#ifndef GRBDA_TEST_HELPERS_H
#define GRBDA_TEST_HELPERS_H

#include <complex>
#include <type_traits>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Utils/OrientationTools.h"

using namespace grbda;

namespace TestHelpers
{
    inline ClusterTreeModel<> extractGenericJointModel(const ClusterTreeModel<> &model)
    {
        using namespace ClusterJoints;

        ClusterTreeModel<> generic_model{};

        for (const auto &cluster : model.clusters())
        {
            std::vector<Body<>> bodies;
            std::vector<JointPtr<double>> joints;

            // Register bodies
            for (auto pair : cluster->bodiesAndJoints())
            {
                Body<> body_i = pair.first;
                bodies.push_back(body_i);

                JointPtr<double> joint_i = pair.second;
                joints.push_back(joint_i);

                bool is_base = body_i.parent_index_ == -1;
                std::string parent_name = is_base ? "ground" : model.body(body_i.parent_index_).name_;
                generic_model.registerBody(body_i.name_, body_i.inertia_, parent_name, body_i.Xtree_);
            }

            // Extract Loop Constraint and Append Cluster
            std::shared_ptr<LoopConstraint::Base<double>> constraint = cluster->joint_->cloneLoopConstraint();
            generic_model.appendRegisteredBodiesAsCluster<Generic<>>(cluster->name_, bodies,
                                                                     joints, constraint);
        }

        // Extract End Effectors & Contact Points
        for (const auto &contact_point : model.contactPoints())
        {
            generic_model.appendContactPoint(contact_point);
        }

        return generic_model;
    }

    inline DVec<casadi::SX> plus(ClusterJointTypes joint_type,
                                 DVec<casadi::SX> q, DVec<casadi::SX> dq)
    {
        using SX = casadi::SX;

        if (joint_type == ClusterJointTypes::Free)
        {
            const Vec3<SX> pos = q.head<3>();
            const Quat<SX> quat = q.tail<4>();

            const Vec3<SX> dquat = dq.head<3>();
            const Vec3<SX> dpos = dq.tail<3>();

            Vec7<SX> q_plus_dq_vec;

            const Mat3<SX> R = ori::quaternionToRotationMatrix(quat);
            q_plus_dq_vec.head<3>() = pos + R.transpose() * dpos;

            Quat<casadi::SX> dquat_vec(0, dquat[0], dquat[1], dquat[2]);
            q_plus_dq_vec.template tail<4>() = quat + 0.5 * ori::quatProduct(quat, dquat_vec);

            return q_plus_dq_vec;
        }
        else
        {
            return q + dq;
        }
    }

    inline casadi::DMVector plus(const casadi::DMVector q, const casadi::DMVector dq)
    {
        casadi::DMVector q_plus_dq_vec = q;

        int pos_idx = 0;
        int vel_idx = 0;

        if (q.size() != dq.size()) // floating base joint
        {
            DVec<casadi::SX> q_vec(7);
            q_vec << q[0], q[1], q[2], q[3], q[4], q[5], q[6];

            DVec<casadi::SX> dq_vec(6);
            dq_vec << dq[0], dq[1], dq[2], dq[3], dq[4], dq[5];

            DVec<casadi::SX> qfb_plus_dq_vec = plus(ClusterJointTypes::Free, q_vec, dq_vec);

            for (int i = 0; i < 7; i++)
            {
                q_plus_dq_vec[i] = qfb_plus_dq_vec[i];
            }

            pos_idx = 7;
            vel_idx = 6;
        }

        for (int i = pos_idx; i < q.size(); i++)
        {
            q_plus_dq_vec[i] = q[i] + dq[vel_idx];
            vel_idx++;
        }

        return q_plus_dq_vec;
    }

} // namespace TestHelpers

// ─── Lie-group state perturbation helpers (shared by simple and complex-step tests) ───

// Normalize quaternion for real scalar types; no-op for complex (to preserve imaginary part).
template<typename Derived>
typename std::enable_if<std::is_arithmetic<typename Derived::Scalar>::value, void>::type
normalizeQuaternionIfReal(Eigen::MatrixBase<Derived>& quat) {
    const_cast<Eigen::MatrixBase<Derived>&>(quat).normalize();
}
template<typename Derived>
typename std::enable_if<!std::is_arithmetic<typename Derived::Scalar>::value, void>::type
normalizeQuaternionIfReal(Eigen::MatrixBase<Derived>&) {}

// Retraction map for a single joint cluster's configuration.
//   q0 : spanning position (7D for free joint, 1D for revolute, etc.)
//   dq : velocity-space perturbation (6D for free joint, 1D for revolute, etc.)
template<typename T>
DVec<T> lieGroupConfigurationAddition(const DVec<T>& q0, const DVec<T>& dq, bool floating_base) {
    if (!floating_base)
        return q0 + dq;

    const int nj = (int)dq.size() - 6;
    DVec<T> q_new = q0;
    q_new.tail(nj) += dq.tail(nj);

    Eigen::Matrix<T, 3, 1> p    = q0.head(3);
    Eigen::Matrix<T, 4, 1> quat = q0.template segment<4>(3);
    Eigen::Matrix<T, 3, 1> omega = dq.head(3);

    bool has_imag = false;
    if constexpr (!std::is_arithmetic<T>::value) {
        for (int i = 0; i < 3; ++i)
            if (std::abs(std::imag(omega[i])) > 1e-30) { has_imag = true; break; }
    }

    Eigen::Matrix<T, 4, 1> delta_quat;
    if (has_imag) {
        delta_quat[0] = T(0.0);
        delta_quat.template tail<3>() = omega / T(2.0);
    } else {
        T theta = omega.norm();
        if (std::abs(theta) < 1e-10) {
            delta_quat[0] = T(1.0);
            delta_quat.template tail<3>() = omega / T(2.0);
        } else {
            T ht = theta / T(2.0);
            delta_quat[0] = std::cos(ht);
            delta_quat.template tail<3>() = std::sin(ht) * omega / theta;
        }
    }

    Eigen::Matrix<T, 4, 1> quat_new;
    if (has_imag) {
        T sca = delta_quat[0];
        Eigen::Matrix<T, 3, 1> vec = delta_quat.template tail<3>();
        auto qt = quat.template tail<3>();
        T vdq = (vec.transpose() * qt)(0, 0);
        quat_new[0] = (T(1.0) + sca) * quat[0] - vdq;
        Eigen::Matrix<T, 3, 1> cross;
        cross[0] = qt[1]*vec[2] - qt[2]*vec[1];
        cross[1] = qt[2]*vec[0] - qt[0]*vec[2];
        cross[2] = qt[0]*vec[1] - qt[1]*vec[0];
        quat_new.template tail<3>() = vec*quat[0] + (T(1.0)+sca)*qt + cross;
    } else {
        quat_new = ori::quatProduct(quat, delta_quat);
        normalizeQuaternionIfReal(quat_new);
    }

    Eigen::Matrix<T, 4, 1> qn = quat / quat.norm();
    Eigen::Matrix<T, 3, 3> R  = ori::quaternionToRotationMatrix(qn);
    Eigen::Matrix<T, 3, 1> vb = dq.template segment<3>(3);
    q_new.head(3) = p + R.transpose() * vb;
    q_new.template segment<4>(3) = quat_new;
    return q_new;
}

// Build a ModelState<T> from flat spanning-coordinate vectors.
// Position JointCoordinates are tagged isSpanning=(np>nv) so toSpanningTreeState
// copies them directly rather than calling gamma().
template<typename T>
ModelState<T> makeModelState(const ClusterTreeModel<double>& model,
                             const DVec<double>& q, const DVec<double>& qd) {
    ModelState<T> result;
    result.reserve(model.clusters().size());
    int q_off = 0, qd_off = 0;
    for (const auto& cluster : model.clusters()) {
        const int np = cluster->num_positions_;
        const int nv = cluster->num_velocities_;
        result.push_back(JointState<T>(
            JointCoordinate<T>(q.segment(q_off, np).template cast<T>(), (np > nv)),
            JointCoordinate<T>(qd.segment(qd_off, nv).template cast<T>(), false)));
        q_off  += np;
        qd_off += nv;
    }
    return result;
}

// Apply a perturbation in minimal (independent) coordinates to a ModelState.
//   Floating-base clusters  : Lie group retraction via lieGroupConfigurationAddition.
//   Implicit-constraint clusters : G-based spanning perturbation (dq_span = G * dq_ind).
//   Simple joints            : direct addition.
// dq / dqd are flat vectors in independent coordinates across all clusters.
template<typename T>
ModelState<T> applyMinimalPerturbation(const ClusterTreeModel<double>& model_ref,
                                       const ModelState<T>& state,
                                       const DVec<T>& dq, const DVec<T>& dqd) {
    ModelState<T> result;
    result.reserve(state.size());
    int off = 0;
    for (size_t c = 0; c < model_ref.clusters().size(); ++c) {
        const auto& cluster = model_ref.clusters()[c];
        const int np = cluster->num_positions_;
        const int nv = cluster->num_velocities_;
        const bool is_fb     = (np == 7 && nv == 6);
        const bool is_implicit = (np > nv) && !is_fb;

        DVec<T> new_pos(state[c].position);
        DVec<T> new_vel = DVec<T>(state[c].velocity) + dqd.segment(off, nv);

        if (is_fb) {
            new_pos = lieGroupConfigurationAddition(
                DVec<T>(state[c].position), DVec<T>(dq.segment(off, nv)), true);
        } else if (is_implicit) {
            DVec<double> q_real(np);
            for (int k = 0; k < np; ++k) q_real(k) = std::real(state[c].position[k]);
            auto lc = cluster->joint_->cloneLoopConstraint();
            lc->updateJacobians(JointCoordinate<double>(q_real, true));
            new_pos += lc->G().template cast<T>() * DVec<T>(dq.segment(off, nv));
        } else {
            new_pos += dq.segment(off, nv);
        }

        result.push_back(JointState<T>(
            JointCoordinate<T>(new_pos, state[c].position.isSpanning()),
            JointCoordinate<T>(new_vel, state[c].velocity.isSpanning())));
        off += nv;
    }
    return result;
}

// Five-point stencil finite-difference Jacobian.
// f'(x) ≈ [-f(x+2h) + 8f(x+h) - 8f(x-h) + f(x-2h)] / (12h)
template<typename Func>
Eigen::MatrixXd finiteDifferenceJacobian(Func func, const Eigen::VectorXd& point, double h) {
    const int n = point.size();
    Eigen::VectorXd f0 = func(point);
    const int m = f0.size();
    Eigen::MatrixXd jacobian(m, n);
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd p1 = point, p2 = point, p3 = point, p4 = point;
        p1[i] += 2*h; p2[i] += h; p3[i] -= h; p4[i] -= 2*h;
        jacobian.col(i) = (-func(p1) + 8*func(p2) - 8*func(p3) + func(p4)) / (12*h);
    }
    return jacobian;
}

// Build a random ModelState from each cluster's randomJointState().
// Pass enforce_constraints=true for models with loop constraints.
inline ModelState<double> randomModelState(const ClusterTreeModel<double>& model,
                                           bool enforce_constraints = false) {
    ModelState<double> state;
    for (const auto& c : model.clusters())
        state.push_back(c->joint_->randomJointState(enforce_constraints));
    return state;
}

#endif // GRBDA_TEST_HELPERS_H
