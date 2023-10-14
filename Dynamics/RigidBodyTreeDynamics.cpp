#include "RigidBodyTreeModel.h"

namespace grbda
{

    template <typename Scalar>
    const D6Mat<Scalar> &
    RigidBodyTreeModel<Scalar>::contactJacobianWorldFrame(const std::string &cp_name)
    {
        this->forwardKinematics();
        updateLoopConstraints();

        D6Mat<Scalar> J_spanning = D6Mat<Scalar>::Zero(6, this->getNumDegreesOfFreedom());

        ContactPoint<Scalar> &cp = this->contact_points_[this->contact_name_to_contact_index_.at(cp_name)];
        const size_t &i = cp.body_index_;
        const auto node_i = this->getNodeContainingBody(i);
        const spatial::Transform<Scalar> Xa = node_i->Xa_[0];
        const Mat3<Scalar> R_link_to_world = Xa.getRotation().transpose();
        Mat6<Scalar> Xout = spatial::createSXform(R_link_to_world, cp.local_offset_);

        int j = (int)i;
        while (j > -1)
        {
            const auto node_j = this->getNodeContainingBody(j);
            const int &vel_idx = node_j->velocity_index_;
            const int &num_vel = node_j->num_velocities_;

            const D6Mat<Scalar> &S = node_j->S();
            J_spanning.middleCols(vel_idx, num_vel) = Xout * S;

            const Mat6<Scalar> Xup = node_j->Xup_[0].toMatrix();
            Xout = Xout * Xup;

            j = node_j->parent_index_;
        }

        cp.jacobian_ = J_spanning * loop_constraints_.G();
        return cp.jacobian_;
    }

    template <typename Scalar>
    D6Mat<Scalar> RigidBodyTreeModel<Scalar>::contactJacobianBodyFrame(const std::string &cp_name)
    {
        this->forwardKinematics();
        updateLoopConstraints();

        D6Mat<Scalar> J_spanning = D6Mat<Scalar>::Zero(6, this->getNumDegreesOfFreedom());

        ContactPoint<Scalar> &cp = this->contact_points_[this->contact_name_to_contact_index_.at(cp_name)];
        const size_t &i = cp.body_index_;
        const auto node_i = this->getNodeContainingBody(i);
        Mat6<Scalar> Xout = spatial::createSXform(Mat3<Scalar>::Identity(), cp.local_offset_);

        int j = (int)i;
        while (j > -1)
        {
            const auto node_j = this->getNodeContainingBody(j);
            const int &vel_idx = node_j->velocity_index_;
            const int &num_vel = node_j->num_velocities_;

            const D6Mat<Scalar> &S = node_j->S();
            J_spanning.middleCols(vel_idx, num_vel) = Xout * S;

            const Mat6<Scalar> Xup = node_j->Xup_[0].toMatrix();
            Xout = Xout * Xup;

            j = node_j->parent_index_;
        }

        return J_spanning * loop_constraints_.G();
    }

    template <typename Scalar>
    DVec<Scalar> RigidBodyTreeModel<Scalar>::forwardDynamics(const DVec<Scalar> &tau)
    {
        typedef typename CorrectMatrixLltType<Scalar>::type LltType;
     
        this->forwardKinematics();
        updateLoopConstraints();
        this->compositeRigidBodyAlgorithm();
        this->updateBiasForceVector();

        switch (forward_dynamics_method_)
        {
        case FwdDynMethod::Projection:
        {
            // Based on Method 3 in Featherstone Ch 8.5
            const DMat<Scalar> &G = loop_constraints_.G();
            const DMat<Scalar> &G_transpose = loop_constraints_.G_transpose();
            const DVec<Scalar> &g = loop_constraints_.g();

            const DMat<Scalar> A = G_transpose * this->H_ * G;
            const DVec<Scalar> b = tau - G_transpose * (this->C_ + this->H_ * g);
            const DVec<Scalar> ydd = LltType(A).solve(b);
            return G * ydd + g;
        }

        case FwdDynMethod::LagrangeMultiplierCustom:
        {
            // Based on Method 2 in Featherstone Ch 8.5 (using custom sparse factorization)

            // Factorize H into L^T*L
            factorization::LTL L(this->H_, expanded_tree_parent_indices_);

            // Calculate tau_prime
            DVec<Scalar> tau_full = loop_constraints_.G_tranpose_pinv() * tau;
            DVec<Scalar> tau_prime = tau_full - this->C_;

            // Calculate Y and z via back-subsition
            DMat<Scalar> Y = L.inverseTransposeMatrixProduct(loop_constraints_.K_transpose());
            DVec<Scalar> z = L.inverseTransposeProduct(tau_prime);

            // Calculate A and b
            DMat<Scalar> A = Y.transpose() * Y;
            DVec<Scalar> b = loop_constraints_.k() - Y.transpose() * z;

            // Solve Linear System A*lambda = b
            DVec<Scalar> lambda = A.size() > 0 ? solveLinearSystem(A, b)
                                               : DVec<Scalar>::Zero(0);

            // Solve for qdd using the factors from step 1
            return L.solve(tau_prime + loop_constraints_.K_transpose() * lambda);
        }

        case FwdDynMethod::LagrangeMultiplierEigen:
        {
            // Based on Method 2 in Featherstone Ch 8.5 (using Eigen factorization)

            LltType lltOfH = LltType(this->H_);
            DMat<Scalar> A = loop_constraints_.K() * lltOfH.solve(loop_constraints_.K_transpose());
            DVec<Scalar> tau_full = loop_constraints_.G_tranpose_pinv() * tau;
            DVec<Scalar> tau_prime = tau_full - this->C_;
            DVec<Scalar> b = loop_constraints_.k() -
                             loop_constraints_.K() * lltOfH.solve(tau_prime);
            DVec<Scalar> lambda = A.size() > 0 ? solveLinearSystem(A, b)
                                               : DVec<Scalar>::Zero(0);
            return lltOfH.solve(tau_prime + loop_constraints_.K_transpose() * lambda);
        }

        default:
            throw std::runtime_error("Invalid forward dynamics method specified.");
        }
    }

    template <typename Scalar>
    DVec<Scalar> RigidBodyTreeModel<Scalar>::inverseDynamics(const DVec<Scalar> &ydd)
    {
        this->forwardKinematics();
        updateLoopConstraints();

        const DVec<Scalar> qdd_spanning = loop_constraints_.G() * ydd + loop_constraints_.g();
        const DVec<Scalar> tau_id = this->recursiveNewtonEulerAlgorithm(qdd_spanning);
        return loop_constraints_.G_transpose() * tau_id;
    }

    template <typename Scalar>
    DMat<Scalar> RigidBodyTreeModel<Scalar>::inverseOperationalSpaceInertiaMatrix()
    {
        const DMat<Scalar> H = getMassMatrix();
        DMat<Scalar> J_stacked = DMat<Scalar>::Zero(6 * this->getNumEndEffectors(), H.rows());

        int ee_cnt = 0;
        for (int i = 0; i < (int)this->contact_points_.size(); i++)
        {
            const ContactPoint<Scalar> &cp = this->contact_points_[i];
            if (!cp.is_end_effector_)
                continue;
            J_stacked.template middleRows<6>(6 * ee_cnt++) = contactJacobianBodyFrame(cp.name_);
        }
        return J_stacked * matrixInverse(H) * J_stacked.transpose();
    }

    template <typename Scalar>
    Scalar RigidBodyTreeModel<Scalar>::applyTestForce(
        const std::string &cp_name, const Vec3<Scalar> &force, DVec<Scalar> &dstate_out)
    {
        const D3Mat<Scalar> J = contactJacobianWorldFrame(cp_name).template bottomRows<3>();
        const DMat<Scalar> H = getMassMatrix();
        const DMat<Scalar> H_inv = matrixInverse(H);
        const DMat<Scalar> inv_ops_inertia = J * H_inv * J.transpose();
        dstate_out = H_inv * (J.transpose() * force);
        return force.dot(inv_ops_inertia * force);
    }

    template class RigidBodyTreeModel<double>;
    template class RigidBodyTreeModel<float>;
    template class RigidBodyTreeModel<casadi::SX>;

} // namespace grbda
