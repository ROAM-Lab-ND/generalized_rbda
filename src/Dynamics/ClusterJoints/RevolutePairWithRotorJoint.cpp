#include "grbda/Dynamics/ClusterJoints/RevolutePairWithRotorJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        namespace
        {
            template <typename Scalar>
            std::vector<Body<Scalar>> makeRPWRBodies(
                const ParallelBeltTransmissionModule<1, Scalar> &m1,
                const ParallelBeltTransmissionModule<2, Scalar> &m2)
            {
                // Body has const members so is not copy-assignable; sort a permutation instead.
                const Body<Scalar> *src[4] = {&m1.body_, &m1.rotor_, &m2.rotor_, &m2.body_};
                int order[4] = {0, 1, 2, 3};
                std::sort(std::begin(order), std::end(order), [&](int a, int b) {
                    return src[a]->sub_index_within_cluster_ < src[b]->sub_index_within_cluster_;
                });
                std::vector<Body<Scalar>> bodies;
                for (int i : order)
                    bodies.push_back(*src[i]);
                return bodies;
            }

            template <typename Scalar>
            std::vector<JointPtr<Scalar>> makeRPWRJoints(
                const ParallelBeltTransmissionModule<1, Scalar> &m1,
                const ParallelBeltTransmissionModule<2, Scalar> &m2)
            {
                using Rev = Joints::Revolute<Scalar>;
                // Indices must match makeRPWRBodies insertion order.
                int sub[4] = {
                    m1.body_.sub_index_within_cluster_,
                    m1.rotor_.sub_index_within_cluster_,
                    m2.rotor_.sub_index_within_cluster_,
                    m2.body_.sub_index_within_cluster_,
                };
                JointPtr<Scalar> src[4] = {
                    std::make_shared<Rev>(m1.joint_axis_),
                    std::make_shared<Rev>(m1.rotor_axis_),
                    std::make_shared<Rev>(m2.rotor_axis_),
                    std::make_shared<Rev>(m2.joint_axis_),
                };
                int order[4] = {0, 1, 2, 3};
                std::sort(std::begin(order), std::end(order),
                          [&](int a, int b) { return sub[a] < sub[b]; });
                std::vector<JointPtr<Scalar>> joints;
                for (int i : order)
                    joints.push_back(src[i]);
                return joints;
            }

            template <typename Scalar>
            std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> makeRPWRConstraint(
                const ParallelBeltTransmissionModule<1, Scalar> &m1,
                const ParallelBeltTransmissionModule<2, Scalar> &m2)
            {
                using SX = casadi::SX;

                const int l1 = m1.body_.sub_index_within_cluster_;
                const int l2 = m2.body_.sub_index_within_cluster_;
                const int r1 = m1.rotor_.sub_index_within_cluster_;
                const int r2 = m2.rotor_.sub_index_within_cluster_;

                Vec2<Scalar> gear_ratios{m1.gear_ratio_, m2.gear_ratio_};
                Eigen::DiagonalMatrix<Scalar, 2> rotor_matrix(gear_ratios);
                Mat2<Scalar> belt_matrix;
                belt_matrix << beltMatrixRowFromBeltRatios(m1.belt_ratios_), Scalar(0),
                    beltMatrixRowFromBeltRatios(m2.belt_ratios_);
                Mat2<Scalar> ratio_product = rotor_matrix * belt_matrix;

                // K * q_span = 0: rotor velocities are linearly determined by link velocities
                DMat<Scalar> K = DMat<Scalar>::Zero(2, 4);
                int cnstr1 = (r1 > r2) ? 1 : 0;
                int cnstr2 = (r2 > r1) ? 1 : 0;
                K(cnstr1, r1) = Scalar(-1.);
                K(cnstr1, l1) = ratio_product(0, 0);
                K(cnstr2, r2) = Scalar(-1.);
                K(cnstr2, l1) = ratio_product(1, 0);
                K(cnstr2, l2) = ratio_product(1, 1);

                // sym_phi is the CasADi version of K * q required by GenericImplicit
                DMat<double> K_double(2, 4);
                for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 4; j++) {
                        if constexpr (std::is_same_v<Scalar, std::complex<double>>)
                            K_double(i, j) = std::real(K(i, j));
                        else
                            K_double(i, j) = static_cast<double>(K(i, j));
                    }
                std::vector<bool> is_ind(4, false);
                is_ind[l1] = true;
                is_ind[l2] = true;

                auto sym_phi = [K_double](const JointCoordinate<SX> &jp) -> DVec<SX>
                {
                    DVec<SX> phi = DVec<SX>::Zero(2);
                    for (int i = 0; i < 2; i++)
                        for (int j = 0; j < 4; j++)
                            phi(i) += SX(K_double(i, j)) * jp(j);
                    return phi;
                };

                return std::make_shared<LoopConstraint::GenericImplicit<Scalar>>(
                    is_ind, sym_phi);
            }
        } // anonymous namespace

        template <typename Scalar>
        RevolutePairWithRotor<Scalar>::RevolutePairWithRotor(
            ProximalTransmission &module_1, DistalTransmission &module_2)
            : Generic<Scalar>(
                  makeRPWRBodies<Scalar>(module_1, module_2),
                  makeRPWRJoints<Scalar>(module_1, module_2),
                  makeRPWRConstraint<Scalar>(module_1, module_2)),
              link1_(module_1.body_), link2_(module_2.body_),
              rotor1_(module_1.rotor_), rotor2_(module_2.rotor_),
              link1_index_(module_1.body_.sub_index_within_cluster_),
              link2_index_(module_2.body_.sub_index_within_cluster_),
              rotor1_index_(module_1.rotor_.sub_index_within_cluster_),
              rotor2_index_(module_2.rotor_.sub_index_within_cluster_)
        {
            Vec2<Scalar> gear_ratios{module_1.gear_ratio_, module_2.gear_ratio_};
            Eigen::DiagonalMatrix<Scalar, 2> rotor_matrix(gear_ratios);
            Mat2<Scalar> belt_matrix;
            belt_matrix << beltMatrixRowFromBeltRatios(module_1.belt_ratios_), Scalar(0),
                beltMatrixRowFromBeltRatios(module_2.belt_ratios_);
            ratio_product_ = rotor_matrix * belt_matrix;
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevolutePairWithRotor<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> result;

            // The gear/belt constraint is linear (constant K), so G is state-independent.
            // Rotor 1 velocity = ratio_product_(0,0)*q_link1 + 0*q_link2
            // Rotor 2 velocity = ratio_product_(1,0)*q_link1 + ratio_product_(1,1)*q_link2
            // S_dep_i = S_rotor_i * [ratio_product row i]  (6 x 2 matrix)
            // This avoids relying on this->S_ which is only set after updateKinematics().

            const DMat<Scalar> S_rotor1 = this->single_joints_[rotor1_index_]->S();
            DMat<Scalar> S_dep_1 = S_rotor1 * ratio_product_.row(0);
            Mat6<Scalar> Ir1 = rotor1_.inertia_.getMatrix();
            DMat<Scalar> ref_inertia_1 = S_dep_1.transpose() * Ir1 * S_dep_1;
            result.push_back(std::make_tuple(link1_, this->single_joints_[link1_index_], ref_inertia_1));

            const DMat<Scalar> S_rotor2 = this->single_joints_[rotor2_index_]->S();
            DMat<Scalar> S_dep_2 = S_rotor2 * ratio_product_.row(1);
            Mat6<Scalar> Ir2 = rotor2_.inertia_.getMatrix();
            DMat<Scalar> ref_inertia_2 = S_dep_2.transpose() * Ir2 * S_dep_2;
            result.push_back(std::make_tuple(link2_, this->single_joints_[link2_index_], ref_inertia_2));

            return result;
        }

        template class RevolutePairWithRotor<double>;
        template class RevolutePairWithRotor<std::complex<double>>;
        template class RevolutePairWithRotor<float>;
        template class RevolutePairWithRotor<casadi::SX>;

    }

} // namespace grbda
