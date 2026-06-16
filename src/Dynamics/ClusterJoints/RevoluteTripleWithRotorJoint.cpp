#include "grbda/Dynamics/ClusterJoints/RevoluteTripleWithRotorJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        namespace
        {
            template <typename Scalar>
            std::vector<Body<Scalar>> makeRTWRBodies(
                const ParallelBeltTransmissionModule<1, Scalar> &m1,
                const ParallelBeltTransmissionModule<2, Scalar> &m2,
                const ParallelBeltTransmissionModule<3, Scalar> &m3)
            {
                const Body<Scalar> *src[6] = {
                    &m1.body_, &m1.rotor_,
                    &m2.body_, &m2.rotor_,
                    &m3.body_, &m3.rotor_,
                };
                int order[6] = {0, 1, 2, 3, 4, 5};
                std::sort(std::begin(order), std::end(order), [&](int a, int b) {
                    return src[a]->sub_index_within_cluster_ < src[b]->sub_index_within_cluster_;
                });
                std::vector<Body<Scalar>> bodies;
                for (int i : order)
                    bodies.push_back(*src[i]);
                return bodies;
            }

            template <typename Scalar>
            std::vector<JointPtr<Scalar>> makeRTWRJoints(
                const ParallelBeltTransmissionModule<1, Scalar> &m1,
                const ParallelBeltTransmissionModule<2, Scalar> &m2,
                const ParallelBeltTransmissionModule<3, Scalar> &m3)
            {
                using Rev = Joints::Revolute<Scalar>;
                int sub[6] = {
                    m1.body_.sub_index_within_cluster_,
                    m1.rotor_.sub_index_within_cluster_,
                    m2.body_.sub_index_within_cluster_,
                    m2.rotor_.sub_index_within_cluster_,
                    m3.body_.sub_index_within_cluster_,
                    m3.rotor_.sub_index_within_cluster_,
                };
                JointPtr<Scalar> src[6] = {
                    std::make_shared<Rev>(m1.joint_axis_),
                    std::make_shared<Rev>(m1.rotor_axis_),
                    std::make_shared<Rev>(m2.joint_axis_),
                    std::make_shared<Rev>(m2.rotor_axis_),
                    std::make_shared<Rev>(m3.joint_axis_),
                    std::make_shared<Rev>(m3.rotor_axis_),
                };
                int order[6] = {0, 1, 2, 3, 4, 5};
                std::sort(std::begin(order), std::end(order),
                          [&](int a, int b) { return sub[a] < sub[b]; });
                std::vector<JointPtr<Scalar>> joints;
                for (int i : order)
                    joints.push_back(src[i]);
                return joints;
            }

            template <typename Scalar>
            std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> makeRTWRConstraint(
                const ParallelBeltTransmissionModule<1, Scalar> &m1,
                const ParallelBeltTransmissionModule<2, Scalar> &m2,
                const ParallelBeltTransmissionModule<3, Scalar> &m3)
            {
                using SX = casadi::SX;

                const int l1 = m1.body_.sub_index_within_cluster_;
                const int l2 = m2.body_.sub_index_within_cluster_;
                const int l3 = m3.body_.sub_index_within_cluster_;
                const int r1 = m1.rotor_.sub_index_within_cluster_;
                const int r2 = m2.rotor_.sub_index_within_cluster_;
                const int r3 = m3.rotor_.sub_index_within_cluster_;

                Vec3<Scalar> gear_ratios{m1.gear_ratio_, m2.gear_ratio_, m3.gear_ratio_};
                Eigen::DiagonalMatrix<Scalar, 3> rotor_matrix(gear_ratios);

                Mat3<Scalar> belt_matrix = Mat3<Scalar>::Zero();
                belt_matrix.template block<1,1>(0,0) = beltMatrixRowFromBeltRatios(m1.belt_ratios_);
                belt_matrix.template block<1,2>(1,0) = beltMatrixRowFromBeltRatios(m2.belt_ratios_);
                belt_matrix.template block<1,3>(2,0) = beltMatrixRowFromBeltRatios(m3.belt_ratios_);

                Mat3<Scalar> ratio_product = rotor_matrix * belt_matrix;

                // K * q_span = 0: rotor i velocity is a linear combination of link velocities.
                // Row ordering follows ascending rotor sub_index for determinism.
                DMat<Scalar> K = DMat<Scalar>::Zero(3, 6);

                int rotor_cols[3] = {r1, r2, r3};
                int order[3] = {0, 1, 2};
                std::sort(std::begin(order), std::end(order),
                          [&](int a, int b) { return rotor_cols[a] < rotor_cols[b]; });

                // rotor 0 (proximal):     depends on l1 only
                // rotor 1 (intermediate): depends on l1, l2
                // rotor 2 (distal):       depends on l1, l2, l3
                int link_sets[3][3] = {{l1, -1, -1}, {l1, l2, -1}, {l1, l2, l3}};
                int ratio_cols[3][3] = {{0, -1, -1}, {0, 1, -1}, {0, 1, 2}};

                for (int ci = 0; ci < 3; ci++) {
                    int src = order[ci];
                    K(ci, rotor_cols[src]) = Scalar(-1.);
                    for (int j = 0; j < 3; j++)
                        if (link_sets[src][j] >= 0)
                            K(ci, link_sets[src][j]) = ratio_product(src, ratio_cols[src][j]);
                }

                DMat<double> K_double(3, 6);
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 6; j++) {
                        if constexpr (std::is_same_v<Scalar, std::complex<double>>)
                            K_double(i, j) = std::real(K(i, j));
                        else
                            K_double(i, j) = static_cast<double>(K(i, j));
                    }

                std::vector<bool> is_ind(6, false);
                is_ind[l1] = true;
                is_ind[l2] = true;
                is_ind[l3] = true;

                auto sym_phi = [K_double](const JointCoordinate<SX> &jp) -> DVec<SX>
                {
                    DVec<SX> phi = DVec<SX>::Zero(3);
                    for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 6; j++)
                            phi(i) += SX(K_double(i, j)) * jp(j);
                    return phi;
                };

                return std::make_shared<LoopConstraint::GenericImplicit<Scalar>>(
                    is_ind, sym_phi);
            }
        } // anonymous namespace

        template <typename Scalar>
        RevoluteTripleWithRotor<Scalar>::RevoluteTripleWithRotor(
            const ProximalTransmission &module_1,
            const IntermediateTransmission &module_2,
            const DistalTransmission &module_3)
            : Generic<Scalar>(
                  makeRTWRBodies<Scalar>(module_1, module_2, module_3),
                  makeRTWRJoints<Scalar>(module_1, module_2, module_3),
                  makeRTWRConstraint<Scalar>(module_1, module_2, module_3)),
              link1_(module_1.body_), link2_(module_2.body_), link3_(module_3.body_),
              rotor1_(module_1.rotor_), rotor2_(module_2.rotor_), rotor3_(module_3.rotor_),
              link1_index_(module_1.body_.sub_index_within_cluster_),
              link2_index_(module_2.body_.sub_index_within_cluster_),
              link3_index_(module_3.body_.sub_index_within_cluster_),
              rotor1_index_(module_1.rotor_.sub_index_within_cluster_),
              rotor2_index_(module_2.rotor_.sub_index_within_cluster_),
              rotor3_index_(module_3.rotor_.sub_index_within_cluster_)
        {
            Vec3<Scalar> gear_ratios{module_1.gear_ratio_, module_2.gear_ratio_, module_3.gear_ratio_};
            Eigen::DiagonalMatrix<Scalar, 3> rotor_matrix(gear_ratios);

            Mat3<Scalar> belt_matrix = Mat3<Scalar>::Zero();
            belt_matrix.template block<1,1>(0,0) = beltMatrixRowFromBeltRatios(module_1.belt_ratios_);
            belt_matrix.template block<1,2>(1,0) = beltMatrixRowFromBeltRatios(module_2.belt_ratios_);
            belt_matrix.template block<1,3>(2,0) = beltMatrixRowFromBeltRatios(module_3.belt_ratios_);

            ratio_product_ = rotor_matrix * belt_matrix;
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevoluteTripleWithRotor<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> result;

            const DMat<Scalar> S_rotor1 = this->single_joints_[rotor1_index_]->S();
            DMat<Scalar> S_dep_1 = S_rotor1 * ratio_product_.row(0);
            Mat6<Scalar> Ir1 = rotor1_.inertia_.getMatrix();
            result.push_back(std::make_tuple(link1_, this->single_joints_[link1_index_],
                                             S_dep_1.transpose() * Ir1 * S_dep_1));

            const DMat<Scalar> S_rotor2 = this->single_joints_[rotor2_index_]->S();
            DMat<Scalar> S_dep_2 = S_rotor2 * ratio_product_.row(1);
            Mat6<Scalar> Ir2 = rotor2_.inertia_.getMatrix();
            result.push_back(std::make_tuple(link2_, this->single_joints_[link2_index_],
                                             S_dep_2.transpose() * Ir2 * S_dep_2));

            const DMat<Scalar> S_rotor3 = this->single_joints_[rotor3_index_]->S();
            DMat<Scalar> S_dep_3 = S_rotor3 * ratio_product_.row(2);
            Mat6<Scalar> Ir3 = rotor3_.inertia_.getMatrix();
            result.push_back(std::make_tuple(link3_, this->single_joints_[link3_index_],
                                             S_dep_3.transpose() * Ir3 * S_dep_3));

            return result;
        }

        template class RevoluteTripleWithRotor<double>;
        template class RevoluteTripleWithRotor<std::complex<double>>;
        template class RevoluteTripleWithRotor<casadi::SX>;

    }

} // namespace grbda
