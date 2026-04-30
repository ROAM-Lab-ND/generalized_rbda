#include "grbda/Dynamics/ClusterJoints/RevolutePairJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        namespace
        {
            template <typename Scalar>
            std::vector<Body<Scalar>> makeRPBodies(Body<Scalar> &link_1, Body<Scalar> &link_2)
            {
                return {link_1, link_2};
            }

            template <typename Scalar>
            std::vector<JointPtr<Scalar>> makeRPJoints(ori::CoordinateAxis axis1,
                                                       ori::CoordinateAxis axis2)
            {
                using Rev = Joints::Revolute<Scalar>;
                return {std::make_shared<Rev>(axis1), std::make_shared<Rev>(axis2)};
            }

            template <typename Scalar>
            std::shared_ptr<LoopConstraint::Static<Scalar>> makeRPConstraint()
            {
                DMat<Scalar> G = DMat<Scalar>::Identity(2, 2);
                DMat<Scalar> K = DMat<Scalar>::Identity(0, 2);
                return std::make_shared<LoopConstraint::Static<Scalar>>(G, K);
            }
        } // anonymous namespace

        template <typename Scalar>
        RevolutePair<Scalar>::RevolutePair(Body<Scalar> &link_1, Body<Scalar> &link_2,
                                           ori::CoordinateAxis joint_axis_1,
                                           ori::CoordinateAxis joint_axis_2)
            : Generic<Scalar>(
                  makeRPBodies<Scalar>(link_1, link_2),
                  makeRPJoints<Scalar>(joint_axis_1, joint_axis_2),
                  makeRPConstraint<Scalar>()),
              link_1_(link_1), link_2_(link_2)
        {
            link_1_joint_ = this->single_joints_[0];
            link_2_joint_ = this->single_joints_[1];
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevolutePair<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> result;

            const DMat<Scalar> zero = DMat<Scalar>::Zero(this->numVelocities(),
                                                         this->numVelocities());
            result.push_back(std::make_tuple(link_1_, link_1_joint_, zero));
            result.push_back(std::make_tuple(link_2_, link_2_joint_, zero));

            return result;
        }

        template class RevolutePair<double>;
        template class RevolutePair<std::complex<double>>;
        template class RevolutePair<float>;
        template class RevolutePair<casadi::SX>;

    }

} // namespace grbda
