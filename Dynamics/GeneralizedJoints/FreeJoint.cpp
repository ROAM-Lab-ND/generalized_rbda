#include "FreeJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        Free::Free(const Body &body) : Base(7, 6, 1)
        {
            if (body.parent_index_ >= 0)
                throw std::runtime_error("Free joint is only valid as the first joint in a tree and thus cannot have a parent body");

            S_.setIdentity();
            Psi_.setIdentity();
            vJ_ = SVec<double>::Zero();

            single_joints_.emplace_back(new Joints::Free());

            gamma_ = [](DVec<double> y)
            { return y; };
            G_ = DMat<double>::Identity(6, 6);
            g_ = DVec<double>::Zero(6);

            phi_ = [](DVec<double> q)
            { return DVec<double>::Zero(0); };
            K_ = DMat<double>::Identity(0, 6);
            k_ = DVec<double>::Zero(0);

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Identity(6, 6);
        }

        void Free::updateKinematics(const State<double> &joint_pos, const State<double> &joint_vel)
        {
            single_joints_[0]->updateKinematics(joint_pos, joint_vel);
            vJ_ = S_ * joint_vel;
        }

        void Free::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
            if (Xup.getNumOutputBodies() != 1 || Xup.getNumParentBodies() != 1)
                throw std::runtime_error("[Free Joint] Xup must be 6x6");
            Xup[0] = single_joints_[0]->XJ();
        }

    }

} // namespace grbda
