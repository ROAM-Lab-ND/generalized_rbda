#include "RevoluteWithMultipleRotorsJoint.h"
#include "Utils/Utilities/utilities.h"

namespace grbda
{
    namespace GeneralizedJoints
    {

        RevoluteWithMultipleRotorsJoint::RevoluteWithMultipleRotorsJoint(
            Body &link, std::vector<Body> &rotors,
            CoordinateAxis joint_axis, std::vector<CoordinateAxis> &rotor_axes,
            std::vector<double> &gear_ratios)
            : Base(1, 1, 1 + rotors.size()), link_(link), rotors_(rotors)
        {
            const size_t num_rotors = rotors.size();
            if (num_rotors != rotor_axes.size() || num_rotors != gear_ratios.size())
            {
                throw std::invalid_argument("Number of rotor axes and gear ratios must be equal to the number of rotors");
            }

            link_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis));
            for (size_t i(0); i < num_rotors; i++)
            {
                auto rotor = single_joints_.emplace_back(new Joints::Revolute(rotor_axes[i]));
                rotor_joints_.push_back(rotor);
            }

            gamma_ = [gear_ratios, num_rotors](const DVec<double> &y)
            {
                DVec<double> q = DVec<double>::Zero(1 + num_rotors);
                q[0] = y[0];
                for (size_t j(0); j < num_rotors; j++)
                {
                    q[j + 1] = gear_ratios[j] * y[0];
                }
                return q;
            };

            G_ = DMat<double>::Zero(1 + num_rotors, 1);
            G_(0, 0) = 1.;
            for (size_t j(0); j < num_rotors; j++)
            {
                G_(j + 1, 0) = gear_ratios[j];
            }

            g_ = DVec<double>::Zero(1 + num_rotors);

            phi_ = [gear_ratios, num_rotors](const DVec<double> &q)
            {
                DVec<double> out = DVec<double>::Zero(num_rotors);
                for (size_t j(0); j < num_rotors; j++)
                {
                    out[j] = gear_ratios[j] * q[0] - q[1 + j];
                }
                return out;
            };

            K_ = DMat<double>::Zero(num_rotors, 1 + num_rotors);
            for (size_t j(0); j < num_rotors; j++)
            {
                K_(j, 0) = gear_ratios[j];
                K_(j, j + 1) = -1.;
            }

            k_ = DVec<double>::Zero(num_rotors);

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(1, 1 + num_rotors);
            spanning_tree_to_independent_coords_conversion_(0, 0) = 1.;

            DMat<double> S_spanning_tree = DMat<double>::Zero(0, 0);
            for (auto &joint : single_joints_)
            {
                S_spanning_tree = appendEigenMatrix(S_spanning_tree, joint->S());
            }
            S_ = S_spanning_tree * G_;

            // TODO(@MatthewChignoli): How to compute Psi?

            vJ_ = DVec<double>::Zero(6 * (1 + num_rotors));
        }

        void RevoluteWithMultipleRotorsJoint::updateKinematics(const JointState &joint_state)
        {
            // TODO(@MatthewChignoli): Commented out because this depends on the State type
            // if (y.size() != num_independent_positions_ || yd.size() != num_independent_velocities_)
            // throw std::runtime_error("[Revolute w/ Multiple Rotors] Dimension of y or yd is wrong");

            const JointState spanning_joint_state = toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            link_joint_->updateKinematics(q.segment<1>(0), qd.segment<1>(0));
            for (size_t i(0); i < rotors_.size(); i++)
            {
                rotor_joints_[i]->updateKinematics(q.segment<1>(i + 1), qd.segment<1>(i + 1));
            }

            // TODO(@MatthewChignoli): Issue if joint_vel is spanning
            vJ_ = S_ * joint_state.velocity;
        }

        void RevoluteWithMultipleRotorsJoint::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
            if (Xup.getNumOutputBodies() != (int)(1 + rotors_.size()))
                throw std::runtime_error("[Revolute + Mutl Rotor Joint] Xup invalid size");

            Xup[0] = link_joint_->XJ() * link_.Xtree_;
            for (size_t i(0); i < rotors_.size(); i++)
            {
                Xup[i + 1] = rotor_joints_[i]->XJ() * rotors_[i].Xtree_;
            }
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        RevoluteWithMultipleRotorsJoint::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_reflected_inertias_;

            DMat<double> reflected_inertia = DMat<double>::Zero(1, 1);
            for (size_t i(0); i < rotors_.size(); i++)
            {
                DMat<double> S_i = S_.middleRows<6>(6 * (1 + i));
                reflected_inertia += S_i.transpose() * rotors_[i].inertia_.getMatrix() * S_i;
            }

            bodies_joints_and_reflected_inertias_.push_back(
                std::make_tuple(link_, link_joint_, reflected_inertia));

            return bodies_joints_and_reflected_inertias_;
        }

    } // namespace GeneralizedJoints

} // namespace grbda
