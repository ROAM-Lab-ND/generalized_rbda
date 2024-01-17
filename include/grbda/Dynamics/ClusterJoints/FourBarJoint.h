#ifndef GRBDA_FOUR_BAR_JOINT_H
#define GRBDA_FOUR_BAR_JOINT_H

#include "grbda/Utils/Utilities.h"
// #include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
        // TODO(@MatthewChignoli): Later we can worry about how we handle the fixed offset with this loop constraint compared to the generic one
        // TODO(@MatthewChignoli): Missing the fixed offset between the joints of link 1 and link 3
        template <typename Scalar = double>
        struct FourBar : Base<Scalar>
        {
            typedef typename CorrectMatrixInverseType<Scalar>::type InverseType;

            FourBar(std::vector<Scalar> path1_link_lengths, std::vector<Scalar> path2_link_lengths,
                    Vec2<Scalar> offset, int independent_coordinate);

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<FourBar<Scalar>>(*this);
            }

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;

            void updateBiases(const JointState<Scalar> &joint_state) override;

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override
            {
                throw std::runtime_error("FourBar: Explicit constraint does not exist");
            }

            void createPhiRootFinder() override
            {
                using SX = casadi::SX;

                // Create a symbolic LoopConstraint::FourBar
                std::vector<SX> path1_link_lengths_sym, path2_link_lengths_sym;
                for (size_t i = 0; i < path1_link_lengths_.size(); i++)
                {
                    path1_link_lengths_sym.push_back(path1_link_lengths_[i]);
                }
                for (size_t i = 0; i < path2_link_lengths_.size(); i++)
                {
                    path2_link_lengths_sym.push_back(path2_link_lengths_[i]);
                }
                Vec2<SX> offset_sym{offset_[0], offset_[1]};
                FourBar<SX> four_bar_sym(path1_link_lengths_sym, path2_link_lengths_sym, offset_sym, independent_coordinate_);

                // Define symbolic variables
                SX cs_q_sym = SX::sym("q", this->numSpanningPos(), 1);
                DVec<SX> q_sym(this->numSpanningPos());
                casadi::copy(cs_q_sym, q_sym);

                // Compute constraint violation
                JointCoordinate<SX> joint_pos(q_sym, false);
                DVec<SX> phi_sx = four_bar_sym.phi(joint_pos);
                SX f = phi_sx.transpose() * phi_sx;

                // Create root finder nlp
                casadi::SXDict nlp = {{"x", cs_q_sym}, {"f", f}};
                casadi::Dict opts = {};
                opts.insert(std::make_pair("print_time", false));
                opts.insert(std::make_pair("ipopt.linear_solver", "ma27"));
                opts.insert(std::make_pair("ipopt.print_level", 0));
                this->phi_root_finder = casadi::nlpsol("solver", "ipopt", nlp, opts);
            }

        private:
            void updateImplicitJacobian(const JointCoordinate<Scalar> &joint_pos);
            void updateExplicitJacobian(const DMat<Scalar> &K);

            // TODO(@MatthewChignoli): Move to source file
            // TODO(@MatthewChignoli): This only works of Scalar type is SX...

            const size_t links_in_path1_;
            const size_t links_in_path2_;
            const std::vector<Scalar> path1_link_lengths_;
            const std::vector<Scalar> path2_link_lengths_;
            const Vec2<Scalar> offset_;

            const int independent_coordinate_;
            // The independent coordinate map is a 3x3 matrix that maps the stacked indepedent
            // coordinates [y;q_dep] to the spanning coordinate vector [q1;q2;q3]
            Mat3<Scalar> indepenent_coordinate_map_;

            InverseType Kd_inv_;
        };
    }

    namespace ClusterJoints
    {

        // TODO(@MatthewChignoli): Before specializing, use the generic joint first
        template <typename Scalar = double>
        class FourBar : public Generic<Scalar>
        {
        public:
            // TODO(@MatthewChignoli): Figure out later exactly what we need
            // TODO(@MatthewChignoli): For now, assume a certain type of path
            // using LinkJointPair = std::pair<Body<Scalar>, JointPtr<Scalar>>;
            // FourBar(std::vector<LinkJointPair> link_joint_pairs) : Base<Scalar>(3, 1, 1)
            // {
            // }
            // virtual ~FourBar() {}

            FourBar(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::FourBar<Scalar>> loop_constraint)
                : Generic<Scalar>(bodies, joints, loop_constraint) {}

            virtual ~FourBar() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::FourBar; }

            // void updateKinematics(const JointState<Scalar> &joint_state) override
            // {
            //     const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            //     const DVec<Scalar> &q = spanning_joint_state.position;
            //     const DVec<Scalar> &qd = spanning_joint_state.velocity;

            //     // Need to compute: S, vJ, cJ
            // }

            // TODO(@MatthewChignoli): Move to source file
            JointState<double> randomJointState() const override
            {
                // TODO(@MatthewChignoli): YORO this
                this->loop_constraint_->createPhiRootFinder();

                // Find a feasible joint position
                casadi::DMDict arg;
                arg["x0"] = random<casadi::DM>(this->loop_constraint_->numSpanningPos());
                casadi::DM q_dm = this->loop_constraint_->phi_root_finder(arg).at("x");
                DVec<double> q(this->loop_constraint_->numSpanningPos());
                casadi::copy(q_dm, q);
                JointCoordinate<double> joint_pos(q, true);

                // Compute the explicit constraint jacobians
                this->loop_constraint_->updateJacobians(joint_pos);
                DMat<double> G = this->loop_constraint_->G();

                // Compute a valid joint velocity
                DVec<double> v = G * DVec<double>::Random(this->loop_constraint_->numIndependentVel());
                casadi::DM v_dm = casadi::DM::zeros(this->loop_constraint_->numSpanningVel(), 1);
                casadi::copy(v, v_dm);
                JointCoordinate<double> joint_vel(v, true);

                return JointState<double>(joint_pos, joint_vel);
            }
        };

    } // namespace ClusterJoints

} // namespace grbda

#endif // GRBDA_FOUR_BAR_JOINT_H
