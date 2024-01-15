#ifndef GRBDA_FOUR_BAR_JOINT_H
#define GRBDA_FOUR_BAR_JOINT_H

namespace grbda
{

    namespace LoopConstraint
    {
        // TODO(@MatthewChignoli): Later we can worry about how we handle the fixed offset with this loop constraint compared to the generic one
        // TODO(@MatthewChignoli): Missing the fixed offset between the joints of link 1 and link 3
        template <typename Scalar = double>
        struct FourBar : Base<Scalar>
        {
            // TODO(@MatthewChignoli): Pass by reference?
            FourBar(std::vector<Scalar> path1_lengths,
                    std::vector<Scalar> path2_lengths,
                    Vec2<Scalar> offset) :
                path1_size_(path1_lengths.size()), path2_size_(path2_lengths.size()),
                path1_lengths_(path1_lengths), path2_lengths_(path2_lengths)
            {
                if (path1_size_ + path2_size_ != 3)
                {
                    throw std::runtime_error("FourBar: Must contain 3 links");
                }

                this->phi_ = [this, offset](const JointCoordinate<Scalar> &joint_pos)
                {
                    DVec<Scalar> phi = DVec<Scalar>::Zero(2);

                    DVec<Scalar> path1_joints = joint_pos.head(path1_size_);
                    DVec<Scalar> path2_joints = joint_pos.tail(path2_size_);

                    Scalar cumulative_angle = 0.;
                    DVec<Scalar> path1 = DVec<Scalar>::Zero(2);
                    for (size_t i = 0; i < path1_size_; i++)
                    {
                        cumulative_angle += path1_joints(i);
                        path1(0) += path1_lengths_[i] * cos(cumulative_angle);
                        path1(1) += path1_lengths_[i] * sin(cumulative_angle);
                    }

                    cumulative_angle = 0.;
                    DVec<Scalar> path2 = offset;
                    for (size_t i = 0; i < path2_size_; i++)
                    {
                        cumulative_angle += path2_joints(i);
                        path2(0) += path2_lengths_[i] * cos(cumulative_angle);
                        path2(1) += path2_lengths_[i] * sin(cumulative_angle);
                    }

                    phi = path1 - path2;
                    return phi;
                };

                this->G_ = DMat<Scalar>::Zero(3, 1);
                this->g_ = DVec<Scalar>::Zero(3);

                this->K_ = DMat<Scalar>::Zero(2, 3);
                this->k_ = DVec<Scalar>::Zero(2);
            }

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<FourBar<Scalar>>(*this);
            }

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override
            {
                DVec<Scalar> path1_joints = joint_pos.head(path1_size_);
                DVec<Scalar> path2_joints = joint_pos.tail(path2_size_);

                // Update K
                Scalar cumulative_angle = 0.;
                DMat<Scalar> K1 = DMat<Scalar>::Zero(2, path1_size_);
                for (size_t i = 0; i < path1_lengths_.size(); i++)
                {
                    cumulative_angle += path1_joints(i);
                    for (size_t j = 0; j <= i; j++)
                    {
                        K1(0, j) += -path1_lengths_[i] * sin(cumulative_angle);
                        K1(1, j) += path1_lengths_[i] * cos(cumulative_angle);
                    }
                }

                cumulative_angle = 0.;
                DMat<Scalar> K2 = DMat<Scalar>::Zero(2, path2_size_);
                for (size_t i = 0; i < path2_size_; i++)
                {
                    cumulative_angle += path2_joints(i);
                    for (size_t j = 0; j <= i; j++)
                    {
                        K2(0, j) += path2_lengths_[i] * sin(cumulative_angle);
                        K2(1, j) += -path2_lengths_[i] * cos(cumulative_angle);
                    }
                }

                this->K_ << K1, K2;

                // TODO(@MatthewChignoli): Do this part later
                // Update G
                DMat<Scalar> G(1, 3);
                G.setZero();
            }

            void updateBiases(const JointState<Scalar> &joint_state) override {}

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override { return DVec<Scalar>::Zero(0); }

            private:
                // TODO(@MatthewChignoli): The naming here is pretty confusing
                const size_t path1_size_;
                const size_t path2_size_;
                const std::vector<Scalar> path1_lengths_;
                const std::vector<Scalar> path2_lengths_;
        };
    }

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class FourBar : public Base<Scalar>
        {
        public:
            FourBar(Body<Scalar> &link_1, Body<Scalar> &link_2,
                    ori::CoordinateAxis joint_axis_1, ori::CoordinateAxis joint_axis_2);
            virtual ~FourBar() {}
        };

    } // namespace ClusterJoints

} // namespace grbda

#endif // GRBDA_FOUR_BAR_JOINT_H
