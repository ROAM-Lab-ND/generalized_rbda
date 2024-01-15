#ifndef GRBDA_FOUR_BAR_JOINT_H
#define GRBDA_FOUR_BAR_JOINT_H

namespace grbda
{

    namespace LoopConstraint
    {
        // TODO(@MatthewChignoli): Missing the fixed offset between the joints of link 1 and link 3
        template <typename Scalar = double>
        struct FourBar : Base<Scalar>
        {
            FourBar(std::vector<Scalar> path1_lengths, std::vector<Scalar> path2_lengths)
            {
                if (path1_lengths.size() + path2_lengths.size() != 3)
                {
                    throw std::runtime_error("FourBar: path lengths must be of size 3");
                }
                path1_lengths_ = path1_lengths;
                path2_lengths_ = path2_lengths;

                this->phi_ = [this](const JointCoordinate<Scalar> &joint_pos)
                {
                    DVec<Scalar> phi = DVec<Scalar>::Zero(2);

                    Scalar cumulative_angle = 0.;
                    for (size_t i = 0; i < path1_lengths_.size(); i++)
                    {
                        cumulative_angle += joint_pos(i);
                        phi(0) += path1_lengths_[i] * cos(cumulative_angle);
                        phi(1) += path1_lengths_[i] * sin(cumulative_angle);
                    }

                    cumulative_angle = 0.;
                    for (size_t i = 0; i < path2_lengths_.size(); i++)
                    {
                        cumulative_angle += joint_pos(path1_lengths_.size() + i);
                        phi(0) -= path2_lengths_[i] * cos(cumulative_angle);
                        phi(1) -= path2_lengths_[i] * sin(cumulative_angle);
                    }

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
                // Update K
                DMat<Scalar> K(2, 3);
                K.setZero();

                Scalar cumulative_angle = 0.;
                for (size_t i = 0; i < path1_lengths_.size(); i++)
                {
                    cumulative_angle += joint_pos(i);
                    for (size_t j = 0; j <= i; j++)
                    {
                        K(0, j) += -path1_lengths_[i] * sin(cumulative_angle);
                        K(1, j) += path1_lengths_[i] * cos(cumulative_angle);
                    }
                }

                cumulative_angle = 0.;
                for (size_t i = 0; i < path2_lengths_.size(); i++)
                {
                    size_t offset = path1_lengths_.size();
                    cumulative_angle += joint_pos(offset + i);
                    for (size_t j = 0; j <= i; j++)
                    {
                        K(0, offset + j) += path2_lengths_[i] * sin(cumulative_angle);
                        K(1, offset + j) += -path2_lengths_[i] * cos(cumulative_angle);
                    }
                }

                this->K_ = K;

                // TODO(@MatthewChignoli): Do this part later
                // Update G
                DMat<Scalar> G(1, 3);
                G.setZero();
            }

            void updateBiases(const JointState<Scalar> &joint_state) override {}

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override { return DVec<Scalar>::Zero(0); }

            private:
                std::vector<Scalar> path1_lengths_;
                std::vector<Scalar> path2_lengths_;
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
