#ifndef GRBDA_JOINT_H
#define GRBDA_JOINT_H

#include "grbda/Utils/SpatialTransforms.h"
#include "grbda/Dynamics/Joints/OrientationRepresentation.h"

namespace grbda
{

    namespace Joints
    {

        template <typename Scalar = double>
        class Base
        {
        public:
            Base(int num_positions, int num_velocities, std::string name)
                : name_(name), num_positions_(num_positions), num_velocities_(num_velocities)  {}
            virtual ~Base() {}

            virtual std::shared_ptr<Base<Scalar>> clone() const = 0;

            virtual void updateKinematics(const DVec<Scalar> &q, const DVec<Scalar> &qd) = 0;

            const std::string& name() const { return name_; }
            int numPositions() const { return num_positions_; }
            int numVelocities() const { return num_velocities_; }

            const DMat<Scalar> &S() const { return S_; }
            const DMat<Scalar> &Psi() const { return Psi_; }
            const spatial::Transform<Scalar> &XJ() const { return XJ_; }

            // Derivative interface for configuration-dependent motion subspaces
            // Returns zero by default for most joint types (Revolute, Free, etc.)
            // Override for joints with absolute coordinates or configuration-dependent kinematics

            // Returns ∂S/∂q as a vector of nv matrices, each of size (6 x nv)
            // S_q[i](j,k) = ∂S(j,k)/∂q(i)
            virtual std::vector<DMat<Scalar>> getSq() const {
                return std::vector<DMat<Scalar>>(num_velocities_,
                                                 DMat<Scalar>::Zero(6, num_velocities_));
            }

            // Returns ∂(Ṡ·q̇)/∂q as a (6 x nv) matrix
            virtual DMat<Scalar> getSdotqd_q() const {
                return DMat<Scalar>::Zero(6, num_velocities_);
            }

            // Returns ∂(Ṡ·q̇)/∂q̇ as a (6 x nv) matrix
            virtual DMat<Scalar> getSdotqd_qd() const {
                return DMat<Scalar>::Zero(6, num_velocities_);
            }

        protected:
            const std::string name_;             
            const int num_positions_;
            const int num_velocities_;

            spatial::Transform<Scalar> XJ_;
            DMat<Scalar> S_;
            DMat<Scalar> Psi_;
        };

        template <typename Scalar = double,
                  typename OrientationRepresentation = ori_representation::Quaternion>
        class Free : public Base<Scalar>
        {
        public:
            Free(std::string name = "unnamed_free_joint")
                : Base<Scalar>(OrientationRepresentation::num_ori_parameter + 3, 6, name)
            {
                this->S_ = D6Mat<Scalar>::Identity(6, 6);
                this->Psi_ = D6Mat<Scalar>::Identity(6, 6);
            }
            ~Free() {}

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<Free<Scalar, OrientationRepresentation>>(*this);
            }

            void updateKinematics(const DVec<Scalar> &q, const DVec<Scalar> &qd) override
            {
                const int& num_ori_param = OrientationRepresentation::num_ori_parameter;

                // Extract orientation parameters and normalize if quaternion
                // CRITICAL: MATLAB's rq() function normalizes quaternions before converting to rotation matrix
                // Unnormalized quaternions produce incorrect rotation matrices
                auto orientation_segment = q.template tail<num_ori_param>();

                if constexpr (num_ori_param == 4) {
                    // For quaternions, normalize before converting to rotation matrix
                    // CRITICAL: MATLAB's rq() function normalizes quaternions (line 28: q = q / norm(q))
                    // We must match this exactly, including for complex types!
                    // For complex-step differentiation, normalization is differentiable and the
                    // imaginary part will carry through correctly via the chain rule.
                    Quat<Scalar> quat_segment = orientation_segment;
                    Scalar norm_val = quat_segment.norm();
                    quat_segment = quat_segment / norm_val;

                    const RotMat<Scalar> R = OrientationRepresentation::getRotationMatrix(quat_segment);
                    const Vec3<Scalar> q_pos = q.template head<3>();
                    this->XJ_ = spatial::Transform<Scalar>(R, q_pos);
                } else {
                    // For RPY, use as-is
                    const RotMat<Scalar> R = OrientationRepresentation::getRotationMatrix(orientation_segment);
                    const Vec3<Scalar> q_pos = q.template head<3>();
                    this->XJ_ = spatial::Transform<Scalar>(R, q_pos);
                }
            }

            OrientationRepresentation orientation_representation_;
        };

        template <typename Scalar = double>
        class Revolute : public Base<Scalar>
        {
        public:
            Revolute(ori::CoordinateAxis axis, std::string name = "unnamed_revolute_joint")
                : Base<Scalar>(1, 1, name), axis_(axis)
            {
                spatial::JointType Rev = spatial::JointType::Revolute;

                this->S_ = D6Mat<Scalar>::Zero(6, 1);
                this->S_.template leftCols<1>() = spatial::jointMotionSubspace<Scalar>(Rev, axis);

                this->Psi_ = D6Mat<Scalar>::Zero(6, 1);
                this->Psi_.template leftCols<1>() = spatial::jointMotionSubspace<Scalar>(Rev, axis);
            }
            ~Revolute() {}

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<Revolute<Scalar>>(*this);
            }

            void updateKinematics(const DVec<Scalar> &q, const DVec<Scalar> &qd) override
            {
                this->XJ_ = spatial::rotation<Scalar>(axis_, q[0]);
            }

            ori::CoordinateAxis getAxis() const { return axis_; }

        private:
            const ori::CoordinateAxis axis_;
        };

    }

} // namespace grbda

#endif // GRBDA_JOINT_H
