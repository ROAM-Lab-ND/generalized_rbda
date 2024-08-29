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
                : num_positions_(num_positions), num_velocities_(num_velocities), name_(name) {}
            virtual ~Base() {}

            virtual std::shared_ptr<Base<Scalar>> clone() const = 0;

            virtual void updateKinematics(const DVec<Scalar> &q, const DVec<Scalar> &qd) = 0;

            int numPositions() const { return num_positions_; }
            int numVelocities() const { return num_velocities_; }

            const DMat<Scalar> &S() const { return S_; }
            const DMat<Scalar> &Psi() const { return Psi_; }
            const spatial::Transform<Scalar> &XJ() const { return XJ_; }

            // TODO(@MatthewChignoli): Maybe should be protected
            const std::string name_;             

        protected:
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
                const RotMat<Scalar> R =
                    OrientationRepresentation::getRotationMatrix(q.template tail<num_ori_param>());
                const Vec3<Scalar> q_pos = q.template head<3>();
                this->XJ_ = spatial::Transform<Scalar>(R, q_pos);
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

        private:
            const ori::CoordinateAxis axis_;
        };

    }

} // namespace grbda

#endif // GRBDA_JOINT_H
