#ifndef GRBDA_JOINT_H
#define GRBDA_JOINT_H

#include "Utils/SpatialTransforms.h"

namespace grbda
{

    namespace Joints
    {

        class Base
        {
        public:
            Base(int num_positions, int num_velocities)
                : num_positions_(num_positions), num_velocities_(num_velocities) {}
            virtual ~Base() {}

            virtual std::shared_ptr<Base> clone() const = 0;

            virtual void updateKinematics(const DVec<double> &q, const DVec<double> &qd) = 0;

            int numPositions() const { return num_positions_; }
            int numVelocities() const { return num_velocities_; }

            const DMat<double> &S() const { return S_; }
            const DMat<double> &Psi() const { return Psi_; }
            const spatial::Transform<> &XJ() const { return XJ_; }

        protected:
            const int num_positions_;
            const int num_velocities_;

            spatial::Transform<> XJ_;
            DMat<double> S_;
            DMat<double> Psi_;
        };

        class Free : public Base
        {
        public:
            Free() : Base(7, 6)
            {
                S_ = D6Mat<double>::Identity(6, 6);
                Psi_ = D6Mat<double>::Identity(6, 6);
            }
            ~Free() {}

            std::shared_ptr<Base> clone() const override { return std::make_shared<Free>(*this); }

            void updateKinematics(const DVec<double> &q, const DVec<double> &qd) override
            {
                XJ_ = spatial::Transform<>(ori::quaternionToRotationMatrix(q.tail<4>()),
                                           q.head<3>());
            }
        };

        class Revolute : public Base
        {
        public:
            Revolute(ori::CoordinateAxis axis) : Base(1, 1), axis_(axis)
            {
                spatial::JointType Rev = spatial::JointType::Revolute;

                S_ = D6Mat<double>::Zero(6, 1);
                S_.leftCols<1>() = spatial::jointMotionSubspace<double>(Rev, axis);

                Psi_ = D6Mat<double>::Zero(6, 1);
                Psi_.leftCols<1>() = spatial::jointMotionSubspace<double>(Rev, axis);
            }
            ~Revolute() {}

            std::shared_ptr<Base> clone() const override
            {
                return std::make_shared<Revolute>(*this);
            }

            void updateKinematics(const DVec<double> &q, const DVec<double> &qd) override
            {
                XJ_ = spatial::spatialRotation<double>(axis_, q[0]);
            }

        private:
            const ori::CoordinateAxis axis_;
        };

    }

} // namespace grbda

#endif // GRBDA_JOINT_H
