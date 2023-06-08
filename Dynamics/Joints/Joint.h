#pragma once

#include "Utils/Utilities/SpatialTransforms.h"

namespace grbda
{

    namespace Joints
    {

        using namespace ori;
        using namespace spatial;

        class Base
        {
        public:
            Base(int num_positions, int num_velocities)
                : num_positions_(num_positions), num_velocities_(num_velocities) {}
            virtual ~Base() {}

            virtual void updateKinematics(const DVec<double> &q, const DVec<double> &qd) = 0;

            int numPositions() const { return num_positions_; }
            int numVelocities() const { return num_velocities_; }

            const DMat<double> &S() const { return S_; }
            const DMat<double> &S_ring() const { return S_ring_; }
            const DMat<double> &Psi() const { return Psi_; }
            const SpatialTransform &XJ() const { return XJ_; }

        protected:
            const int num_positions_;
            const int num_velocities_;

            SpatialTransform XJ_;
            DMat<double> S_;
            DMat<double> S_ring_;
            DMat<double> Psi_;
        };

        class Free : public Base
        {
        public:
            Free() : Base(7, 6)
            {
                S_ = D6Mat<double>::Identity(6, 6);
                S_ring_ = D6Mat<double>::Zero(6, 6);
                Psi_ = D6Mat<double>::Identity(6, 6);
            }
            ~Free() {}

            void updateKinematics(const DVec<double> &q, const DVec<double> &qd) override {}
        };

        class Revolute : public Base
        {
        public:
            Revolute(CoordinateAxis axis) : Base(1, 1), axis_(axis)
            {
                S_ = D6Mat<double>::Zero(6, 1);
                S_.leftCols<1>() = jointMotionSubspace<double>(JointType::Revolute, axis);

                S_ring_ = D6Mat<double>::Zero(6, 1);

                Psi_ = D6Mat<double>::Zero(6, 1);
                Psi_.leftCols<1>() = jointMotionSubspace<double>(JointType::Revolute, axis);
            }
            ~Revolute() {}

            void updateKinematics(const DVec<double> &q, const DVec<double> &qd) override
            {
                XJ_ = spatialRotation(axis_, q[0]);
            }

        private:
            const CoordinateAxis axis_;
        };

    }

} // namespace grbda
