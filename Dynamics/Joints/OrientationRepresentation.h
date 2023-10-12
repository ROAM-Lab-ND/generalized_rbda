#ifndef GRBDA_ORI_REPRESENTATION_H
#define GRBDA_ORI_REPRESENTATION_H

#include "Utils/SpatialTransforms.h"
#include "cppTypes.h"

namespace grbda
{
    namespace ori_representation
    {
        struct Quaternion
        {
            static const int num_ori_parameter = 4;
            static const int numSpanningPos = 7;
            static const int numIndependentPos = 7;

            template <typename Derived>
            static const RotMat<typename Derived::Scalar>
            getRotationMatrix(const Eigen::MatrixBase<Derived> &q)
            {
                return ori::quaternionToRotationMatrix(q);
            }

            template <typename Scalar>
            static const Quat<Scalar> randomOrientation()
            {
                return ori::rpyToQuat(Vec3<Scalar>::Random(3));
            }
        };

        struct RollPitchYaw
        {
            static const int num_ori_parameter = 3;
            static const int numSpanningPos = 6;
            static const int numIndependentPos = 6;

            template <typename Derived>
            static const RotMat<typename Derived::Scalar>
            getRotationMatrix(const Eigen::MatrixBase<Derived> &q)
            {
                return ori::rpyToRotMat(q);
            }

            template <typename Scalar>
            static const Vec3<Scalar> randomOrientation()
            {
                return Vec3<Scalar>::Random(3);
            }
        };

    } // namespace ori_representation

} // namespace grbda

#endif // GRBDA_JOINT_H
