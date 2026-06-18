#ifndef GRBDA_ORI_REPRESENTATION_H
#define GRBDA_ORI_REPRESENTATION_H

#include "grbda/Utils/SpatialTransforms.h"
#include "grbda/Utils/cppTypes.h"

namespace grbda
{
    namespace ori_representation
    {
        struct Quaternion
        {
            static constexpr int num_ori_parameter = 4;
            static constexpr int numSpanningPos = 7;
            static constexpr int numIndependentPos = 7;

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
            static constexpr int num_ori_parameter = 3;
            static constexpr int numSpanningPos = 6;
            static constexpr int numIndependentPos = 6;

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

#endif // GRBDA_ORI_REPRESENTATION_H
