#ifndef GRBDA_ORI_REPRESENTATION_H
#define GRBDA_ORI_REPRESENTATION_H

#include "Utils/SpatialTransforms.h"
#include "cppTypes.h"

namespace grbda
{
    namespace ori_representation
    {
        template <typename Derived>
        struct Base {
            static const int num_ori_parameter;      
            static const int numSpanningPos;         
            static const int numIndependentPos;

            virtual const RotMat<typename Derived::Scalar> getRotationMatrix(const Eigen::MatrixBase<Derived> & q) const = 0;
            virtual const Derived randomOrientation() const = 0;
        };

        template <typename Derived>
        struct QuaternionRepresentation : public Base<Derived> {
            static const int num_ori_parameter = 4;
            static const int numSpanningPos = 7;
            static const int numIndependentPos = 7;

            const RotMat<typename Derived::Scalar> getRotationMatrix(const Eigen::MatrixBase<Derived> & q) const override {
                const RotMat<typename Derived::Scalar> R = ori::quaternionToRotationMatrix(q);
                return R;
            }
            const Derived randomOrientation() const override {
                return ori::rpyToQuat(Vec3<typename Derived::Scalar>::Random(3));
            }
        };

        template <typename Derived>
        struct RollPitchYawRepresentation : public Base<Derived> {
            static const int num_ori_parameter = 3;
            static const int numSpanningPos = 6;
            static const int numIndependentPos = 6;

            const RotMat<typename Derived::Scalar> getRotationMatrix(const Eigen::MatrixBase<Derived> & q) const override {
                return ori::rpyToRotMat(q);
            }

            const Derived randomOrientation() const override {
                return Derived::Random(3);
            }
        };
    }  // namespace ori_representation
}  // namespace grbda


#endif // GRBDA_JOINT_H
