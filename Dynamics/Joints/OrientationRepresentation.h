#ifndef GRBDA_ORI_REPRESENTATION_H
#define GRBDA_ORI_REPRESENTATION_H

#include "Utils/SpatialTransforms.h"
#include "cppTypes.h"

namespace grbda
{
    namespace ori_representation
    {
        template <typename T = double>
        struct Base {
            static const int num_ori_parameter;      
            static const int numSpanningPos;         
            static const int numIndependentPos;

            virtual const RotMat<T> getRotationMatrix(const DVec<T>& q) const = 0; // pure virtual method
        };

        template <typename T = double>
        struct QuaternionRepresentation : public Base<T> {
            static const int num_ori_parameter = 4;
            static const int numSpanningPos = 7;
            static const int numIndependentPos = 7;

            const RotMat<T> getRotationMatrix(const DVec<T>& q) const override {
                const RotMat<T> R = ori::quaternionToRotationMatrix(q.template tail<4>());
                return R;
            }
        };

        template <typename T = double>
        struct RollPitchYawRepresentation : public Base<T> {
            static const int num_ori_parameter = 3;
            static const int numSpanningPos = 6;
            static const int numIndependentPos = 6;

            const RotMat<T> getRotationMatrix(const DVec<T>& q) const override {
                return ori::rpyToRotMat(q);
            }
        };
    }  // namespace ori_representation
}  // namespace grbda


#endif // GRBDA_JOINT_H
