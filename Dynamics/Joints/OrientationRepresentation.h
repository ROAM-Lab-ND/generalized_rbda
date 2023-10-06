#ifndef GRBDA_ORI_REPRESENTATION_H
#define GRBDA_ORI_REPRESENTATION_H

#include "Utils/SpatialTransforms.h"
#include "cppTypes.h"

namespace grbda
{
    namespace ori_representation
    {
        struct Base {
            static const int num_ori_parameter;      
            static const int numSpanningPos;         
            static const int numIndependentPos;

            virtual RotMat<double> getRotationMatrix(const DVec<double>& q) const = 0; // pure virtual method
        };

        struct Quaternion : public Base {
            static const int num_ori_parameter = 4;
            static const int numSpanningPos = 7;
            static const int numIndependentPos = 7;

            RotMat<double> getRotationMatrix(const DVec<double>& q) const override {
                const RotMat<Scalar> R = ori::quaternionToRotationMatrix(q.template tail<4>());
                // ... 
                return rotationMatrix;
            }
        };

        struct RollPitchYaw : public Base {
            static const int num_ori_parameter = 3;
            static const int numSpanningPos = 6;
            static const int numIndependentPos = 6;

            RotMat<double> getRotationMatrix(const DVec<double>& q) const override {
                return ori::rpyToRotMat(q);
            }
        };
    }  // namespace ori_representation
}  // namespace grbda


#endif // GRBDA_JOINT_H
