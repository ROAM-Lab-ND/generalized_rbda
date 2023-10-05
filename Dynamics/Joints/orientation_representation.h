#ifndef GRBDA_ORI_REPRESENTATION_H
#define GRBDA_ORI_REPRESENTATION_H

#include "Utils/SpatialTransforms.h"

#include <Eigen/Core>  // for Eigen core functionalities

namespace grbda
{
    namespace ori_representation
    {
        struct OrientationBase {
            static const int num_ori_parameter;      // Initialize in source file or inline if C++17
            static const int numSpanningPos;         // Initialize in source file or inline if C++17
            static const int numIndependentPos;      // Initialize in source file or inline if C++17

            virtual RotMat<double> getRotationMatrix(const DVec<double>& q) const = 0; // pure virtual method
        };

        struct Quaternion : public OrientationBase {
            static const int num_ori_parameter = 4;
            static const int numSpanningPos = 7;
            static const int numIndependentPos = 7;

            RotMat<double> getRotationMatrix(const DVec<double>& q) const override {
                // Implement the logic to convert quaternion to rotation matrix
                RotMat<double> rotationMatrix;
                // ... 
                return rotationMatrix;
            }
        };

        struct RollPitchYaw : public OrientationBase {
            static const int num_ori_parameter = 3;
            static const int numSpanningPos = 6;
            static const int numIndependentPos = 6;

            RotMat<double> getRotationMatrix(const DVec<double>& q) const override {
                // Implement the logic to convert roll-pitch-yaw to rotation matrix
                RotMat<double> rotationMatrix;
                // ...
                return rotationMatrix;
            }
        };
    }  // namespace ori_representation
}  // namespace grbda


#endif // GRBDA_JOINT_H
