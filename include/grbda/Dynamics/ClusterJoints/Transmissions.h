#ifndef GRBDA_TRANSMISSIONS_H
#define GRBDA_TRANSMISSIONS_H

#include <memory>
#include "grbda/Dynamics/Body.h"

namespace grbda
{
    namespace ClusterJoints
    {
        template <typename Scalar = double>
        struct GearedTransmissionModule
        {
            Body<Scalar> body_;
            Body<Scalar> rotor_;
            std::string body_joint_name_;
            std::string rotor_joint_name_;
            ori::CoordinateAxis joint_axis_;
            ori::CoordinateAxis rotor_axis_;
            Scalar gear_ratio_;
        };

        template <size_t N_belts, typename Scalar = double>
        struct ParallelBeltTransmissionModule
        {
            Body<Scalar> body_;
            Body<Scalar> rotor_;
            ori::CoordinateAxis joint_axis_;
            ori::CoordinateAxis rotor_axis_;
            Scalar gear_ratio_;
            Eigen::Matrix<Scalar, N_belts, 1> belt_ratios_;
        };

        template <typename DerivedScalar, int DerivedSize>
        inline Eigen::Matrix<DerivedScalar, 1, DerivedSize>
        beltMatrixRowFromBeltRatios(Eigen::Matrix<DerivedScalar, DerivedSize, 1> ratios)
        {
            for (int i = 1; i < ratios.size(); ++i)
            {
                ratios(i) = ratios(i - 1) * ratios(i);
            }
            return ratios.transpose();
        }
        
    }
}

#endif // GRBDA_TRANSMISSIONS_H
