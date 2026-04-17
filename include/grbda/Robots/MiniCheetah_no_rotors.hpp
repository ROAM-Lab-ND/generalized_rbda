#ifndef GRBDA_ROBOTS_MINI_CHEETAH_NO_ROTORS_H
#define GRBDA_ROBOTS_MINI_CHEETAH_NO_ROTORS_H

#include "grbda/Robots/Robot.h"

namespace grbda
{

    template <typename Scalar = double,
              typename OrientationRepresentation = ori_representation::Quaternion>
    class MiniCheetah_no_rotors : public Robot<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MiniCheetah_no_rotors();

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    private:
        Scalar _bodyMass = 3.3;
        Scalar _abadMass = 0.54;
        Scalar _hipMass = 0.634;
        Scalar _kneeMass = 0.064;

        Scalar _bodyLength = 0.19 * 2;
        Scalar _bodyWidth = 0.049 * 2;
        Scalar _bodyHeight = 0.05 * 2;
        Scalar _hipLinkLength = 0.209;
        Scalar _kneeLinkLength = 0.195;
        Scalar _kneeLinkY_offset = 0.004;

        Vec3<Scalar> _bodyCOM = Vec3<Scalar>(0, 0, 0);
        Vec3<Scalar> _abadCOM = Vec3<Scalar>(0, 0.036, 0);
        Vec3<Scalar> _hipCOM = Vec3<Scalar>(0, 0.016, -0.02);
        Vec3<Scalar> _kneeCOM = Vec3<Scalar>(0, 0, -0.061);

        Mat3<Scalar> _bodyRotationalInertia;
        Mat3<Scalar> _abadRotationalInertia;
        Mat3<Scalar> _hipRotationalInertia;
        Mat3<Scalar> _kneeRotationalInertia;

        Vec3<Scalar> _abadLocation = Vec3<Scalar>(0.19, 0.049, 0);
        Vec3<Scalar> _hipLocation = Vec3<Scalar>(0, 0.062, 0);
        Vec3<Scalar> _kneeLocation = Vec3<Scalar>(0, 0, -0.209);

        template <typename T>
        Vec3<Scalar> withLegSigns(const Eigen::MatrixBase<T> &v, int side) const
        {
            static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                          "Must have 3x1 matrix");
            switch (side)
            {
            case 0:
                return Vec3<Scalar>(v[0], -v[1], v[2]);
            case 1:
                return Vec3<Scalar>(v[0], v[1], v[2]);
            case 2:
                return Vec3<Scalar>(-v[0], -v[1], v[2]);
            case 3:
                return Vec3<Scalar>(-v[0], v[1], v[2]);
            default:
                throw std::runtime_error("Invalid leg id!");
            }
        }

        std::string withLegSigns(const std::string &s, int side) const
        {
            switch (side)
            {
            case 0:
                return "FR_" + s;
            case 1:
                return "FL_" + s;
            case 2:
                return "HR_" + s;
            case 3:
                return "HL_" + s;
            default:
                throw std::runtime_error("Invalid leg id!");
            }
        }

        SpatialInertia<Scalar> withLeftRightSigns(const SpatialInertia<Scalar> &I, int side) const
        {
            switch (side)
            {
            case -1:
            case 0:
                return I.flipAlongAxis(ori::CoordinateAxis::Y);
            case 1:
                return I;
            default:
                throw std::runtime_error("Invalid side id!");
            }
        }
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_MINI_CHEETAH_NO_ROTORS_H