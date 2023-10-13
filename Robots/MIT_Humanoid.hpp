#ifndef GRBDA_ROBOTS_MIT_HUMANOID_H
#define GRBDA_ROBOTS_MIT_HUMANOID_H

#include "Robot.h"

namespace grbda
{
    // TODO(@MatthewChignoli): Validate that I did not break anything by playing with the inertia params
    template <typename Scalar = double>
    class MIT_Humanoid : public Robot<Scalar>
    {
    public:
        MIT_Humanoid();

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

        LinearInertialParams<Scalar> _torsoLinearInertialParams;
        LinearInertialParams<Scalar> _hipRzLinearInertialParams;
        LinearInertialParams<Scalar> _hipRzRotorLinearInertialParams;
        LinearInertialParams<Scalar> _hipRxLinearInertialParams;
        LinearInertialParams<Scalar> _hipRxRotorLinearInertialParams;
        LinearInertialParams<Scalar> _hipRyLinearInertialParams;
        LinearInertialParams<Scalar> _hipRyRotorLinearInertialParams;
        LinearInertialParams<Scalar> _kneeLinearInertialParams;
        LinearInertialParams<Scalar> _kneeRotorLinearInertialParams;
        LinearInertialParams<Scalar> _ankleLinearInertialParams;
        LinearInertialParams<Scalar> _ankleRotorLinearInertialParams;
        LinearInertialParams<Scalar> _shoulderRyLinearInertialParams;
        LinearInertialParams<Scalar> _shoulderRyRotorLinearInertialParams;
        LinearInertialParams<Scalar> _shoulderRxLinearInertialParams;
        LinearInertialParams<Scalar> _shoulderRxRotorLinearInertialParams;
        LinearInertialParams<Scalar> _shoulderRzLinearInertialParams;
        LinearInertialParams<Scalar> _shoulderRzRotorLinearInertialParams;
        LinearInertialParams<Scalar> _elbowLinearInertialParams;
        LinearInertialParams<Scalar> _elbowRotorLinearInertialParams;

    protected:
        Scalar _hipRzPitch = -0.174533;
        Scalar _hipRxPitch = 0.436332;
        Scalar _hipRyPitch = -(_hipRxPitch + _hipRzPitch);

        Vec3<Scalar> _hipRzLocation = Vec3<Scalar>(-0.00565, -0.082, -0.05735);
        Vec3<Scalar> _hipRxLocation = Vec3<Scalar>(-0.06435, 0.0, -.07499);
        Vec3<Scalar> _hipRyLocation = Vec3<Scalar>(0.071, 0.0018375, 0.0);
        Vec3<Scalar> _kneeLocation = Vec3<Scalar>(0.0, 0.0, -0.267);
        Vec3<Scalar> _ankleLocation = Vec3<Scalar>(0.0, 0.0, -0.2785);

        Vec3<Scalar> _shoulderRyLocation = Vec3<Scalar>(0.01346, -0.17608, 0.24657);
        Vec3<Scalar> _shoulderRxLocation = Vec3<Scalar>(0.0, -0.0575, 0.0);
        Vec3<Scalar> _shoulderRzLocation = Vec3<Scalar>(0.0, 0.0, -0.10250);
        Vec3<Scalar> _elbowLocation = Vec3<Scalar>(0.0, 0.0, -0.1455);

        Scalar _hipRzGearRatio = 6.0;
        Scalar _hipRxGearRatio = 6.0;
        Scalar _hipRyGearRatio = 6.0;
        Scalar _kneeGearRatio = 6.0;
        Scalar _kneeBeltRatio = 2.0;
        Scalar _ankleGearRatio = 6.0;
        Scalar _ankleBeltRatio = 2.0;

        Scalar _shoulderRxGearRatio = 6.0;
        Scalar _shoulderRzGearRatio = 6.0;
        Scalar _shoulderRyGearRatio = 6.0;
        Scalar _elbowGearRatio = 9.0;

        Vec3<Scalar> _hipRzRotorLocation = Vec3<Scalar>(-0.00842837, -0.082, -0.041593);
        Vec3<Scalar> _hipRxRotorLocation = Vec3<Scalar>(-0.0827, 0.0, -0.066436);
        Vec3<Scalar> _hipRyRotorLocation = Vec3<Scalar>(0.071, 0.024, 0.0);
        Vec3<Scalar> _kneeRotorLocation = Vec3<Scalar>(0.013, -0.0497, -0.0178);
        Vec3<Scalar> _ankleRotorLocation = Vec3<Scalar>(0., 0., 0.);

        Vec3<Scalar> _shoulderRyRotorLocation = Vec3<Scalar>(0.01346, -0.16, 0.24657);
        Vec3<Scalar> _shoulderRxRotorLocation = Vec3<Scalar>(0, -0.0575, 0);
        Vec3<Scalar> _shoulderRzRotorLocation = Vec3<Scalar>(0., 0., -0.1025);
        Vec3<Scalar> _elbowRotorLocation = Vec3<Scalar>(0., 0.0325, -0.06);

        Scalar _torsoLength = .1;
        Scalar _torsoWidth = .164;
        Scalar _torsoHeight = 0.30392;

        Scalar _thighLength = 0.267;
        Scalar _footToeLength = 0.1;
        Scalar _footHeelLength = 0.05;
        Scalar _footHeight = 0.041;

        Scalar _lowerArmLength = 0.27;

        template <typename T>
        Vec3<Scalar> withLeftRightSigns(const Eigen::MatrixBase<T> &v, int side) const
        {
            static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                          "Must have 3x1 matrix");
            switch (side)
            {
            case 0: // right
                return Vec3<Scalar>(v[0], v[1], v[2]);
            case 1: // left
                return Vec3<Scalar>(v[0], -v[1], v[2]);
            default:
                throw std::runtime_error("Invalid side id!");
            }
        }

        std::string withLeftRightSigns(const std::string &s, int side) const
        {
            switch (side)
            {
            case 0: // right
                return "right_" + s;
            case 1: // left
                return "left_" + s;
            default:
                throw std::runtime_error("Invalid side id!");
            }
        }

        SpatialInertia<Scalar> withLeftRightSigns(const SpatialInertia<Scalar> &I, int side) const
        {
            switch (side)
            {
            case -1: // right
            case 0:  // right
                return I.flipAlongAxis(ori::CoordinateAxis::Y);
            case 1: // left
                return I;
            default:
                throw std::runtime_error("Invalid side id!");
            }
        }
    };
} // namespace grbda

#endif // GRBDA_ROBOTS_MIT_HUMANOID_H
