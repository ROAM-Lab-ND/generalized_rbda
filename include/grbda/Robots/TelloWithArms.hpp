#ifndef GRBDA_ROBOTS_TELLO_WITH_ARMS_H
#define GRBDA_ROBOTS_TELLO_WITH_ARMS_H

#include "grbda/Robots/Tello.hpp"

namespace grbda
{

    class TelloWithArms : public Tello
    {
    public:
        TelloWithArms()
        {
            _shoulderRyRotInertia << 0.0013678, 0.0000266, 0.0000021,
                0.0000266, 0.0007392, -0.0000012,
                0.0000021, -0.0000012, 0.000884;
            _shoulderRxRotInertia << 0.0011524, 0.0000007, 0.0000396,
                0.0000007, 0.0011921, 0.0000014,
                0.0000396, 0.0000014, 0.0012386;
            _shoulderRzRotInertia << 0.0012713, 0.000001, -0.000008,
                0.000001, 0.0017477, -0.0000225,
                -0.000008, -0.0000225, 0.0008191;
            _elbowRotInertia << 0.001570, 0.0000002, 0.0000335,
                0.0000002, 0.0016167, 0.000003,
                0.0000335, 0.000003, 0.0000619;

            _smallRotorRotationalInertiaZ << 1.084e-4, 0, 0,
                0, 1.084e-4, 0,
                0, 0, 1.6841e-4;

            Mat3<double> RY = ori::coordinateRotation<double>(ori::CoordinateAxis::Y, M_PI / 2);
            Mat3<double> RX = ori::coordinateRotation<double>(ori::CoordinateAxis::X, -M_PI / 2);

            Mat3<double> smallRotorRotationalInertiaX = RY.transpose() * _smallRotorRotationalInertiaZ * RY;
            Mat3<double> smallRotorRotationalInertiaY = RX.transpose() * _smallRotorRotationalInertiaZ * RX;

            _smallRotorRotInertiaX = smallRotorRotationalInertiaX;
            _smallRotorRotInertiaY = smallRotorRotationalInertiaY;
            _smallRotorRotInertiaZ = _smallRotorRotationalInertiaZ;
        }

        ClusterTreeModel<> buildClusterTreeModel() const override;

    private:
        double _shoulderRyMass = 0.788506;
        double _shoulderRxMass = 0.80125;
        double _shoulderRzMass = 0.905588;
        double _elbowMass = 0.34839;

        Vec3<double> _shoulderRyLocation = Vec3<double>(0.01346, -0.17608, 0.24657);
        Vec3<double> _shoulderRxLocation = Vec3<double>(0.0, -0.0575, 0.0);
        Vec3<double> _shoulderRzLocation = Vec3<double>(0.0, 0.0, -0.10250);
        Vec3<double> _elbowLocation = Vec3<double>(0.0, 0.0, -0.1455);

        Vec3<double> _shoulderRyCOM = Vec3<double>(0.009265, 0.052623, -0.0001249);
        Vec3<double> _shoulderRxCOM = Vec3<double>(0.0006041, 0.0001221, -0.082361);
        Vec3<double> _shoulderRzCOM = Vec3<double>(0.0001703, -0.016797, -0.060);
        Vec3<double> _elbowCOM = Vec3<double>(-0.0059578, 0.000111, -0.0426735);

        Vec3<double> _smallRotorCOM = Vec3<double>(0., 0., 0.);

        double _shoulderRxGearRatio = 6.0;
        double _shoulderRzGearRatio = 6.0;
        double _shoulderRyGearRatio = 6.0;
        double _elbowGearRatio = 9.0;

        Vec3<double> _shoulderRyRotorLocation = Vec3<double>(0.01346, -0.16, 0.24657);
        Vec3<double> _shoulderRxRotorLocation = Vec3<double>(0, -0.0575, 0);
        Vec3<double> _shoulderRzRotorLocation = Vec3<double>(0., 0., -0.1025);
        Vec3<double> _elbowRotorLocation = Vec3<double>(0., 0.0325, -0.06);

        Mat3<double> _shoulderRyRotInertia;
        Mat3<double> _shoulderRxRotInertia;
        Mat3<double> _shoulderRzRotInertia;
        Mat3<double> _elbowRotInertia;

        Mat3<double> _smallRotorRotationalInertiaZ;

        Mat3<double> _smallRotorRotInertiaX;
        Mat3<double> _smallRotorRotInertiaY;
        Mat3<double> _smallRotorRotInertiaZ;

        double _lowerArmLength = 0.27;

        template <typename T>
        Vec3<double> withLeftRightSigns(const Eigen::MatrixBase<T> &v, int side) const
        {
            static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                          "Must have 3x1 matrix");
            switch (side)
            {
            case 0: // right
                return Vec3<double>(v[0], v[1], v[2]);
            case 1: // left
                return Vec3<double>(v[0], -v[1], v[2]);
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

        SpatialInertia<double> withLeftRightSigns(const SpatialInertia<double> &I, int side) const
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

#endif // GRBDA_ROBOTS_TELLO_WITH_ARMS_H
