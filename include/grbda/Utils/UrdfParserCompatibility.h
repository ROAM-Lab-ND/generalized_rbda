#ifndef GRBDA_URDF_PARSER_COMPATIBILITY_H
#define GRBDA_URDF_PARSER_COMPATIBILITY_H

#include "custom_urdf/cluster.h"
#include "grbda/Utils/SpatialInertia.h"

namespace grbda
{
    template <typename Scalar>
    SpatialInertia<Scalar> urdfInertialToSpatialInertia(const std::shared_ptr<dynacore::urdf::Inertial> &inertial)
    {
        Scalar mass = inertial->mass;
        Vec3<Scalar> COM = Vec3<Scalar>(inertial->origin.position.x,
                                        inertial->origin.position.y,
                                        inertial->origin.position.z);
        Mat3<Scalar> inertia;
        inertia.row(0) << inertial->ixx, inertial->ixy, inertial->ixz;
        inertia.row(1) << inertial->ixy, inertial->iyy, inertial->iyz;
        inertia.row(2) << inertial->ixz, inertial->iyz, inertial->izz;
        return SpatialInertia<Scalar>(mass, COM, inertia);
    }

    template <typename Scalar>
    Vec3<Scalar> urdfVector3ToVec3(const dynacore::urdf::Vector3 &vector)
    {
        return Vec3<Scalar>(vector.x, vector.y, vector.z);
    }

    template <typename Scalar>
    Mat3<Scalar> urdfRotationToRotationMatrix(const dynacore::urdf::Rotation &rotation)
    {
        Quat<Scalar> quat = Quat<Scalar>(rotation.w, rotation.x, rotation.y, rotation.z);
        return ori::quaternionToRotationMatrix(quat);
    }

    // TODO(@MatthewChignoli): What if axis is something like [1 0.3 0.5]?
    // TODO(@MatthewChignoli): How to account for when an axis has a negative sign? I think we need to change Xtree?
    static ori::CoordinateAxis urdfAxisToCoordinateAxis(const dynacore::urdf::Vector3 &axis)
    {
        if (axis.x == 1 || axis.x == -1)
        {
            return ori::CoordinateAxis::X;
        }
        else if (axis.y == 1 || axis.y == -1)
        {
            return ori::CoordinateAxis::Y;
        }
        else if (axis.z == 1 || axis.z == -1)
        {
            return ori::CoordinateAxis::Z;
        }
        else
        {
            throw std::runtime_error("Error: Joint axis not defined");
        }
    }
} // namespace grbda

#endif // GRBDA_URDF_PARSER_COMPATIBILITY_H
