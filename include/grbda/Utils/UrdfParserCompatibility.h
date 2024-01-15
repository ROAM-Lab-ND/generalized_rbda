#ifndef GRBDA_URDF_PARSER_COMPATIBILITY_H
#define GRBDA_URDF_PARSER_COMPATIBILITY_H

// TODO(@MatthewChignoli): Can we get away with including less?
#include "custom_urdf/cluster.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{
    // TODO(@MatthewChignoli): Make these a utilities somewhere?
    SpatialInertia<double> urdfInertialToSpatialInertia(const std::shared_ptr<dynacore::urdf::Inertial> &inertial)
    {
        double mass = inertial->mass;
        Vec3<double> COM = Vec3<double>(inertial->origin.position.x,
                                        inertial->origin.position.y,
                                        inertial->origin.position.z);
        Mat3<double> inertia;
        inertia.row(0) << inertial->ixx, inertial->ixy, inertial->ixz;
        inertia.row(1) << inertial->ixy, inertial->iyy, inertial->iyz;
        inertia.row(2) << inertial->ixz, inertial->iyz, inertial->izz;
        return SpatialInertia<double>(mass, COM, inertia);
    }

    Vec3<double> urdfVector3ToVec3(const dynacore::urdf::Vector3 &vector)
    {
        return Vec3<double>(vector.x, vector.y, vector.z);
    }

    Mat3<double> urdfRotationToRotationMatrix(const dynacore::urdf::Rotation &rotation)
    {
        Quat<double> quat = Quat<double>(rotation.w, rotation.x, rotation.y, rotation.z);
        return ori::quaternionToRotationMatrix(quat);
    }

    // TODO(@MatthewChignoli): What if axis is something like [1 0.3 0.5]?
    // TODO(@MatthewChignoli): How to account for when an axis has a negative sign? I think we need to change Xtree?
    ori::CoordinateAxis urdfAxisToCoordinateAxis(const dynacore::urdf::Vector3 &axis)
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