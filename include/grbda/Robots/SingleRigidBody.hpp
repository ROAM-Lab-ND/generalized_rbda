#ifndef GRBDA_ROBOTS_SRB_H
#define GRBDA_ROBOTS_SRB_H

#include "grbda/Robots/Robot.h"

namespace grbda
{

    template <typename Scalar = double,
              typename OrientationRepresentation = ori_representation::Quaternion>
    class SingleRigidBody : public Robot<Scalar>
    {
    public:
        SingleRigidBody()
        {
            _mass = random<Scalar>();
            _COM = Vec3<Scalar>::Random();
            _rotational_inertia = Vec3<Scalar>::Random().asDiagonal();
        }

        SingleRigidBody(Scalar mass, Vec3<Scalar> COM, Mat3<Scalar> rotational_inertia)
        {
            _mass = mass;
            _COM = COM;
            _rotational_inertia = rotational_inertia;
        }

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override
        {
            ClusterTreeModel<Scalar> model;

            const std::string name = "Floating Base";
            const std::string parent_name = "ground";
            const SpatialInertia<Scalar> inertia(_mass, _COM, _rotational_inertia);
            model.template appendBody<ClusterJoints::Free<Scalar, OrientationRepresentation>>(name, inertia, parent_name,
                                                                   spatial::Transform<Scalar>{});

            Vec3<Scalar> dims = Vec3<Scalar>::Random();
            model.appendContactBox(name, dims);

            return model;
        }

    private:
        Scalar _mass;
        Vec3<Scalar> _COM;
        Mat3<Scalar> _rotational_inertia;
    };

};

#endif // GRBDA_ROBOTS_SRB_H
