#include "grbda/Robots/TelloWithArms.hpp"

namespace grbda
{

    template <typename Scalar>
    ClusterTreeModel<Scalar> TelloWithArms<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;

        ClusterTreeModel<Scalar> model = Tello<Scalar>::buildClusterTreeModel();

        const Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

        std::vector<std::string> sides = {"left", "right"};

        for (int armID = 0; armID < 2; armID++)
        {
            const std::string side = sides[armID];

            // ShoulderRy
            const std::string shoulder_ry_parent_name = this->base;
            const std::string shoulder_ry_name = withLeftRightSigns("shoulder-ry", armID);
            const std::string shoulder_ry_link_name = withLeftRightSigns("shoulder-ry", armID);
            const std::string shoulder_ry_rotor_name = withLeftRightSigns("shoulder-ry-rotor", armID);
            const std::string shoulder_ry_link_joint_name = this->base + "-to-" + side + "-shoulder-ry";
            const std::string shoulder_ry_rotor_joint_name = this->base + "-to-" + side + "-shoulder-ry-rotor";

            SpatialInertia<Scalar> shoulder_ry_link_inertia(_shoulderRyMass, _shoulderRyCOM,
                                                            _shoulderRyRotInertia);
            shoulder_ry_link_inertia = withLeftRightSigns(shoulder_ry_link_inertia, armID);

            SpatialInertia<Scalar> shoulder_ry_rotor_inertia(0., _smallRotorCOM,
                                                             _smallRotorRotInertiaY);
            shoulder_ry_rotor_inertia = withLeftRightSigns(shoulder_ry_rotor_inertia, armID);

            const spatial::Transform<Scalar> xtreeShoulderRy(I3, withLeftRightSigns(_shoulderRyLocation, armID));
            const spatial::Transform<Scalar> xtreeShoulderRyRotor(I3, withLeftRightSigns(_shoulderRyRotorLocation, armID));

            Body<Scalar> shoulder_ry_link = model.registerBody(shoulder_ry_link_name,
                                                               shoulder_ry_link_inertia,
                                                               shoulder_ry_parent_name,
                                                               xtreeShoulderRy);
            Body<Scalar> shoulder_ry_rotor = model.registerBody(shoulder_ry_rotor_name,
                                                                shoulder_ry_rotor_inertia,
                                                                shoulder_ry_parent_name,
                                                                xtreeShoulderRyRotor);
            GearedTransmissionModule<Scalar> shoulder_ry_module{shoulder_ry_link, shoulder_ry_rotor,
                                                                shoulder_ry_link_joint_name, 
                                                                shoulder_ry_rotor_joint_name,
                                                                ori::CoordinateAxis::Y,
                                                                ori::CoordinateAxis::Y,
                                                                _shoulderRyGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<Scalar>>(
                shoulder_ry_name, shoulder_ry_module);

            // ShoulderRx
            const std::string shoulder_rx_parent_name = shoulder_ry_link_name;
            const std::string shoulder_rx_name = withLeftRightSigns("shoulder-rx", armID);
            const std::string shoulder_rx_link_name = withLeftRightSigns("shoulder-rx", armID);
            const std::string shoulder_rx_rotor_name = withLeftRightSigns("shoulder-rx-rotor", armID);
            const std::string shoulder_rx_link_joint_name = withLeftRightSigns("shoulder-ry-to-shoulder-rx", armID);
            const std::string shoulder_rx_rotor_joint_name = withLeftRightSigns("shoulder-ry-to-shoulder-rx-rotor", armID);

            SpatialInertia<Scalar> shoulder_rx_link_inertia(_shoulderRxMass, _shoulderRxCOM,
                                                            _shoulderRxRotInertia);
            shoulder_rx_link_inertia = withLeftRightSigns(shoulder_rx_link_inertia, armID);

            SpatialInertia<Scalar> shoulder_rx_rotor_inertia(0., _smallRotorCOM,
                                                             _smallRotorRotInertiaX);
            shoulder_rx_rotor_inertia = withLeftRightSigns(shoulder_rx_rotor_inertia, armID);

            const spatial::Transform<Scalar> xtreeShoulderRx(I3, withLeftRightSigns(_shoulderRxLocation, armID));
            const spatial::Transform<Scalar> xtreeShoulderRxRotor(I3, withLeftRightSigns(_shoulderRxRotorLocation, armID));

            Body<Scalar> shoulder_rx_link = model.registerBody(shoulder_rx_link_name,
                                                               shoulder_rx_link_inertia,
                                                               shoulder_rx_parent_name,
                                                               xtreeShoulderRx);
            Body<Scalar> shoulder_rx_rotor = model.registerBody(shoulder_rx_rotor_name,
                                                                shoulder_rx_rotor_inertia,
                                                                shoulder_rx_parent_name,
                                                                xtreeShoulderRxRotor);
            GearedTransmissionModule<Scalar> shoulder_rx_module{shoulder_rx_link, shoulder_rx_rotor,
                                                                shoulder_rx_link_joint_name,
                                                                shoulder_rx_rotor_joint_name,
                                                                ori::CoordinateAxis::X,
                                                                ori::CoordinateAxis::X,
                                                                _shoulderRxGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<Scalar>>(
                shoulder_rx_name, shoulder_rx_module);

            // ShoulderRz
            const std::string shoulder_rz_parent_name = shoulder_rx_link_name;
            const std::string shoulder_rz_name = withLeftRightSigns("shoulder-rz", armID);
            const std::string shoulder_rz_link_name = withLeftRightSigns("shoulder-rz-link", armID);
            const std::string shoulder_rz_rotor_name = withLeftRightSigns("shoulder-rz-rotor", armID);
            const std::string shoulder_rz_link_joint_name = withLeftRightSigns("shoulder-rx-to-shoulder-rz", armID);
            const std::string shoulder_rz_rotor_joint_name = withLeftRightSigns("shoulder-rx-to-shoulder-rz-rotor", armID);

            SpatialInertia<Scalar> shoulder_rz_link_inertia(_shoulderRzMass, _shoulderRzCOM,
                                                            _shoulderRzRotInertia);
            shoulder_rz_link_inertia = withLeftRightSigns(shoulder_rz_link_inertia, armID);

            SpatialInertia<Scalar> shoulder_rz_rotor_inertia(0., _smallRotorCOM,
                                                             _smallRotorRotInertiaZ);
            shoulder_rz_rotor_inertia = withLeftRightSigns(shoulder_rz_rotor_inertia, armID);

            const spatial::Transform<Scalar> xtreeShoulderRz(I3, withLeftRightSigns(_shoulderRzLocation, armID));
            const spatial::Transform<Scalar> xtreeShoulderRzRotor(I3, withLeftRightSigns(_shoulderRzRotorLocation, armID));

            Body<Scalar> shoulder_rz_link = model.registerBody(shoulder_rz_link_name,
                                                               shoulder_rz_link_inertia,
                                                               shoulder_rz_parent_name,
                                                               xtreeShoulderRz);
            Body<Scalar> shoulder_rz_rotor = model.registerBody(shoulder_rz_rotor_name,
                                                                shoulder_rz_rotor_inertia,
                                                                shoulder_rz_parent_name,
                                                                xtreeShoulderRzRotor);
            GearedTransmissionModule<Scalar> shoulder_rz_module{shoulder_rz_link, shoulder_rz_rotor,
                                                                shoulder_rz_link_joint_name,
                                                                shoulder_rz_rotor_joint_name,
                                                                ori::CoordinateAxis::Z,
                                                                ori::CoordinateAxis::Z,
                                                                _shoulderRzGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<Scalar>>(
                shoulder_rz_name, shoulder_rz_module);

            // Elbow
            const std::string elbow_parent_name = shoulder_rz_link_name;
            const std::string elbow_name = withLeftRightSigns("elbow", armID);
            const std::string elbow_link_name = withLeftRightSigns("elbow-link", armID);
            const std::string elbow_rotor_name = withLeftRightSigns("elbow-rotor", armID);
            const std::string elbow_link_joint_name = withLeftRightSigns("shoulder-rz-to-elbow", armID);
            const std::string elbow_rotor_joint_name = withLeftRightSigns("shoulder-rz-to-elbow-rotor", armID);

            SpatialInertia<Scalar> elbow_link_inertia(_elbowMass, _elbowCOM, _elbowRotInertia);
            elbow_link_inertia = withLeftRightSigns(elbow_link_inertia, armID);

            SpatialInertia<Scalar> elbow_rotor_inertia(0., _smallRotorCOM, _smallRotorRotInertiaY);
            elbow_rotor_inertia = withLeftRightSigns(elbow_rotor_inertia, armID);

            const spatial::Transform<Scalar> xtreeElbow(I3, withLeftRightSigns(_elbowLocation, armID));
            const spatial::Transform<Scalar> xtreeElbowRotor(I3, withLeftRightSigns(_elbowRotorLocation, armID));

            Body<Scalar> elbow_link = model.registerBody(elbow_link_name, elbow_link_inertia,
                                                         elbow_parent_name, xtreeElbow);
            Body<Scalar> elbow_rotor = model.registerBody(elbow_rotor_name, elbow_rotor_inertia,
                                                          elbow_parent_name, xtreeElbowRotor);
            GearedTransmissionModule<Scalar> elbow_module{elbow_link, elbow_rotor,
                                                          elbow_link_joint_name,
                                                          elbow_rotor_joint_name,
                                                          ori::CoordinateAxis::Y,
                                                          ori::CoordinateAxis::Y,
                                                          _elbowGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<Scalar>>(elbow_name, elbow_module);

            const std::string elbow_contact_name = withLeftRightSigns("elbow-contact", armID);
            const std::string hand_contact_name = withLeftRightSigns("hand-contact", armID);
            model.appendContactPoint(elbow_link_name, Vec3<Scalar>(0, 0, 0), elbow_contact_name);
            model.appendContactPoint(elbow_link_name, Vec3<Scalar>(0, 0, -_lowerArmLength),
                                     hand_contact_name);
        }

        return model;
    }

    template class TelloWithArms<double>;
    template class TelloWithArms<casadi::SX>;

} // namespace grbda
