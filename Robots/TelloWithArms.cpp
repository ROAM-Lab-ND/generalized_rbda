#include "TelloWithArms.hpp"

namespace grbda
{

    ClusterTreeModel TelloWithArms::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;

        ClusterTreeModel model = Tello::buildClusterTreeModel();

        const Mat3<double> I3 = Mat3<double>::Identity();

        for (int armID = 0; armID < 2; armID++)
        {
            // ShoulderRy
            const std::string shoulder_ry_parent_name = "torso";
            const std::string shoulder_ry_name = withLeftRightSigns("shoulder_ry", armID);
            const std::string shoulder_ry_link_name = withLeftRightSigns("shoulder_ry_link", armID);
            const std::string shoulder_ry_rotor_name = withLeftRightSigns("shoulder_ry_rotor", armID);

            SpatialInertia<double> shoulder_ry_link_inertia(_shoulderRyMass, _shoulderRyCOM,
                                                            _shoulderRyRotInertia);
            shoulder_ry_link_inertia = withLeftRightSigns(shoulder_ry_link_inertia, armID);

            SpatialInertia<double> shoulder_ry_rotor_inertia(0., _smallRotorCOM,
                                                             _smallRotorRotInertiaY);
            shoulder_ry_rotor_inertia = withLeftRightSigns(shoulder_ry_rotor_inertia, armID);

            const spatial::Transform<> xtreeShoulderRy(I3, withLeftRightSigns(_shoulderRyLocation, armID));
            const spatial::Transform<> xtreeShoulderRyRotor(I3, withLeftRightSigns(_shoulderRyRotorLocation, armID));

            Body<> shoulder_ry_link = model.registerBody(shoulder_ry_link_name,
                                                       shoulder_ry_link_inertia,
                                                       shoulder_ry_parent_name,
                                                       xtreeShoulderRy);
            Body<> shoulder_ry_rotor = model.registerBody(shoulder_ry_rotor_name,
                                                        shoulder_ry_rotor_inertia,
                                                        shoulder_ry_parent_name,
                                                        xtreeShoulderRyRotor);
            GearedTransmissionModule<> shoulder_ry_module{shoulder_ry_link, shoulder_ry_rotor,
                                                          ori::CoordinateAxis::Y,
                                                          ori::CoordinateAxis::Y,
                                                          _shoulderRyGearRatio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor<>>(shoulder_ry_name,
                                                                       shoulder_ry_module);

            // ShoulderRx
            const std::string shoulder_rx_parent_name = shoulder_ry_link_name;
            const std::string shoulder_rx_name = withLeftRightSigns("shoulder_rx", armID);
            const std::string shoulder_rx_link_name = withLeftRightSigns("shoulder_rx_link", armID);
            const std::string shoulder_rx_rotor_name = withLeftRightSigns("shoulder_rx_rotor", armID);

            SpatialInertia<double> shoulder_rx_link_inertia(_shoulderRxMass, _shoulderRxCOM,
                                                            _shoulderRxRotInertia);
            shoulder_rx_link_inertia = withLeftRightSigns(shoulder_rx_link_inertia, armID);

            SpatialInertia<double> shoulder_rx_rotor_inertia(0., _smallRotorCOM,
                                                             _smallRotorRotInertiaX);
            shoulder_rx_rotor_inertia = withLeftRightSigns(shoulder_rx_rotor_inertia, armID);

            const spatial::Transform<> xtreeShoulderRx(I3, withLeftRightSigns(_shoulderRxLocation, armID));
            const spatial::Transform<> xtreeShoulderRxRotor(I3, withLeftRightSigns(_shoulderRxRotorLocation, armID));

            Body<> shoulder_rx_link = model.registerBody(shoulder_rx_link_name,
                                                       shoulder_rx_link_inertia,
                                                       shoulder_rx_parent_name,
                                                       xtreeShoulderRx);
            Body<> shoulder_rx_rotor = model.registerBody(shoulder_rx_rotor_name,
                                                        shoulder_rx_rotor_inertia,
                                                        shoulder_rx_parent_name,
                                                        xtreeShoulderRxRotor);
            GearedTransmissionModule<> shoulder_rx_module{shoulder_rx_link, shoulder_rx_rotor,
                                                          ori::CoordinateAxis::X,
                                                          ori::CoordinateAxis::X,
                                                          _shoulderRxGearRatio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor<>>(shoulder_rx_name,
                                                                       shoulder_rx_module);

            // ShoulderRz
            const std::string shoulder_rz_parent_name = shoulder_rx_link_name;
            const std::string shoulder_rz_name = withLeftRightSigns("shoulder_rz", armID);
            const std::string shoulder_rz_link_name = withLeftRightSigns("shoulder_rz_link", armID);
            const std::string shoulder_rz_rotor_name = withLeftRightSigns("shoulder_rz_rotor", armID);

            SpatialInertia<double> shoulder_rz_link_inertia(_shoulderRzMass, _shoulderRzCOM,
                                                            _shoulderRzRotInertia);
            shoulder_rz_link_inertia = withLeftRightSigns(shoulder_rz_link_inertia, armID);

            SpatialInertia<double> shoulder_rz_rotor_inertia(0., _smallRotorCOM,
                                                             _smallRotorRotInertiaZ);
            shoulder_rz_rotor_inertia = withLeftRightSigns(shoulder_rz_rotor_inertia, armID);

            const spatial::Transform<> xtreeShoulderRz(I3, withLeftRightSigns(_shoulderRzLocation, armID));
            const spatial::Transform<> xtreeShoulderRzRotor(I3, withLeftRightSigns(_shoulderRzRotorLocation, armID));

            Body<> shoulder_rz_link = model.registerBody(shoulder_rz_link_name,
                                                       shoulder_rz_link_inertia,
                                                       shoulder_rz_parent_name,
                                                       xtreeShoulderRz);
            Body<> shoulder_rz_rotor = model.registerBody(shoulder_rz_rotor_name,
                                                        shoulder_rz_rotor_inertia,
                                                        shoulder_rz_parent_name,
                                                        xtreeShoulderRzRotor);
            GearedTransmissionModule<> shoulder_rz_module{shoulder_rz_link, shoulder_rz_rotor,
                                                          ori::CoordinateAxis::Z,
                                                          ori::CoordinateAxis::Z,
                                                          _shoulderRzGearRatio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor<>>(shoulder_rz_name,
                                                                       shoulder_rz_module);

            // Elbow
            const std::string elbow_parent_name = shoulder_rz_link_name;
            const std::string elbow_name = withLeftRightSigns("elbow", armID);
            const std::string elbow_link_name = withLeftRightSigns("elbow_link", armID);
            const std::string elbow_rotor_name = withLeftRightSigns("elbow_rotor", armID);

            SpatialInertia<double> elbow_link_inertia(_elbowMass, _elbowCOM, _elbowRotInertia);
            elbow_link_inertia = withLeftRightSigns(elbow_link_inertia, armID);

            SpatialInertia<double> elbow_rotor_inertia(0., _smallRotorCOM,
                                                       _smallRotorRotInertiaY);
            elbow_rotor_inertia = withLeftRightSigns(elbow_rotor_inertia, armID);

            const spatial::Transform<> xtreeElbow(I3, withLeftRightSigns(_elbowLocation, armID));
            const spatial::Transform<> xtreeElbowRotor(I3, withLeftRightSigns(_elbowRotorLocation, armID));

            Body<> elbow_link = model.registerBody(elbow_link_name, elbow_link_inertia,
                                                 elbow_parent_name, xtreeElbow);
            Body<> elbow_rotor = model.registerBody(elbow_rotor_name, elbow_rotor_inertia,
                                                  elbow_parent_name, xtreeElbowRotor);
            GearedTransmissionModule<> elbow_module{elbow_link, elbow_rotor, ori::CoordinateAxis::Y,
                                                    ori::CoordinateAxis::Y, _elbowGearRatio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor<>>(elbow_name, elbow_module);

            const std::string elbow_contact_name = withLeftRightSigns("elbow_contact", armID);
            const std::string hand_contact_name = withLeftRightSigns("hand_contact", armID);
            model.appendContactPoint(elbow_link_name, Vec3<double>(0, 0, 0), elbow_contact_name);
            model.appendContactPoint(elbow_link_name, Vec3<double>(0, 0, -_lowerArmLength),
                                     hand_contact_name);
        }

        return model;
    }

} // namespace grbda
