#include "grbda/Robots/TeleopArm.hpp"

namespace grbda
{

    ClusterTreeModel<double> TeleopArm::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;
        using SpatialTransform = spatial::Transform<double>;

        typedef typename ClusterJoints::ParallelBeltTransmissionModule<1, double> ProximalTransModule;
        typedef typename ClusterJoints::ParallelBeltTransmissionModule<2, double> IntermedTransModule;
        typedef typename ClusterJoints::ParallelBeltTransmissionModule<3, double> DistalTransModule;

        ClusterTreeModel<double> model{};

        Mat3<double> I3 = Mat3<double>::Identity();

        // Base
        SpatialTransform base_Xtree = SpatialTransform(I3, base_location_);
        Body<double> base = model.registerBody(base_name_, base_spatial_inertia_,
                                               base_parent_name_, base_Xtree);

        SpatialTransform base_rotor_Xtree = SpatialTransform(I3, base_rotor_location_);
        Body<double> base_rotor = model.registerBody(base_rotor_name_, base_rotor_spatial_inertia_,
                                                     base_rotor_parent_name_, base_rotor_Xtree);

        GearedTransmissionModule<double> base_module{base, base_rotor,
                                                     "base_link_joint", "base_rotor_joint",
                                                     ori::CoordinateAxis::Z,
                                                     ori::CoordinateAxis::Z, base_rotor_gear_ratio_};
        model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<double>>(base_cluster_name_, base_module);

        // Shoulder Rx
        SpatialTransform shoulder_rx_link_Xtree = SpatialTransform(I3, shoulder_rx_link_location_);
        Body<double> shoulder_rx_link = model.registerBody(shoulder_rx_link_name_,
                                                           shoulder_rx_link_spatial_inertia_,
                                                           shoulder_rx_link_parent_name_,
                                                           shoulder_rx_link_Xtree);

        SpatialTransform shoulder_rx_rotor_Xtree = SpatialTransform(I3, shoulder_rx_rotor_location_);
        Body<double> shoulder_rx_rotor = model.registerBody(shoulder_rx_rotor_name_,
                                                            shoulder_rx_rotor_spatial_inertia_,
                                                            shoulder_rx_rotor_parent_name_,
                                                            shoulder_rx_rotor_Xtree);

        GearedTransmissionModule<double> shoulder_rx_module{shoulder_rx_link, shoulder_rx_rotor,
                                                            "shoulder_rx_link_joint",
                                                            "shoulder_rx_rotor_joint",
                                                            ori::CoordinateAxis::X,
                                                            ori::CoordinateAxis::X,
                                                            shoulder_rx_rotor_gear_ratio_};
        model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<double>>(
            shoulder_rx_cluster_name_, shoulder_rx_module);

        // Shoulder Ry
        SpatialTransform shoulder_ry_link_Xtree = SpatialTransform(I3, shoulder_ry_link_location_);
        Body<double> shoulder_ry_link = model.registerBody(shoulder_ry_link_name_,
                                                           shoulder_ry_link_spatial_inertia_,
                                                           shoulder_ry_link_parent_name_,
                                                           shoulder_ry_link_Xtree);

        SpatialTransform shoulder_ry_rotor_Xtree = SpatialTransform(I3, shoulder_ry_rotor_location_);
        Body<double> shoulder_ry_rotor = model.registerBody(shoulder_ry_rotor_name_,
                                                            shoulder_ry_rotor_spatial_inertia_,
                                                            shoulder_ry_rotor_parent_name_,
                                                            shoulder_ry_rotor_Xtree);

        GearedTransmissionModule<double> shoulder_ry_module{shoulder_ry_link, shoulder_ry_rotor,
                                                            "shoulder_ry_link_joint",
                                                            "shoulder_ry_rotor_joint",
                                                            ori::CoordinateAxis::Y,
                                                            ori::CoordinateAxis::Y,
                                                            shoulder_ry_rotor_gear_ratio_};
        model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<double>>(
            shoulder_ry_cluster_name_, shoulder_ry_module);

        // Upper Link
        SpatialTransform upper_link_Xtree = SpatialTransform(I3, upper_link_location_);
        Body<double> upper_link = model.registerBody(upper_link_name_, upper_link_spatial_inertia_,
                                                     upper_link_parent_name_, upper_link_Xtree);

        // Wrist Pitch Link
        SpatialTransform wrist_pitch_link_Xtree = SpatialTransform(I3, wrist_pitch_link_location_);
        Body<double> wrist_pitch_link = model.registerBody(wrist_pitch_link_name_,
                                                           wrist_pitch_link_spatial_inertia_,
                                                           wrist_pitch_link_parent_name_,
                                                           wrist_pitch_link_Xtree);

        // Wrist Roll Link
        SpatialTransform wrist_roll_link_Xtree = SpatialTransform(I3, wrist_roll_link_location_);
        Body<double> wrist_roll_link = model.registerBody(wrist_roll_link_name_,
                                                          wrist_roll_link_spatial_inertia_,
                                                          wrist_roll_link_parent_name_,
                                                          wrist_roll_link_Xtree);

        // Elbow Rotor
        SpatialTransform elbow_rotor_Xtree = SpatialTransform(I3, elbow_rotor_location_);
        Body<double> elbow_rotor = model.registerBody(elbow_rotor_name_, elbow_rotor_spatial_inertia_,
                                                      elbow_rotor_parent_name_, elbow_rotor_Xtree);

        // Wrist Pitch Rotor
        SpatialTransform wrist_pitch_rotor_Xtree = SpatialTransform(I3, wrist_pitch_rotor_location_);
        Body<double> wrist_pitch_rotor = model.registerBody(wrist_pitch_rotor_name_,
                                                            wrist_pitch_rotor_spatial_inertia_,
                                                            wrist_pitch_rotor_parent_name_,
                                                            wrist_pitch_rotor_Xtree);

        // Wrist Roll Rotor
        SpatialTransform wrist_roll_rotor_Xtree = SpatialTransform(I3, wrist_roll_rotor_location_);
        Body<double> wrist_roll_rotor = model.registerBody(wrist_roll_rotor_name_,
                                                           wrist_roll_rotor_spatial_inertia_,
                                                           wrist_roll_rotor_parent_name_,
                                                           wrist_roll_rotor_Xtree);

        // Upper Arm Cluster
        ProximalTransModule upper_arm_module{upper_link, elbow_rotor,
                                             ori::CoordinateAxis::Y,
                                             ori::CoordinateAxis::Y,
                                             elbow_rotor_gear_ratio_,
                                             elbow_rotor_belt_ratios_};
        IntermedTransModule wrist_pitch_module{wrist_pitch_link, wrist_pitch_rotor,
                                               ori::CoordinateAxis::Y,
                                               ori::CoordinateAxis::Y,
                                               wrist_pitch_rotor_gear_ratio_,
                                               wrist_pitch_rotor_belt_ratios_};
        DistalTransModule wrist_roll_module{wrist_roll_link, wrist_roll_rotor,
                                            ori::CoordinateAxis::Z,
                                            ori::CoordinateAxis::Z,
                                            wrist_roll_rotor_gear_ratio_,
                                            wrist_roll_rotor_belt_ratios_};
        model.template appendRegisteredBodiesAsCluster<RevoluteTripleWithRotor<double>>(
            upper_arm_cluster_name_, upper_arm_module, wrist_pitch_module, wrist_roll_module);

        // Gripper
        SpatialTransform gripper_Xtree = SpatialTransform(I3, gripper_location_);
        Body<double> gripper = model.registerBody(gripper_name_, gripper_spatial_inertia_,
                                                  gripper_parent_name_, gripper_Xtree);

        SpatialTransform gripper_rotor_Xtree = SpatialTransform(I3, gripper_rotor_location_);
        Body<double> gripper_rotor = model.registerBody(gripper_rotor_name_, gripper_rotor_spatial_inertia_,
                                                        gripper_rotor_parent_name_, gripper_rotor_Xtree);

        GearedTransmissionModule<double> gripper_module{gripper, gripper_rotor,
                                                        "gripper_joint", "gripper_rotor_joint",
                                                        ori::CoordinateAxis::X,
                                                        ori::CoordinateAxis::X,
                                                        gripper_rotor_gear_ratio_};
        model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<double>>(gripper_cluster_name_,
                                                                                  gripper_module);

        model.appendContactPoint("upper-link", Vec3<double>(0., 0., 0.), "elbow-contact");
        model.appendContactPoint("wrist-pitch-link", Vec3<double>(0., 0., 0.), "wrist-pitch-contact");
        model.appendContactPoint("wrist-roll-link", Vec3<double>(0., 0., 0.), "wrist-roll-contact");

        return model;
    }

    TeleopArm::TeleopArm()
    {
        base_location_ = Vec3<double>(0, 0, 0.051);
        base_rotor_location_ = Vec3<double>(0, 0, 0);
        shoulder_rx_link_location_ = Vec3<double>(0, 0, 0.106);
        shoulder_rx_rotor_location_ = Vec3<double>(0, 0, 0);
        shoulder_ry_link_location_ = Vec3<double>(0, 0, 0.071);
        shoulder_ry_rotor_location_ = Vec3<double>(0, 0, 0);
        upper_link_location_ = Vec3<double>(0, -0.0095, 0.3855);
        wrist_pitch_link_location_ = Vec3<double>(0, 0, 0.362);
        wrist_roll_link_location_ = Vec3<double>(0, 0.004, 0.03574);
        elbow_rotor_location_ = Vec3<double>(0., 0., 0.);
        wrist_pitch_rotor_location_ = Vec3<double>(0., 0., 0.);
        wrist_roll_rotor_location_ = Vec3<double>(0., 0., 0.);
        gripper_location_ = Vec3<double>(0.0004, 0.0375, 0.070995);
        gripper_rotor_location_ = Vec3<double>(0, 0, 0);

        base_rotor_gear_ratio_ = 6.0;
        shoulder_rx_rotor_gear_ratio_ = 6.0;
        shoulder_ry_rotor_gear_ratio_ = 6.0;
        elbow_rotor_gear_ratio_ = 6.0;
        wrist_pitch_rotor_gear_ratio_ = 6.0;
        wrist_roll_rotor_gear_ratio_ = 6.0;
        gripper_rotor_gear_ratio_ = 2.0;

        elbow_rotor_belt_ratios_ = DVec<double>(1);
        elbow_rotor_belt_ratios_ << 1.0;

        wrist_pitch_rotor_belt_ratios_ = DVec<double>(2);
        wrist_pitch_rotor_belt_ratios_ << 1.0, 1.0;

        wrist_roll_rotor_belt_ratios_ = DVec<double>(3);
        wrist_roll_rotor_belt_ratios_ << -1.0, 1.0, -1.0;

        Mat3<double> largeRotorRotationalInertiaZ;
        largeRotorRotationalInertiaZ << 1.052, 0, 0, 0, 1.046, 0, 0, 0, 1.811;
        largeRotorRotationalInertiaZ = 1e-3 * largeRotorRotationalInertiaZ;

        Mat3<double> smallRotorRotationalInertiaZ;
        smallRotorRotationalInertiaZ << 1.052, 0, 0, 0, 1.046, 0, 0, 0, 1.811;
        smallRotorRotationalInertiaZ = 1e-3 * largeRotorRotationalInertiaZ;

        Mat3<double> RY = ori::coordinateRotation<double>(ori::CoordinateAxis::Y, M_PI / 2);
        Mat3<double> RX = ori::coordinateRotation<double>(ori::CoordinateAxis::X, M_PI / 2);

        Mat3<double> smallRotorRotationalInertiaX = RY * smallRotorRotationalInertiaZ * RY.transpose();
        Mat3<double> smallRotorRotationalInertiaY = RX * smallRotorRotationalInertiaZ * RX.transpose();
        Mat3<double> largeRotorRotationalInertiaX = RY * largeRotorRotationalInertiaZ * RY.transpose();
        Mat3<double> largeRotorRotationalInertiaY = RX * largeRotorRotationalInertiaZ * RX.transpose();

        Vec3<double> smallRotorCOM(0, 0, 0);
        SpatialInertia<double> smallRotorInertiaZ(0.055, smallRotorCOM, smallRotorRotationalInertiaZ);
        SpatialInertia<double> smallRotorInertiaX(0.055, smallRotorCOM, smallRotorRotationalInertiaX);
        SpatialInertia<double> smallRotorInertiaY(0.055, smallRotorCOM, smallRotorRotationalInertiaY);

        Vec3<double> largeRotorCOM(0, 0, 0);
        SpatialInertia<double> largeRotorInertiaZ(1.081, largeRotorCOM, largeRotorRotationalInertiaZ);
        SpatialInertia<double> largeRotorInertiaX(1.081, largeRotorCOM, largeRotorRotationalInertiaX);
        SpatialInertia<double> largeRotorInertiaY(1.081, largeRotorCOM, largeRotorRotationalInertiaY);

        Mat3<double> base_rotational_inertia;
        base_rotational_inertia << 0.003411721, -2.78E-09, -1.45E-09, -2.78E-09, 0.003047159, -0.000609866, -1.45E-09, -0.000609866, 0.003412964;
        Vec3<double> base_com(-1.32E-07, -0.001991858, 0.042209332);
        base_spatial_inertia_ = SpatialInertia<double>(0.996728196, base_com, base_rotational_inertia);

        base_rotor_spatial_inertia_ = SpatialInertia<double>(largeRotorInertiaZ.getMass(),
            largeRotorInertiaZ.getCOM(), largeRotorInertiaZ.getInertiaTensor());

        Mat3<double> shoulder_rx_link_rotational_inertia;
        shoulder_rx_link_rotational_inertia << 0.001190777, 3.21E-12, 2.07E-11, 3.21E-12, 0.001028401, 0.00022441, 2.07E-11, 0.00022441, 0.001459421;
        Vec3<double> shoulder_rx_link_com(-4.96E-10, -0.004546817, 0.045690967);
        shoulder_rx_link_spatial_inertia_ = SpatialInertia<double>(0.4796, shoulder_rx_link_com, shoulder_rx_link_rotational_inertia);

        shoulder_rx_rotor_spatial_inertia_ = SpatialInertia<double>(largeRotorInertiaX.getMass(),
            largeRotorInertiaX.getCOM(), largeRotorInertiaX.getInertiaTensor());

        Mat3<double> shoulder_ry_link_rotational_inertia;
        shoulder_ry_link_rotational_inertia << 0.016496934, 2.23E-07, 7.79E-05, 2.23E-07, 0.018757685, 0.000273553, 7.79E-05, 0.000273553, 0.003224214;
        Vec3<double> shoulder_ry_link_com(0.000482465, -0.000659857, 0.168192061);
        shoulder_ry_link_spatial_inertia_ = SpatialInertia<double>(1.670980082, shoulder_ry_link_com, shoulder_ry_link_rotational_inertia);

        shoulder_ry_rotor_spatial_inertia_ = SpatialInertia<double>(largeRotorInertiaY.getMass(),
            largeRotorInertiaY.getCOM(), largeRotorInertiaY.getInertiaTensor());

        Mat3<double> upper_link_rotational_inertia;
        upper_link_rotational_inertia << 0.006286029, -1.39E-10, -3.25E-09, -1.39E-10, 0.006437035, 6.24E-06, -3.25E-09, 6.24E-06, 0.00020455;
        Vec3<double> upper_link_com(3.99E-08, 0.003531916, 0.130185501);
        upper_link_spatial_inertia_ = SpatialInertia<double>(0.4617247, upper_link_com, upper_link_rotational_inertia);

        Mat3<double> wrist_pitch_link_rotational_inertia;
        wrist_pitch_link_rotational_inertia << 1.73E-05, -2.13E-12, -3.11E-13, -2.13E-12, 1.42E-05, -4.98E-07, -3.11E-13, -4.98E-07, 2.03E-05;
        Vec3<double> wrist_pitch_link_com(-1.70E-09, 0.004167466, 0.01683806);
        wrist_pitch_link_spatial_inertia_ = SpatialInertia<double>(0.063603259, wrist_pitch_link_com, wrist_pitch_link_rotational_inertia);

        Mat3<double> wrist_roll_link_rotational_inertia;
        wrist_roll_link_rotational_inertia << 0.000182127, 2.15E-06, -1.57E-06, 2.15E-06, 8.28E-05, -9.44E-06, -1.57E-06, -9.44E-06, 0.00011452;
        Vec3<double> wrist_roll_link_com(-0.001177177, 0.004697849, 0.040664076);
        wrist_roll_link_spatial_inertia_ = SpatialInertia<double>(0.167365855, wrist_roll_link_com, wrist_roll_link_rotational_inertia);

        elbow_rotor_spatial_inertia_ = SpatialInertia<double>(largeRotorInertiaY.getMass(),
            largeRotorInertiaY.getCOM(), largeRotorInertiaY.getInertiaTensor());

        wrist_pitch_rotor_spatial_inertia_ = SpatialInertia<double>(smallRotorInertiaY.getMass(),
            smallRotorInertiaY.getCOM(), smallRotorInertiaY.getInertiaTensor());

        wrist_roll_rotor_spatial_inertia_ = SpatialInertia<double>(smallRotorInertiaZ.getMass(),
            smallRotorInertiaZ.getCOM(), smallRotorInertiaZ.getInertiaTensor());

        Mat3<double> gripper_rotational_inertia;
        gripper_rotational_inertia << 6.20E-05, 5.26E-09, 1.04E-06, 5.26E-09, 6.64E-05, 8.69E-07, 1.04E-06, 8.69E-07, 1.81E-05;
        Vec3<double> gripper_com(0.000564355, -0.003238555, 0.105873754);
        gripper_spatial_inertia_ = SpatialInertia<double>(0.170943071, gripper_com, gripper_rotational_inertia);

        gripper_rotor_spatial_inertia_ = SpatialInertia<double>(smallRotorInertiaX.getMass(),
            smallRotorInertiaX.getCOM(), smallRotorInertiaX.getInertiaTensor());
    }

} // namespace grbda
