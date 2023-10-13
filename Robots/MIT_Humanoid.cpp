#include "MIT_Humanoid.hpp"

namespace grbda
{

    template <typename Scalar>
    MIT_Humanoid<Scalar>::MIT_Humanoid()
    {
        // Compute rotor inertias about different axes
        Vec3<Scalar> smallRotorCOM = Vec3<Scalar>(0., 0., 0.);
        Vec3<Scalar> largeRotorCOM = Vec3<Scalar>(0., 0., 0.);

        Mat3<Scalar> largeRotorRotationalInertiaZ;
        largeRotorRotationalInertiaZ << 3.443e-4, 0, 0,
            0, 3.443e-4, 0,
            0, 0, 5.548e-4;

        Mat3<Scalar> smallRotorRotationalInertiaZ;
        smallRotorRotationalInertiaZ << 1.084e-4, 0, 0,
            0, 1.084e-4, 0,
            0, 0, 1.6841e-4;

        Mat3<Scalar> RY = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y, M_PI / 2);
        Mat3<Scalar> RX = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::X, -M_PI / 2);

        Mat3<Scalar> smallRotorRotationalInertiaX =
            RY.transpose() * smallRotorRotationalInertiaZ * RY;
        Mat3<Scalar> smallRotorRotationalInertiaY =
            RX.transpose() * smallRotorRotationalInertiaZ * RX;
        Mat3<Scalar> largeRotorRotationalInertiaY =
            RX.transpose() * largeRotorRotationalInertiaZ * RX;

        // Set default inertial params
        InertialParams<Scalar> torsoInertialParams;
        torsoInertialParams.m = 8.52;
        torsoInertialParams.c = Vec3<Scalar>(0.009896, 0.004771, 0.100522);
        torsoInertialParams.Ic << 0.172699, 0.001419, 0.004023,
            0.001419, 0.105949, -0.001672,
            0.004023, -0.001672, 0.091906;
        _torsoLinearInertialParams = torsoInertialParams;

        InertialParams<Scalar> hipRzInertialParams;
        hipRzInertialParams.m = 0.84563;
        hipRzInertialParams.c = Vec3<Scalar>(-0.064842, -0.000036, -0.063090);
        hipRzInertialParams.Ic << 0.0015373, 0.0000011, 0.0005578,
            0.0000011, 0.0014252, 0.0000024,
            0.0005578, 0.0000024, 0.0012028;
        _hipRzLinearInertialParams = hipRzInertialParams;

        InertialParams<Scalar> hipRzRotorInertialParams;
        hipRzRotorInertialParams.c = smallRotorCOM;
        hipRzRotorInertialParams.Ic = smallRotorRotationalInertiaZ;
        _hipRzRotorLinearInertialParams = hipRzRotorInertialParams;

        InertialParams<Scalar> hipRxInertialParams;
        hipRxInertialParams.m = 1.908683;
        hipRxInertialParams.c = Vec3<Scalar>(0.067232, -0.013018, 0.0001831);
        hipRxInertialParams.Ic << 0.0017535, -0.0000063, -0.000080,
            -0.0000063, 0.003338, -0.000013,
            -0.000080, -0.000013, 0.0019927;
        _hipRxLinearInertialParams = hipRxInertialParams;

        InertialParams<Scalar> hipRxRotorInertialParams;
        hipRxRotorInertialParams.c = smallRotorCOM;
        hipRxRotorInertialParams.Ic = smallRotorRotationalInertiaX;
        _hipRxRotorLinearInertialParams = hipRxRotorInertialParams;

        InertialParams<Scalar> hipRyInertialParams;
        hipRyInertialParams.m = 2.64093;
        hipRyInertialParams.c = Vec3<Scalar>(0.0132054, 0.0269864, -0.096021);
        hipRyInertialParams.Ic << 0.0243761, 0.0000996, 0.0006548,
            0.0000996, 0.0259015, 0.0026713,
            0.0006548, 0.0026713, 0.0038929;
        _hipRyLinearInertialParams = hipRyInertialParams;

        InertialParams<Scalar> hipRyRotorInertialParams;
        hipRyRotorInertialParams.c = largeRotorCOM;
        hipRyRotorInertialParams.Ic = largeRotorRotationalInertiaY;
        _hipRyRotorLinearInertialParams = hipRyRotorInertialParams;

        InertialParams<Scalar> kneeInertialParams;
        kneeInertialParams.m = 0.3543355;
        kneeInertialParams.c = Vec3<Scalar>(0.00528, 0.0014762, -0.13201);
        kneeInertialParams.Ic << 0.003051, 0.000000, 0.0000873,
            0.000000, 0.003033, 0.0000393,
            0.0000873, 0.0000393, 0.0002529;
        _kneeLinearInertialParams = kneeInertialParams;

        InertialParams<Scalar> kneeRotorInertialParams;
        kneeRotorInertialParams.c = largeRotorCOM;
        kneeRotorInertialParams.Ic = largeRotorRotationalInertiaY;
        _kneeRotorLinearInertialParams = kneeRotorInertialParams;

        InertialParams<Scalar> ankleInertialParams;
        ankleInertialParams.m = 0.280951;
        ankleInertialParams.c = Vec3<Scalar>(0.022623, 0.0, -0.012826);
        ankleInertialParams.Ic << 0.0000842, 0.000000, -0.0000488,
            0.000000, 0.0007959, -0.000000,
            -0.0000488, -0.000000, 0.0007681;
        _ankleLinearInertialParams = ankleInertialParams;

        InertialParams<Scalar> ankleRotorInertialParams;
        ankleRotorInertialParams.c = smallRotorCOM;
        ankleRotorInertialParams.Ic = smallRotorRotationalInertiaY;
        _ankleRotorLinearInertialParams = ankleRotorInertialParams;

        InertialParams<Scalar> shoulderRyInertialParams;
        shoulderRyInertialParams.m = 0.788506;
        shoulderRyInertialParams.c = Vec3<Scalar>(0.009265, 0.052623, -0.0001249);
        shoulderRyInertialParams.Ic << 0.0013678, 0.0000266, 0.0000021,
            0.0000266, 0.0007392, -0.0000012,
            0.0000021, -0.0000012, 0.000884;
        _shoulderRyLinearInertialParams = shoulderRyInertialParams;

        InertialParams<Scalar> shoulderRyRotorInertialParams;
        shoulderRyRotorInertialParams.c = smallRotorCOM;
        shoulderRyRotorInertialParams.Ic = smallRotorRotationalInertiaY;
        _shoulderRyRotorLinearInertialParams = shoulderRyRotorInertialParams;

        InertialParams<Scalar> shoulderRxInertialParams;
        shoulderRxInertialParams.m = 0.80125;
        shoulderRxInertialParams.c = Vec3<Scalar>(0.0006041, 0.0001221, -0.082361);
        shoulderRxInertialParams.Ic << 0.0011524, 0.0000007, 0.0000396,
            0.0000007, 0.0011921, 0.0000014,
            0.0000396, 0.0000014, 0.0012386;
        _shoulderRxLinearInertialParams = shoulderRxInertialParams;

        InertialParams<Scalar> shoulderRxRotorInertialParams;
        shoulderRxRotorInertialParams.c = smallRotorCOM;
        shoulderRxRotorInertialParams.Ic = smallRotorRotationalInertiaX;
        _shoulderRxRotorLinearInertialParams = shoulderRxRotorInertialParams;

        InertialParams<Scalar> shoulderRzInertialParams;
        shoulderRzInertialParams.m = 0.905588;
        shoulderRzInertialParams.c = Vec3<Scalar>(0.0001703, -0.016797, -0.060);
        shoulderRzInertialParams.Ic << 0.0012713, 0.000001, -0.000008,
            0.000001, 0.0017477, -0.0000225,
            -0.000008, -0.0000225, 0.0008191;
        _shoulderRzLinearInertialParams = shoulderRzInertialParams; 

        InertialParams<Scalar> shoulderRzRotorInertialParams;
        shoulderRzRotorInertialParams.c = smallRotorCOM;
        shoulderRzRotorInertialParams.Ic = smallRotorRotationalInertiaZ;
        _shoulderRzRotorLinearInertialParams = shoulderRzRotorInertialParams;

        InertialParams<Scalar> elbowInertialParams;
        elbowInertialParams.m = 0.34839;
        elbowInertialParams.c = Vec3<Scalar>(-0.0059578, 0.000111, -0.0426735);
        elbowInertialParams.Ic << 0.001570, 0.0000002, 0.0000335,
            0.0000002, 0.0016167, 0.000003,
            0.0000335, 0.000003, 0.0000619;
        _elbowLinearInertialParams = elbowInertialParams;

        InertialParams<Scalar> elbowRotorInertialParams;
        elbowRotorInertialParams.c = smallRotorCOM;
        elbowRotorInertialParams.Ic = smallRotorRotationalInertiaY;
        _elbowRotorLinearInertialParams = elbowRotorInertialParams;
    }
    template <typename Scalar>
    ClusterTreeModel<Scalar> MIT_Humanoid<Scalar>::buildClusterTreeModel() const
    {
        typedef spatial::Transform<Scalar> Xform;
        typedef ClusterJoints::GearedTransmissionModule<Scalar> GearedTransModule;
        typedef ClusterJoints::ParallelBeltTransmissionModule<Scalar> ParallelBeltTransModule;
        typedef ClusterJoints::RevoluteWithRotor<Scalar> RevoluteWithRotor;
        typedef ClusterJoints::RevolutePairWithRotor<Scalar> RevolutePairWithRotor;

        ClusterTreeModel<Scalar> model;

        const Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

        // Torso
        const std::string torso_name = "Floating Base";
        const std::string torso_parent_name = "ground";
        const SpatialInertia<Scalar> torsoInertia(_torsoLinearInertialParams);
        model.template appendBody<ClusterJoints::Free<Scalar>>(torso_name, torsoInertia,
                                                               torso_parent_name, Xform{});

        Vec3<Scalar> torsoDims(_torsoLength, _torsoWidth, _torsoHeight);
        model.appendContactBox(torso_name, torsoDims);

        for (int legID = 0; legID < 2; legID++)
        {

            // HipRz
            const std::string hip_rz_parent_name = torso_name;
            const std::string hip_rz_name = withLeftRightSigns("hip_rz", legID);
            const std::string hip_rz_link_name = withLeftRightSigns("hip_rz_link", legID);
            const std::string hip_rz_rotor_name = withLeftRightSigns("hip_rz_rotor", legID);

            SpatialInertia<Scalar> hip_rz_link_inertia(_hipRzLinearInertialParams);
            hip_rz_link_inertia = withLeftRightSigns(hip_rz_link_inertia, legID);

            SpatialInertia<Scalar> hip_rz_rotor_inertia(_hipRzRotorLinearInertialParams);
            hip_rz_rotor_inertia = withLeftRightSigns(hip_rz_rotor_inertia, legID);

            Mat3<Scalar> Xrot_HipZ = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y,
                                                                     _hipRzPitch);
            const Vec3<Scalar> hipRzLocation = withLeftRightSigns(_hipRzLocation, legID);
            const Vec3<Scalar> hipRzRotorLocation = withLeftRightSigns(_hipRzRotorLocation, legID);
            const spatial::Transform<Scalar> xtreeHipRz(Xrot_HipZ, hipRzLocation);
            const spatial::Transform<Scalar> xtreeHipRzRotor(Xrot_HipZ, hipRzRotorLocation);

            Body<Scalar> hip_rz_link = model.registerBody(hip_rz_link_name, hip_rz_link_inertia,
                                                          hip_rz_parent_name, xtreeHipRz);
            Body<Scalar> hip_rz_rotor = model.registerBody(hip_rz_rotor_name, hip_rz_rotor_inertia,
                                                           hip_rz_parent_name, xtreeHipRzRotor);
            GearedTransModule hip_rz_module{hip_rz_link, hip_rz_rotor,
                                            ori::CoordinateAxis::Z, ori::CoordinateAxis::Z,
                                            _hipRzGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_rz_name,
                                                                              hip_rz_module);

            // HipRx
            const std::string hip_rx_parent_name = hip_rz_link_name;
            const std::string hip_rx_name = withLeftRightSigns("hip_rx", legID);
            const std::string hip_rx_link_name = withLeftRightSigns("hip_rx_link", legID);
            const std::string hip_rx_rotor_name = withLeftRightSigns("hip_rx_rotor", legID);

            SpatialInertia<Scalar> hip_rx_link_inertia(_hipRxLinearInertialParams);
            hip_rx_link_inertia = withLeftRightSigns(hip_rx_link_inertia, legID);

            SpatialInertia<Scalar> hip_rx_rotor_inertia(_hipRxRotorLinearInertialParams);
            hip_rx_rotor_inertia = withLeftRightSigns(hip_rx_rotor_inertia, legID);

            Mat3<Scalar> Xrot_HipX = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y,
                                                                     _hipRxPitch);
            const Vec3<Scalar> hipRxLocation = withLeftRightSigns(_hipRxLocation, legID);
            const Vec3<Scalar> hipRxRotorLocation = withLeftRightSigns(_hipRxRotorLocation, legID);
            const spatial::Transform<Scalar> xtreeHipRx(Xrot_HipX, hipRxLocation);
            const spatial::Transform<Scalar> xtreeHipRxRotor(Xrot_HipX, hipRxRotorLocation);

            Body<Scalar> hip_rx_link = model.registerBody(hip_rx_link_name, hip_rx_link_inertia,
                                                          hip_rx_parent_name, xtreeHipRx);
            Body<Scalar> hip_rx_rotor = model.registerBody(hip_rx_rotor_name, hip_rx_rotor_inertia,
                                                           hip_rx_parent_name, xtreeHipRxRotor);
            GearedTransModule hip_rx_module{hip_rx_link, hip_rx_rotor,
                                            ori::CoordinateAxis::X, ori::CoordinateAxis::X,
                                            _hipRxGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_rx_name,
                                                                              hip_rx_module);

            // HipRy
            const std::string hip_ry_parent_name = hip_rx_link_name;
            const std::string hip_ry_name = withLeftRightSigns("hip_ry", legID);
            const std::string hip_ry_link_name = withLeftRightSigns("hip_ry_link", legID);
            const std::string hip_ry_rotor_name = withLeftRightSigns("hip_ry_rotor", legID);

            SpatialInertia<Scalar> hip_ry_link_inertia(_hipRyLinearInertialParams);
            hip_ry_link_inertia = withLeftRightSigns(hip_ry_link_inertia, legID);

            SpatialInertia<Scalar> hip_ry_rotor_inertia(_hipRyRotorLinearInertialParams);
            hip_ry_rotor_inertia = withLeftRightSigns(hip_ry_rotor_inertia, legID);

            Mat3<Scalar> Xrot_HipY = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y,
                                                                     _hipRyPitch);
            const Vec3<Scalar> hipRyLocation = withLeftRightSigns(_hipRyLocation, legID);
            const Vec3<Scalar> hipRyRotorLocation = withLeftRightSigns(_hipRyRotorLocation, legID);
            const spatial::Transform<Scalar> xtreeHipRy(Xrot_HipY, hipRyLocation);
            const spatial::Transform<Scalar> xtreeHipRyRotor(Xrot_HipY, hipRyRotorLocation);

            Body<Scalar> hip_ry_link = model.registerBody(hip_ry_link_name, hip_ry_link_inertia,
                                                          hip_ry_parent_name, xtreeHipRy);
            Body<Scalar> hip_ry_rotor = model.registerBody(hip_ry_rotor_name, hip_ry_rotor_inertia,
                                                           hip_ry_parent_name, xtreeHipRyRotor);
            GearedTransModule hip_ry_module{hip_ry_link, hip_ry_rotor,
                                            ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                            _hipRyGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_ry_name,
                                                                              hip_ry_module);

            const std::string knee_contact_name = withLeftRightSigns("knee_contact", legID);
            model.appendContactPoint(hip_ry_link_name, Vec3<Scalar>(0, 0, -_thighLength),
                                     knee_contact_name);

            // Knee
            const std::string knee_parent_name = withLeftRightSigns("hip_ry_link", legID);
            const std::string knee_link_name = withLeftRightSigns("knee_link", legID);
            const std::string knee_rotor_name = withLeftRightSigns("knee_rotor", legID);

            SpatialInertia<Scalar> knee_link_inertia(_kneeLinearInertialParams);
            knee_link_inertia = withLeftRightSigns(knee_link_inertia, legID);

            SpatialInertia<Scalar> knee_rotor_inertia(_kneeRotorLinearInertialParams);
            knee_rotor_inertia = withLeftRightSigns(knee_rotor_inertia, legID);

            const Vec3<Scalar> kneeLocation = withLeftRightSigns(_kneeLocation, legID);
            const Vec3<Scalar> kneeRotorLocation = withLeftRightSigns(_kneeRotorLocation, legID);
            const spatial::Transform<Scalar> xtreeKnee(I3, kneeLocation);
            const spatial::Transform<Scalar> xtreeKneeRotor(I3, kneeRotorLocation);

            Body<Scalar> knee_link = model.registerBody(knee_link_name, knee_link_inertia,
                                                        knee_parent_name, xtreeKnee);
            Body<Scalar> knee_rotor = model.registerBody(knee_rotor_name, knee_rotor_inertia,
                                                         knee_parent_name, xtreeKneeRotor);
            ParallelBeltTransModule knee_module{knee_link, knee_rotor,
                                                ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                                _kneeGearRatio, _kneeBeltRatio};

            // Ankle
            const std::string ankle_parent_name = knee_link_name;
            const std::string ankle_link_name = withLeftRightSigns("ankle_link", legID);
            const std::string ankle_rotor_name = withLeftRightSigns("ankle_rotor", legID);

            SpatialInertia<Scalar> ankle_link_inertia(_ankleLinearInertialParams);
            ankle_link_inertia = withLeftRightSigns(ankle_link_inertia, legID);

            SpatialInertia<Scalar> ankle_rotor_inertia(_ankleRotorLinearInertialParams);
            ankle_rotor_inertia = withLeftRightSigns(ankle_rotor_inertia, legID);

            const Vec3<Scalar> ankleLocation = withLeftRightSigns(_ankleLocation, legID);
            const Vec3<Scalar> ankleRotorLocation = withLeftRightSigns(_ankleRotorLocation, legID);
            const spatial::Transform<Scalar> xtreeAnkle(I3, ankleLocation);
            const spatial::Transform<Scalar> xtreeAnkleRotor(I3, ankleRotorLocation);

            Body<Scalar> ankle_rotor = model.registerBody(ankle_rotor_name, ankle_rotor_inertia,
                                                          knee_parent_name, xtreeAnkleRotor);
            Body<Scalar> ankle_link = model.registerBody(ankle_link_name, ankle_link_inertia,
                                                         ankle_parent_name, xtreeAnkle);
            ParallelBeltTransModule ankle_module{ankle_link, ankle_rotor,
                                                 ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                                 _ankleGearRatio, _ankleBeltRatio};

            // Cluster
            const std::string knee_and_ankle_name = withLeftRightSigns("knee_and_ankle", legID);
            model.template appendRegisteredBodiesAsCluster<RevolutePairWithRotor>(
                knee_and_ankle_name, knee_module, ankle_module);

            // Contact Points
            const std::string toe_contact_name = withLeftRightSigns("toe_contact", legID);
            const std::string heel_contact_name = withLeftRightSigns("heel_contact", legID);
            if (legID == 0)
                model.appendEndEffector(ankle_link_name,
                                        Vec3<Scalar>(_footToeLength, 0, -_footHeight),
                                        toe_contact_name);
            else
                model.appendContactPoint(ankle_link_name,
                                         Vec3<Scalar>(_footToeLength, 0, -_footHeight),
                                         toe_contact_name);
            model.appendContactPoint(ankle_link_name,
                                     Vec3<Scalar>(-_footHeelLength, 0, -_footHeight),
                                     heel_contact_name);
        }

        for (int armID = 0; armID < 2; armID++)
        {
            // ShoulderRy
            const std::string shoulder_ry_parent_name = torso_name;
            const std::string shoulder_ry_name = withLeftRightSigns("shoulder_ry", armID);
            const std::string shoulder_ry_link_name = withLeftRightSigns("shoulder_ry_link", armID);
            const std::string shoulder_ry_rotor_name = withLeftRightSigns("shoulder_ry_rotor", armID);

            SpatialInertia<Scalar> shoulder_ry_link_inertia(_shoulderRyLinearInertialParams);
            shoulder_ry_link_inertia = withLeftRightSigns(shoulder_ry_link_inertia, armID);

            SpatialInertia<Scalar> shoulder_ry_rotor_inertia(_shoulderRyRotorLinearInertialParams);
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
            GearedTransModule shoulder_ry_module{shoulder_ry_link, shoulder_ry_rotor,
                                                 ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                                 _shoulderRyGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(shoulder_ry_name,
                                                                              shoulder_ry_module);

            // ShoulderRx
            const std::string shoulder_rx_parent_name = shoulder_ry_link_name;
            const std::string shoulder_rx_name = withLeftRightSigns("shoulder_rx", armID);
            const std::string shoulder_rx_link_name = withLeftRightSigns("shoulder_rx_link", armID);
            const std::string shoulder_rx_rotor_name = withLeftRightSigns("shoulder_rx_rotor", armID);

            SpatialInertia<Scalar> shoulder_rx_link_inertia(_shoulderRxLinearInertialParams);
            shoulder_rx_link_inertia = withLeftRightSigns(shoulder_rx_link_inertia, armID);

            SpatialInertia<Scalar> shoulder_rx_rotor_inertia(_shoulderRxRotorLinearInertialParams);
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
            GearedTransModule shoulder_rx_module{shoulder_rx_link, shoulder_rx_rotor,
                                                 ori::CoordinateAxis::X, ori::CoordinateAxis::X,
                                                 _shoulderRxGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(shoulder_rx_name,
                                                                              shoulder_rx_module);

            // ShoulderRz
            const std::string shoulder_rz_parent_name = shoulder_rx_link_name;
            const std::string shoulder_rz_name = withLeftRightSigns("shoulder_rz", armID);
            const std::string shoulder_rz_link_name = withLeftRightSigns("shoulder_rz_link", armID);
            const std::string shoulder_rz_rotor_name = withLeftRightSigns("shoulder_rz_rotor", armID);

            SpatialInertia<Scalar> shoulder_rz_link_inertia(_shoulderRzLinearInertialParams);
            shoulder_rz_link_inertia = withLeftRightSigns(shoulder_rz_link_inertia, armID);

            SpatialInertia<Scalar> shoulder_rz_rotor_inertia(_shoulderRzRotorLinearInertialParams);
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
            GearedTransModule shoulder_rz_module{shoulder_rz_link, shoulder_rz_rotor,
                                                 ori::CoordinateAxis::Z, ori::CoordinateAxis::Z,
                                                 _shoulderRzGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(shoulder_rz_name,
                                                                              shoulder_rz_module);

            // Elbow
            const std::string elbow_parent_name = shoulder_rz_link_name;
            const std::string elbow_name = withLeftRightSigns("elbow", armID);
            const std::string elbow_link_name = withLeftRightSigns("elbow_link", armID);
            const std::string elbow_rotor_name = withLeftRightSigns("elbow_rotor", armID);

            SpatialInertia<Scalar> elbow_link_inertia(_elbowLinearInertialParams);
            elbow_link_inertia = withLeftRightSigns(elbow_link_inertia, armID);

            SpatialInertia<Scalar> elbow_rotor_inertia(_elbowRotorLinearInertialParams);
            elbow_rotor_inertia = withLeftRightSigns(elbow_rotor_inertia, armID);

            const spatial::Transform<Scalar> xtreeElbow(I3, withLeftRightSigns(_elbowLocation, armID));
            const spatial::Transform<Scalar> xtreeElbowRotor(I3, withLeftRightSigns(_elbowRotorLocation, armID));

            Body<Scalar> elbow_link = model.registerBody(elbow_link_name, elbow_link_inertia,
                                                         elbow_parent_name, xtreeElbow);
            Body<Scalar> elbow_rotor = model.registerBody(elbow_rotor_name, elbow_rotor_inertia,
                                                          elbow_parent_name, xtreeElbowRotor);
            GearedTransModule elbow_module{elbow_link, elbow_rotor, ori::CoordinateAxis::Y,
                                           ori::CoordinateAxis::Y, _elbowGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(elbow_name,
                                                                              elbow_module);

            const std::string elbow_contact_name = withLeftRightSigns("elbow_contact", armID);
            const std::string hand_contact_name = withLeftRightSigns("hand_contact", armID);
            model.appendContactPoint(elbow_link_name, Vec3<Scalar>(0, 0, 0), elbow_contact_name);
            model.appendContactPoint(elbow_link_name, Vec3<Scalar>(0, 0, -_lowerArmLength),
                                     hand_contact_name);
        }

        return model;
    }

    template class MIT_Humanoid<double>;
    template class MIT_Humanoid<casadi::SX>;
}
