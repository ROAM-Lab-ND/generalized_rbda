#include "MIT_Humanoid.hpp"

namespace grbda
{

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
        const SpatialInertia<Scalar> torsoInertia(_torsoMass, _torsoCOM, _torsoRotInertia);
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

            SpatialInertia<Scalar> hip_rz_link_inertia(_hipRzMass, _hipRzCOM, _hipRzRotInertia);
            hip_rz_link_inertia = withLeftRightSigns(hip_rz_link_inertia, legID);

            SpatialInertia<Scalar> hip_rz_rotor_inertia(0., _smallRotorCOM, _smallRotorRotInertiaZ);
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

            SpatialInertia<Scalar> hip_rx_link_inertia(_hipRxMass, _hipRxCOM, _hipRxRotInertia);
            hip_rx_link_inertia = withLeftRightSigns(hip_rx_link_inertia, legID);

            SpatialInertia<Scalar> hip_rx_rotor_inertia(0., _smallRotorCOM, _smallRotorRotInertiaX);
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

            SpatialInertia<Scalar> hip_ry_link_inertia(_hipRyMass, _hipRyCOM,
                                                       _hipRyRotInertia);
            hip_ry_link_inertia = withLeftRightSigns(hip_ry_link_inertia, legID);

            SpatialInertia<Scalar> hip_ry_rotor_inertia(0., _largeRotorCOM,
                                                        _largeRotorRotInertiaY);
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

            SpatialInertia<Scalar> knee_link_inertia(_kneeMass, _kneeCOM, _kneeRotInertia);
            knee_link_inertia = withLeftRightSigns(knee_link_inertia, legID);

            SpatialInertia<Scalar> knee_rotor_inertia(0., _largeRotorCOM, _largeRotorRotInertiaY);
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

            SpatialInertia<Scalar> ankle_link_inertia(_ankleMass, _ankleCOM, _ankleRotInertia);
            ankle_link_inertia = withLeftRightSigns(ankle_link_inertia, legID);

            SpatialInertia<Scalar> ankle_rotor_inertia(0., _smallRotorCOM, _smallRotorRotInertiaY);
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

            SpatialInertia<Scalar> elbow_link_inertia(_elbowMass, _elbowCOM, _elbowRotInertia);
            elbow_link_inertia = withLeftRightSigns(elbow_link_inertia, armID);

            SpatialInertia<Scalar> elbow_rotor_inertia(0., _smallRotorCOM,
                                                       _smallRotorRotInertiaY);
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
