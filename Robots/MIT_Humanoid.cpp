#include "MIT_Humanoid.hpp"

namespace grbda
{

    ClusterTreeModel MIT_Humanoid::buildClusterTreeModel() const
    {
        ClusterTreeModel model;

        const Mat3<double> I3 = Mat3<double>::Identity();

        // Torso
        const std::string torso_name = "Floating Base";
        const std::string torso_parent_name = "ground";
        const SpatialInertia<double> torsoInertia(_torsoMass, _torsoCOM, _torsoRotInertia);
        Body torso = model.registerBody(torso_name, torsoInertia, torso_parent_name,
                                        spatial::SpatialTransform{});
        auto torso_generalized_joint = std::make_shared<GeneralizedJoints::Free>(torso);
        model.appendRegisteredBodiesAsCluster(torso_name, torso_generalized_joint);

        Vec3<double> torsoDims(_torsoLength, _torsoWidth, _torsoHeight);
        model.appendContactBox(torso_name, torsoDims);

        for (int legID = 0; legID < 2; legID++)
        {

            // HipRz
            const std::string hip_rz_parent_name = torso_name;
            const std::string hip_rz_name = withLeftRightSigns("hip_rz", legID);
            const std::string hip_rz_link_name = withLeftRightSigns("hip_rz_link", legID);
            const std::string hip_rz_rotor_name = withLeftRightSigns("hip_rz_rotor", legID);

            SpatialInertia<double> hip_rz_link_inertia(_hipRzMass, _hipRzCOM, _hipRzRotInertia);
            hip_rz_link_inertia = withLeftRightSigns(hip_rz_link_inertia, legID);

            SpatialInertia<double> hip_rz_rotor_inertia(0., _smallRotorCOM, _smallRotorRotInertiaZ);
            hip_rz_rotor_inertia = withLeftRightSigns(hip_rz_rotor_inertia, legID);

            Mat3<double> Xrot_HipZ = coordinateRotation<double>(CoordinateAxis::Y, _hipRzPitch);
            const Vec3<double> hipRzLocation = withLeftRightSigns(_hipRzLocation, legID);
            const Vec3<double> hipRzRotorLocation = withLeftRightSigns(_hipRzRotorLocation, legID);
            const spatial::SpatialTransform xtreeHipRz(Xrot_HipZ, hipRzLocation);
            const spatial::SpatialTransform xtreeHipRzRotor(Xrot_HipZ, hipRzRotorLocation);

            Body hip_rz_link = model.registerBody(hip_rz_link_name, hip_rz_link_inertia,
                                                  hip_rz_parent_name, xtreeHipRz);
            Body hip_rz_rotor = model.registerBody(hip_rz_rotor_name, hip_rz_rotor_inertia,
                                                   hip_rz_parent_name, xtreeHipRzRotor);

            auto hip_rz_generalized_joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(hip_rz_link, hip_rz_rotor, CoordinateAxis::Z, CoordinateAxis::Z, _hipRzGearRatio);
            model.appendRegisteredBodiesAsCluster(hip_rz_name, hip_rz_generalized_joint);

            // HipRx
            const std::string hip_rx_parent_name = hip_rz_link_name;
            const std::string hip_rx_name = withLeftRightSigns("hip_rx", legID);
            const std::string hip_rx_link_name = withLeftRightSigns("hip_rx_link", legID);
            const std::string hip_rx_rotor_name = withLeftRightSigns("hip_rx_rotor", legID);

            SpatialInertia<double> hip_rx_link_inertia(_hipRxMass, _hipRxCOM, _hipRxRotInertia);
            hip_rx_link_inertia = withLeftRightSigns(hip_rx_link_inertia, legID);

            SpatialInertia<double> hip_rx_rotor_inertia(0., _smallRotorCOM, _smallRotorRotInertiaX);
            hip_rx_rotor_inertia = withLeftRightSigns(hip_rx_rotor_inertia, legID);

            Mat3<double> Xrot_HipX = coordinateRotation<double>(CoordinateAxis::Y, _hipRxPitch);
            const Vec3<double> hipRxLocation = withLeftRightSigns(_hipRxLocation, legID);
            const Vec3<double> hipRxRotorLocation = withLeftRightSigns(_hipRxRotorLocation, legID);
            const spatial::SpatialTransform xtreeHipRx(Xrot_HipX, hipRxLocation);
            const spatial::SpatialTransform xtreeHipRxRotor(Xrot_HipX, hipRxRotorLocation);

            Body hip_rx_link = model.registerBody(hip_rx_link_name, hip_rx_link_inertia,
                                                  hip_rx_parent_name, xtreeHipRx);
            Body hip_rx_rotor = model.registerBody(hip_rx_rotor_name, hip_rx_rotor_inertia,
                                                   hip_rx_parent_name, xtreeHipRxRotor);

            auto hip_rx_generalized_joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(hip_rx_link, hip_rx_rotor, CoordinateAxis::X, CoordinateAxis::X, _hipRxGearRatio);
            model.appendRegisteredBodiesAsCluster(hip_rx_name, hip_rx_generalized_joint);

            // HipRy
            const std::string hip_ry_parent_name = hip_rx_link_name;
            const std::string hip_ry_name = withLeftRightSigns("hip_ry", legID);
            const std::string hip_ry_link_name = withLeftRightSigns("hip_ry_link", legID);
            const std::string hip_ry_rotor_name = withLeftRightSigns("hip_ry_rotor", legID);

            SpatialInertia<double> hip_ry_link_inertia(_hipRyMass, _hipRyCOM,
                                                       _hipRyRotInertia);
            hip_ry_link_inertia = withLeftRightSigns(hip_ry_link_inertia, legID);

            SpatialInertia<double> hip_ry_rotor_inertia(0., _largeRotorCOM,
                                                        _largeRotorRotInertiaY);
            hip_ry_rotor_inertia = withLeftRightSigns(hip_ry_rotor_inertia, legID);

            Mat3<double> Xrot_HipY = coordinateRotation<double>(CoordinateAxis::Y, _hipRyPitch);
            const Vec3<double> hipRyLocation = withLeftRightSigns(_hipRyLocation, legID);
            const Vec3<double> hipRyRotorLocation = withLeftRightSigns(_hipRyRotorLocation, legID);
            const spatial::SpatialTransform xtreeHipRy(Xrot_HipY, hipRyLocation);
            const spatial::SpatialTransform xtreeHipRyRotor(Xrot_HipY, hipRyRotorLocation);

            Body hip_ry_link = model.registerBody(hip_ry_link_name, hip_ry_link_inertia,
                                                  hip_ry_parent_name, xtreeHipRy);
            Body hip_ry_rotor = model.registerBody(hip_ry_rotor_name, hip_ry_rotor_inertia,
                                                   hip_ry_parent_name, xtreeHipRyRotor);

            auto hip_ry_generalized_joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(hip_ry_link, hip_ry_rotor, CoordinateAxis::Y, CoordinateAxis::Y, _hipRyGearRatio);
            model.appendRegisteredBodiesAsCluster(hip_ry_name, hip_ry_generalized_joint);

            const std::string knee_contact_name = withLeftRightSigns("knee_contact", legID);
            model.appendContactPoint(hip_ry_link_name, Vec3<double>(0, 0, -_thighLength),
                                     knee_contact_name);

            // Knee
            const std::string knee_parent_name = withLeftRightSigns("hip_ry_link", legID);
            const std::string knee_link_name = withLeftRightSigns("knee_link", legID);
            const std::string knee_rotor_name = withLeftRightSigns("knee_rotor", legID);

            SpatialInertia<double> knee_link_inertia(_kneeMass, _kneeCOM, _kneeRotInertia);
            knee_link_inertia = withLeftRightSigns(knee_link_inertia, legID);

            SpatialInertia<double> knee_rotor_inertia(0., _largeRotorCOM, _largeRotorRotInertiaY);
            knee_rotor_inertia = withLeftRightSigns(knee_rotor_inertia, legID);

            const Vec3<double> kneeLocation = withLeftRightSigns(_kneeLocation, legID);
            const Vec3<double> kneeRotorLocation = withLeftRightSigns(_kneeRotorLocation, legID);
            const spatial::SpatialTransform xtreeKnee(I3, kneeLocation);
            const spatial::SpatialTransform xtreeKneeRotor(I3, kneeRotorLocation);

            Body knee_link = model.registerBody(knee_link_name, knee_link_inertia,
                                                knee_parent_name, xtreeKnee);
            Body knee_rotor = model.registerBody(knee_rotor_name, knee_rotor_inertia,
                                                 knee_parent_name, xtreeKneeRotor);

            // Ankle
            const std::string ankle_parent_name = knee_link_name;
            const std::string ankle_link_name = withLeftRightSigns("ankle_link", legID);
            const std::string ankle_rotor_name = withLeftRightSigns("ankle_rotor", legID);

            SpatialInertia<double> ankle_link_inertia(_ankleMass, _ankleCOM, _ankleRotInertia);
            ankle_link_inertia = withLeftRightSigns(ankle_link_inertia, legID);

            SpatialInertia<double> ankle_rotor_inertia(0., _smallRotorCOM, _smallRotorRotInertiaY);
            ankle_rotor_inertia = withLeftRightSigns(ankle_rotor_inertia, legID);

            const Vec3<double> ankleLocation = withLeftRightSigns(_ankleLocation, legID);
            const Vec3<double> ankleRotorLocation = withLeftRightSigns(_ankleRotorLocation, legID);
            const spatial::SpatialTransform xtreeAnkle(I3, ankleLocation);
            const spatial::SpatialTransform xtreeAnkleRotor(I3, ankleRotorLocation);

            Body ankle_rotor = model.registerBody(ankle_rotor_name, ankle_rotor_inertia,
                                                  knee_parent_name, xtreeAnkleRotor);
            Body ankle_link = model.registerBody(ankle_link_name, ankle_link_inertia,
                                                 ankle_parent_name, xtreeAnkle);

            // Cluster
            const std::string knee_and_ankle_name = withLeftRightSigns("knee_and_ankle", legID);
            auto knee_and_ankle_generalized_joint = std::make_shared<GeneralizedJoints::RevolutePairWithRotor>(
                knee_link, ankle_link, knee_rotor, ankle_rotor,
                CoordinateAxis::Y, CoordinateAxis::Y, CoordinateAxis::Y, CoordinateAxis::Y,
                _kneeGearRatio / _kneeBeltRatio,
                _ankleGearRatio / _ankleBeltRatio,
                _kneeBeltRatio, _ankleBeltRatio);
            model.appendRegisteredBodiesAsCluster(knee_and_ankle_name,
                                                  knee_and_ankle_generalized_joint);

            // Contact Points
            const std::string toe_contact_name = withLeftRightSigns("toe_contact", legID);
            const std::string heel_contact_name = withLeftRightSigns("heel_contact", legID);
            model.appendContactPoint(ankle_link_name,
                                     Vec3<double>(_footToeLength, 0, -_footHeight),
                                     toe_contact_name);
            model.appendContactPoint(ankle_link_name,
                                     Vec3<double>(-_footHeelLength, 0, -_footHeight),
                                     heel_contact_name);
        }

        // TODO(@MatthewChignoli): Add the arms
        for (int armID = 0; armID < 2; armID++)
        {
            // ShoulderRy
            const std::string shoulder_ry_parent_name = torso_name;
            const std::string shoulder_ry_name = withLeftRightSigns("shoulder_ry", armID);
            const std::string shoulder_ry_link_name = withLeftRightSigns("shoulder_ry_link", armID);
            const std::string shoulder_ry_rotor_name = withLeftRightSigns("shoulder_ry_rotor", armID);

            SpatialInertia<double> shoulder_ry_link_inertia(_shoulderRyMass, _shoulderRyCOM,
                                                            _shoulderRyRotInertia);
            shoulder_ry_link_inertia = withLeftRightSigns(shoulder_ry_link_inertia, armID);

            SpatialInertia<double> shoulder_ry_rotor_inertia(0., _smallRotorCOM,
                                                             _smallRotorRotInertiaY);
            shoulder_ry_rotor_inertia = withLeftRightSigns(shoulder_ry_rotor_inertia, armID);

            const spatial::SpatialTransform xtreeShoulderRy(I3, withLeftRightSigns(_shoulderRyLocation, armID));
            const spatial::SpatialTransform xtreeShoulderRyRotor(I3, withLeftRightSigns(_shoulderRyRotorLocation, armID));

            Body shoulder_ry_link = model.registerBody(shoulder_ry_link_name,
                                                       shoulder_ry_link_inertia,
                                                       shoulder_ry_parent_name,
                                                       xtreeShoulderRy);
            Body shoulder_ry_rotor = model.registerBody(shoulder_ry_rotor_name,
                                                        shoulder_ry_rotor_inertia,
                                                        shoulder_ry_parent_name,
                                                        xtreeShoulderRyRotor);

            auto shoulder_ry_generalized_joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(shoulder_ry_link, shoulder_ry_rotor, CoordinateAxis::Y, CoordinateAxis::Y, _shoulderRyGearRatio);
            model.appendRegisteredBodiesAsCluster(shoulder_ry_name, shoulder_ry_generalized_joint);

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

            const spatial::SpatialTransform xtreeShoulderRx(I3, withLeftRightSigns(_shoulderRxLocation, armID));
            const spatial::SpatialTransform xtreeShoulderRxRotor(I3, withLeftRightSigns(_shoulderRxRotorLocation, armID));

            Body shoulder_rx_link = model.registerBody(shoulder_rx_link_name,
                                                       shoulder_rx_link_inertia,
                                                       shoulder_rx_parent_name,
                                                       xtreeShoulderRx);
            Body shoulder_rx_rotor = model.registerBody(shoulder_rx_rotor_name,
                                                        shoulder_rx_rotor_inertia,
                                                        shoulder_rx_parent_name,
                                                        xtreeShoulderRxRotor);

            auto shoulder_rx_generalized_joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(shoulder_rx_link, shoulder_rx_rotor, CoordinateAxis::X, CoordinateAxis::X, _shoulderRxGearRatio);
            model.appendRegisteredBodiesAsCluster(shoulder_rx_name, shoulder_rx_generalized_joint);

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

            const spatial::SpatialTransform xtreeShoulderRz(I3, withLeftRightSigns(_shoulderRzLocation, armID));
            const spatial::SpatialTransform xtreeShoulderRzRotor(I3, withLeftRightSigns(_shoulderRzRotorLocation, armID));

            Body shoulder_rz_link = model.registerBody(shoulder_rz_link_name,
                                                       shoulder_rz_link_inertia,
                                                       shoulder_rz_parent_name,
                                                       xtreeShoulderRz);
            Body shoulder_rz_rotor = model.registerBody(shoulder_rz_rotor_name,
                                                        shoulder_rz_rotor_inertia,
                                                        shoulder_rz_parent_name,
                                                        xtreeShoulderRzRotor);

            auto shoulder_rz_generalized_joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(shoulder_rz_link, shoulder_rz_rotor, CoordinateAxis::Z, CoordinateAxis::Z, _shoulderRzGearRatio);
            model.appendRegisteredBodiesAsCluster(shoulder_rz_name, shoulder_rz_generalized_joint);

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

            const spatial::SpatialTransform xtreeElbow(I3, withLeftRightSigns(_elbowLocation, armID));
            const spatial::SpatialTransform xtreeElbowRotor(I3, withLeftRightSigns(_elbowRotorLocation, armID));

            Body elbow_link = model.registerBody(elbow_link_name, elbow_link_inertia,
                                                 elbow_parent_name, xtreeElbow);
            Body elbow_rotor = model.registerBody(elbow_rotor_name, elbow_rotor_inertia,
                                                  elbow_parent_name, xtreeElbowRotor);

            auto elbow_generalized_joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(elbow_link, elbow_rotor, CoordinateAxis::Y, CoordinateAxis::Y, _elbowGearRatio);
            model.appendRegisteredBodiesAsCluster(elbow_name, elbow_generalized_joint);

            const std::string elbow_contact_name = withLeftRightSigns("elbow_contact", armID);
            const std::string hand_contact_name = withLeftRightSigns("hand_contact", armID);
            model.appendContactPoint(elbow_link_name, Vec3<double>(0, 0, 0), elbow_contact_name);
            model.appendContactPoint(elbow_link_name, Vec3<double>(0, 0, -_lowerArmLength),
                                     hand_contact_name);
        }

        return model;
    }
}
