#include "grbda/Robots/MIT_Humanoid_no_rotors.hpp"

namespace grbda
{

    template <typename Scalar, typename OrientationRepresentation>
    ClusterTreeModel<Scalar>
    MIT_Humanoid_no_rotors<Scalar, OrientationRepresentation>::buildClusterTreeModel() const
    {
        typedef spatial::Transform<Scalar> Xform;
        typedef ClusterJoints::Revolute<Scalar> Revolute;
        typedef ClusterJoints::RevolutePair<Scalar> RevolutePair;
        typedef ori::CoordinateAxis Axis;

        ClusterTreeModel<Scalar> model;

        const Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

        // Torso
        const std::string torso_name = "Floating Base";
        const std::string torso_parent_name = "ground";
        const SpatialInertia<Scalar> torsoInertia(this->_torsoMass, this->_torsoCOM,
                                                  this->_torsoRotInertia);
        model.template appendBody<ClusterJoints::Free<Scalar, OrientationRepresentation>>(torso_name, torsoInertia,
                                                               torso_parent_name, Xform{});

        Vec3<Scalar> torsoDims(this->_torsoLength, this->_torsoWidth, this->_torsoHeight);
        model.appendContactBox(torso_name, torsoDims);

        for (int legID = 0; legID < 2; legID++)
        {

            // HipRz
            const std::string hip_rz_parent_name = torso_name;
            const std::string hip_rz_name = this->withLeftRightSigns("hip_rz_link", legID);

            SpatialInertia<Scalar> hip_rz_link_inertia(this->_hipRzMass, this->_hipRzCOM,
                                                       this->_hipRzRotInertia);
            hip_rz_link_inertia = this->withLeftRightSigns(hip_rz_link_inertia, legID);

            Mat3<Scalar> Xrot_HipZ = ori::coordinateRotation<Scalar>(Axis::Y, this->_hipRzPitch);
            const Vec3<Scalar> hipRzLocation = this->withLeftRightSigns(this->_hipRzLocation, legID);
            const Xform xtreeHipRz(Xrot_HipZ, hipRzLocation);

            model.template appendBody<Revolute>(hip_rz_name, hip_rz_link_inertia,
                                                hip_rz_parent_name, xtreeHipRz, Axis::Z);

            // HipRx
            const std::string hip_rx_parent_name = hip_rz_name;
            const std::string hip_rx_name = this->withLeftRightSigns("hip_rx_link", legID);

            SpatialInertia<Scalar> hip_rx_link_inertia(this->_hipRxMass, this->_hipRxCOM,
                                                       this->_hipRxRotInertia);
            hip_rx_link_inertia = this->withLeftRightSigns(hip_rx_link_inertia, legID);

            Mat3<Scalar> Xrot_HipX = ori::coordinateRotation<Scalar>(Axis::Y, this->_hipRxPitch);
            const Vec3<Scalar> hipRxLocation = this->withLeftRightSigns(this->_hipRxLocation, legID);
            const Xform xtreeHipRx(Xrot_HipX, hipRxLocation);

            model.template appendBody<Revolute>(hip_rx_name, hip_rx_link_inertia,
                                                hip_rx_parent_name, xtreeHipRx, Axis::X);

            // HipRy
            const std::string hip_ry_parent_name = hip_rx_name;
            const std::string hip_ry_name = this->withLeftRightSigns("hip_ry_link", legID);

            SpatialInertia<Scalar> hip_ry_link_inertia(this->_hipRyMass, this->_hipRyCOM,
                                                       this->_hipRyRotInertia);
            hip_ry_link_inertia = this->withLeftRightSigns(hip_ry_link_inertia, legID);

            Mat3<Scalar> Xrot_HipY = ori::coordinateRotation<Scalar>(Axis::Y, this->_hipRyPitch);
            const Vec3<Scalar> hipRyLocation = this->withLeftRightSigns(this->_hipRyLocation, legID);
            const Xform xtreeHipRy(Xrot_HipY, hipRyLocation);

            model.template appendBody<Revolute>(hip_ry_name, hip_ry_link_inertia,
                                                hip_ry_parent_name, xtreeHipRy, Axis::Y);

            const std::string knee_contact_name = this->withLeftRightSigns("knee_contact", legID);
            model.appendContactPoint(hip_ry_name, Vec3<Scalar>(0, 0, -this->_thighLength),
                                     knee_contact_name);

            // Knee
            const std::string knee_parent_name = hip_ry_name;
            const std::string knee_name = this->withLeftRightSigns("knee_link", legID);

            SpatialInertia<Scalar> knee_link_inertia(this->_kneeMass, this->_kneeCOM,
                                                     this->_kneeRotInertia);
            knee_link_inertia = this->withLeftRightSigns(knee_link_inertia, legID);

            const Vec3<Scalar> kneeLocation = this->withLeftRightSigns(this->_kneeLocation, legID);
            const Xform xtreeKnee(I3, kneeLocation);

            Body<Scalar> knee = model.registerBody(knee_name, knee_link_inertia,
                                                   knee_parent_name, xtreeKnee);

            // Ankle
            const std::string ankle_parent_name = knee_name;
            const std::string ankle_name = this->withLeftRightSigns("ankle_link", legID);

            SpatialInertia<Scalar> ankle_link_inertia(this->_ankleMass, this->_ankleCOM,
                                                      this->_ankleRotInertia);
            ankle_link_inertia = this->withLeftRightSigns(ankle_link_inertia, legID);

            const Vec3<Scalar> ankleLocation = this->withLeftRightSigns(this->_ankleLocation, legID);
            const Xform xtreeAnkle(I3, ankleLocation);

            Body<Scalar> ankle = model.registerBody(ankle_name, ankle_link_inertia,
                                                    ankle_parent_name, xtreeAnkle);

            // Knee/Ankle Cluster
            const std::string knee_ankle_cluster_name = this->withLeftRightSigns("knee_ankle_cluster", legID);
            model.template appendRegisteredBodiesAsCluster<RevolutePair>(
                knee_ankle_cluster_name, knee, ankle, Axis::Y, Axis::Y);

            // Contact Points
            const std::string toe_contact_name = this->withLeftRightSigns("toe_contact", legID);
            const std::string heel_contact_name = this->withLeftRightSigns("heel_contact", legID);
            if (legID == 0)
                model.appendEndEffector(ankle_name,
                                        Vec3<Scalar>(this->_footToeLength, 0, -this->_footHeight),
                                        toe_contact_name);
            else
                model.appendContactPoint(ankle_name,
                                         Vec3<Scalar>(this->_footToeLength, 0, -this->_footHeight),
                                         toe_contact_name);
            model.appendContactPoint(ankle_name,
                                     Vec3<Scalar>(-this->_footHeelLength, 0, -this->_footHeight),
                                     heel_contact_name);
        }

        for (int armID = 0; armID < 2; armID++)
        {
            // ShoulderRy
            const std::string shoulder_ry_parent_name = torso_name;
            const std::string shoulder_ry_name = this->withLeftRightSigns("shoulder_ry_link", armID);

            SpatialInertia<Scalar> shoulder_ry_link_inertia(
                this->_shoulderRyMass, this->_shoulderRyCOM, this->_shoulderRyRotInertia);
            shoulder_ry_link_inertia = this->withLeftRightSigns(shoulder_ry_link_inertia, armID);

            const Xform xtreeShoulderRy(I3, this->withLeftRightSigns(this->_shoulderRyLocation, armID));

            model.template appendBody<Revolute>(shoulder_ry_name, shoulder_ry_link_inertia,
                                                shoulder_ry_parent_name, xtreeShoulderRy, Axis::Y);

            // ShoulderRx
            const std::string shoulder_rx_parent_name = shoulder_ry_name;
            const std::string shoulder_rx_name = this->withLeftRightSigns("shoulder_rx_link", armID);

            SpatialInertia<Scalar> shoulder_rx_link_inertia(
                this->_shoulderRxMass, this->_shoulderRxCOM, this->_shoulderRxRotInertia);
            shoulder_rx_link_inertia = this->withLeftRightSigns(shoulder_rx_link_inertia, armID);

            Xform xtreeShoulderRx(I3, this->withLeftRightSigns(this->_shoulderRxLocation, armID));

            model.template appendBody<Revolute>(shoulder_rx_name, shoulder_rx_link_inertia,
                                                shoulder_rx_parent_name, xtreeShoulderRx, Axis::X);

            // ShoulderRz
            const std::string shoulder_rz_parent_name = shoulder_rx_name;
            const std::string shoulder_rz_name = this->withLeftRightSigns("shoulder_rz_link", armID);

            SpatialInertia<Scalar> shoulder_rz_link_inertia(
                this->_shoulderRzMass, this->_shoulderRzCOM, this->_shoulderRzRotInertia);
            shoulder_rz_link_inertia = this->withLeftRightSigns(shoulder_rz_link_inertia, armID);

            const Xform xtreeShoulderRz(I3, this->withLeftRightSigns(this->_shoulderRzLocation, armID));

            model.template appendBody<Revolute>(shoulder_rz_name, shoulder_rz_link_inertia,
                                                shoulder_rz_parent_name, xtreeShoulderRz, Axis::Z);

            // Elbow
            const std::string elbow_parent_name = shoulder_rz_name;
            const std::string elbow_name = this->withLeftRightSigns("elbow_link", armID);

            SpatialInertia<Scalar> elbow_link_inertia(this->_elbowMass, this->_elbowCOM,
                                                      this->_elbowRotInertia);
            elbow_link_inertia = this->withLeftRightSigns(elbow_link_inertia, armID);

            const Xform xtreeElbow(I3, this->withLeftRightSigns(this->_elbowLocation, armID));

            model.template appendBody<Revolute>(elbow_name, elbow_link_inertia,
                                                elbow_parent_name, xtreeElbow, Axis::Y);

            const std::string elbow_contact_name = this->withLeftRightSigns("elbow_contact", armID);
            const std::string hand_contact_name = this->withLeftRightSigns("hand_contact", armID);
            model.appendContactPoint(elbow_name, Vec3<Scalar>(0, 0, 0), elbow_contact_name);
            model.appendContactPoint(elbow_name, Vec3<Scalar>(0, 0, -this->_lowerArmLength),
                                     hand_contact_name);
        }

        return model;
    }

    template class MIT_Humanoid_no_rotors<double, ori_representation::RollPitchYaw>;
    template class MIT_Humanoid_no_rotors<double, ori_representation::Quaternion>;
    template class MIT_Humanoid_no_rotors<casadi::SX, ori_representation::RollPitchYaw>;
    template class MIT_Humanoid_no_rotors<casadi::SX, ori_representation::Quaternion>;
}
