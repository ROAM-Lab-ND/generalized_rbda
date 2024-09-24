#include "grbda/Robots/MiniCheetah.hpp"

namespace grbda
{

    template <typename Scalar, typename OrientationRepresentation>
    ClusterTreeModel<Scalar>
    MiniCheetah<Scalar, OrientationRepresentation>::buildClusterTreeModel() const
    {
        typedef spatial::Transform<Scalar> Xform;
        typedef ClusterJoints::GearedTransmissionModule<Scalar> TransmissionModule;
        typedef ClusterJoints::RevoluteWithRotor<Scalar> RevoluteWithRotor;
        typedef ClusterJoints::Free<Scalar, OrientationRepresentation> Free;

        ClusterTreeModel<Scalar> model;

        const Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

        // Torso
        const std::string torso_name = "Floating Base";
        const std::string torso_parent_name = "ground";
        const SpatialInertia<Scalar> torsoInertia(_bodyMass, _bodyCOM, _bodyRotationalInertia);
        model.template appendBody<Free>(torso_name, torsoInertia, torso_parent_name, Xform{});

        Vec3<Scalar> torsoDims(_bodyLength, _bodyWidth, _bodyHeight);
        model.appendContactBox(torso_name, torsoDims);

        int sideSign = -1;
        for (int legID = 0; legID < 4; legID++)
        {
            // Ab/Ad joint
            const std::string abad_parent_name = torso_name;
            const std::string abad_name = withLegSigns("abad", legID);
            const std::string abad_link_name = withLegSigns("abad_link", legID);
            const std::string abad_rotor_name = withLegSigns("abad_rotor", legID);
            const std::string abad_link_joint_name = withLegSigns("abad_link_joint", legID);
            const std::string abad_rotor_joint_name = withLegSigns("abad_rotor_joint", legID);

            SpatialInertia<Scalar> abad_link_inertia(_abadMass, _abadCOM, _abadRotationalInertia);
            abad_link_inertia = withLeftRightSigns(abad_link_inertia, sideSign);

            SpatialInertia<Scalar> abad_rotor_inertia(_rotorMass, _rotorCOM,
                                                      _rotorRotationalInertiaX);
            abad_rotor_inertia = withLeftRightSigns(abad_rotor_inertia, sideSign);

            const Xform xtree_abad(I3, withLegSigns(_abadLocation, legID));
            const Xform xtree_abad_rotor(I3, withLegSigns(_abadRotorLocation, legID));

            Body<Scalar> abad_link = model.registerBody(abad_link_name, abad_link_inertia,
                                                        abad_parent_name, xtree_abad);
            Body<Scalar> abad_rotor = model.registerBody(abad_rotor_name, abad_rotor_inertia,
                                                         abad_parent_name, xtree_abad_rotor);
            TransmissionModule abad_module{abad_link, abad_rotor,
                                           abad_link_joint_name, abad_rotor_joint_name,
                                           ori::CoordinateAxis::X, ori::CoordinateAxis::X,
                                           _abadGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(abad_name,
                                                                              abad_module);

            // Hip Joint
            const std::string hip_parent_name = abad_link_name;
            const std::string hip_name = withLegSigns("hip", legID);
            const std::string hip_link_name = withLegSigns("hip_link", legID);
            const std::string hip_rotor_name = withLegSigns("hip_rotor", legID);
            const std::string hip_link_joint_name = withLegSigns("hip_link_joint", legID);
            const std::string hip_rotor_joint_name = withLegSigns("hip_rotor_joint", legID);

            SpatialInertia<Scalar> hip_link_inertia(_hipMass, _hipCOM, _hipRotationalInertia);
            hip_link_inertia = withLeftRightSigns(hip_link_inertia, sideSign);

            SpatialInertia<Scalar> hip_rotor_inertia(_rotorMass, _rotorCOM,
                                                     _rotorRotationalInertiaY);
            hip_rotor_inertia = withLeftRightSigns(hip_rotor_inertia, sideSign);

            Mat3<Scalar> RZ = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Z, Scalar(M_PI));
            const Xform xtree_hip(RZ, withLegSigns(_hipLocation, legID));
            const Xform xtree_hip_rotor(RZ, withLegSigns(_hipRotorLocation, legID));

            Body<Scalar> hip_link = model.registerBody(hip_link_name, hip_link_inertia,
                                                       hip_parent_name, xtree_hip);
            Body<Scalar> hip_rotor = model.registerBody(hip_rotor_name, hip_rotor_inertia,
                                                        hip_parent_name, xtree_hip_rotor);
            TransmissionModule hip_module{hip_link, hip_rotor,
                                          hip_link_joint_name, hip_rotor_joint_name,
                                          ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                          _hipGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_name, hip_module);

            const std::string hip_contact_name = withLegSigns("hip_contact", legID);
            model.appendContactPoint(hip_link_name, Vec3<Scalar>(0, 0, 0), hip_contact_name);

            const std::string knee_contact_name = withLegSigns("knee_contact", legID);
            model.appendContactPoint(hip_link_name, Vec3<Scalar>(0, 0, -_hipLinkLength),
                                     knee_contact_name);

            // Knee Joint
            const std::string knee_parent_name = hip_link_name;
            const std::string knee_name = withLegSigns("knee", legID);
            const std::string knee_link_name = withLegSigns("knee_link", legID);
            const std::string knee_rotor_name = withLegSigns("knee_rotor", legID);
            const std::string knee_link_joint_name = withLegSigns("knee_link_joint", legID);
            const std::string knee_rotor_joint_name = withLegSigns("knee_rotor_joint", legID);

            SpatialInertia<Scalar> knee_link_inertia(_kneeMass, _kneeCOM,
                                                     _kneeRotationalInertiaRotated);
            knee_link_inertia = withLeftRightSigns(knee_link_inertia, sideSign);

            SpatialInertia<Scalar> knee_rotor_inertia(_rotorMass, _rotorCOM,
                                                      _rotorRotationalInertiaY);
            knee_rotor_inertia = withLeftRightSigns(knee_rotor_inertia, sideSign);

            const Xform xtree_knee(I3, withLegSigns(_kneeLocation, legID));
            const Xform xtree_knee_rotor(I3, withLegSigns(_kneeRotorLocation, legID));

            Body<Scalar> knee_link = model.registerBody(knee_link_name, knee_link_inertia,
                                                        knee_parent_name, xtree_knee);
            Body<Scalar> knee_rotor = model.registerBody(knee_rotor_name, knee_rotor_inertia,
                                                         knee_parent_name, xtree_knee_rotor);
            TransmissionModule knee_module{knee_link, knee_rotor,
                                           knee_link_joint_name, knee_rotor_joint_name,
                                           ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                           _kneeGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(knee_name,
                                                                              knee_module);

            const std::string foot_contact_name = withLegSigns("foot_contact", legID);
            const Vec3<Scalar> foot_contact_offset =
                withLegSigns(Vec3<Scalar>(0, -_kneeLinkY_offset, -_kneeLinkLength), legID);

            if (legID == 0)
                model.appendEndEffector(knee_link_name, foot_contact_offset, foot_contact_name);
            else
                model.appendContactPoint(knee_link_name, foot_contact_offset, foot_contact_name);

            sideSign *= -1;
        }

        return model;
    }

    template class MiniCheetah<double, ori_representation::RollPitchYaw>;
    template class MiniCheetah<double, ori_representation::Quaternion>;
    template class MiniCheetah<casadi::SX, ori_representation::RollPitchYaw>;
    template class MiniCheetah<casadi::SX, ori_representation::Quaternion>;
}
