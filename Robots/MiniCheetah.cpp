#include "MiniCheetah.hpp"

namespace grbda
{

    ClusterTreeModel MiniCheetah::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;

        ClusterTreeModel model;

        const Mat3<double> I3 = Mat3<double>::Identity();

        // Torso
        const std::string torso_name = "Floating Base";
        const std::string torso_parent_name = "ground";
        const SpatialInertia<double> torsoInertia(_bodyMass, _bodyCOM, _bodyRotationalInertia);
        model.appendBody<Free>(torso_name, torsoInertia, torso_parent_name,
                               spatial::Transform{});

        Vec3<double> torsoDims(_bodyLength, _bodyWidth, _bodyHeight);
        model.appendContactBox(torso_name, torsoDims);

        int sideSign = -1;
        for (int legID = 0; legID < 4; legID++)
        {
            // Ab/Ad joint
            const std::string abad_parent_name = torso_name;
            const std::string abad_name = withLegSigns("abad", legID);
            const std::string abad_link_name = withLegSigns("abad_link", legID);
            const std::string abad_rotor_name = withLegSigns("abad_rotor", legID);

            SpatialInertia<double> abad_link_inertia(_abadMass, _abadCOM, _abadRotationalInertia);
            abad_link_inertia = withLeftRightSigns(abad_link_inertia, sideSign);

            SpatialInertia<double> abad_rotor_inertia(0., _rotorCOM, _rotorRotationalInertiaX);
            abad_rotor_inertia = withLeftRightSigns(abad_rotor_inertia, sideSign);

            const spatial::Transform xtree_abad(I3, withLegSigns(_abadLocation, legID));
            const spatial::Transform xtree_abad_rotor(I3, withLegSigns(_abadRotorLocation, legID));

            Body abad_link = model.registerBody(abad_link_name, abad_link_inertia,
                                                abad_parent_name, xtree_abad);
            Body abad_rotor = model.registerBody(abad_rotor_name, abad_rotor_inertia,
                                                 abad_parent_name, xtree_abad_rotor);
            GearedTransmissionModule abad_module{abad_link, abad_rotor, ori::CoordinateAxis::X,
                                                 ori::CoordinateAxis::X, _abadGearRatio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(abad_name, abad_module);

            // Hip Joint
            const std::string hip_parent_name = abad_link_name;
            const std::string hip_name = withLegSigns("hip", legID);
            const std::string hip_link_name = withLegSigns("hip_link", legID);
            const std::string hip_rotor_name = withLegSigns("hip_rotor", legID);

            SpatialInertia<double> hip_link_inertia(_hipMass, _hipCOM, _hipRotationalInertia);
            hip_link_inertia = withLeftRightSigns(hip_link_inertia, sideSign);

            SpatialInertia<double> hip_rotor_inertia(0., _rotorCOM, _rotorRotationalInertiaY);
            hip_rotor_inertia = withLeftRightSigns(hip_rotor_inertia, sideSign);

            const spatial::Transform xtree_hip(I3, withLegSigns(_hipLocation, legID));
            const spatial::Transform xtree_hip_rotor(I3, withLegSigns(_hipRotorLocation, legID));

            Body hip_link = model.registerBody(hip_link_name, hip_link_inertia,
                                               hip_parent_name, xtree_hip);
            Body hip_rotor = model.registerBody(hip_rotor_name, hip_rotor_inertia,
                                                hip_parent_name, xtree_hip_rotor);
            GearedTransmissionModule hip_module{hip_link, hip_rotor, ori::CoordinateAxis::Y,
                                                ori::CoordinateAxis::Y, _hipGearRatio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_name, hip_module);

            const std::string hip_contact_name = withLegSigns("hip_contact", legID);
            model.appendContactPoint(hip_link_name, Vec3<double>(0, 0, 0), hip_contact_name);

            const std::string knee_contact_name = withLegSigns("knee_contact", legID);
            model.appendContactPoint(hip_link_name, Vec3<double>(0, 0, -_hipLinkLength), knee_contact_name);

            // Knee Joint
            const std::string knee_parent_name = hip_link_name;
            const std::string knee_name = withLegSigns("knee", legID);
            const std::string knee_link_name = withLegSigns("knee_link", legID);
            const std::string knee_rotor_name = withLegSigns("knee_rotor", legID);

            SpatialInertia<double> knee_link_inertia(_kneeMass, _kneeCOM, _kneeRotationalInertiaRotated);
            knee_link_inertia = withLeftRightSigns(knee_link_inertia, sideSign);

            SpatialInertia<double> knee_rotor_inertia(0., _rotorCOM, _rotorRotationalInertiaZ);
            knee_rotor_inertia = withLeftRightSigns(knee_rotor_inertia, sideSign);

            const spatial::Transform xtree_knee(I3, withLegSigns(_kneeLocation, legID));
            const spatial::Transform xtree_knee_rotor(I3, withLegSigns(_kneeRotorLocation, legID));

            Body knee_link = model.registerBody(knee_link_name, knee_link_inertia,
                                                knee_parent_name, xtree_knee);
            Body knee_rotor = model.registerBody(knee_rotor_name, knee_rotor_inertia,
                                                 knee_parent_name, xtree_knee_rotor);
            GearedTransmissionModule knee_module{knee_link, knee_rotor, ori::CoordinateAxis::Z,
                                                 ori::CoordinateAxis::Z, _kneeGearRatio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(knee_name, knee_module);

            const std::string foot_contact_name = withLegSigns("foot_contact", legID);
            const Vec3<double> foot_contact_offset = withLegSigns(Vec3<double>(0, -_kneeLinkY_offset, -_kneeLinkLength), legID);

            if (legID == 0)
                model.appendEndEffector(knee_link_name, foot_contact_offset, foot_contact_name);
            else
                model.appendContactPoint(knee_link_name, foot_contact_offset, foot_contact_name);

            sideSign *= -1;
        }

        return model;
    }
}
