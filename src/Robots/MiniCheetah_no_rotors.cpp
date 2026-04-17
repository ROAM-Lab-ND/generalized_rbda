#include "grbda/Robots/MiniCheetah_no_rotors.hpp"

namespace grbda
{

    template <typename Scalar, typename OrientationRepresentation>
    MiniCheetah_no_rotors<Scalar, OrientationRepresentation>::MiniCheetah_no_rotors()
    {
        Mat3<Scalar> RY = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y, M_PI / 2);

        _bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
        _bodyRotationalInertia = _bodyRotationalInertia * 1e-6;

        _abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
        _abadRotationalInertia = _abadRotationalInertia * 1e-6;

        _hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
        _hipRotationalInertia = _hipRotationalInertia * 1e-6;

        Mat3<Scalar> kneeRotationalInertiaRotated;
        kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
        kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
        _kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    }

    template <typename Scalar, typename OrientationRepresentation>
    ClusterTreeModel<Scalar>
    MiniCheetah_no_rotors<Scalar, OrientationRepresentation>::buildClusterTreeModel() const
    {
        typedef spatial::Transform<Scalar> Xform;
        typedef ClusterJoints::Revolute<Scalar> Revolute;
        typedef ClusterJoints::Free<Scalar, OrientationRepresentation> Free;

        ClusterTreeModel<Scalar> model;
        const Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

        const std::string torso_name = "Floating Base";
        const std::string torso_parent_name = "ground";
        const SpatialInertia<Scalar> torsoInertia(_bodyMass, _bodyCOM, _bodyRotationalInertia);
        model.template appendBody<Free>(torso_name, torsoInertia, torso_parent_name, Xform{});

        Vec3<Scalar> torsoDims(_bodyLength, _bodyWidth, _bodyHeight);
        model.appendContactBox(torso_name, torsoDims);

        int sideSign = -1;
        for (int legID : {2, 3, 0, 1})
        {
            const std::string abad_parent_name = torso_name;
            const std::string abad_name = withLegSigns("abad", legID);
            const std::string abad_link_name = withLegSigns("abad_link", legID);

            SpatialInertia<Scalar> abad_link_inertia(_abadMass, _abadCOM, _abadRotationalInertia);
            abad_link_inertia = withLeftRightSigns(abad_link_inertia, sideSign);

            const Xform xtree_abad(I3, withLegSigns(_abadLocation, legID));
            model.template appendBody<Revolute>(abad_link_name, abad_link_inertia,
                                                abad_parent_name, xtree_abad,
                                                ori::CoordinateAxis::X);

            const std::string hip_parent_name = abad_link_name;
            const std::string hip_name = withLegSigns("hip", legID);
            const std::string hip_link_name = withLegSigns("hip_link", legID);

            SpatialInertia<Scalar> hip_link_inertia(_hipMass, _hipCOM, _hipRotationalInertia);
            hip_link_inertia = withLeftRightSigns(hip_link_inertia, sideSign);

            Mat3<Scalar> RZ = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Z, Scalar(M_PI));
            const Xform xtree_hip(RZ, withLegSigns(_hipLocation, legID));
            model.template appendBody<Revolute>(hip_link_name, hip_link_inertia,
                                                hip_parent_name, xtree_hip,
                                                ori::CoordinateAxis::Y);

            const std::string knee_contact_name = withLegSigns("knee_contact", legID);
            model.appendContactPoint(hip_link_name, Vec3<Scalar>(0, 0, -_hipLinkLength), knee_contact_name);

            const std::string knee_parent_name = hip_link_name;
            const std::string knee_name = withLegSigns("knee", legID);
            const std::string knee_link_name = withLegSigns("knee_link", legID);

            SpatialInertia<Scalar> knee_link_inertia(_kneeMass, _kneeCOM, _kneeRotationalInertia);
            knee_link_inertia = withLeftRightSigns(knee_link_inertia, sideSign);

            const Xform xtree_knee(I3, withLegSigns(_kneeLocation, legID));
            model.template appendBody<Revolute>(knee_link_name, knee_link_inertia,
                                                knee_parent_name, xtree_knee,
                                                ori::CoordinateAxis::Y);

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

    template class MiniCheetah_no_rotors<double, ori_representation::RollPitchYaw>;
    template class MiniCheetah_no_rotors<double, ori_representation::Quaternion>;
    template class MiniCheetah_no_rotors<casadi::SX, ori_representation::RollPitchYaw>;
    template class MiniCheetah_no_rotors<casadi::SX, ori_representation::Quaternion>;

} // namespace grbda