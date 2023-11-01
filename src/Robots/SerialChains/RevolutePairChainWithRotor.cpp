#include "grbda/Robots/SerialChains/RevolutePairChainWithRotor.hpp"

namespace grbda
{

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar>
    RevolutePairChainWithRotor<N, Scalar>::buildRandomClusterTreeModel() const
    {
        ClusterTreeModel<Scalar> model{};

        std::string parent_name = "ground";
        for (size_t i(0); i < N / 2; i++)
        {
            // Link A
            const std::string linkA_name = "link-A-" + std::to_string(i);
            const auto linkA_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkA_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkA_joint_axis = ori::randomCoordinateAxis();
            auto linkA = model.registerBody(linkA_name, linkA_inertia, parent_name, linkA_Xtree);

            // Rotor A
            const std::string rotorA_name = "rotor-A-" + std::to_string(i);
            const auto rotorA_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto rotorA_inertia = this->randomRotorSpatialInertia();
            ori::CoordinateAxis rotorA_joint_axis = ori::randomCoordinateAxis();
            auto rotorA = model.registerBody(rotorA_name, rotorA_inertia,
                                             parent_name, rotorA_Xtree);

            // Rotor B
            const std::string rotorB_name = "rotor-B-" + std::to_string(i);
            const auto rotorB_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto rotorB_inertia = this->randomRotorSpatialInertia();
            ori::CoordinateAxis rotorB_joint_axis = ori::randomCoordinateAxis();
            auto rotorB = model.registerBody(rotorB_name, rotorB_inertia,
                                             parent_name, rotorB_Xtree);

            // Link B
            const std::string linkB_name = "link-B-" + std::to_string(i);
            const auto linkB_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkB_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkB_joint_axis = ori::randomCoordinateAxis();
            auto linkB = model.registerBody(linkB_name, linkB_inertia, linkA_name, linkB_Xtree);

            // Cluster
            const std::string name = "cluster-" + std::to_string(i);
            ProxTransModule moduleA{linkA, rotorA, linkA_joint_axis, rotorA_joint_axis,
                                    this->randomGearRatio(), this->template randomBeltRatios<1>()};
            DistTransModule moduleB{linkB, rotorB, linkB_joint_axis, rotorB_joint_axis,
                                    this->randomGearRatio(), this->template randomBeltRatios<2>()};
            model.template appendRegisteredBodiesAsCluster<RevPairRotor>(name, moduleA, moduleB);

            // Contact points
            appendContactPoints(model, i, linkA_name, linkB_name);

            parent_name = linkB_name;
        }

        return model;
    }

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar>
    RevolutePairChainWithRotor<N, Scalar>::buildUniformClusterTreeModel() const
    {
        ClusterTreeModel<Scalar> model{};

        Mat3<Scalar> I3 = Mat3<Scalar>::Identity();
        Vec3<Scalar> z3 = Vec3<Scalar>::Zero();

        const Scalar grav = 9.81;
        model.setGravity(Vec3<Scalar>{grav, 0., 0.});

        // Inertia params
        const Scalar I = 1.;
        const Scalar Irot = 1e-4;
        const Scalar m = 1.;
        const Scalar l = 1.;
        const Scalar c = 0.5;
        const Scalar gr = 2.;
        const Scalar br = 3.;

        // Uniform quantities
        ori::CoordinateAxis axis = ori::CoordinateAxis::Z;

        const spatial::Transform<Scalar> Xtree2 =
            spatial::Transform<Scalar>(I3, Vec3<Scalar>(l, 0, 0.));

        Mat3<Scalar> link_inertia;
        link_inertia << 0., 0., 0., 0., 0., 0., 0., 0., I;
        const SpatialInertia<Scalar> link_spatiala_inertia(m, Vec3<Scalar>(c, 0., 0.),
                                                           link_inertia);

        Mat3<Scalar> rotor_inertia;
        rotor_inertia << 0., 0., 0., 0., 0., 0., 0., 0., Irot;
        const SpatialInertia<Scalar> rotor_spatial_inertia(0., Vec3<Scalar>::Zero(),
                                                           rotor_inertia);

        std::string parent_name = "ground";
        for (size_t i(0); i < N / 2; i++)
        {
            const spatial::Transform<Scalar> Xtree1 = i == 0 ? spatial::Transform<Scalar>(I3, z3)
                                                             : Xtree2;

            // Link A
            const std::string linkA_name = "link-A-" + std::to_string(i);
            auto linkA = model.registerBody(linkA_name, link_spatiala_inertia,
                                            parent_name, Xtree1);

            // Rotor A
            const std::string rotorA_name = "rotor-A-" + std::to_string(i);
            auto rotorA = model.registerBody(rotorA_name, rotor_spatial_inertia,
                                             parent_name, Xtree1);

            // Rotor B
            const std::string rotorB_name = "rotor-B-" + std::to_string(i);
            auto rotorB = model.registerBody(rotorB_name, rotor_spatial_inertia,
                                             parent_name, Xtree1);

            // Link B
            const std::string linkB_name = "link-B-" + std::to_string(i);
            auto linkB = model.registerBody(linkB_name, link_spatiala_inertia,
                                            linkA_name, Xtree2);

            // Cluster
            const std::string name = "cluster-" + std::to_string(i);
            ProxTransModule moduleA{linkA, rotorA, axis, axis, gr, Vec1<Scalar>{br}};
            DistTransModule moduleB{linkB, rotorB, axis, axis, gr, Vec2<Scalar>{br, 1.}};
            model.template appendRegisteredBodiesAsCluster<RevPairRotor>(name, moduleA, moduleB);

            // Contact points
            appendContactPoints(model, i, linkA_name, linkB_name);

            parent_name = linkB_name;
        }

        return model;
    }

    template <size_t N, typename Scalar>
    void RevolutePairChainWithRotor<N, Scalar>::appendContactPoints(
        ClusterTreeModel<Scalar> &model, const int i,
        const std::string linkA_name, const std::string linkB_name) const
    {
        const std::string cpA_name = "cp-A-" + std::to_string(i);
        const Vec3<Scalar> cpA_local_offset = Vec3<Scalar>::Random();
        model.appendContactPoint(linkA_name, cpA_local_offset, cpA_name);

        const std::string cpB_name = "cp-B-" + std::to_string(i);
        const Vec3<Scalar> cpB_local_offset = Vec3<Scalar>::Random();
        if (i == N / 2 - 1)
            model.appendEndEffector(linkB_name, cpB_local_offset, cpB_name);
        else
            model.appendContactPoint(linkB_name, cpB_local_offset, cpB_name);
    }

    template <size_t N, typename Scalar>
    DVec<double> RevolutePairChainWithRotor<N, Scalar>::forwardDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        std::vector<DVec<double>> arg = {y, yd, tau};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevPairWithRotors2DofFwdDyn,
                             RevPairWithRotors2DofFwdDyn_sparsity_out,
                             RevPairWithRotors2DofFwdDyn_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevPairWithRotors4DofFwdDyn,
                             RevPairWithRotors4DofFwdDyn_sparsity_out,
                             RevPairWithRotors4DofFwdDyn_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N, typename Scalar>
    DVec<double> RevolutePairChainWithRotor<N, Scalar>::forwardDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        std::vector<DVec<double>> arg = {y, yd, tau};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevPairWithRotors2DofFwdDynReflectedInertia,
                             RevPairWithRotors2DofFwdDynReflectedInertia_sparsity_out,
                             RevPairWithRotors2DofFwdDynReflectedInertia_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevPairWithRotors4DofFwdDynReflectedInertia,
                             RevPairWithRotors4DofFwdDynReflectedInertia_sparsity_out,
                             RevPairWithRotors4DofFwdDynReflectedInertia_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N, typename Scalar>
    DVec<double> RevolutePairChainWithRotor<N, Scalar>::forwardDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        std::vector<DVec<double>> arg = {y, yd, tau};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevPairWithRotors2DofFwdDynReflectedInertiaDiag,
                             RevPairWithRotors2DofFwdDynReflectedInertiaDiag_sparsity_out,
                             RevPairWithRotors2DofFwdDynReflectedInertiaDiag_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevPairWithRotors4DofFwdDynReflectedInertiaDiag,
                             RevPairWithRotors4DofFwdDynReflectedInertiaDiag_sparsity_out,
                             RevPairWithRotors4DofFwdDynReflectedInertiaDiag_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N, typename Scalar>
    DVec<double> RevolutePairChainWithRotor<N, Scalar>::inverseDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        std::vector<DVec<double>> arg = {y, yd, ydd};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevPairWithRotors2DofInvDyn,
                             RevPairWithRotors2DofInvDyn_sparsity_out,
                             RevPairWithRotors2DofInvDyn_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevPairWithRotors4DofInvDyn,
                             RevPairWithRotors4DofInvDyn_sparsity_out,
                             RevPairWithRotors4DofInvDyn_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N, typename Scalar>
    DVec<double> RevolutePairChainWithRotor<N, Scalar>::inverseDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        std::vector<DVec<double>> arg = {y, yd, ydd};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevPairWithRotors2DofInvDynReflectedInertia,
                             RevPairWithRotors2DofInvDynReflectedInertia_sparsity_out,
                             RevPairWithRotors2DofInvDynReflectedInertia_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevPairWithRotors4DofInvDynReflectedInertia,
                             RevPairWithRotors4DofInvDynReflectedInertia_sparsity_out,
                             RevPairWithRotors4DofInvDynReflectedInertia_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N, typename Scalar>
    DVec<double> RevolutePairChainWithRotor<N, Scalar>::inverseDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        std::vector<DVec<double>> arg = {y, yd, ydd};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevPairWithRotors2DofInvDynReflectedInertiaDiag,
                             RevPairWithRotors2DofInvDynReflectedInertiaDiag_sparsity_out,
                             RevPairWithRotors2DofInvDynReflectedInertiaDiag_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevPairWithRotors4DofInvDynReflectedInertiaDiag,
                             RevPairWithRotors4DofInvDynReflectedInertiaDiag_sparsity_out,
                             RevPairWithRotors4DofInvDynReflectedInertiaDiag_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template class RevolutePairChainWithRotor<2ul>;
    template class RevolutePairChainWithRotor<4ul>;
    template class RevolutePairChainWithRotor<6ul>;
    template class RevolutePairChainWithRotor<8ul>;
    template class RevolutePairChainWithRotor<10ul>;
    template class RevolutePairChainWithRotor<12ul>;
    template class RevolutePairChainWithRotor<16ul>;
    template class RevolutePairChainWithRotor<20ul>;
    template class RevolutePairChainWithRotor<24ul>;
    template class RevolutePairChainWithRotor<28ul>;
    template class RevolutePairChainWithRotor<32ul>;

    template class RevolutePairChainWithRotor<2ul, casadi::SX>;
    template class RevolutePairChainWithRotor<6ul, casadi::SX>;

} // namespace grbda
