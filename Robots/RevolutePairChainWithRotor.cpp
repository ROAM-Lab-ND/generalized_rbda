#include "RevolutePairChainWithRotor.hpp"

namespace grbda
{

    template <size_t N>
    ClusterTreeModel RevolutePairChainWithRotor<N>::buildRandomClusterTreeModel() const
    {
        ClusterTreeModel model{};

        std::string parent_name = "ground";
        for (size_t i(0); i < N / 2; i++)
        {
            // Link A
            const std::string linkA_name = "link-A-" + std::to_string(i);
            const auto linkA_Xtree = randomSpatialRotation<double>();
            const auto linkA_inertia = SpatialInertia<double>::createRandomInertia();
            CoordinateAxis linkA_joint_axis = ori::randomCoordinateAxis<double>();
            auto linkA = model.registerBody(linkA_name, linkA_inertia, parent_name, linkA_Xtree);

            // Rotor A
            const std::string rotorA_name = "rotor-A-" + std::to_string(i);
            const auto rotorA_Xtree = randomSpatialRotation<double>();
            const auto rotorA_inertia = SpatialInertia<double>::createRandomInertia();
            CoordinateAxis rotorA_joint_axis = ori::randomCoordinateAxis<double>();
            auto rotorA = model.registerBody(rotorA_name, rotorA_inertia,
                                             parent_name, rotorA_Xtree);

            // Rotor B
            const std::string rotorB_name = "rotor-B-" + std::to_string(i);
            const auto rotorB_Xtree = randomSpatialRotation<double>();
            const auto rotorB_inertia = SpatialInertia<double>::createRandomInertia();
            CoordinateAxis rotorB_joint_axis = ori::randomCoordinateAxis<double>();
            auto rotorB = model.registerBody(rotorB_name, rotorB_inertia,
                                             parent_name, rotorB_Xtree);

            // Link B
            const std::string linkB_name = "link-B-" + std::to_string(i);
            const auto linkB_Xtree = randomSpatialRotation<double>();
            const auto linkB_inertia = SpatialInertia<double>::createRandomInertia();
            CoordinateAxis linkB_joint_axis = ori::randomCoordinateAxis<double>();
            auto linkB = model.registerBody(linkB_name, linkB_inertia, linkA_name, linkB_Xtree);

            // Cluster
            const int gear_ratioA = rand() % 5 + 1;
            const int belt_ratioA = rand() % 5 + 1;
            const int gear_ratioB = rand() % 5 + 1;
            const int belt_ratioB = rand() % 5 + 1;

            const std::string cluster_name = "cluster-" + std::to_string(i);
            auto joint = std::make_shared<GeneralizedJoints::RevolutePairWithRotor>(
                linkA, linkB, rotorA, rotorB, linkA_joint_axis, linkB_joint_axis,
                rotorA_joint_axis, rotorB_joint_axis, gear_ratioA, gear_ratioB,
                belt_ratioA, belt_ratioB);

            model.appendRegisteredBodiesAsCluster(cluster_name, joint);

            parent_name = linkA_name;
        }

        return model;
    }

    template <size_t N>
    ClusterTreeModel RevolutePairChainWithRotor<N>::buildUniformClusterTreeModel() const
    {
        ClusterTreeModel model{};

        Mat3<double> I3 = Mat3<double>::Identity();
        Vec3<double> z3 = Vec3<double>::Zero();

        const double grav = 9.81;
        model.setGravity(Vec3<double>{grav, 0., 0.});

        // Inertia params
        const double I = 1.;
        const double Irot = 1e-4;
        const double m = 1.;
        const double l = 1.;
        const double c = 0.5;
        const double gr = 2.;
        const double br = 3.;

        // Uniform quantities
        CoordinateAxis axis = CoordinateAxis::Z;

        const SpatialTransform Xtree2 = SpatialTransform(I3, Vec3<double>(l, 0, 0.));

        Mat3<double> link_inertia;
        link_inertia << 0., 0., 0., 0., 0., 0., 0., 0., I;
        const SpatialInertia<double> link_spatiala_inertia(m, Vec3<double>(c, 0., 0.),
                                                           link_inertia);

        Mat3<double> rotor_inertia;
        rotor_inertia << 0., 0., 0., 0., 0., 0., 0., 0., Irot;
        const SpatialInertia<double> rotor_spatial_inertia(0., Vec3<double>::Zero(),
                                                           rotor_inertia);

        std::string parent_name = "ground";
        for (size_t i(0); i < N / 2; i++)
        {
            const SpatialTransform Xtree1 = i == 0 ? SpatialTransform(I3, z3) : Xtree2;

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
            const std::string cluster_name = "cluster-" + std::to_string(i);
            auto joint = std::make_shared<GeneralizedJoints::RevolutePairWithRotor>(
                linkA, linkB, rotorA, rotorB, axis, axis, axis, axis, gr, gr, br, br);

            model.appendRegisteredBodiesAsCluster(cluster_name, joint);

            parent_name = linkB_name;
        }

        return model;
    }

    template <size_t N>
    DVec<double> RevolutePairChainWithRotor<N>::forwardDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        vector<DVec<double>> arg = {y, yd, tau};
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

    template <size_t N>
    DVec<double> RevolutePairChainWithRotor<N>::forwardDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        vector<DVec<double>> arg = {y, yd, tau};
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

    template <size_t N>
    DVec<double> RevolutePairChainWithRotor<N>::forwardDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        vector<DVec<double>> arg = {y, yd, tau};
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

    template <size_t N>
    DVec<double> RevolutePairChainWithRotor<N>::inverseDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        vector<DVec<double>> arg = {y, yd, ydd};
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

    template <size_t N>
    DVec<double> RevolutePairChainWithRotor<N>::inverseDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        vector<DVec<double>> arg = {y, yd, ydd};
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

    template <size_t N>
    DVec<double> RevolutePairChainWithRotor<N>::inverseDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        vector<DVec<double>> arg = {y, yd, ydd};
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

} // namespace grbda
