#include "RevoluteChainWithRotor.hpp"

namespace grbda
{

    template <size_t N>
    ClusterTreeModel RevoluteChainWithRotor<N>::buildRandomClusterTreeModel() const
    {
        ClusterTreeModel model{};

        std::string prev_link_name = "ground";
        for (size_t i(0); i < N; i++)
        {
            // Link
            const std::string link_name = "link-" + std::to_string(i);
            const auto link_Xtree = randomSpatialRotation<double>();
            const auto link_inertia = randomLinkSpatialInertia();
            CoordinateAxis link_joint_axis = ori::randomCoordinateAxis<double>();
            auto link = model.registerBody(link_name, link_inertia, prev_link_name, link_Xtree);

            // Rotor
            const std::string rotor_name = "rotor-" + std::to_string(i);
            const auto rotor_Xtree = randomSpatialRotation<double>();
            const auto rotor_inertia = randomRotorSpatialInertia();
            CoordinateAxis rotor_joint_axis = ori::randomCoordinateAxis<double>();
            auto rotor = model.registerBody(rotor_name, rotor_inertia, prev_link_name, rotor_Xtree);

            // Cluster
            const int gear_ratio = rand() % this->_gear_ratio_scale + 1;
            const std::string cluster_name = "cluster-" + std::to_string(i);
            auto joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(
                link, rotor, link_joint_axis, rotor_joint_axis, gear_ratio);
            model.appendRegisteredBodiesAsCluster(cluster_name, joint);

            // Contact point
            appendContactPoints(model, i, link_name);

            prev_link_name = link_name;
        }

        return model;
    }

    template <size_t N>
    ClusterTreeModel RevoluteChainWithRotor<N>::buildUniformClusterTreeModel() const
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

        Mat3<double> link_inertia;
        link_inertia << 0., 0., 0., 0., 0., 0., 0., 0., I;
        const SpatialInertia<double> link_spatiala_inertia(m, Vec3<double>(c, 0., 0.),
                                                           link_inertia);

        Mat3<double> rotor_inertia;
        rotor_inertia << 0., 0., 0., 0., 0., 0., 0., 0., Irot;
        const SpatialInertia<double> rotor_spatial_inertia(0., Vec3<double>::Zero(),
                                                           rotor_inertia);

        std::string prev_link_name = "ground";
        for (size_t i(0); i < N; i++)
        {
            const SpatialTransform Xtree = i == 0 ? SpatialTransform(I3, z3)
                                                  : SpatialTransform(I3, Vec3<double>(l, 0., 0.));

            // Link
            const std::string link_name = "link-" + std::to_string(i);
            auto link = model.registerBody(link_name, link_spatiala_inertia,
                                           prev_link_name, Xtree);

            // Rotor
            const std::string rotor_name = "rotor-" + std::to_string(i);
            auto rotor = model.registerBody(rotor_name, rotor_spatial_inertia,
                                            prev_link_name, Xtree);

            // Cluster
            const std::string cluster_name = "cluster-" + std::to_string(i);
            auto joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(
                link, rotor, axis, axis, gr * br);
            model.appendRegisteredBodiesAsCluster(cluster_name, joint);

            // Contact point
            appendContactPoints(model, i, link_name);

            prev_link_name = link_name;
        }

        return model;
    }

    template <size_t N>
    void RevoluteChainWithRotor<N>::appendContactPoints(ClusterTreeModel &model, const int i,
                                                        const std::string link_name) const
    {
        const std::string cp_name = "cp-" + std::to_string(i);
        const Vec3<double> cp_local_offset = Vec3<double>::Random();
        model.appendEndEffector(link_name, cp_local_offset, cp_name);
    }

    template <size_t N>
    DVec<double> RevoluteChainWithRotor<N>::forwardDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        vector<DVec<double>> arg = {y, yd, tau};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevWithRotors2DofFwdDyn,
                             RevWithRotors2DofFwdDyn_sparsity_out,
                             RevWithRotors2DofFwdDyn_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevWithRotors4DofFwdDyn,
                             RevWithRotors4DofFwdDyn_sparsity_out,
                             RevWithRotors4DofFwdDyn_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N>
    DVec<double> RevoluteChainWithRotor<N>::forwardDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        vector<DVec<double>> arg = {y, yd, tau};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevWithRotors2DofFwdDynReflectedInertia,
                             RevWithRotors2DofFwdDynReflectedInertia_sparsity_out,
                             RevWithRotors2DofFwdDynReflectedInertia_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevWithRotors4DofFwdDynReflectedInertia,
                             RevWithRotors4DofFwdDynReflectedInertia_sparsity_out,
                             RevWithRotors4DofFwdDynReflectedInertia_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N>
    DVec<double> RevoluteChainWithRotor<N>::forwardDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        vector<DVec<double>> arg = {y, yd, tau};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevWithRotors2DofFwdDynReflectedInertiaDiag,
                             RevWithRotors2DofFwdDynReflectedInertiaDiag_sparsity_out,
                             RevWithRotors2DofFwdDynReflectedInertiaDiag_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevWithRotors4DofFwdDynReflectedInertiaDiag,
                             RevWithRotors4DofFwdDynReflectedInertiaDiag_sparsity_out,
                             RevWithRotors4DofFwdDynReflectedInertiaDiag_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N>
    DVec<double> RevoluteChainWithRotor<N>::inverseDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        vector<DVec<double>> arg = {y, yd, ydd};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevWithRotors2DofInvDyn,
                             RevWithRotors2DofInvDyn_sparsity_out,
                             RevWithRotors2DofInvDyn_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevWithRotors4DofInvDyn,
                             RevWithRotors4DofInvDyn_sparsity_out,
                             RevWithRotors4DofInvDyn_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N>
    DVec<double> RevoluteChainWithRotor<N>::inverseDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        vector<DVec<double>> arg = {y, yd, ydd};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevWithRotors2DofInvDynReflectedInertia,
                             RevWithRotors2DofInvDynReflectedInertia_sparsity_out,
                             RevWithRotors2DofInvDynReflectedInertia_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevWithRotors4DofInvDynReflectedInertia,
                             RevWithRotors4DofInvDynReflectedInertia_sparsity_out,
                             RevWithRotors4DofInvDynReflectedInertia_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    template <size_t N>
    DVec<double> RevoluteChainWithRotor<N>::inverseDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        vector<DVec<double>> arg = {y, yd, ydd};
        DMat<double> eom = DVec<double>::Zero(N);

        if (N == 2)
            casadi_interface(arg, eom,
                             RevWithRotors2DofInvDynReflectedInertiaDiag,
                             RevWithRotors2DofInvDynReflectedInertiaDiag_sparsity_out,
                             RevWithRotors2DofInvDynReflectedInertiaDiag_work);
        else if (N == 4)
            casadi_interface(arg, eom,
                             RevWithRotors4DofInvDynReflectedInertiaDiag,
                             RevWithRotors4DofInvDynReflectedInertiaDiag_sparsity_out,
                             RevWithRotors4DofInvDynReflectedInertiaDiag_work);
        else
            throw std::runtime_error("N must be 2 or 4");

        return eom;
    }

    // TODO(@MatthewChignoli): If we put all of the code in the header file, we don't have to worry about this nonsense
    template class RevoluteChainWithRotor<2ul>;
    template class RevoluteChainWithRotor<4ul>;
    template class RevoluteChainWithRotor<6ul>;
    template class RevoluteChainWithRotor<8ul>;
    template class RevoluteChainWithRotor<10ul>;
    template class RevoluteChainWithRotor<12ul>;
    template class RevoluteChainWithRotor<16ul>;
    template class RevoluteChainWithRotor<20ul>;
    template class RevoluteChainWithRotor<24ul>;

} // namespace grbda
