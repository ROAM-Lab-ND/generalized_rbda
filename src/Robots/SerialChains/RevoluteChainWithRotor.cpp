#include "grbda/Robots/SerialChains/RevoluteChainWithRotor.hpp"

namespace grbda
{

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar> RevoluteChainWithRotor<N, Scalar>::buildRandomClusterTreeModel() const
    {
        ClusterTreeModel<Scalar> model{};

        std::string prev_link_name = "ground";
        for (size_t i(0); i < N; i++)
        {
            // Link
            const std::string link_name = "link-" + std::to_string(i);
            const auto link_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto link_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis link_axis = ori::randomCoordinateAxis();
            auto link = model.registerBody(link_name, link_inertia, prev_link_name, link_Xtree);

            // Rotor
            const std::string rotor_name = "rotor-" + std::to_string(i);
            const auto rotor_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto rotor_inertia = this->randomRotorSpatialInertia();
            ori::CoordinateAxis rotor_axis = ori::randomCoordinateAxis();
            auto rotor = model.registerBody(rotor_name, rotor_inertia, prev_link_name, rotor_Xtree);

            // Cluster
            const std::string cluster_name = "cluster-" + std::to_string(i);
            const std::string link_joint_name = "link-joint-" + std::to_string(i);
            const std::string rotor_joint_name = "rotor-joint-" + std::to_string(i);
            TransmissionModule module{link, rotor, link_joint_name, rotor_joint_name,
                                      link_axis, rotor_axis, this->randomGearRatio()};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(cluster_name, module);

            // Contact point
            appendContactPoints(model, i, link_name);

            prev_link_name = link_name;
        }

        return model;
    }

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar> RevoluteChainWithRotor<N, Scalar>::buildUniformClusterTreeModel() const
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

        Mat3<Scalar> link_inertia;
        link_inertia << 0., 0., 0., 0., 0., 0., 0., 0., I;
        const SpatialInertia<Scalar> link_spatiala_inertia(m, Vec3<Scalar>(c, 0., 0.),
                                                           link_inertia);

        Mat3<Scalar> rotor_inertia;
        rotor_inertia << 0., 0., 0., 0., 0., 0., 0., 0., Irot;
        const SpatialInertia<Scalar> rotor_spatial_inertia(0., Vec3<Scalar>::Zero(),
                                                           rotor_inertia);

        std::string prev_link_name = "ground";
        for (size_t i(0); i < N; i++)
        {
            spatial::Transform Xtree = i == 0 ? spatial::Transform(I3, z3)
                                              : spatial::Transform(I3, Vec3<Scalar>(l, 0., 0.));

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
            const std::string link_joint_name = "link-joint-" + std::to_string(i);
            const std::string rotor_joint_name = "rotor-joint-" + std::to_string(i);
            TransmissionModule module{link, rotor, link_joint_name, rotor_joint_name,
                                      axis, axis, gr * br};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(cluster_name, module);

            // Contact point
            appendContactPoints(model, i, link_name);

            prev_link_name = link_name;
        }

        return model;
    }

    template <size_t N, typename Scalar>
    void RevoluteChainWithRotor<N, Scalar>::appendContactPoints(
        ClusterTreeModel<Scalar> &model, const int i, const std::string link_name) const
    {
        const std::string cp_name = "cp-" + std::to_string(i);
        const Vec3<Scalar> cp_local_offset = Vec3<Scalar>::Random();
        if (i == N - 1)
            model.appendEndEffector(link_name, cp_local_offset, cp_name);
        else
            model.appendContactPoint(link_name, cp_local_offset, cp_name);
    }

    template <size_t N, typename Scalar>
    DVec<double> RevoluteChainWithRotor<N, Scalar>::forwardDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        std::vector<DVec<double>> arg = {y, yd, tau};
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

    template <size_t N, typename Scalar>
    DVec<double> RevoluteChainWithRotor<N, Scalar>::forwardDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        std::vector<DVec<double>> arg = {y, yd, tau};
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

    template <size_t N, typename Scalar>
    DVec<double> RevoluteChainWithRotor<N, Scalar>::forwardDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
        std::vector<DVec<double>> arg = {y, yd, tau};
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

    template <size_t N, typename Scalar>
    DVec<double> RevoluteChainWithRotor<N, Scalar>::inverseDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        std::vector<DVec<double>> arg = {y, yd, ydd};
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

    template <size_t N, typename Scalar>
    DVec<double> RevoluteChainWithRotor<N, Scalar>::inverseDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        std::vector<DVec<double>> arg = {y, yd, ydd};
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

    template <size_t N, typename Scalar>
    DVec<double> RevoluteChainWithRotor<N, Scalar>::inverseDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
        std::vector<DVec<double>> arg = {y, yd, ydd};
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

    template class RevoluteChainWithRotor<2ul>;
    template class RevoluteChainWithRotor<3ul>;
    template class RevoluteChainWithRotor<4ul>;
    template class RevoluteChainWithRotor<6ul>;
    template class RevoluteChainWithRotor<8ul>;
    template class RevoluteChainWithRotor<10ul>;
    template class RevoluteChainWithRotor<12ul>;
    template class RevoluteChainWithRotor<16ul>;
    template class RevoluteChainWithRotor<20ul>;
    template class RevoluteChainWithRotor<24ul>;

    template class RevoluteChainWithRotor<2ul, casadi::SX>;
    template class RevoluteChainWithRotor<4ul, casadi::SX>;
    template class RevoluteChainWithRotor<8ul, casadi::SX>;

} // namespace grbda
