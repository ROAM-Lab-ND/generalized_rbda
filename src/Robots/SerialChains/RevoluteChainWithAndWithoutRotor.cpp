#include "grbda/Robots/SerialChains/RevoluteChainWithAndWithoutRotor.hpp"

namespace grbda
{

    template <size_t N, size_t M, typename Scalar>
    ClusterTreeModel<Scalar>
    RevoluteChainWithAndWithoutRotor<N, M, Scalar>::buildRandomClusterTreeModel() const
    {
        using namespace ClusterJoints;

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

            prev_link_name = link_name;
        }

        for (size_t i(N); i < N + M; i++)
        {
            // Link
            const std::string link_name = "link-" + std::to_string(i);
            const auto link_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto link_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis link_joint_axis = ori::randomCoordinateAxis();
            model.template appendBody<Revolute>(link_name, link_inertia, prev_link_name,
                                                link_Xtree, link_joint_axis);

            prev_link_name = link_name;
        }

        return model;
    }

    template <size_t N, size_t M, typename Scalar>
    ClusterTreeModel<Scalar>
    RevoluteChainWithAndWithoutRotor<N, M, Scalar>::buildUniformClusterTreeModel() const
    {
        using namespace ClusterJoints;

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

        // Uniform quantities
        ori::CoordinateAxis axis = ori::CoordinateAxis::Z;

        const spatial::Transform<Scalar> Xtree_link = spatial::Transform(I3, Vec3<Scalar>(l, 0, 0.));

        Mat3<Scalar> link_inertia;
        link_inertia << 0., 0., 0., 0., 0., 0., 0., 0., I;
        const SpatialInertia<Scalar> link_spatial_inertia(m, Vec3<Scalar>(c, 0., 0.),
                                                          link_inertia);

        Mat3<Scalar> rotor_inertia;
        rotor_inertia << 0., 0., 0., 0., 0., 0., 0., 0., Irot;
        const SpatialInertia<Scalar> rotor_spatial_inertia(0., Vec3<Scalar>::Zero(),
                                                           rotor_inertia);

        std::string prev_link_name = "ground";

        // First N links with rotors (cluster joints)
        for (size_t i(0); i < N; i++)
        {
            const spatial::Transform<Scalar> Xtree1 = i == 0 ? spatial::Transform<Scalar>(I3, z3)
                                                             : Xtree_link;

            // Link
            const std::string link_name = "link-" + std::to_string(i);
            auto link = model.registerBody(link_name, link_spatial_inertia,
                                          prev_link_name, Xtree1);

            // Rotor
            const std::string rotor_name = "rotor-" + std::to_string(i);
            auto rotor = model.registerBody(rotor_name, rotor_spatial_inertia,
                                           prev_link_name, Xtree1);

            // Cluster
            const std::string cluster_name = "cluster-" + std::to_string(i);
            const std::string link_joint_name = "link-joint-" + std::to_string(i);
            const std::string rotor_joint_name = "rotor-joint-" + std::to_string(i);
            TransmissionModule module{link, rotor, link_joint_name, rotor_joint_name,
                                      axis, axis, gr};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(cluster_name, module);

            prev_link_name = link_name;
        }

        // Next M links without rotors (simple Revolute joints)
        for (size_t i(N); i < N + M; i++)
        {
            const spatial::Transform<Scalar> Xtree1 = i == N && N == 0 ?
                                                      spatial::Transform<Scalar>(I3, z3) : Xtree_link;

            const std::string link_name = "link-" + std::to_string(i);
            model.template appendBody<Revolute>(link_name, link_spatial_inertia, prev_link_name,
                                                Xtree1, axis);

            prev_link_name = link_name;
        }

        return model;
    }

    template class RevoluteChainWithAndWithoutRotor<0ul, 2ul>;
    template class RevoluteChainWithAndWithoutRotor<0ul, 3ul>;
    template class RevoluteChainWithAndWithoutRotor<0ul, 4ul>;
    template class RevoluteChainWithAndWithoutRotor<0ul, 8ul>;
    template class RevoluteChainWithAndWithoutRotor<1ul, 7ul>;
    template class RevoluteChainWithAndWithoutRotor<2ul, 0ul>;
    template class RevoluteChainWithAndWithoutRotor<2ul, 6ul>;
    template class RevoluteChainWithAndWithoutRotor<3ul, 0ul>;
    template class RevoluteChainWithAndWithoutRotor<3ul, 5ul>;
    template class RevoluteChainWithAndWithoutRotor<4ul, 0ul>;
    template class RevoluteChainWithAndWithoutRotor<4ul, 4ul>;
    template class RevoluteChainWithAndWithoutRotor<5ul, 3ul>;
    template class RevoluteChainWithAndWithoutRotor<6ul, 2ul>;
    template class RevoluteChainWithAndWithoutRotor<7ul, 1ul>;
    template class RevoluteChainWithAndWithoutRotor<8ul, 0ul>;

    template class RevoluteChainWithAndWithoutRotor<0ul, 2ul, std::complex<double>>;
    template class RevoluteChainWithAndWithoutRotor<0ul, 3ul, std::complex<double>>;
    template class RevoluteChainWithAndWithoutRotor<0ul, 4ul, std::complex<double>>;

} // namespace grbda
