#include "grbda/Robots/SerialChains/RevoluteTripleChainWithRotor.hpp"

namespace grbda
{

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar> 
    RevoluteTripleChainWithRotor<N, Scalar>::buildRandomClusterTreeModel() const
    {
        ClusterTreeModel<Scalar> model{};

        std::string parent_name = "ground";
        for (size_t i(0); i < N / 3; i++)
        {
            // Link A
            const std::string linkA_name = "link-A-" + std::to_string(i);
            const auto linkA_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkA_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkA_joint_axis = ori::randomCoordinateAxis();
            auto linkA = model.registerBody(linkA_name, linkA_inertia, parent_name, linkA_Xtree);

            // Link B
            const std::string linkB_name = "link-B-" + std::to_string(i);
            const auto linkB_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkB_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkB_joint_axis = ori::randomCoordinateAxis();
            auto linkB = model.registerBody(linkB_name, linkB_inertia, linkA_name, linkB_Xtree);

            // Link C
            const std::string linkC_name = "link-C-" + std::to_string(i);
            const auto linkC_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkC_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkC_joint_axis = ori::randomCoordinateAxis();
            auto linkC = model.registerBody(linkC_name, linkC_inertia, linkB_name, linkC_Xtree);

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

            // Rotor C
            const std::string rotorC_name = "rotor-C-" + std::to_string(i);
            const auto rotorC_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto rotorC_inertia = this->randomRotorSpatialInertia();
            ori::CoordinateAxis rotorC_joint_axis = ori::randomCoordinateAxis();
            auto rotorC = model.registerBody(rotorC_name, rotorC_inertia,
                                             parent_name, rotorC_Xtree);

            // Cluster
            ProxTransModule moduleA{linkA, rotorA, linkA_joint_axis, rotorA_joint_axis,
                                    this->randomGearRatio(), this->template randomBeltRatios<1>()};
            InterTransModule moduleB{linkB, rotorB, linkB_joint_axis, rotorB_joint_axis,
                                     this->randomGearRatio(), this->template randomBeltRatios<2>()};
            DistTransModule moduleC{linkC, rotorC, linkC_joint_axis, rotorC_joint_axis,
                                    this->randomGearRatio(), this->template randomBeltRatios<3>()};

            const std::string cluster_name = "cluster-" + std::to_string(i);
            model.template appendRegisteredBodiesAsCluster<RevTripleWithRotor>(
                cluster_name, moduleA, moduleB, moduleC);

            // Contact points
            appendContactPoints(model, i, linkA_name, linkB_name, linkC_name);

            parent_name = linkC_name;
        }

        return model;
    }

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar>
    RevoluteTripleChainWithRotor<N, Scalar>::buildUniformClusterTreeModel() const
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
        DVec<Scalar> br1(1), br2(2), br3(3);
        br1 << 3.;
        br2 << 3., 3.;
        br3 << 3., 3., 3.;

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

        std::string parent_name = "ground";
        for (size_t i(0); i < N / 3; i++)
        {
            const spatial::Transform<Scalar> Xtree1 = i == 0 ? spatial::Transform<Scalar>(I3, z3)
                                                             : Xtree_link;

            // Link A
            const std::string linkA_name = "link-A-" + std::to_string(i);
            auto linkA = model.registerBody(linkA_name, link_spatial_inertia,
                                           parent_name, Xtree1);

            // Link B
            const std::string linkB_name = "link-B-" + std::to_string(i);
            auto linkB = model.registerBody(linkB_name, link_spatial_inertia,
                                           linkA_name, Xtree_link);

            // Link C
            const std::string linkC_name = "link-C-" + std::to_string(i);
            auto linkC = model.registerBody(linkC_name, link_spatial_inertia,
                                           linkB_name, Xtree_link);

            // Rotor A
            const std::string rotorA_name = "rotor-A-" + std::to_string(i);
            auto rotorA = model.registerBody(rotorA_name, rotor_spatial_inertia,
                                            parent_name, Xtree1);

            // Rotor B
            const std::string rotorB_name = "rotor-B-" + std::to_string(i);
            auto rotorB = model.registerBody(rotorB_name, rotor_spatial_inertia,
                                            parent_name, Xtree1);

            // Rotor C
            const std::string rotorC_name = "rotor-C-" + std::to_string(i);
            auto rotorC = model.registerBody(rotorC_name, rotor_spatial_inertia,
                                            parent_name, Xtree1);

            // Cluster
            ProxTransModule moduleA{linkA, rotorA, axis, axis, gr, br1};
            InterTransModule moduleB{linkB, rotorB, axis, axis, gr, br2};
            DistTransModule moduleC{linkC, rotorC, axis, axis, gr, br3};

            const std::string cluster_name = "cluster-" + std::to_string(i);
            model.template appendRegisteredBodiesAsCluster<RevTripleWithRotor>(
                cluster_name, moduleA, moduleB, moduleC);

            parent_name = linkC_name;
        }

        return model;
    }

    template <size_t N, typename Scalar>
    void RevoluteTripleChainWithRotor<N, Scalar>::appendContactPoints(
        ClusterTreeModel<Scalar> &model, const int i, const std::string linkA_name,
        const std::string linkB_name, const std::string linkC_name) const
    {
        const std::string cpA_name = "cp-A-" + std::to_string(i);
        const Vec3<Scalar> cpA_local_offset = Vec3<Scalar>::Random();
        model.appendContactPoint(linkA_name, cpA_local_offset, cpA_name);

        const std::string cpB_name = "cp-B-" + std::to_string(i);
        const Vec3<Scalar> cpB_local_offset = Vec3<Scalar>::Random();
        model.appendContactPoint(linkB_name, cpB_local_offset, cpB_name);

        const std::string cpC_name = "cp-C-" + std::to_string(i);
        const Vec3<Scalar> cpC_local_offset = Vec3<Scalar>::Random();
        if (i == N / 3 - 1)
            model.appendEndEffector(linkC_name, cpC_local_offset, cpC_name);
        else
            model.appendContactPoint(linkC_name, cpC_local_offset, cpC_name);
    }

    template class RevoluteTripleChainWithRotor<3ul>;
    template class RevoluteTripleChainWithRotor<6ul>;
    template class RevoluteTripleChainWithRotor<9ul>;
    template class RevoluteTripleChainWithRotor<12ul>;
    template class RevoluteTripleChainWithRotor<15ul>;

    template class RevoluteTripleChainWithRotor<3ul, std::complex<double>>;
    template class RevoluteTripleChainWithRotor<6ul, std::complex<double>>;

} // namespace grbda
