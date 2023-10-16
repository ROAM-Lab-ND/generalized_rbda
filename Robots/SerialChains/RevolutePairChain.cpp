#include "RevolutePairChain.hpp"

namespace grbda
{

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar> RevolutePairChain<N, Scalar>::buildRandomClusterTreeModel() const
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

            // Link B
            const std::string linkB_name = "link-B-" + std::to_string(i);
            const auto linkB_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkB_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkB_joint_axis = ori::randomCoordinateAxis();
            auto linkB = model.registerBody(linkB_name, linkB_inertia, linkA_name, linkB_Xtree);

            // Cluster
            const std::string cluster_name = "cluster-" + std::to_string(i);
            model.template appendRegisteredBodiesAsCluster<RevolutePair>(
                cluster_name, linkA, linkB, linkA_joint_axis, linkB_joint_axis);

            // Contact points
            const std::string cpA_name = "cp-A-" + std::to_string(i);
            const Vec3<Scalar> cpA_local_offset = Vec3<Scalar>::Zero();
            model.appendContactPoint(linkA_name, cpA_local_offset, cpA_name);

            const std::string cpB_name = "cp-B-" + std::to_string(i);
            const Vec3<Scalar> cpB_local_offset = Vec3<Scalar>::Zero();
            model.appendContactPoint(linkB_name, cpB_local_offset, cpB_name);

            parent_name = linkA_name;
        }

        return model;
    }

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar> RevolutePairChain<N, Scalar>::buildUniformClusterTreeModel() const
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

        const spatial::Transform<Scalar> Xtree2 = spatial::Transform(I3, Vec3<Scalar>(l, 0, 0.));

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

            // Link B
            const std::string linkB_name = "link-B-" + std::to_string(i);
            auto linkB = model.registerBody(linkB_name, link_spatiala_inertia,
                                            linkA_name, Xtree2);

            // Cluster
            const std::string cluster_name = "cluster-" + std::to_string(i);
            model.template appendRegisteredBodiesAsCluster<RevolutePair>(
                cluster_name, linkA, linkB, axis, axis);

            parent_name = linkB_name;
        }

        return model;
    }

    template class RevolutePairChain<2ul>;
    template class RevolutePairChain<4ul>;

} // namespace grbda
