#include "RevoluteChainWithAndWithoutRotor.hpp"

namespace grbda
{

    template <size_t N, size_t M>
    ClusterTreeModel<> RevoluteChainWithAndWithoutRotor<N, M>::buildRandomClusterTreeModel() const
    {
        using namespace ClusterJoints;

        ClusterTreeModel<> model{};

        std::string prev_link_name = "ground";
        for (size_t i(0); i < N; i++)
        {
            // Link
            const std::string link_name = "link-" + std::to_string(i);
            const auto link_Xtree = spatial::randomSpatialRotation<double>();
            const auto link_inertia = randomLinkSpatialInertia();
            ori::CoordinateAxis link_axis = ori::randomCoordinateAxis<double>();
            auto link = model.registerBody(link_name, link_inertia, prev_link_name, link_Xtree);

            // Rotor
            const std::string rotor_name = "rotor-" + std::to_string(i);
            const auto rotor_Xtree = spatial::randomSpatialRotation<double>();
            const auto rotor_inertia = randomRotorSpatialInertia();
            ori::CoordinateAxis rotor_axis = ori::randomCoordinateAxis<double>();
            auto rotor = model.registerBody(rotor_name, rotor_inertia, prev_link_name, rotor_Xtree);

            // Cluster
            const std::string cluster_name = "cluster-" + std::to_string(i);
            GearedTransmissionModule<> module{link, rotor, link_axis, rotor_axis, randomGearRatio()};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor<>>(cluster_name, module);

            prev_link_name = link_name;
        }

        for (size_t i(N); i < N + M; i++)
        {
            // Link
            const std::string link_name = "link-" + std::to_string(i);
            const auto link_Xtree = spatial::randomSpatialRotation<double>();
            const auto link_inertia = randomLinkSpatialInertia();
            ori::CoordinateAxis link_joint_axis = ori::randomCoordinateAxis<double>();
            model.appendBody<ClusterJoints::Revolute<>>(link_name, link_inertia, prev_link_name,
                                                        link_Xtree, link_joint_axis);

            prev_link_name = link_name;
        }

        return model;
    }

    template <size_t N, size_t M>
    ClusterTreeModel<> RevoluteChainWithAndWithoutRotor<N, M>::buildUniformClusterTreeModel() const
    {
        throw std::runtime_error("Not implemented");
    }

    template class RevoluteChainWithAndWithoutRotor<0ul, 8ul>;
    template class RevoluteChainWithAndWithoutRotor<1ul, 7ul>;
    template class RevoluteChainWithAndWithoutRotor<2ul, 6ul>;
    template class RevoluteChainWithAndWithoutRotor<3ul, 5ul>;
    template class RevoluteChainWithAndWithoutRotor<4ul, 4ul>;
    template class RevoluteChainWithAndWithoutRotor<5ul, 3ul>;
    template class RevoluteChainWithAndWithoutRotor<6ul, 2ul>;
    template class RevoluteChainWithAndWithoutRotor<7ul, 1ul>;
    template class RevoluteChainWithAndWithoutRotor<8ul, 0ul>;

} // namespace grbda
