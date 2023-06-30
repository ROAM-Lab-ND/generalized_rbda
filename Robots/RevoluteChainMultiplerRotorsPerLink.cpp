#include "RevoluteChainMultiplerRotorsPerLink.hpp"

namespace grbda
{

    template <size_t N, size_t M>
    ClusterTreeModel RevoluteChainMultipleRotorsPerLink<N, M>::buildRandomClusterTreeModel() const
    {
        ClusterTreeModel model{};

        std::string prev_link_name = "ground";
        for (size_t i(0); i < N; i++)
        {
            // Link
            const std::string link_name = "link-" + std::to_string(i);
            const auto link_Xtree = randomSpatialRotation<double>();
            const auto link_inertia = SpatialInertia<double>::createRandomInertia();
            const CoordinateAxis link_joint_axis = ori::randomCoordinateAxis<double>();
            auto link = model.registerBody(link_name, link_inertia, prev_link_name, link_Xtree);

            // Rotors
            std::vector<Body> rotors;
            std::vector<CoordinateAxis> rotor_axes;
            std::vector<double> gear_ratios;
            for (size_t j(0); j < M; j++)
            {
                const std::string rotor_name = "rotor-" + std::to_string(i) + "-" + std::to_string(j);
                const auto rotor_Xtree = randomSpatialRotation<double>();
                const auto rotor_inertia = SpatialInertia<double>::createRandomInertia();
                auto rotor = model.registerBody(rotor_name, rotor_inertia, prev_link_name, rotor_Xtree);
                rotors.push_back(rotor);

                rotor_axes.push_back(ori::randomCoordinateAxis<double>());
                gear_ratios.push_back(rand() % 5 + 1);
            }

            // Cluster
            auto cluster_joint = std::make_shared<GeneralizedJoints::RevoluteWithMultipleRotorsJoint>(
                link, rotors, link_joint_axis, rotor_axes, gear_ratios);
            const std::string cluster_name = "cluster-" + std::to_string(i);
            model.appendRegisteredBodiesAsCluster(cluster_name, cluster_joint);

            prev_link_name = link_name;
        }

        return model;
    }

    template <size_t N, size_t M>
    ClusterTreeModel RevoluteChainMultipleRotorsPerLink<N, M>::buildUniformClusterTreeModel() const
    {
        throw std::runtime_error("Not implemented");
    }

    template class RevoluteChainMultipleRotorsPerLink<2ul, 2ul>;
    template class RevoluteChainMultipleRotorsPerLink<4ul, 1ul>;
    template class RevoluteChainMultipleRotorsPerLink<4ul, 2ul>;
    template class RevoluteChainMultipleRotorsPerLink<4ul, 3ul>;
    template class RevoluteChainMultipleRotorsPerLink<4ul, 4ul>;
    template class RevoluteChainMultipleRotorsPerLink<4ul, 5ul>;
    template class RevoluteChainMultipleRotorsPerLink<4ul, 6ul>;
    template class RevoluteChainMultipleRotorsPerLink<4ul, 7ul>;
    template class RevoluteChainMultipleRotorsPerLink<8ul, 1ul>;
    template class RevoluteChainMultipleRotorsPerLink<8ul, 2ul>;
    template class RevoluteChainMultipleRotorsPerLink<8ul, 3ul>;
    template class RevoluteChainMultipleRotorsPerLink<8ul, 4ul>;
    template class RevoluteChainMultipleRotorsPerLink<8ul, 5ul>;
    template class RevoluteChainMultipleRotorsPerLink<8ul, 6ul>;
    template class RevoluteChainMultipleRotorsPerLink<8ul, 7ul>;

} // namespace grbda
