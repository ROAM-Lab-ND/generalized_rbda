#ifndef GRBDA_URDF_MODEL_H
#define GRBDA_URDF_MODEL_H

#include <iostream>
#include <stack>
#include "urdf_parser/urdf_parser.h"

namespace grbda
{
    using UrdfCluster = std::vector<urdf::LinkSharedPtr>;

    class UrdfClusterModel : public urdf::ModelInterface
    {
    public:
        UrdfClusterModel(const urdf::ModelInterface &model) : urdf::ModelInterface(model)
        {
            clusterStronglyConnectedComponents();
        }

        const UrdfCluster &getClusterContaining(const urdf::LinkConstSharedPtr &link) const;

        std::vector<UrdfCluster> getChildClusters(const UrdfCluster &cluster) const;

    private:
        void clusterStronglyConnectedComponents();

        void dfsFirstPass(const std::string &link_name,
                          std::map<std::string, bool> &visited,
                          std::stack<std::string> &finishing_order);

        void dfsSecondPass(const std::map<std::string, UrdfCluster> &reverse_graph,
                           const std::string &link_name,
                           std::map<std::string, bool> &visited, UrdfCluster &scc);

        std::vector<UrdfCluster> clusters_;
    };

} // namespace grbda

#endif
