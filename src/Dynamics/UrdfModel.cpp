#include "grbda/Dynamics/UrdfModel.h"

namespace grbda
{
    void UrdfClusterModel::clusterStronglyConnectedComponents()
    {
        // Clustering
        {
            std::map<std::string, bool> visited;
            std::stack<std::string> finishing_order;

            // Build the reverse graph
            std::map<std::string, UrdfCluster> reverse_link_graph;
            for (const auto &[link_name, link] : this->links_)
            {
                reverse_link_graph[link->name] = UrdfCluster();
            }
            for (const auto &[link_name, link] : this->links_)
            {
                for (urdf::LinkSharedPtr &child : link->child_links)
                {
                    reverse_link_graph[child->name].push_back(link);
                }
                for (urdf::LinkSharedPtr &loop_link : link->loop_links)
                {
                    reverse_link_graph[loop_link->name].push_back(link);
                }
            }
            std::cout << "Built reverse graph" << std::endl;

            // Print the reverse graph
            for (const auto &[link_name, neighbors] : reverse_link_graph)
            {
                std::cout << link_name << " -> ";
                for (const urdf::LinkSharedPtr &neighbor : neighbors)
                {
                    std::cout << neighbor->name << ", ";
                }
                std::cout << std::endl;
            }

            // First Pass: Calculate finishing times
            for (const auto &[link_name, link] : this->links_)
            {
                if (!visited[link->name])
                {
                    dfsFirstPass(link->name, visited, finishing_order);
                }
            }
            std::cout << "Finished first pass" << std::endl;

            visited.clear();

            // Second Pass: Build SCCs
            while (!finishing_order.empty())
            {
                const std::string &link_name = finishing_order.top();
                finishing_order.pop();

                if (!visited[link_name])
                {
                    UrdfCluster cluster;
                    dfsSecondPass(reverse_link_graph, link_name, visited, cluster);
                    clusters_.push_back(cluster);
                }
            }
            std::cout << "Finished second pass" << std::endl;
        }
    }

    void UrdfClusterModel::dfsFirstPass(const std::string &link_name,
                                 std::map<std::string, bool> &visited,
                                 std::stack<std::string> &finishing_order)
    {
        visited[link_name] = true;
        for (const urdf::LinkSharedPtr &child : this->links_[link_name]->child_links)
        {
            const std::string &child_name = child->name;
            if (!visited[child_name])
            {
                dfsFirstPass(child_name, visited, finishing_order);
            }
        }
        for (const urdf::LinkSharedPtr &loop_link : this->links_[link_name]->loop_links)
        {
            const std::string &loop_link_name = loop_link->name;
            if (!visited[loop_link_name])
            {
                dfsFirstPass(loop_link_name, visited, finishing_order);
            }
        }
        finishing_order.push(link_name);
    }

    void UrdfClusterModel::dfsSecondPass(const std::map<std::string, UrdfCluster> &reverse_graph,
                                  const std::string &link_name,
                                  std::map<std::string, bool> &visited, UrdfCluster &scc)
    {
        std::cout << "Visiting " << link_name << std::endl;
        visited[link_name] = true;
        scc.push_back(this->links_[link_name]);
        for (const urdf::LinkSharedPtr &neighbor : reverse_graph.at(link_name))
        {
            std::cout << "Neighbor: " << neighbor->name << std::endl;
            const std::string &neighbor_name = neighbor->name;
            if (!visited[neighbor_name])
            {
                dfsSecondPass(reverse_graph, neighbor_name, visited, scc);
            }
        }
    }

    const UrdfCluster &
    UrdfClusterModel::getClusterContaining(const urdf::LinkConstSharedPtr &link) const
    {
        for (const UrdfCluster &cluster : clusters_)
        {
            if (std::find(cluster.begin(), cluster.end(), link) != cluster.end())
            {
                return cluster;
            }
        }
        throw std::runtime_error("Link not found in any cluster");
    }

    std::vector<UrdfCluster> UrdfClusterModel::getChildClusters(const UrdfCluster &cluster) const
    {
        std::vector<UrdfCluster> child_clusters;
        for (const urdf::LinkSharedPtr &link : cluster)
        {
            for (const urdf::LinkSharedPtr &child : link->child_links)
            {
                const UrdfCluster &child_cluster = getClusterContaining(child);

                if (child_cluster == cluster)
                {
                    continue;
                }

                if (std::find(child_clusters.begin(), child_clusters.end(), child_cluster) == child_clusters.end())
                {
                    child_clusters.push_back(child_cluster);
                }
            }
        }
        return child_clusters;
    }

} // namespace grbda
