#ifndef GRBDA_CLUSTER_TREE_MODEL_H
#define GRBDA_CLUSTER_TREE_MODEL_H

#include <string>
#include <vector>
#include <stdio.h>
#include <stdexcept>

#include "TreeModel.h"
#include "Dynamics/Nodes/ClusterTreeNode.h"

namespace grbda
{
    using ClusterTreeNodePtr = std::shared_ptr<ClusterTreeNode>;

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class ClusterTreeModel : public TreeModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ClusterTreeModel()
        {
            body_name_to_body_index_["ground"] = -1;
            body_index_to_cluster_index_[-1] = -1;
        }
        ~ClusterTreeModel() {}

        // The standard process for appending a cluster to the tree is to register all the bodies 
        // in  given cluster and then append them as a cluster by specifying the type of cluster 
        // joint that connects them
        Body registerBody(const std::string name, const SpatialInertia<double> inertia,
                          const std::string parent_name, const spatial::Transform Xtree);

        template <typename T, typename... Args>
        void appendRegisteredBodiesAsCluster(const std::string name, Args&&... args)
        {
            auto cluster_joint = std::make_shared<T>(args...);
            appendRegisteredBodiesAsCluster(name, cluster_joint);
        }

        // Alternatively, this function can be used when appending individual bodies to the model
        template <typename T, typename... Args>
        void appendBody(const std::string name, const SpatialInertia<double> inertia,
                        const std::string parent_name, const spatial::Transform Xtree,
                        Args &&...args)
        {
            Body body = registerBody(name, inertia, parent_name, Xtree);
            std::shared_ptr<GeneralizedJoints::Base> joint = std::make_shared<T>(body, args...);
            appendRegisteredBodiesAsCluster(name, joint);
        }

        void appendContactPoint(const std::string body_name, const Vec3<double> &local_offset,
                                const std::string contact_name, const bool is_end_eff = false);
        void appendContactBox(const std::string body_name, const Vec3<double> &box_dimensions);
        void appendEndEffector(const std::string body_name, const Vec3<double> &local_offset,
                               const std::string end_effector_name);

        void print() const;

        void setState(const ModelState &model_state);

        int getNumBodies() const override { return (int)bodies_.size(); }

        // NOTE: A body's "cluster ancestor" is the nearest member in the body's supporting tree
        // that belongs to a different cluster. This function is only intended to be run when
        // registering bodies. At all other times, a bodies cluster ancestor should be accesses via
        // body.cluster_ancestor_index_
        int getClusterAncestorIndexFromParent(const int body_index);

        int getSubIndexWithinClusterForBody(const Body &body) const;
        int getSubIndexWithinClusterForBody(const int body_index) const;
        int getSubIndexWithinClusterForBody(const std::string &body_name) const;

        int getNumBodiesInCluster(const ClusterTreeNodePtr cluster) const;
        int getNumBodiesInCluster(const int cluster_index) const;
        int getNumBodiesInCluster(const std::string &cluster_name) const;

        int getIndexOfClusterContainingBody(const Body &body);
        int getIndexOfClusterContainingBody(const int body_index);
        int getIndexOfClusterContainingBody(const std::string &body_name);

        ClusterTreeNodePtr getClusterContainingBody(const Body &body);
        ClusterTreeNodePtr getClusterContainingBody(const int body_index);
        ClusterTreeNodePtr getClusterContainingBody(const std::string &body_name);

        int getIndexOfParentClusterFromBodies(const std::vector<Body> &bodies);

        const Body &getBody(int index) const override { return bodies_[index]; }
        const TreeNodePtr getNodeContainingBody(int index) override
        {
            return nodes_[getIndexOfClusterContainingBody(index)];
        }

        const std::vector<Body> &bodies() const { return bodies_; }
        const std::vector<ClusterTreeNodePtr> &clusters() const { return cluster_nodes_; }

        const Body &body(const int body_index) const { return bodies_[body_index]; }
        const Body &body(const std::string body_name) const
        {
            return bodies_[body_name_to_body_index_.at(body_name)];
        }

        const ClusterTreeNodePtr cluster(const int cluster_index) const { return cluster_nodes_[cluster_index]; }
        const ClusterTreeNodePtr cluster(const std::string &cluster_name) const
        {
            return cluster_nodes_[cluster_name_to_cluster_index_.at(cluster_name)];
        }

        Vec3<double> getPosition(const std::string &body_name) override;
        Mat3<double> getOrientation(const std::string &body_name) override;
        Vec3<double> getLinearVelocity(const std::string &body_name) override;
        Vec3<double> getAngularVelocity(const std::string &body_name) override;

        D6Mat<double> contactJacobianBodyFrame(const std::string &cp_name) override;
        const D6Mat<double> &contactJacobianWorldFrame(const std::string &cp_name) override;

        DVec<double> inverseDynamics(const DVec<double> &qdd) override;
        DVec<double> forwardDynamics(const DVec<double> &tau) override;
        DMat<double> inverseOperationalSpaceInertiaMatrix() override;
        double applyTestForce(const std::string &contact_point_name,
                              const Vec3<double> &force, DVec<double> &dstate_out) override;

        DMat<double> getMassMatrix() override;
        DVec<double> getBiasForceVector() override;

    protected:
        void appendRegisteredBodiesAsCluster(const std::string name,
                                             std::shared_ptr<GeneralizedJoints::Base> joint);

        void checkValidParentClusterForBodiesInCluster(const ClusterTreeNodePtr cluster);
        void checkValidParentClusterForBodiesInCluster(const int cluster_index);
        void checkValidParentClusterForBodiesInCluster(const std::string &cluster_nam);
        std::optional<int> searchClustersForBody(const int body_index);

        void resizeSystemMatrices();
        void resetCache() override;

        void updateArticulatedBodies();
        void updateForcePropagators();
        void updateQddEffects();

        DVec<double> localCartesianForceAtPointToWorldPluckerForceOnCluster(
            const Vec3<double> &force, const ContactPoint &contact_point);

        std::vector<Body> bodies_;
        std::vector<ClusterTreeNodePtr> cluster_nodes_;

        std::vector<Body> bodies_in_current_cluster_;

        std::unordered_map<std::string, int> body_name_to_body_index_;
        std::unordered_map<std::string, int> cluster_name_to_cluster_index_;
        std::unordered_map<int, int> body_index_to_cluster_index_;

        bool articulated_bodies_updated_ = false;
        bool force_propagators_updated_ = false;
        bool qdd_effects_updated_ = false;

        friend class RigidBodyTreeModel;
        friend class ReflectedInertiaTreeModel;
    };

} // namespace grbda

#endif // GRBDA_CLUSTER_TREE_MODEL_H
