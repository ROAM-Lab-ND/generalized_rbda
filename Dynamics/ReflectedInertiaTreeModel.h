#pragma once

#include "ClusterTreeModel.h"
#include "Dynamics/Nodes/ReflectedInertiaTreeNode.h"

namespace grbda
{

    using namespace ori;
    using namespace spatial;

    using ReflectedInertiaTreeNodePtr = std::shared_ptr<ReflectedInertiaTreeNode>;

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class ReflectedInertiaTreeModel : public TreeModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReflectedInertiaTreeModel(const ClusterTreeModel &cluster_tree_model);

        int getNumBodies() const override { return (int)reflected_inertia_nodes_.size(); }

        const Body &getBody(int spanning_tree_index) const override;
        const TreeNodePtr getNodeContainingBody(int spanning_tree_index) override;
        int getIndexOfParentNodeForBody(const int spanning_tree_index);

        const DVec<int> &getIndependentCoordinateIndices() const { return independent_coord_indices_; }

        void initializeIndependentStates(const DVec<double> &y, const DVec<double> &yd) override;

        DVec<double> forwardDynamics(const DVec<double> &tau) override;
        DVec<double> forwardDynamicsHandC(const DVec<double> &tau);

        DMat<double> getMassMatrix() override;
        DVec<double> getBiasForceVector() override;

    private:
        void extractRigidBodiesAndJointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractIndependentCoordinatesFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractContactPointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);

        void resetCache() override;

        void updateArticulatedBodies();

        std::vector<ReflectedInertiaTreeNodePtr> reflected_inertia_nodes_;

        DMat<double> reflected_inertia_;

        DMat<double> spanning_tree_to_independent_coords_conversion_;
        DVec<int> independent_coord_indices_;

        bool articulated_bodies_updated_ = false;
    };

} // namespace grbda
