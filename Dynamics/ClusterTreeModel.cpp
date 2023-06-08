/*! @file ClusterTreeModel.cpp
 *
 */

#include "ClusterTreeModel.h"

namespace grbda
{

    using namespace ori;
    using namespace spatial;

    Body ClusterTreeModel::registerBody(const std::string name,
                                        const SpatialInertia<double> inertia,
                                        const std::string parent_name,
                                        const SpatialTransform Xtree)
    {
        body_name_to_body_index_.checkForKey(parent_name);

        const int body_index = (int)bodies_.size();
        body_name_to_body_index_[name] = body_index;

        const int parent_body_index = body_name_to_body_index_[parent_name];
        const int cluster_ancestor_index = getClusterAncestorIndexFromParent(parent_body_index);
        const int cluster_ancestor_sub_index_within_cluster =
            getSubIndexWithinClusterForBody(cluster_ancestor_index);

        auto body = Body(body_index, name, parent_body_index, Xtree, inertia,
                         (int)bodies_in_current_cluster_.size(),
                         cluster_ancestor_sub_index_within_cluster);

        bodies_.push_back(body);
        bodies_in_current_cluster_.push_back(body);
        return body;
    }

    void ClusterTreeModel::appendBody(const std::string name, const SpatialInertia<double> inertia,
                                      const std::string parent_name, const SpatialTransform Xtree,
                                      std::shared_ptr<GeneralizedJoints::Base> joint)
    {
        registerBody(name, inertia, parent_name, Xtree);
        appendRegisteredBodiesAsCluster(name, joint);
    }

    void ClusterTreeModel::appendRegisteredBodiesAsCluster(
        const std::string name, std::shared_ptr<GeneralizedJoints::Base> joint)
    {
        const int parent_cluster_index = getIndexOfParentClusterFromBodies(bodies_in_current_cluster_);
        const int num_bodies_in_parent_cluster = getNumBodiesInCluster(parent_cluster_index);

        const int cluster_index = (int)cluster_nodes_.size();
        cluster_name_to_cluster_index_[name] = cluster_index;

        auto node = std::make_shared<ClusterTreeNode>(cluster_index, name, bodies_in_current_cluster_,
                                                      joint, parent_cluster_index,
                                                      num_bodies_in_parent_cluster,
                                                      position_index_, velocity_index_);
        cluster_nodes_.push_back(node);
        nodes_.push_back(node);

        checkValidParentClusterForBodiesInCluster(cluster_nodes_.back());

        position_index_ += joint->numPositions();
        velocity_index_ += joint->numVelocities();

        resizeSystemMatrices();
        bodies_in_current_cluster_.clear();
    }

    void ClusterTreeModel::print() const
    {
        for (const auto &cluster : cluster_nodes_)
        {
            int parent = cluster->parent_index_;
            std::string parent_name = parent > -1 ? cluster_nodes_[parent]->name_ : "ground";
            printf("Cluster: %s (%s)\n", cluster->name_.c_str(), parent_name.c_str());

            for (const auto &body : cluster->bodies_)
            {
                parent = body.parent_index_;
                parent_name = parent > -1 ? bodies_[parent].name_ : "ground";
                printf("\t\t >%s (%s)\n", body.name_.c_str(), parent_name.c_str());
            }
        }
    }

    DMat<double> ClusterTreeModel::getMassMatrix()
    {
        compositeRigidBodyAlgorithm();
        return H_;
    }

    DVec<double> ClusterTreeModel::getBiasForceVector()
    {
        updateBiasForceVector();
        return C_;
    }

    void ClusterTreeModel::checkValidParentClusterForBodiesInCluster(const ClusterTreeNodePtr cluster)
    {
        const int cluster_index = cluster->index_;
        const int parent_cluster_index = cluster->parent_index_;
        for (size_t i = 0; i < cluster->bodies_.size(); i++)
        {
            int other_parent_cluster_index =
                getIndexOfClusterContainingBody(cluster->bodies_[i].parent_index_);
            if (other_parent_cluster_index != cluster_index &&
                other_parent_cluster_index != parent_cluster_index)
                throw std::runtime_error("The parents of all bodies in a cluster must have parents in the current cluster OR in the same parent cluster");
        }
    }

    void ClusterTreeModel::checkValidParentClusterForBodiesInCluster(const int cluster_index)
    {
        checkValidParentClusterForBodiesInCluster(cluster_nodes_[cluster_index]);
    }

    void ClusterTreeModel::checkValidParentClusterForBodiesInCluster(const std::string &cluster_name)
    {
        checkValidParentClusterForBodiesInCluster(cluster_nodes_[cluster_name_to_cluster_index_
                                                                     .at(cluster_name)]);
    }

    void ClusterTreeModel::resizeSystemMatrices()
    {
        const int num_degrees_of_freedom = getNumDegreesOfFreedom();

        H_ = DMat<double>::Zero(num_degrees_of_freedom, num_degrees_of_freedom);
        C_ = DVec<double>::Zero(num_degrees_of_freedom);

        for (auto &cluster : cluster_nodes_)
            cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd
                .setZero(num_degrees_of_freedom, cluster->num_velocities_);

        for (auto &contact_point : contact_points_)
            contact_point.jacobian_.setZero(6, num_degrees_of_freedom);
    }

    void ClusterTreeModel::appendContactPoint(const std::string body_name,
                                              const Vec3<double> &local_offset,
                                              const std::string contact_point_name)
    {
        body_name_to_body_index_.checkForKey(body_name);
        const int contact_point_index = (int)contact_points_.size();
        contact_name_to_contact_index_[contact_point_name] = contact_point_index;
        contact_points_.emplace_back(body_name_to_body_index_[body_name], local_offset,
                                     contact_point_name, getNumDegreesOfFreedom());
    }

    void ClusterTreeModel::initializeIndependentStates(const DVec<double> &y, const DVec<double> &yd)
    {
        for (auto &cluster : cluster_nodes_)
        {
            cluster->q_ = y.segment(cluster->position_index_, cluster->num_positions_);
            cluster->qd_ = yd.segment(cluster->velocity_index_, cluster->num_velocities_);
        }
        initializeExternalForces();
    }

    void ClusterTreeModel::resetCache()
    {
        TreeModel::resetCache();
        articulated_bodies_updated_ = false;
        force_propagators_updated_ = false;
        qdd_effects_updated_ = false;
    }

    int ClusterTreeModel::getClusterAncestorIndexFromParent(const int body_index)
    {
        int cluster_ancestor_index = body_index;
        while (!searchClustersForBody(cluster_ancestor_index) && cluster_ancestor_index != -1)
        {
            cluster_ancestor_index = bodies_[cluster_ancestor_index].parent_index_;
        }
        return cluster_ancestor_index;
    }

    int ClusterTreeModel::getSubIndexWithinClusterForBody(const int body_index) const
    {
        return body_index >= 0 ? bodies_[body_index].sub_index_within_cluster_ : 0;
    }

    int ClusterTreeModel::getSubIndexWithinClusterForBody(const Body &body) const
    {
        return getSubIndexWithinClusterForBody(body.index_);
    }

    int ClusterTreeModel::getSubIndexWithinClusterForBody(const std::string &body_name) const
    {
        return getSubIndexWithinClusterForBody(body_name_to_body_index_.at(body_name));
    }

    int ClusterTreeModel::getNumBodiesInCluster(const int cluster_index) const
    {
        return cluster_index >= 0 ? cluster_nodes_[cluster_index]->bodies_.size() : 1;
    }

    int ClusterTreeModel::getNumBodiesInCluster(const ClusterTreeNodePtr cluster) const
    {
        return getNumBodiesInCluster(cluster->index_);
    }

    int ClusterTreeModel::getNumBodiesInCluster(const std::string &cluster_name) const
    {
        return getNumBodiesInCluster(cluster_name_to_cluster_index_.at(cluster_name));
    }

    int ClusterTreeModel::getIndexOfParentClusterFromBodies(const std::vector<Body> &bodies)
    {
        int parent_cluster_index;
        bool parent_cluster_detected = false;

        // Check that at least one body has a parent from different cluster
        int i = 0;
        while (!parent_cluster_detected && i < (int)bodies.size())
        {
            try
            {
                parent_cluster_index = getIndexOfClusterContainingBody(bodies[i].parent_index_);
                parent_cluster_detected = true;
            }
            catch (...)
            {
                // Parent body is either in current cluster or does not belong to any cluster
                i++;
            }
        }

        // Error handling
        if (!parent_cluster_detected)
            throw std::runtime_error("At least one body in every cluster must have a parent in a different clusters");

        return parent_cluster_index;
    }

    int ClusterTreeModel::getIndexOfClusterContainingBody(const int body_index)
    {
        auto body_found_in_map = body_index_to_cluster_index_.find(body_index);
        if (body_found_in_map == body_index_to_cluster_index_.end())
        {
            auto cluster_index = searchClustersForBody(body_index);
            if (cluster_index)
                return cluster_index.value();
            else
                throw std::runtime_error("Body is not found in any registered cluster");
        }
        else
            return body_found_in_map->second;
    }

    int ClusterTreeModel::getIndexOfClusterContainingBody(const Body &body)
    {
        return getIndexOfClusterContainingBody(body.index_);
    }

    int ClusterTreeModel::getIndexOfClusterContainingBody(const std::string &body_name)
    {
        return getIndexOfClusterContainingBody(body_name_to_body_index_.at(body_name));
    }

    std::optional<int> ClusterTreeModel::searchClustersForBody(const int body_index)
    {
        for (size_t i = 0; i < cluster_nodes_.size(); i++)
            if (cluster_nodes_[i]->containsBody(body_index))
            {
                body_index_to_cluster_index_[body_index] = i;
                return i;
            }
        return std::nullopt;
    }

    ClusterTreeNodePtr ClusterTreeModel::getClusterContainingBody(const int body_index)
    {
        return cluster_nodes_[getIndexOfClusterContainingBody(body_index)];
    }

    ClusterTreeNodePtr ClusterTreeModel::getClusterContainingBody(const Body &body)
    {
        return cluster_nodes_[getIndexOfClusterContainingBody(body)];
    }

    ClusterTreeNodePtr ClusterTreeModel::getClusterContainingBody(const std::string &body_name)
    {
        return cluster_nodes_[getIndexOfClusterContainingBody(body_name)];
    }

    DVec<double> ClusterTreeModel::localCartesianForceAtPointToWorldPluckerForceOnCluster(
        const Vec3<double> &force, const ContactPoint &contact_point)
    {
        const auto &body = bodies_[contact_point.body_index_];
        auto cluster = getClusterContainingBody(contact_point.body_index_);

        const auto &Xa = cluster->Xa_.getTransformForOutputBody(body.sub_index_within_cluster_);
        Mat3<double> Rai = Xa.getRotation().transpose();
        SpatialTransform X_cartesian_to_plucker{Rai, contact_point.local_offset_};

        SVec<double> spatial_force = SVec<double>::Zero();
        spatial_force.tail<3>() = force;

        DVec<double> F = DVec<double>::Zero(cluster->motion_subspace_dimension_);
        F.segment<6>(body.sub_index_within_cluster_) =
            X_cartesian_to_plucker.inverseTransformForceVector(spatial_force);

        return F;
    }

} // namespace grbda
