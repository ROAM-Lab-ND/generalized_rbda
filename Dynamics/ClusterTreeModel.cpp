/*! @file ClusterTreeModel.cpp
 *
 */

#include "ClusterTreeModel.h"

namespace grbda
{

    using namespace ori;
    using namespace spatial;

    Body ClusterTreeModel::registerBody(const string name, const SpatialInertia<double> inertia,
                                        const string parent_name, const SpatialTransform Xtree)
    {
        const int body_index = (int)bodies_.size();
        body_name_to_body_index_[name] = body_index;

        const int parent_body_index = body_name_to_body_index_.at(parent_name);
        const int cluster_ancestor_index = getClusterAncestorIndexFromParent(parent_body_index);
        const int cluster_ancestor_sub_index_within_cluster =
            getSubIndexWithinClusterForBody(cluster_ancestor_index);

        auto body = Body(body_index, name, parent_body_index, Xtree, inertia,
                         (int)bodies_in_current_cluster_.size(),
                         cluster_ancestor_index,
                         cluster_ancestor_sub_index_within_cluster);

        bodies_.push_back(body);
        bodies_in_current_cluster_.push_back(body);
        return body;
    }

    // void ClusterTreeModel::appendBody(const string name, const SpatialInertia<double> inertia,
    //                                   const string parent_name, const SpatialTransform Xtree,
    //                                   shared_ptr<GeneralizedJoints::Base> joint)
    // {
    //     throw runtime_error("No longer valid");
    //     registerBody(name, inertia, parent_name, Xtree);
    //     appendRegisteredBodiesAsCluster(name, joint);
    // }

    void ClusterTreeModel::appendRegisteredBodiesAsCluster(
        const string name, shared_ptr<GeneralizedJoints::Base> joint)
    {
        const int parent_cluster_index = getIndexOfParentClusterFromBodies(bodies_in_current_cluster_);
        const int num_bodies_in_parent_cluster = getNumBodiesInCluster(parent_cluster_index);

        const int cluster_index = (int)cluster_nodes_.size();
        cluster_name_to_cluster_index_[name] = cluster_index;

        auto node = make_shared<ClusterTreeNode>(cluster_index, name, bodies_in_current_cluster_,
                                                 joint, parent_cluster_index,
                                                 num_bodies_in_parent_cluster,
                                                 position_index_, velocity_index_,
                                                 motion_subspace_index_);
        cluster_nodes_.push_back(node);
        nodes_.push_back(node);

        checkValidParentClusterForBodiesInCluster(cluster_nodes_.back());

        position_index_ += joint->numPositions();
        velocity_index_ += joint->numVelocities();
        motion_subspace_index_ += node->motion_subspace_dimension_;
        unactuated_dofs_ += joint->numUnactuatedVelocities();

        resizeSystemMatrices();
        bodies_in_current_cluster_.clear();
    }

    void ClusterTreeModel::print() const
    {
        printf("\nCluster Tree Model:\n");
        printf("** Clusters **\n");
        for (const auto &cluster : cluster_nodes_)
        {
            int parent = cluster->parent_index_;
            string parent_name = parent > -1 ? cluster_nodes_[parent]->name_ : "ground";
            printf("Cluster: %s (%s)\n", cluster->name_.c_str(), parent_name.c_str());

            for (const auto &body : cluster->bodies_)
            {
                parent = body.parent_index_;
                parent_name = parent > -1 ? bodies_[parent].name_ : "ground";
                printf("\t\t >%s (%s)\n", body.name_.c_str(), parent_name.c_str());
            }
        }

        printf("** Contact Points **\n");
        for (const auto &contact_point : contact_points_)
        {
            const int parent = contact_point.body_index_;
            string parent_name = bodies_.at(parent).name_;
            string type = contact_point.is_end_effector_ ? "End Effector" : "Contact Point";
            printf("%s: %s (%s)\n", type.c_str(), contact_point.name_.c_str(), parent_name.c_str());
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
                throw runtime_error("The parents of all bodies in a cluster must have parents in the current cluster OR in the same parent cluster");
        }
    }

    void ClusterTreeModel::checkValidParentClusterForBodiesInCluster(const int cluster_index)
    {
        checkValidParentClusterForBodiesInCluster(cluster_nodes_[cluster_index]);
    }

    void ClusterTreeModel::checkValidParentClusterForBodiesInCluster(const string &cluster_name)
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
        {
            contact_point.jacobian_.setZero(6, num_degrees_of_freedom);
            if (!contact_point.is_end_effector_)
                continue;
            contact_point.ChiUp_.push_back(DMat<double>::Zero(0, 0));
        }
    }

    void ClusterTreeModel::appendContactPoint(const string body_name,
                                              const Vec3<double> &local_offset,
                                              const string contact_point_name,
                                              const bool is_end_effector)
    {
        const int contact_point_index = (int)contact_points_.size();
        contact_name_to_contact_index_[contact_point_name] = contact_point_index;

        if (!is_end_effector)
        {
            contact_points_.emplace_back(body_name_to_body_index_.at(body_name), local_offset,
                                         contact_point_name, getNumDegreesOfFreedom());
            return;
        }

        contact_points_.emplace_back(body_name_to_body_index_.at(body_name), local_offset,
                                     contact_point_name, getNumDegreesOfFreedom(),
                                     num_end_effectors_++);
        ContactPoint &contact_point = contact_points_.back();

        // Keep track of which nodes support this new end effector
        int i = getIndexOfClusterContainingBody(body_name_to_body_index_.at(body_name));
        while (i > -1)
        {
            cluster_nodes_[i]->supported_end_effectors_.push_back(contact_point_index);
            contact_point.supporting_nodes_.push_back(i);
            i = cluster_nodes_[i]->parent_index_;
        }

        // Initialize the force propagators for this end effector
        for (int j = 0; j < (int)cluster_nodes_.size(); j++)
        {
            contact_point.ChiUp_.push_back(DMat<double>::Zero(0, 0));
        }

        // Get the nearest shared supporting cluster for every existing end effector
        for (int k = 0; k < (int)contact_points_.size() - 1; k++)
        {
            if (!contact_points_[k].is_end_effector_)
                continue; 
            std::pair<int, int> cp_pair(k, contact_point_index);
            const int nearest_shared_support = getNearestSharedSupportingNode(cp_pair);
            cluster_nodes_[nearest_shared_support]->nearest_supported_ee_pairs_.push_back(cp_pair);
        }
    }

    void ClusterTreeModel::appendContactBox(const string body_name, const Vec3<double> &dims)
    {
        using V3d = Vec3<double>;
        appendContactPoint(body_name, V3d(dims(0), dims(1), dims(2)) / 2, "torso-contact-1");
        appendContactPoint(body_name, V3d(-dims(0), dims(1), dims(2)) / 2, "torso-contact-2");
        appendContactPoint(body_name, V3d(dims(0), -dims(1), dims(2)) / 2, "torso-contact-3");
        appendContactPoint(body_name, V3d(-dims(0), -dims(1), dims(2)) / 2, "torso-contact-4");

        appendContactPoint(body_name, V3d(dims(0), dims(1), -dims(2)) / 2, "torso-contact-5");
        appendContactPoint(body_name, V3d(-dims(0), dims(1), -dims(2)) / 2, "torso-contact-6");
        appendContactPoint(body_name, V3d(dims(0), -dims(1), -dims(2)) / 2, "torso-contact-7");
        appendContactPoint(body_name, V3d(-dims(0), -dims(1), -dims(2)) / 2, "torso-contact-8");
    }

    void ClusterTreeModel::appendEndEffector(const string body_name,
                                             const Vec3<double> &local_offset,
                                             const string end_effector_name)
    {
        appendContactPoint(body_name, local_offset, end_effector_name, true);
    }

    void ClusterTreeModel::setState(const ModelState &model_state)
    {
        size_t i = 0;
        for (auto &cluster : cluster_nodes_)
        {
            cluster->joint_state_ = model_state.at(i);
            i++;
        }

        setExternalForces();
    }

    void ClusterTreeModel::resetCache()
    {
        TreeModel::resetCache();
        articulated_bodies_updated_ = false;
        force_propagators_updated_ = false;
        qdd_effects_updated_ = false;
    }

    Vec3<double> ClusterTreeModel::getPosition(const string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        forwardKinematics();
        const SpatialTransform &Xa = cluster_nodes_[cluster_idx]->Xa_[subindex_within_cluster];
        const Mat6<double> Xai = invertSXform(Xa.toMatrix().cast<double>());
        Vec3<double> link_pos = sXFormPoint(Xai, Vec3<double>::Zero());
        return link_pos;
    }

    Mat3<double> ClusterTreeModel::getOrientation(const string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        forwardKinematics();
        const SpatialTransform &Xa = cluster_nodes_[cluster_idx]->Xa_[subindex_within_cluster];
        Mat3<double> Rai = Xa.getRotation();
        Rai.transposeInPlace();
        return Rai;
    }

    Vec3<double> ClusterTreeModel::getLinearVelocity(const string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        forwardKinematics();
        const Mat3<double> Rai = getOrientation(body_name);
        const DVec<double> &v_cluster = cluster_nodes_[cluster_idx]->v_;
        const SVec<double> v = v_cluster.segment<6>(6 * subindex_within_cluster);
        return Rai * spatialToLinearVelocity(v, Vec3<double>::Zero());
    }

    Vec3<double> ClusterTreeModel::getAngularVelocity(const string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        forwardKinematics();
        const Mat3<double> Rai = getOrientation(body_name);
        const DVec<double> &v_cluster = cluster_nodes_[cluster_idx]->v_;
        const SVec<double> v = v_cluster.segment<6>(6 * subindex_within_cluster);
        return Rai * v.head<3>();
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

    int ClusterTreeModel::getSubIndexWithinClusterForBody(const string &body_name) const
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

    int ClusterTreeModel::getNumBodiesInCluster(const string &cluster_name) const
    {
        return getNumBodiesInCluster(cluster_name_to_cluster_index_.at(cluster_name));
    }

    int ClusterTreeModel::getIndexOfParentClusterFromBodies(const vector<Body> &bodies)
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
            throw runtime_error("At least one body in every cluster must have a parent in a different clusters");

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
                throw runtime_error("Body is not found in any registered cluster");
        }
        else
            return body_found_in_map->second;
    }

    int ClusterTreeModel::getIndexOfClusterContainingBody(const Body &body)
    {
        return getIndexOfClusterContainingBody(body.index_);
    }

    int ClusterTreeModel::getIndexOfClusterContainingBody(const string &body_name)
    {
        return getIndexOfClusterContainingBody(body_name_to_body_index_.at(body_name));
    }

    optional<int> ClusterTreeModel::searchClustersForBody(const int body_index)
    {
        for (size_t i = 0; i < cluster_nodes_.size(); i++)
            if (cluster_nodes_[i]->containsBody(body_index))
            {
                body_index_to_cluster_index_[body_index] = i;
                return i;
            }
        return nullopt;
    }

    ClusterTreeNodePtr ClusterTreeModel::getClusterContainingBody(const int body_index)
    {
        return cluster_nodes_[getIndexOfClusterContainingBody(body_index)];
    }

    ClusterTreeNodePtr ClusterTreeModel::getClusterContainingBody(const Body &body)
    {
        return cluster_nodes_[getIndexOfClusterContainingBody(body)];
    }

    ClusterTreeNodePtr ClusterTreeModel::getClusterContainingBody(const string &body_name)
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
        F.segment<6>(6 * body.sub_index_within_cluster_) =
            X_cartesian_to_plucker.inverseTransformForceVector(spatial_force);

        return F;
    }

} // namespace grbda
