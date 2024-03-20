/*! @file ClusterTreeModel.cpp
 *
 */

#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{
    template <typename Scalar>
    Body<Scalar> ClusterTreeModel<Scalar>::registerBody(const std::string name,
                                                        const SpatialInertia<Scalar> inertia,
                                                        const std::string parent_name,
                                                        const spatial::Transform<Scalar> Xtree)
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

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendRegisteredBodiesAsCluster(
        const std::string name, std::shared_ptr<ClusterJoints::Base<Scalar>> joint)
    {
        const int parent_cluster_index = getIndexOfParentClusterFromBodies(bodies_in_current_cluster_);
        const int num_bodies_in_parent_cluster = getNumBodiesInCluster(parent_cluster_index);

        const int cluster_index = (int)cluster_nodes_.size();
        cluster_name_to_cluster_index_[name] = cluster_index;

        auto node = std::make_shared<ClusterTreeNode<Scalar>>(cluster_index, name,
                                                              bodies_in_current_cluster_,
                                                              joint, parent_cluster_index,
                                                              num_bodies_in_parent_cluster,
                                                              this->position_index_,
                                                              this->velocity_index_,
                                                              this->motion_subspace_index_);
        cluster_nodes_.push_back(node);
        this->nodes_.push_back(node);

        checkValidParentClusterForBodiesInCluster(cluster_nodes_.back());

        // TODO(@MatthewChignoli): Much of the code is built to support spanning or independent 
        // representations of the joint state. However, this method of assigning the position and 
        // velocity indices determine whether the cluster tree model is compatible with spanning or 
        // non spanning joints states. Needs to be changed if we want to support both options
        this->position_index_ += joint->numPositions();
        this->velocity_index_ += joint->numVelocities();
        this->motion_subspace_index_ += node->motion_subspace_dimension_;
        this->unactuated_dofs_ += joint->numUnactuatedVelocities();

        resizeSystemMatrices();
        bodies_in_current_cluster_.clear();
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::print() const
    {
        printf("\nCluster Tree Model:\n");
        printf("** Clusters **\n");
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

        printf("** Contact Points **\n");
        for (const auto &contact_point : this->contact_points_)
        {
            const int parent = contact_point.body_index_;
            std::string parent_name = bodies_.at(parent).name_;
            std::string type = contact_point.is_end_effector_ ? "End Effector" : "Contact Point";
            printf("%s: %s (%s)\n", type.c_str(), contact_point.name_.c_str(), parent_name.c_str());
        }
    }

    template <typename Scalar>
    DMat<Scalar> ClusterTreeModel<Scalar>::getMassMatrix()
    {
        this->compositeRigidBodyAlgorithm();
        return this->H_;
    }

    template <typename Scalar>
    DVec<Scalar> ClusterTreeModel<Scalar>::getBiasForceVector()
    {
        this->updateBiasForceVector();
        return this->C_;
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::checkValidParentClusterForBodiesInCluster(
        const ClusterTreeNodePtr<Scalar> cluster)
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

    template <typename Scalar>
    void
    ClusterTreeModel<Scalar>::checkValidParentClusterForBodiesInCluster(const int cluster_index)
    {
        checkValidParentClusterForBodiesInCluster(cluster_nodes_[cluster_index]);
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::checkValidParentClusterForBodiesInCluster(
        const std::string &cluster_name)
    {
        checkValidParentClusterForBodiesInCluster(cluster_nodes_[cluster_name_to_cluster_index_
                                                                     .at(cluster_name)]);
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::resizeSystemMatrices()
    {
        const int num_degrees_of_freedom = this->getNumDegreesOfFreedom();

        this->H_ = DMat<Scalar>::Zero(num_degrees_of_freedom, num_degrees_of_freedom);
        this->C_ = DVec<Scalar>::Zero(num_degrees_of_freedom);

        for (auto &cluster : cluster_nodes_)
            cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd
                .setZero(num_degrees_of_freedom, cluster->num_velocities_);

        for (auto &contact_point : this->contact_points_)
        {
            contact_point.jacobian_.setZero(6, num_degrees_of_freedom);
            if (!contact_point.is_end_effector_)
                continue;
            contact_point.ChiUp_.push_back(DMat<Scalar>::Zero(0, 0));
        }
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendContactPoint(const std::string body_name,
                                                      const Vec3<Scalar> &local_offset,
                                                      const std::string contact_point_name,
                                                      const bool is_end_effector)
    {
        const int contact_point_index = (int)this->contact_points_.size();
        this->contact_name_to_contact_index_[contact_point_name] = contact_point_index;

        if (!is_end_effector)
        {
            this->contact_points_.emplace_back(body_name_to_body_index_.at(body_name), local_offset,
                                               contact_point_name, this->getNumDegreesOfFreedom());
            return;
        }

        this->contact_points_.emplace_back(body_name_to_body_index_.at(body_name), local_offset,
                                           contact_point_name, this->getNumDegreesOfFreedom(),
                                           this->num_end_effectors_++);
        ContactPoint<Scalar> &contact_point = this->contact_points_.back();

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
            contact_point.ChiUp_.push_back(DMat<Scalar>::Zero(0, 0));
        }

        // Get the nearest shared supporting cluster for every existing end effector
        for (int k = 0; k < (int)this->contact_points_.size() - 1; k++)
        {
            if (!this->contact_points_[k].is_end_effector_)
                continue;
            std::pair<int, int> cp_pair(k, contact_point_index);
            const int nearest_shared_support = this->getNearestSharedSupportingNode(cp_pair);
            cluster_nodes_[nearest_shared_support]->nearest_supported_ee_pairs_.push_back(cp_pair);
        }
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendContactBox(const std::string body_name,
                                                    const Vec3<Scalar> &dims)
    {
        using V3d = Vec3<Scalar>;
        appendContactPoint(body_name, V3d(dims(0), dims(1), dims(2)) / 2, "torso-contact-1");
        appendContactPoint(body_name, V3d(-dims(0), dims(1), dims(2)) / 2, "torso-contact-2");
        appendContactPoint(body_name, V3d(dims(0), -dims(1), dims(2)) / 2, "torso-contact-3");
        appendContactPoint(body_name, V3d(-dims(0), -dims(1), dims(2)) / 2, "torso-contact-4");

        appendContactPoint(body_name, V3d(dims(0), dims(1), -dims(2)) / 2, "torso-contact-5");
        appendContactPoint(body_name, V3d(-dims(0), dims(1), -dims(2)) / 2, "torso-contact-6");
        appendContactPoint(body_name, V3d(dims(0), -dims(1), -dims(2)) / 2, "torso-contact-7");
        appendContactPoint(body_name, V3d(-dims(0), -dims(1), -dims(2)) / 2, "torso-contact-8");
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendEndEffector(const std::string body_name,
                                                     const Vec3<Scalar> &local_offset,
                                                     const std::string end_effector_name)
    {
        appendContactPoint(body_name, local_offset, end_effector_name, true);
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::setState(const ModelState<Scalar> &model_state)
    {
        size_t i = 0;
        for (auto &cluster : cluster_nodes_)
        {
            cluster->joint_state_ = model_state.at(i);
            i++;
        }

        this->setExternalForces();
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::setState(const StatePair &q_qd_pair)
    {
        ModelState<Scalar> state = stateVectorToModelState(q_qd_pair);
        setState(state);
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::setState(const DVec<Scalar> &q_qd_vec)
    {
        const int nq = this->getNumPositions();
        const int nv = this->getNumDegreesOfFreedom();

        StatePair q_qd_pair = {q_qd_vec.segment(0, nq), q_qd_vec.segment(nq, nv)};
        ModelState<Scalar> state = stateVectorToModelState(q_qd_pair);
        setState(state);
    }

    template <typename Scalar>
    // NOTE: This function is only for non-spanning joint coordinates
    ModelState<Scalar> ClusterTreeModel<Scalar>::stateVectorToModelState(const StatePair &q_qd_pair)
    {

        ModelState<Scalar> state;

        for (const auto &cluster : cluster_nodes_)
        {
            DVec<Scalar> q_cluster = q_qd_pair.first.segment(cluster->position_index_,
                                                             cluster->num_positions_);
            const int &vel_idx = cluster->velocity_index_;
            const int &num_vel = cluster->num_velocities_;
            DVec<Scalar> qd_cluster = q_qd_pair.second.segment(vel_idx, num_vel);

            JointState<Scalar> joint_state(JointCoordinate<Scalar>(q_cluster, false),
                                           JointCoordinate<Scalar>(qd_cluster, false));
            state.push_back(joint_state);
        }

        return state;
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::resetCache()
    {
        TreeModel<Scalar>::resetCache();
        articulated_bodies_updated_ = false;
        force_propagators_updated_ = false;
        qdd_effects_updated_ = false;
    }

    template <typename Scalar>
    Vec3<Scalar> ClusterTreeModel<Scalar>::getPosition(const std::string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        this->forwardKinematics();
        const spatial::Transform<Scalar> &Xa =
            cluster_nodes_[cluster_idx]->Xa_[subindex_within_cluster];
        const Mat6<Scalar> Xai = spatial::invertSXform(Xa.toMatrix().template cast<Scalar>());
        Vec3<Scalar> link_pos = spatial::sXFormPoint(Xai, Vec3<Scalar>::Zero());
        return link_pos;
    }

    template <typename Scalar>
    Mat3<Scalar> ClusterTreeModel<Scalar>::getOrientation(const std::string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        this->forwardKinematics();
        const spatial::Transform<Scalar> &Xa =
            cluster_nodes_[cluster_idx]->Xa_[subindex_within_cluster];
        Mat3<Scalar> Rai = Xa.getRotation();
        Rai.transposeInPlace();
        return Rai;
    }

    template <typename Scalar>
    Vec3<Scalar> ClusterTreeModel<Scalar>::getLinearVelocity(const std::string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        this->forwardKinematics();
        const Mat3<Scalar> Rai = getOrientation(body_name);
        const DVec<Scalar> &v_cluster = cluster_nodes_[cluster_idx]->v_;
        const SVec<Scalar> v = v_cluster.template segment<6>(6 * subindex_within_cluster);
        return Rai * spatial::spatialToLinearVelocity(v, Vec3<Scalar>::Zero());
    }

    template <typename Scalar>
    Vec3<Scalar> ClusterTreeModel<Scalar>::getAngularVelocity(const std::string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        this->forwardKinematics();
        const Mat3<Scalar> Rai = getOrientation(body_name);
        const DVec<Scalar> &v_cluster = cluster_nodes_[cluster_idx]->v_;
        const SVec<Scalar> v = v_cluster.template segment<6>(6 * subindex_within_cluster);
        return Rai * v.template head<3>();
    }

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getClusterAncestorIndexFromParent(const int body_index)
    {
        int cluster_ancestor_index = body_index;
        while (!searchClustersForBody(cluster_ancestor_index) && cluster_ancestor_index != -1)
        {
            cluster_ancestor_index = bodies_[cluster_ancestor_index].parent_index_;
        }
        return cluster_ancestor_index;
    }

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getSubIndexWithinClusterForBody(const int body_index) const
    {
        return body_index >= 0 ? bodies_[body_index].sub_index_within_cluster_ : 0;
    }

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getSubIndexWithinClusterForBody(const Body<Scalar> &body) const
    {
        return getSubIndexWithinClusterForBody(body.index_);
    }

    template <typename Scalar>
    int
    ClusterTreeModel<Scalar>::getSubIndexWithinClusterForBody(const std::string &body_name) const
    {
        return getSubIndexWithinClusterForBody(body_name_to_body_index_.at(body_name));
    }

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getNumBodiesInCluster(const int cluster_index) const
    {
        return cluster_index >= 0 ? cluster_nodes_[cluster_index]->bodies_.size() : 1;
    }

    template <typename Scalar>
    int
    ClusterTreeModel<Scalar>::getNumBodiesInCluster(const ClusterTreeNodePtr<Scalar> cluster) const
    {
        return getNumBodiesInCluster(cluster->index_);
    }

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getNumBodiesInCluster(const std::string &cluster_name) const
    {
        return getNumBodiesInCluster(cluster_name_to_cluster_index_.at(cluster_name));
    }

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getIndexOfParentClusterFromBodies(
        const std::vector<Body<Scalar>> &bodies)
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

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getIndexOfClusterContainingBody(const int body_index)
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

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getIndexOfClusterContainingBody(const Body<Scalar> &body)
    {
        return getIndexOfClusterContainingBody(body.index_);
    }

    template <typename Scalar>
    int ClusterTreeModel<Scalar>::getIndexOfClusterContainingBody(const std::string &body_name)
    {
        return getIndexOfClusterContainingBody(body_name_to_body_index_.at(body_name));
    }

    template <typename Scalar>
    std::optional<int> ClusterTreeModel<Scalar>::searchClustersForBody(const int body_index)
    {
        for (size_t i = 0; i < cluster_nodes_.size(); i++)
            if (cluster_nodes_[i]->containsBody(body_index))
            {
                body_index_to_cluster_index_[body_index] = i;
                return i;
            }
        return std::nullopt;
    }

    template <typename Scalar>
    ClusterTreeNodePtr<Scalar>
    ClusterTreeModel<Scalar>::getClusterContainingBody(const int body_index)
    {
        return cluster_nodes_[getIndexOfClusterContainingBody(body_index)];
    }

    template <typename Scalar>
    ClusterTreeNodePtr<Scalar>
    ClusterTreeModel<Scalar>::getClusterContainingBody(const Body<Scalar> &body)
    {
        return cluster_nodes_[getIndexOfClusterContainingBody(body)];
    }

    template <typename Scalar>
    ClusterTreeNodePtr<Scalar>
    ClusterTreeModel<Scalar>::getClusterContainingBody(const std::string &body_name)
    {
        return cluster_nodes_[getIndexOfClusterContainingBody(body_name)];
    }

    template <typename Scalar>
    DVec<Scalar> ClusterTreeModel<Scalar>::localCartesianForceAtPointToWorldPluckerForceOnCluster(
        const Vec3<Scalar> &force, const ContactPoint<Scalar> &contact_point)
    {
        const auto &body = bodies_[contact_point.body_index_];
        auto cluster = getClusterContainingBody(contact_point.body_index_);

        const auto &Xa = cluster->Xa_.getTransformForOutputBody(body.sub_index_within_cluster_);
        Mat3<Scalar> Rai = Xa.getRotation().transpose();
        spatial::Transform X_cartesian_to_plucker{Rai, contact_point.local_offset_};

        SVec<Scalar> spatial_force = SVec<Scalar>::Zero();
        spatial_force.template tail<3>() = force;

        DVec<Scalar> F = DVec<Scalar>::Zero(cluster->motion_subspace_dimension_);
        F.template segment<6>(6 * body.sub_index_within_cluster_) =
            X_cartesian_to_plucker.inverseTransformForceVector(spatial_force);

        return F;
    }

    template class ClusterTreeModel<double>;
    template class ClusterTreeModel<float>;
    template class ClusterTreeModel<casadi::SX>;

} // namespace grbda
