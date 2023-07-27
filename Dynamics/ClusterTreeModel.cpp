/*! @file ClusterTreeModel.cpp
 *
 */

#include "ClusterTreeModel.h"

namespace grbda
{

    using namespace ori;
    using namespace spatial;
    using namespace ClusterNodeVisitors;

    Body ClusterTreeModel::registerBody(const string name, const SpatialInertia<double> inertia,
                                        const string parent_name, const SpatialTransform Xtree)
    {
        const int body_index = (int)bodies_.size();
        body_name_to_body_index_[name] = body_index;

        const int parent_body_index = body_name_to_body_index_.at(parent_name);
        const int cluster_ancestor_index = getClusterAncestorIndexFromParent(parent_body_index);
        const int cluster_ancestor_sub_index_within_cluster =
            getSubIndexWithinClusterForBody(cluster_ancestor_index);

        Body body = Body(body_index, name, parent_body_index, Xtree, inertia,
                         (int)bodies_in_current_cluster_.size(),
                         cluster_ancestor_index,
                         cluster_ancestor_sub_index_within_cluster);

        bodies_.push_back(body);
        bodies_in_current_cluster_.push_back(body);
        return body;
    }

    // TODO(@MatthewChignoli): This function will eventually be templated on the type of joint
    void ClusterTreeModel::appendRegisteredBodiesAsCluster(
        const string name, shared_ptr<GeneralizedJoints::Base> joint)
    {
        const int parent_cluster_index =
            getIndexOfParentClusterFromBodies(bodies_in_current_cluster_);
        const int num_bodies_in_parent_cluster = getNumBodiesInCluster(parent_cluster_index);

        const int cluster_index = (int)nodes_variants_.size();
        cluster_name_to_cluster_index_[name] = cluster_index;

        ClusterTreeNode<> node(cluster_index, name, bodies_in_current_cluster_, joint,
                               parent_cluster_index, num_bodies_in_parent_cluster,
                               position_index_, velocity_index_);
        nodes_variants_.push_back(node);

        checkValidParentClusterForBodiesInCluster(nodes_variants_.back());

        position_index_ += joint->numPositions();
        velocity_index_ += joint->numVelocities();
        unactuated_dofs_ += joint->numUnactuatedVelocities();

        resizeSystemMatrices();
        bodies_in_current_cluster_.clear();
    }

    void ClusterTreeModel::print() const
    {
        printf("\nCluster Tree Model:\n");
        printf("** Clusters **\n");

        // New Version
        for (const NodeTypeVariants &cluster : nodes_variants_)
        {
            int parent = parentIndex(cluster);
            string parent_name = parent > -1 ? name(nodes_variants_[parent])
                                             : "ground";

            printf("Cluster: %s (%s)\n", name(cluster).c_str(), parent_name.c_str());

            for (const Body &body : ClusterNodeVisitors::bodies(cluster))
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

    void ClusterTreeModel::checkValidParentClusterForBodiesInCluster(const NodeTypeVariants &cluster)
    {
        const int cluster_index = index(cluster);
        const int parent_cluster_index = parentIndex(cluster);
        const std::vector<Body> &bodies = ClusterNodeVisitors::bodies(cluster);
        for (size_t i = 0; i < bodies.size(); i++)
        {
            int other_parent_cluster_index =
                getIndexOfClusterContainingBody(bodies[i].parent_index_);
            if (other_parent_cluster_index != cluster_index &&
                other_parent_cluster_index != parent_cluster_index)
                throw runtime_error("The parents of all bodies in a cluster must have parents in the current cluster OR in the same parent cluster");
        }
    }

    void ClusterTreeModel::resizeSystemMatrices()
    {
        const int num_degrees_of_freedom = getNumDegreesOfFreedom();

        H_ = DMat<double>::Zero(num_degrees_of_freedom, num_degrees_of_freedom);
        C_ = DVec<double>::Zero(num_degrees_of_freedom);

        for (NodeTypeVariants &cluster_var : nodes_variants_)
        {
            qddSubtree(cluster_var).setZero(num_degrees_of_freedom, numVelocities(cluster_var));
        }

        for (ContactPoint &contact_point : contact_points_)
            contact_point.jacobian_.setZero(6, num_degrees_of_freedom);
    }

    void ClusterTreeModel::appendContactPoint(const string body_name,
                                              const Vec3<double> &local_offset,
                                              const string contact_point_name)
    {
        const int contact_point_index = (int)contact_points_.size();
        contact_name_to_contact_index_[contact_point_name] = contact_point_index;
        contact_points_.emplace_back(body_name_to_body_index_.at(body_name), local_offset,
                                     contact_point_name, getNumDegreesOfFreedom());
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

    void ClusterTreeModel::addJointLim(size_t jointID, double joint_lim_value_lower,
                                       double joint_lim_value_upper)
    {

        _JointLimID.push_back(jointID);
        _JointLimValueLower.push_back(joint_lim_value_lower);
        _JointLimValueUpper.push_back(joint_lim_value_upper);
        _nJointLim++;
    }

    void ClusterTreeModel::setState(const DVec<double> &state)
    {
        const int nq = getNumPositions();
        const int nv = getNumDegreesOfFreedom();

#ifdef DEBUG_MODE
        if (state.size() != nq + nv)
            throw runtime_error("State vector has incorrect size");
#endif

        ModelState model_state;
        int pos_idx = 0;
        int vel_idx = nq;
        for (size_t i(0); i < clusterVariants().size(); i++)
        {
            const auto &joint = getJoint(clusterVariant(i));

            const int &num_pos = joint->numPositions();
            const int &num_vel = joint->numVelocities();

            JointState joint_state(joint->positionIsSpanning(),
                                   joint->velocityIsSpanning());
            joint_state.position = state.segment(pos_idx, num_pos);
            joint_state.velocity = state.segment(vel_idx, num_vel);
            model_state.push_back(joint_state);

            pos_idx += num_pos;
            vel_idx += num_vel;
        }

        initializeState(model_state);
    }

    void ClusterTreeModel::initializeState(const ModelState &model_state)
    {
        size_t i = 0;
        for (NodeTypeVariants &cluster_var : nodes_variants_)
        {
            setJointState(cluster_var, model_state.at(i));
            i++;
        }

        resetExternalForces();
    }

    void ClusterTreeModel::resetExternalForces()
    {
        for (const int index : indices_of_nodes_experiencing_external_forces_)
            externalForce(nodes_variants_[index]).setZero();
        indices_of_nodes_experiencing_external_forces_.clear();

        resetCache();
    }

    void ClusterTreeModel::setExternalForces(const string &body_name, const SVec<double> &force)
    {
        const Body &body_i = body(body_name);
        NodeTypeVariants &node_var = nodes_variants_[getIndexOfClusterContainingBody(body_i.index_)];
        applyForceToBody(node_var, force, body_i);

        // Add index to vector if vector does not already contain this cluster
        if (!vectorContainsIndex(indices_of_nodes_experiencing_external_forces_, index(node_var)))
            indices_of_nodes_experiencing_external_forces_.push_back(index(node_var));
    }

    void ClusterTreeModel::setExternalForces(const unordered_map<string, SVec<double>> &ext_forces)
    {
        for (const auto &ext_force : ext_forces)
        {
            const string &body_name = ext_force.first;
            const SVec<double> &force = ext_force.second;
            setExternalForces(body_name, force);
        }
    }

    void ClusterTreeModel::resetCache()
    {
        kinematics_updated_ = false;
        mass_matrix_updated_ = false;
        bias_force_updated_ = false;
        articulated_bodies_updated_ = false;
        force_propagators_updated_ = false;
        qdd_effects_updated_ = false;
    }

    Vec3<double> ClusterTreeModel::getPosition(const string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        forwardKinematics();
        const SpatialTransform &X = Xa(nodes_variants_[cluster_idx])[subindex_within_cluster];
        const Mat6<double> Xai = invertSXform(X.toMatrix());
        Vec3<double> link_pos = sXFormPoint(Xai, Vec3<double>::Zero());
        return link_pos;
    }

    Mat3<double> ClusterTreeModel::getOrientation(const string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        forwardKinematics();
        const SpatialTransform &X = Xa(nodes_variants_[cluster_idx])[subindex_within_cluster];
        Mat3<double> Rai = X.getRotation();
        Rai.transposeInPlace();
        return Rai;
    }

    Vec3<double> ClusterTreeModel::getLinearVelocity(const string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        forwardKinematics();
        const Mat3<double> Rai = getOrientation(body_name);
        const DVec<double> &v_cluster = velocity(nodes_variants_[cluster_idx]);
        const SVec<double> v = v_cluster.segment<6>(6 * subindex_within_cluster);
        return Rai * spatialToLinearVelocity(v, Vec3<double>::Zero());
    }

    Vec3<double> ClusterTreeModel::getAngularVelocity(const string &body_name)
    {
        const int cluster_idx = getIndexOfClusterContainingBody(body_name);
        const int subindex_within_cluster = body(body_name).sub_index_within_cluster_;

        forwardKinematics();
        const Mat3<double> Rai = getOrientation(body_name);
        const DVec<double> &v_cluster = velocity(nodes_variants_[cluster_idx]);
        const SVec<double> v = v_cluster.segment<6>(6 * subindex_within_cluster);
        return Rai * v.head<3>();
    }

    const std::unordered_map<std::string, int> &ClusterTreeModel::contacts() const
    {
        return contact_name_to_contact_index_;
    }

    const Vec3<double> &ClusterTreeModel::pGC(const string &cp_name) const
    {
        return contactPoint(cp_name).position_;
    }

    const Vec3<double> &ClusterTreeModel::vGC(const string &cp_name) const
    {
        return contactPoint(cp_name).velocity_;
    }

    const string &ClusterTreeModel::gcParent(const string &cp_name) const
    {
        const int &body_index = contactPoint(cp_name).body_index_;
        return bodies_.at(body_index).name_;
    }

    D3Mat<double> ClusterTreeModel::Jc(const string &cp_name) const
    {
        return contactPoint(cp_name).jacobian_.bottomRows<3>();
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

    int ClusterTreeModel::getNumBodiesInCluster(const int cluster_index) const
    {
        return cluster_index >= 0 ? ClusterNodeVisitors::bodies(nodes_variants_[cluster_index]).size() : 1;
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
            optional<int> cluster_index = searchClustersForBody(body_index);
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
        for (size_t i = 0; i < nodes_variants_.size(); i++)
            if (containsBody(nodes_variants_[i], body_index))
            {
                body_index_to_cluster_index_[body_index] = i;
                return i;
            }
        return nullopt;
    }

    ClusterTreeModel::NodeTypeVariants &ClusterTreeModel::getClusterVarContainingBody(const int body_index)
    {
        return nodes_variants_[getIndexOfClusterContainingBody(body_index)];
    }

    ClusterTreeModel::NodeTypeVariants &ClusterTreeModel::getClusterVarContainingBody(const Body &body)
    {
        return nodes_variants_[getIndexOfClusterContainingBody(body)];
    }

    ClusterTreeModel::NodeTypeVariants &ClusterTreeModel::getClusterVarContainingBody(const string &body_name)
    {
        return nodes_variants_[getIndexOfClusterContainingBody(body_name)];
    }

    DVec<double> ClusterTreeModel::localCartesianForceAtPointToWorldPluckerForceOnCluster(
        const Vec3<double> &force, const ContactPoint &contact_point)
    {
        const Body &body = bodies_[contact_point.body_index_];
        const NodeTypeVariants &cluster = getClusterVarContainingBody(contact_point.body_index_);

        const SpatialTransform &X = Xa(cluster)[body.sub_index_within_cluster_];
        const Mat3<double> Rai = X.getRotation().transpose();
        const SpatialTransform X_cartesian_to_plucker{Rai, contact_point.local_offset_};

        SVec<double> spatial_force = SVec<double>::Zero();
        spatial_force.tail<3>() = force;

        DVec<double> F = DVec<double>::Zero(motionSubspaceDimension(cluster));
        F.segment<6>(6 * body.sub_index_within_cluster_) =
            X_cartesian_to_plucker.inverseTransformForceVector(spatial_force);

        return F;
    }

} // namespace grbda
