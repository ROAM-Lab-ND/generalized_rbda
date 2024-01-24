/*! @file ClusterTreeModel.cpp
 *
 */

#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::buildModelFromURDF(const std::string &urdf_filename,
                                                      bool floating_base)
    {
        using ConstLinkPtr = std::shared_ptr<const urdf::Link>;
        using ClusterPtr = std::shared_ptr<urdf::Cluster>;

        std::shared_ptr<urdf::ModelInterface> model;
        model = urdf::parseURDFFile(urdf_filename, true);

        if(model == nullptr)
            throw std::runtime_error("Could not parse URDF file");

        ConstLinkPtr root = model->getRoot();
        ClusterPtr root_cluster = model->getClusterContaining(root->name);
        if (root_cluster->links.size() != 1)
        {
            throw std::runtime_error("The root cluster may only contain one body");
        }

        if (floating_base)
        {
            std::string name = root->name;
            std::string parent_name = "ground";
            SpatialInertia<Scalar> inertia(root->inertial);
            spatial::Transform<Scalar> xtree = spatial::Transform<Scalar>{};
            using Free = ClusterJoints::Free<Scalar, ori_representation::Quaternion>;
            appendBody<Free>(name, inertia, parent_name, xtree);
        }
        else
        {
            body_name_to_body_index_.clear();
            body_name_to_body_index_[root->name] = -1;
        }

        // Add remaining bodies
        std::map<std::string, bool> visited;
        for (ClusterPtr child : root_cluster->child_clusters)
        {
            appendClustersViaDFS(child->name, visited, child);
        }
    }

    template <typename Scalar>
    void
    ClusterTreeModel<Scalar>::appendClustersViaDFS(const std::string &cluster_name,
                                                   std::map<std::string, bool> &visited,
                                                   UrdfClusterPtr cluster)
    {
        visited[cluster_name] = true;
        appendClusterFromUrdfCluster(cluster);

        for (const UrdfClusterPtr child : cluster->child_clusters)
        {
            const std::string &child_name = child->name;
            if (!visited[child_name])
            {
                appendClustersViaDFS(child_name, visited, child);
            }
        }
    }

    // TODO(@MatthewChignoli): Eventually add some specialization and detection for common clusters so that we can use sparsity exploiting classes
    // TODO(@MatthewChignoli): Long function with a lot of nested loops, consider refactoring
    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendClusterFromUrdfCluster(UrdfClusterPtr cluster)
    {
        // TODO(@MatthewChignoli): This is kind of a hack, but I think we should have a special exception for when the cluster has one link, a revolute joint, and not constraint joints
        if (cluster->links.size() == 1 && cluster->constraint_joints.size() == 0)
        {
            std::shared_ptr<urdf::Link> link = cluster->links.front();

            if (link->parent_joint->type != urdf::Joint::CONTINUOUS)
            {
                throw std::runtime_error("The only joint in a cluster with one link must be revolute");
            }
            std::string name = link->name;
            std::string parent_name = link->getParent()->name;
            SpatialInertia<Scalar> inertia(link->inertial);
            spatial::Transform<Scalar> xtree(link->parent_joint->parent_to_joint_origin_transform);
            ori::CoordinateAxis axis = ori::urdfAxisToCoordinateAxis(link->parent_joint->axis);
            using Revolute = ClusterJoints::Revolute<Scalar>;
            appendBody<Revolute>(name, inertia, parent_name, xtree, axis);
            return;
        }
        else if (cluster->constraint_joints.size() != 1)
        {
            throw std::runtime_error("Clusters that do not have exactly one constraint joint must contain a single link with a revolute joint");
        }

        // TODO(@MatthewChignoli): to avoid all of this confusion, maybe the body should just contain the parent joint?
        std::vector<Body<Scalar>> bodies;
        std::vector<JointPtr<Scalar>> joints;
        std::vector<bool> independent_coordinates;
        std::vector<Body<SX>> bodies_sx;
        std::map<std::string, JointPtr<SX>> joints_sx;

        std::vector<std::shared_ptr<urdf::Link>> unregistered_links = cluster->links;
        while (unregistered_links.size() > 0)
        {
            std::vector<std::shared_ptr<urdf::Link>> unregistered_links_next;
            for (std::shared_ptr<urdf::Link> link : unregistered_links)
            {
                // If the parent of this link is not registered, then we will try again
                // next iteration
                const std::string parent_name = link->getParent()->name;
                if (body_name_to_body_index_.find(parent_name) == body_name_to_body_index_.end())
                {
                    unregistered_links_next.push_back(link);
                    continue;
                }

                // Register body
                std::string name = link->name;
                SpatialInertia<Scalar> inertia(link->inertial);
                SpatialInertia<SX> inertia_sx(link->inertial);

                urdf::Pose pose = link->parent_joint->parent_to_joint_origin_transform;
                spatial::Transform<Scalar> xtree(pose);
                spatial::Transform<SX> xtree_sx(pose);

                bodies.push_back(registerBody(name, inertia, parent_name, xtree));
                bodies_sx.emplace_back(bodies.back().index_, name, bodies.back().parent_index_, xtree_sx, inertia_sx, bodies.back().sub_index_within_cluster_, bodies.back().cluster_ancestor_index_, bodies.back().cluster_ancestor_sub_index_within_cluster_);

                // Create joint for registered body
                ori::CoordinateAxis axis = ori::urdfAxisToCoordinateAxis(link->parent_joint->axis);
                joints.push_back(std::make_shared<Joints::Revolute<Scalar>>(axis));
                joints_sx.insert({name, std::make_shared<Joints::Revolute<SX>>(axis)});

                // Extract independent coordinates from joint
                independent_coordinates.push_back(link->parent_joint->independent);
            }

            unregistered_links = unregistered_links_next;
        }

        std::function<DVec<SX>(const JointCoordinate<SX> &)> phi;
        std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint;
        for (std::shared_ptr<const urdf::ConstraintJoint> constraint : cluster->constraint_joints)
        {
            // TODO(@MatthewChignoli): Detect the axis of the constraint. For now we are making the very limiting assumption that all joints must be continuous
            urdf::Vector3 constraint_axis = constraint->allLinks().front()->parent_joint->axis;
            for (std::shared_ptr<urdf::Link> link : constraint->allLinks())
            {
                if (link->parent_joint->type != urdf::Joint::CONTINUOUS)
                    throw std::runtime_error("All joints in a constraint must be revolute");
                if (link->parent_joint->axis.x != constraint_axis.x ||
                    link->parent_joint->axis.y != constraint_axis.y ||
                    link->parent_joint->axis.z != constraint_axis.z)
                    throw std::runtime_error("All joints in a constraint must have the same axis");
            }
            if (constraint_axis.norm() != 1)
            {
                throw std::runtime_error("Constraint axis must be a unit vector");
            }

            // Get subtrees for the constraint
            std::vector<Body<SX>> nca_to_parent_subtree, nca_to_child_subtree;
            for (std::shared_ptr<urdf::Link> link : constraint->nca_to_parent_subtree)
            {
                const auto &body_i = body(link->name);
                nca_to_parent_subtree.push_back(bodies_sx[body_i.sub_index_within_cluster_]);
            }
            for (std::shared_ptr<urdf::Link> link : constraint->nca_to_child_subtree)
            {
                const auto &body_i = body(link->name);
                nca_to_child_subtree.push_back(bodies_sx[body_i.sub_index_within_cluster_]);
            }

            // TODO(@MatthewChignoli): Would like to do some more sophisticated detection and specialization here. For example, if the jacobian of phi is constant, then we can use an explicit constraint
            // Create constraints and add clusters
            if (constraint->type == urdf::ConstraintJoint::POSITION)
            {
                phi = implicitPositionConstraint(nca_to_parent_subtree, nca_to_child_subtree,
                                                 constraint, joints_sx, constraint_axis);

                using LoopConstraintType = LoopConstraint::GenericImplicit<Scalar>;
                loop_constraint = std::make_shared<LoopConstraintType>(independent_coordinates, phi);
            }
            else if (constraint->type == urdf::ConstraintJoint::ROTATION)
            {
                // TODO(@MatthewChignoli): This seems like a very long way to do this. I think it can be streamlined. Yeah this desparately needs to be refactored
                phi = implicitRotationConstraint(nca_to_parent_subtree, nca_to_child_subtree,
                                                 constraint, joints_sx, constraint_axis);

                // TODO(@MatthewChignoli): Assumes all joints are 1 DOF
                SX cs_q_sym = SX::sym("cs_q_sym", constraint->allLinks().size());
                DVec<SX> q_sym = DVec<SX>(constraint->allLinks().size());
                casadi::copy(cs_q_sym, q_sym);
                JointCoordinate<SX> joint_pos(q_sym, true);

                DVec<SX> phi_sym = phi(joint_pos);
                SX cs_phi_sym = SX(phi_sym.size());
                casadi::copy(phi_sym, cs_phi_sym);

                SX cs_K_sym = jacobian(cs_phi_sym, cs_q_sym);
                DMat<SX> K_sym(cs_K_sym.size1(), cs_K_sym.size2());
                casadi::copy(cs_K_sym, K_sym);

                // Get explicit jacboian from implicit jacobian
                // TODO(@MatthewChignoli): Throw an error if K contains a nan
                // TODO(@MatthewChignoli): Do we really need to support float?
                using IntermediateCastType = typename std::conditional<std::is_same<Scalar, float>::value, double, Scalar>::type;
                DMat<Scalar> K = K_sym.cast<IntermediateCastType>().template cast<Scalar>();
                DMat<Scalar> Ki(K.rows(), 0);
                DMat<Scalar> Kd(K.rows(), 0);
                for (int i = 0; i < K.cols(); i++)
                {
                    if (independent_coordinates[i])
                    {
                        Ki.conservativeResize(K.rows(), Ki.cols() + 1);
                        Ki.template rightCols<1>() = K.col(i);
                    }
                    else
                    {
                        Kd.conservativeResize(K.rows(), Kd.cols() + 1);
                        Kd.template rightCols<1>() = K.col(i);
                    }
                }

                DMat<Scalar> G(K.cols(), Ki.cols());
                typename CorrectMatrixInverseType<Scalar>::type Kd_inv(Kd);
                G.topRows(Ki.cols()).setIdentity();
                G.bottomRows(Kd.cols()) = -Kd_inv.solve(Ki);

                loop_constraint = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);
            }
            else
            {
                throw std::runtime_error("Constraint type not supported");
            }
        }

        appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
            cluster->name, bodies, joints, loop_constraint);
    }

    template <typename Scalar>
    std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
    ClusterTreeModel<Scalar>::implicitPositionConstraint(
        std::vector<Body<SX>> &nca_to_parent_subtree,
        std::vector<Body<SX>> &nca_to_child_subtree,
        std::shared_ptr<const urdf::ConstraintJoint> constraint,
        std::map<std::string, JointPtr<SX>> joints_sx,
        urdf::Vector3 constraint_axis)
    {
        return [nca_to_parent_subtree, nca_to_child_subtree, constraint, joints_sx, constraint_axis](const JointCoordinate<SX> &q)
        {
            int jidx = 0;
            using Xform = spatial::Transform<SX>;

            // TODO(@MatthewChignoli): A lot of duplicate code here. Maybe make a helper function that we can call once for parent and once for child

            // Through parent
            Xform X_via_parent;
            for (const Body<SX> &body : nca_to_parent_subtree)
            {
                JointPtr<SX> joint = joints_sx.at(body.name_);
                joint->updateKinematics(q.segment(jidx, joint->numPositions()),
                                        DVec<SX>::Zero(joint->numVelocities()));
                jidx += joint->numPositions();
                X_via_parent = joint->XJ() * body.Xtree_ * X_via_parent;
            }
            X_via_parent = Xform(constraint->parent_to_joint_origin_transform) * X_via_parent;
            Vec3<SX> r_nca_to_constraint_through_parent = X_via_parent.getTranslation();

            // Through child
            Xform X_via_child;
            for (const Body<SX> &body : nca_to_child_subtree)
            {
                JointPtr<SX> joint = joints_sx.at(body.name_);
                joint->updateKinematics(q.segment(jidx, joint->numPositions()),
                                        DVec<SX>::Zero(joint->numVelocities()));
                jidx += joint->numPositions();
                X_via_child = joint->XJ() * body.Xtree_ * X_via_child;
            }
            X_via_child = Xform(constraint->child_to_joint_origin_transform) * X_via_child;
            Vec3<SX> r_nca_to_constraint_through_child = X_via_child.getTranslation();

            // Compute constraint
            Vec3<SX> r_constraint = r_nca_to_constraint_through_parent -
                                    r_nca_to_constraint_through_child;

            // TODO(@MatthewChignoli): Make sure unit test covers all of these cases
            // TODO(@MatthewChignoli): Maybe a helper function can clean up this code and the code for computing the radii in the rotation constraint
            if (constraint_axis.x == 1)
            {
                return DVec<SX>(r_constraint.tail<2>());
            }
            else if (constraint_axis.y == 1)
            {
                return DVec<SX>(r_constraint({0, 2}));
            }
            else if (constraint_axis.z == 1)
            {
                return DVec<SX>(r_constraint.head<2>());
            }
            else
            {
                throw std::runtime_error("Constraint axis must be one of the standard axes");
            }
        };
    }

    template <typename Scalar>
    std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
    ClusterTreeModel<Scalar>::implicitRotationConstraint(
        std::vector<Body<SX>> &nca_to_parent_subtree,
        std::vector<Body<SX>> &nca_to_child_subtree,
        std::shared_ptr<const urdf::ConstraintJoint> constraint,
        std::map<std::string, JointPtr<SX>> joints_sx,
        urdf::Vector3 constraint_axis)
    {
        return [nca_to_parent_subtree, nca_to_child_subtree, constraint, joints_sx, constraint_axis](const JointCoordinate<SX> &q)
        {
            // TODO(@MatthewChignoli): A lot of duplicate code here. Maybe make a helper function that we can call once for parent and once for child

            // Compute radii
            using Vector3 = urdf::Vector3;
            Vector3 parent_offset = constraint->parent_to_joint_origin_transform.position;
            Vector3 child_offset = constraint->child_to_joint_origin_transform.position;
            SX parent_radius, child_radius;
            // TODO(@MatthewChignoli): Another instance of it not being allowed that the axis is negative?
            if (constraint_axis.x == 1)
            {
                parent_radius = sqrt(parent_offset.y * parent_offset.y +
                                     parent_offset.z * parent_offset.z);
                child_radius = sqrt(child_offset.y * child_offset.y +
                                    child_offset.z * child_offset.z);
            }
            else if (constraint_axis.y == 1)
            {
                parent_radius = sqrt(parent_offset.x * parent_offset.x +
                                     parent_offset.z * parent_offset.z);
                child_radius = sqrt(child_offset.x * child_offset.x +
                                    child_offset.z * child_offset.z);
            }
            else if (constraint_axis.z == 1)
            {
                parent_radius = sqrt(parent_offset.x * parent_offset.x +
                                     parent_offset.y * parent_offset.y);
                child_radius = sqrt(child_offset.x * child_offset.x +
                                    child_offset.y * child_offset.y);
            }
            else
            {
                throw std::runtime_error("Constraint axis must be one of the standard axes");
            }

            // Through parent
            int jidx = 0;
            SX parent_angle_rel_nca = 0;
            for (const Body<SX> &body : nca_to_parent_subtree)
            {
                parent_angle_rel_nca += q(jidx);
                JointPtr<SX> joint = joints_sx.at(body.name_);
                jidx += joint->numPositions();
            }

            // Through child
            SX child_angle_rel_nca = 0;
            for (const Body<SX> &body : nca_to_child_subtree)
            {
                child_angle_rel_nca += q(jidx);
                JointPtr<SX> joint = joints_sx.at(body.name_);
                jidx += joint->numPositions();
            }

            DVec<SX> phi_out = DVec<SX>(1);
            phi_out(0) = parent_radius * parent_angle_rel_nca -
                         child_radius * child_angle_rel_nca;
            return phi_out;
        };
    }

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
