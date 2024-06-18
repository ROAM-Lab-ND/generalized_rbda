#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{
    template <typename Scalar>
    template <typename OrientationRepresentation>
    void ClusterTreeModel<Scalar>::buildFromUrdfModelInterface(
        const UrdfModelPtr model, bool floating_base)
    {
        if (model == nullptr)
            throw std::runtime_error("Could not parse URDF file");

        std::cout << "Building cluster tree model from URDF model" << std::endl;
        UrdfClusterModelPtr cluster_model = std::make_shared<UrdfClusterModel>(*model);
        urdf::LinkConstSharedPtr root = cluster_model->getRoot();
        std::cout << "Root link: " << root->name << std::endl;
        const UrdfCluster& root_cluster = cluster_model->getClusterContaining(root);
        if (root_cluster.size() != 1)
        {
            throw std::runtime_error("The root cluster may only contain one body");
        }

        if (floating_base)
        {
            std::string name = root->name;
            std::string parent_name = "ground";
            SpatialInertia<Scalar> inertia(root->inertial);
            spatial::Transform<Scalar> xtree = spatial::Transform<Scalar>{};
            using Free = ClusterJoints::Free<Scalar, OrientationRepresentation>;
            appendBody<Free>(name, inertia, parent_name, xtree);
        }
        else
        {
            body_name_to_body_index_.clear();
            body_name_to_body_index_[root->name] = -1;
        }

        // Add remaining bodies
        std::map<UrdfCluster, bool> visited;
        std::vector<UrdfCluster> child_clusters = cluster_model->getChildClusters(root_cluster);
        for (const UrdfCluster& child : cluster_model->getChildClusters(root_cluster))
        {
            appendClustersViaDFS(visited, child, cluster_model);
        }
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendClustersViaDFS(std::map<UrdfCluster, bool> &visited,
                                                        const UrdfCluster &cluster,
                                                        const UrdfClusterModelPtr &cluster_model)
    {
        visited[cluster] = true;
        appendClusterFromUrdfCluster(cluster);

        for (const UrdfCluster &child : cluster_model->getChildClusters(cluster))
        {
            if (!visited[child])
            {
                appendClustersViaDFS(visited, child, cluster_model);
            }
        }
    }

    // TODO(@MatthewChignoli): Eventually add some specialization and detection for common clusters so that we can use sparsity exploiting classes
    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendClusterFromUrdfCluster(const UrdfCluster& cluster)
    {
        // Collect all constraints in the cluster
        std::vector<urdf::ConstraintSharedPtr> constraints;
        for (const urdf::LinkSharedPtr &link : cluster)
        {
            for (const urdf::ConstraintSharedPtr &constraint : link->constraints)
            {
                constraints.push_back(constraint);
            }
        }

        // TODO(@MatthewChignoli): Remove this assumption?
        for (const urdf::ConstraintSharedPtr &constraint : constraints)
        {
            if (constraint->type != constraints.front()->type)
            {
                throw std::runtime_error("All constraints in a cluster must be of the same type");
            }
        }

        // TODO(@MatthewChignoli): This is kind of a hack, but I think we should have a special exception for when the cluster has one link, a revolute joint, and not constraint joints
        if (cluster.size() == 1 && constraints.size() == 0)
        {
            appendSimpleRevoluteJointFromUrdfCluster(cluster.front());
            return;
        }

        // TODO(@MatthewChignoli): to avoid all of this confusion, maybe the body should just contain the parent joint?
        std::vector<JointPtr<Scalar>> joints_in_current_cluster;
        std::vector<bool> independent_coordinates;
        std::vector<Body<SX>> bodies_sx;
        std::map<std::string, JointPtr<SX>> joints_sx;
        registerBodiesInUrdfCluster(cluster, joints_in_current_cluster, independent_coordinates,
                                    bodies_sx, joints_sx);

        // For each constraint, collect the captures that will be supplied to the lambda function
        // encoding the constraint
        std::vector<LoopConstraintCapture> loop_constraint_captures;
        std::vector<JointConstraintCapture> joint_constraint_captures;
        for (urdf::ConstraintSharedPtr constraint : constraints)
        {
            const Body<Scalar>& parent = body(constraint->parent_link_name);
            std::vector<Body<SX>> nca_to_parent_subtree =
                getSubtreeFromNearestAncestor(parent, bodies_sx, constraint);

            const Body<Scalar>& child = body(constraint->child_link_name);
            std::vector<Body<SX>> nca_to_child_subtree =
                getSubtreeFromNearestAncestor(child, bodies_sx, constraint);               

            if (constraint->type == urdf::Constraint::LOOP)
            {
                urdf::LoopConstraintSharedPtr loop_constraint =
                    std::dynamic_pointer_cast<urdf::LoopConstraint>(constraint);
                LoopConstraintCapture capture;
                capture.nca_to_parent_subtree = nca_to_parent_subtree;
                capture.nca_to_child_subtree = nca_to_child_subtree;
                capture.parent_to_constraint_origin_transform = loop_constraint->parent_to_constraint_origin_transform;
                capture.child_to_constraint_origin_transform = loop_constraint->child_to_constraint_origin_transform;
                loop_constraint_captures.push_back(capture);
            }
            else if (constraint->type == urdf::Constraint::JOINT)
            {
                urdf::JointConstraintSharedPtr joint_constraint =
                    std::dynamic_pointer_cast<urdf::JointConstraint>(constraint);
                JointConstraintCapture capture;
                capture.nca_to_parent_subtree = nca_to_parent_subtree;
                capture.nca_to_child_subtree = nca_to_child_subtree;
                capture.ratio = joint_constraint->gear_ratio;
                joint_constraint_captures.push_back(capture);
            }
            else
            {
                throw std::runtime_error("Constraint type not supported");
            }
        }

        // TODO(@MatthewChignoli): The naming here is tough... we have a grbda::LoopConstraint and a urdf::LoopConstraint, and they mean different things

        // TODO(@MatthewChignoli): Would like to do some more sophisticated detection and specialization here. For example, if the jacobian of phi is constant, then we can use an explicit constraint. The best solution might just be codegen
        // Create constraints and add clusters
        std::shared_ptr<LoopConstraint::Base<Scalar>> cluster_constraint;
        if (constraints.front()->type == urdf::Constraint::LOOP)
        {
            std::function<DVec<SX>(const JointCoordinate<SX> &)> phi;
            phi = implicitPositionConstraint(loop_constraint_captures, joints_sx);

            using LoopConstraintType = LoopConstraint::GenericImplicit<Scalar>;
            cluster_constraint = std::make_shared<LoopConstraintType>(independent_coordinates, phi);
        }
        else if (constraints.front()->type == urdf::Constraint::JOINT)
        {
            using KGPair = std::pair<DMat<Scalar>, DMat<Scalar>>;
            KGPair K_G = explicitRollingConstraint(joint_constraint_captures,
                                                   independent_coordinates);
            cluster_constraint = std::make_shared<LoopConstraint::Static<Scalar>>(K_G.second,
                                                                               K_G.first);
        }
        else
        {
            throw std::runtime_error("Constraint type not supported");
        }

        std::string cluster_name = "cluster-" + std::to_string(cluster_nodes_.size());
        appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
            cluster_name, bodies_in_current_cluster_, joints_in_current_cluster, cluster_constraint);
    }

    template <typename Scalar>
    std::vector<Body<casadi::SX>> ClusterTreeModel<Scalar>::getSubtreeFromNearestAncestor(
        const Body<Scalar> &leaf, const std::vector<Body<SX>> &bodies_sx,
        const urdf::ConstraintSharedPtr &constraint) const
    {
        std::vector<Body<SX>> subtree;

        // TODO(@MatthewChignoli): I think we can do this without needing the NCA?
        subtree.push_back(bodies_sx[leaf.sub_index_within_cluster_]);
        while (subtree.back().parent_index_ != -1)
        {
            if (body(subtree.back().parent_index_).name_ ==
                constraint->nearest_common_ancestor_name)
            {
                break;
            }
            const Body<Scalar> &body_i = body(subtree.back().parent_index_);
            subtree.push_back(bodies_sx[body_i.sub_index_within_cluster_]);
        }
        return subtree;
    }

    template <typename Scalar>
    void
    ClusterTreeModel<Scalar>::appendSimpleRevoluteJointFromUrdfCluster(urdf::LinkSharedPtr link)
    {
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
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::registerBodiesInUrdfCluster(
        const UrdfCluster& cluster,
        std::vector<JointPtr<Scalar>> &joints,
        std::vector<bool> &independent_coordinates,
        std::vector<Body<SX>> &bodies_sx,
        std::map<std::string, JointPtr<SX>> &joints_sx)
    {
        std::map<int, urdf::LinkSharedPtr> unregistered_links;
        int i = 0;
        for (const urdf::LinkSharedPtr link : cluster)
        {
            unregistered_links.insert({i++, link});
        }

        while (unregistered_links.size() > 0)
        {
            std::map<int, urdf::LinkSharedPtr> unregistered_links_next;
            for (const auto &[link_idx, link] : unregistered_links)
            {
                // If the parent of this link is not registered, then we will try again
                // next iteration
                const std::string parent_name = link->getParent()->name;
                if (body_name_to_body_index_.find(parent_name) == body_name_to_body_index_.end())
                {
                    unregistered_links_next.insert({link_idx, link});
                    continue;
                }

                // Register body
                std::string name = link->name;
                SpatialInertia<Scalar> inertia(link->inertial);
                SpatialInertia<SX> inertia_sx(link->inertial);

                urdf::Pose pose = link->parent_joint->parent_to_joint_origin_transform;
                spatial::Transform<Scalar> xtree(pose);
                spatial::Transform<SX> xtree_sx(pose);

                registerBody(name, inertia, parent_name, xtree);
                bodies_sx.emplace_back(bodies_in_current_cluster_.back().index_, name, bodies_in_current_cluster_.back().parent_index_, xtree_sx, inertia_sx, bodies_in_current_cluster_.back().sub_index_within_cluster_, bodies_in_current_cluster_.back().cluster_ancestor_index_, bodies_in_current_cluster_.back().cluster_ancestor_sub_index_within_cluster_);

                // Create joint for registered body
                ori::CoordinateAxis axis = ori::urdfAxisToCoordinateAxis(link->parent_joint->axis);
                joints.push_back(std::make_shared<Joints::Revolute<Scalar>>(axis));
                joints_sx.insert({name, std::make_shared<Joints::Revolute<SX>>(axis)});

                // Extract independent coordinates from joint
                independent_coordinates.push_back(link->parent_joint->independent);
            }

            unregistered_links = unregistered_links_next;
        }
    }

    template <typename Scalar>
    std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
    ClusterTreeModel<Scalar>::implicitPositionConstraint(
        std::vector<LoopConstraintCapture> &captures,
        std::map<std::string, JointPtr<SX>> joints_sx)
    {
        return [captures, joints_sx](const JointCoordinate<SX> &q)
        {
            int jidx = 0;
            using Xform = spatial::Transform<SX>;

            DVec<SX> phi_out = DVec<SX>(0);
            for (size_t i = 0; i < captures.size(); i++)
            {
                const LoopConstraintCapture &capture = captures[i];

                // Through parent
                Xform X_via_parent;
                for (const Body<SX> &body : capture.nca_to_parent_subtree)
                {
                    JointPtr<SX> joint = joints_sx.at(body.name_);
                    joint->updateKinematics(q.segment(jidx, joint->numPositions()),
                                            DVec<SX>::Zero(joint->numVelocities()));
                    jidx += joint->numPositions();
                    X_via_parent = joint->XJ() * body.Xtree_ * X_via_parent;
                }
                X_via_parent = Xform(capture.parent_to_constraint_origin_transform) *
                                    X_via_parent;
                Vec3<SX> r_nca_to_constraint_via_parent = X_via_parent.getTranslation();

                // Through child
                Xform X_via_child;
                for (const Body<SX> &body : capture.nca_to_child_subtree)
                {
                    JointPtr<SX> joint = joints_sx.at(body.name_);
                    joint->updateKinematics(q.segment(jidx, joint->numPositions()),
                                            DVec<SX>::Zero(joint->numVelocities()));
                    jidx += joint->numPositions();
                    X_via_child = joint->XJ() * body.Xtree_ * X_via_child;
                }
                X_via_child = Xform(capture.child_to_constraint_origin_transform) *
                                  X_via_child;
                Vec3<SX> r_nca_to_constraint_via_child = X_via_child.getTranslation();

                // Compute constraint
                Vec3<SX> r_constraint = r_nca_to_constraint_via_parent -
                                        r_nca_to_constraint_via_child;

                // TODO(@MatthewChignoli): The lambda function will do this check every time, which is wasted effort...
                // Only enforce the constraint along axes that depend on joint coordinates
                SX cs_r_constraint = SX(r_constraint.size());
                casadi::copy(r_constraint, cs_r_constraint);
                SX cs_q = SX(q.size());
                casadi::copy(q, cs_q);
                std::vector<bool> constraint_axes = which_depends(cs_r_constraint, cs_q, 2, true);

                for (int j = 0; j < 3; j++)
                {
                    if (constraint_axes[j])
                    {
                        phi_out.conservativeResize(phi_out.size() + 1);
                        phi_out(phi_out.size() - 1) = r_constraint(j);
                    }
                }
            }

            return phi_out;
        };
    }

    template <typename Scalar>
    std::pair<DMat<Scalar>, DMat<Scalar>>
    ClusterTreeModel<Scalar>::explicitRollingConstraint(
        std::vector<JointConstraintCapture> &captures,
        std::vector<bool> independent_coordinates)
    {
        // TODO(@MatthewChignoli): Assumes all joints are 1 DOF?
        DMat<Scalar> K = DMat<Scalar>::Zero(captures.size(), independent_coordinates.size());
        for (size_t i = 0; i < captures.size(); i++)
        {
            const JointConstraintCapture &capture = captures[i];

            // Through parent
            for (const Body<SX> &body : capture.nca_to_parent_subtree)
            {
                K(i, body.sub_index_within_cluster_) = capture.ratio;
            }

            // Through child
            for (const Body<SX> &body : capture.nca_to_child_subtree)
            {
                K(i, body.sub_index_within_cluster_) = -1;
            }
        }

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

        int ind_cnt = 0, dep_cnt = 0;
        DMat<Scalar> coordinate_jacobian = DMat<Scalar>::Zero(K.cols(), K.cols());
        for (int i = 0; i < K.cols(); i++)
        {
            if (independent_coordinates[i])
            {
                coordinate_jacobian(i, ind_cnt++) = 1;
            }
            else
            {
                coordinate_jacobian(i, Ki.cols() + dep_cnt++) = 1;
            }
        }

        DMat<Scalar> G(K.cols(), Ki.cols());
        typename CorrectMatrixInverseType<Scalar>::type Kd_inv(Kd);
        G.topRows(Ki.cols()).setIdentity();
        G.bottomRows(Kd.cols()) = -Kd_inv.solve(Ki);
        G = coordinate_jacobian * G;

        return {K, G};
    }

    template class ClusterTreeModel<double>;
    template class ClusterTreeModel<float>;
    template class ClusterTreeModel<casadi::SX>;

    template void ClusterTreeModel<double>::buildFromUrdfModelInterface<ori_representation::Quaternion>(
        const UrdfModelPtr model, bool floating_base);
    template void ClusterTreeModel<float>::buildFromUrdfModelInterface<ori_representation::Quaternion>(
        const UrdfModelPtr model, bool floating_base);
    template void ClusterTreeModel<casadi::SX>::buildFromUrdfModelInterface<ori_representation::Quaternion>(
        const UrdfModelPtr model, bool floating_base);
    template void ClusterTreeModel<double>::buildFromUrdfModelInterface<ori_representation::RollPitchYaw>(
        const UrdfModelPtr model, bool floating_base);
    template void ClusterTreeModel<float>::buildFromUrdfModelInterface<ori_representation::RollPitchYaw>(
        const UrdfModelPtr model, bool floating_base);
    template void ClusterTreeModel<casadi::SX>::buildFromUrdfModelInterface<ori_representation::RollPitchYaw>(
        const UrdfModelPtr model, bool floating_base);


} // namespace grbda
