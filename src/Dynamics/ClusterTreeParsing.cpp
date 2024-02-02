#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{
    template <typename Scalar>
    void ClusterTreeModel<Scalar>::buildModelFromURDF(const std::string &urdf_filename,
                                                      bool floating_base)
    {
        std::shared_ptr<urdf::ModelInterface> model;
        model = urdf::parseURDFFile(urdf_filename, true);

        if (model == nullptr)
            throw std::runtime_error("Could not parse URDF file");

        std::shared_ptr<const urdf::Link> root = model->getRoot();
        UrdfClusterPtr root_cluster = model->getClusterContaining(root->name);
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
        std::map<UrdfClusterPtr, bool> visited;
        for (UrdfClusterPtr child : root_cluster->child_clusters)
        {
            appendClustersViaDFS(visited, child);
        }
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendClustersViaDFS(std::map<UrdfClusterPtr, bool> &visited,
                                                        UrdfClusterPtr cluster)
    {
        visited[cluster] = true;
        appendClusterFromUrdfCluster(cluster);

        for (const UrdfClusterPtr &child : cluster->child_clusters)
        {
            if (!visited[child])
            {
                appendClustersViaDFS(visited, child);
            }
        }
    }

    // TODO(@MatthewChignoli): Eventually add some specialization and detection for common clusters so that we can use sparsity exploiting classes
    // TODO(@MatthewChignoli): Long function with a lot of nested loops, consider refactoring
    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendClusterFromUrdfCluster(UrdfClusterPtr cluster)
    {
        // TODO(@MatthewChignoli): This is kind of a hack, but I think we should have a special exception for when the cluster has one link, a revolute joint, and not constraint joints
        if (cluster->links.size() == 1 && cluster->constraints.size() == 0)
        {
            UrdfLinkPtr link = cluster->links.begin()->second;
            appendSimpleRevoluteJointFromUrdfCluster(link);
            return;
        }

        // TODO(@MatthewChignoli): to avoid all of this confusion, maybe the body should just contain the parent joint?
        std::vector<JointPtr<Scalar>> joints_in_current_cluster;
        std::vector<bool> independent_coordinates;
        std::vector<Body<SX>> bodies_sx;
        std::map<std::string, JointPtr<SX>> joints_sx;
        registerBodiesInUrdfCluster(cluster, joints_in_current_cluster, independent_coordinates,
                                    bodies_sx, joints_sx);

        // Verify that all constraints in the cluster are of the same type
        auto constraint_type = cluster->constraints.front()->type;
        for (UrdfConstraintPtr constraint : cluster->constraints)
        {
            if (constraint->type != constraint_type)
                throw std::runtime_error("All constraints in a cluster must be of the same type");
        }

        // For each constraint, collect the captures that will be supplied to the lambda function
        // encoding the constraint
        std::vector<PositionConstraintCapture> position_constraint_captures;
        std::vector<RollingConstraintCapture> rolling_constraint_captures;
        for (UrdfConstraintPtr constraint : cluster->constraints)
        {
            std::vector<Body<SX>> nca_to_predecessor_subtree, nca_to_successor_subtree;
            for (UrdfLinkPtr link : constraint->nca_to_predecessor_subtree)
            {
                const auto &body_i = body(link->name);
                nca_to_predecessor_subtree.push_back(bodies_sx[body_i.sub_index_within_cluster_]);
            }
            for (UrdfLinkPtr link : constraint->nca_to_successor_subtree)
            {
                const auto &body_i = body(link->name);
                nca_to_successor_subtree.push_back(bodies_sx[body_i.sub_index_within_cluster_]);
            }

            if (constraint_type == urdf::Constraint::POSITION)
            {
                PositionConstraintCapture capture;
                capture.nca_to_predecessor_subtree = nca_to_predecessor_subtree;
                capture.nca_to_successor_subtree = nca_to_successor_subtree;
                capture.predecessor_to_constraint_origin_transform = *constraint->predecessor_to_constraint_origin_transform;
                capture.successor_to_constraint_origin_transform = *constraint->successor_to_constraint_origin_transform;
                position_constraint_captures.push_back(capture);
            }
            else if (constraint_type == urdf::Constraint::ROLLING)
            {
                RollingConstraintCapture capture;
                capture.nca_to_predecessor_subtree = nca_to_predecessor_subtree;
                capture.nca_to_successor_subtree = nca_to_successor_subtree;
                capture.ratio = *constraint->ratio;
                rolling_constraint_captures.push_back(capture);
            }
            else
            {
                throw std::runtime_error("Constraint type not supported");
            }
        }

        // TODO(@MatthewChignoli): Would like to do some more sophisticated detection and specialization here. For example, if the jacobian of phi is constant, then we can use an explicit constraint. The best solution might just be codegen
        // Create constraints and add clusters
        std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint;
        if (constraint_type == urdf::Constraint::POSITION)
        {
            std::function<DVec<SX>(const JointCoordinate<SX> &)> phi;
            phi = implicitPositionConstraint(position_constraint_captures, joints_sx);

            using LoopConstraintType = LoopConstraint::GenericImplicit<Scalar>;
            loop_constraint = std::make_shared<LoopConstraintType>(independent_coordinates, phi);
        }
        else if (constraint_type == urdf::Constraint::ROLLING)
        {
            using KGPair = std::pair<DMat<Scalar>, DMat<Scalar>>;
            KGPair K_G = explicitRollingConstraint(rolling_constraint_captures,
                                                   independent_coordinates);
            loop_constraint = std::make_shared<LoopConstraint::Static<Scalar>>(K_G.second,
                                                                               K_G.first);
        }
        else
        {
            throw std::runtime_error("Constraint type not supported");
        }

        std::string cluster_name = "cluster-" + std::to_string(cluster_nodes_.size());
        appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
            cluster_name, bodies_in_current_cluster_, joints_in_current_cluster, loop_constraint);
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::appendSimpleRevoluteJointFromUrdfCluster(UrdfLinkPtr link)
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
        UrdfClusterPtr cluster,
        std::vector<JointPtr<Scalar>> &joints,
        std::vector<bool> &independent_coordinates,
        std::vector<Body<SX>> &bodies_sx,
        std::map<std::string, JointPtr<SX>> &joints_sx)
    {
        std::map<int, UrdfLinkPtr> unregistered_links = cluster->links;
        while (unregistered_links.size() > 0)
        {
            std::map<int, UrdfLinkPtr> unregistered_links_next;
            for (const auto &pair : unregistered_links)
            {
                UrdfLinkPtr link = pair.second;

                // If the parent of this link is not registered, then we will try again
                // next iteration
                const std::string parent_name = link->getParent()->name;
                if (body_name_to_body_index_.find(parent_name) == body_name_to_body_index_.end())
                {
                    unregistered_links_next.insert({pair.first, link});
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
        std::vector<PositionConstraintCapture> &captures,
        std::map<std::string, JointPtr<SX>> joints_sx)
    {
        return [captures, joints_sx](const JointCoordinate<SX> &q)
        {
            int jidx = 0;
            using Xform = spatial::Transform<SX>;

            DVec<SX> phi_out = DVec<SX>(0);
            for (size_t i = 0; i < captures.size(); i++)
            {
                const PositionConstraintCapture &capture = captures[i];

                // Through predecessor
                Xform X_via_predecessor;
                for (const Body<SX> &body : capture.nca_to_predecessor_subtree)
                {
                    JointPtr<SX> joint = joints_sx.at(body.name_);
                    joint->updateKinematics(q.segment(jidx, joint->numPositions()),
                                            DVec<SX>::Zero(joint->numVelocities()));
                    jidx += joint->numPositions();
                    X_via_predecessor = joint->XJ() * body.Xtree_ * X_via_predecessor;
                }
                X_via_predecessor = Xform(capture.predecessor_to_constraint_origin_transform) *
                                    X_via_predecessor;
                Vec3<SX> r_nca_to_constraint_via_predecessor = X_via_predecessor.getTranslation();

                // Through successor
                Xform X_via_successor;
                for (const Body<SX> &body : capture.nca_to_successor_subtree)
                {
                    JointPtr<SX> joint = joints_sx.at(body.name_);
                    joint->updateKinematics(q.segment(jidx, joint->numPositions()),
                                            DVec<SX>::Zero(joint->numVelocities()));
                    jidx += joint->numPositions();
                    X_via_successor = joint->XJ() * body.Xtree_ * X_via_successor;
                }
                X_via_successor = Xform(capture.successor_to_constraint_origin_transform) *
                                  X_via_successor;
                Vec3<SX> r_nca_to_constraint_via_successor = X_via_successor.getTranslation();

                // Compute constraint
                Vec3<SX> r_constraint = r_nca_to_constraint_via_predecessor -
                                        r_nca_to_constraint_via_successor;

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
        std::vector<RollingConstraintCapture> &captures,
        std::vector<bool> independent_coordinates)
    {
        // TODO(@MatthewChignoli): Assumes all joints are 1 DOF?
        DMat<Scalar> K = DMat<Scalar>::Zero(captures.size(), independent_coordinates.size());
        for (size_t i = 0; i < captures.size(); i++)
        {
            const RollingConstraintCapture &capture = captures[i];

            // Through predecessor
            for (const Body<SX> &body : capture.nca_to_predecessor_subtree)
            {
                K(i, body.sub_index_within_cluster_) = capture.ratio;
            }

            // Through successor
            for (const Body<SX> &body : capture.nca_to_successor_subtree)
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

} // namespace grbda
