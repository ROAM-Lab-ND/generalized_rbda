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

        for (const UrdfClusterPtr child : cluster->child_clusters)
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
            UrdfLinkPtr link = cluster->links.front();
            appendSimpleRevoluteJointFromUrdfCluster(link);
            return;
        }
        else if (cluster->constraints.size() != 1)
        {
            throw std::runtime_error("Clusters that do not have exactly one constraint joint must contain a single link with a revolute joint");
        }

        // TODO(@MatthewChignoli): to avoid all of this confusion, maybe the body should just contain the parent joint?
        std::vector<JointPtr<Scalar>> joints_in_current_cluster;
        std::vector<bool> independent_coordinates;
        std::vector<Body<SX>> bodies_sx;
        std::map<std::string, JointPtr<SX>> joints_sx;
        registerBodiesInUrdfCluster(cluster, joints_in_current_cluster, independent_coordinates,
                                    bodies_sx, joints_sx);

        // Verify that all constraints in the cluster are of the same type and have the same axis
        UrdfConstraintPtr first_constraint = cluster->constraints.front();
        auto constraint_type = first_constraint->type;
        urdf::Vector3 constraint_axis = first_constraint->allLinks().front()->parent_joint->axis;
        for (UrdfConstraintPtr constraint : cluster->constraints)
        {
            if (constraint->type != constraint_type)
                throw std::runtime_error("All constraints in a cluster must be of the same type");

            for (UrdfLinkPtr link : constraint->allLinks())
            {
                // TODO(@MatthewChignoli): For now we are making the very limiting assumption that all joints must be continuous. If not, then axis can be different yet still valid
                if (link->parent_joint->type != urdf::Joint::CONTINUOUS)
                    throw std::runtime_error("All joints in a constraint must be revolute");
                if (link->parent_joint->axis.x != constraint_axis.x ||
                    link->parent_joint->axis.y != constraint_axis.y ||
                    link->parent_joint->axis.z != constraint_axis.z)
                    throw std::runtime_error("All joints in a constraint must have the same axis");
            }
        }
        if (constraint_axis.norm() != 1)
        {
            throw std::runtime_error("Constraint axis must be a unit vector");
        }

        // For each constraint, collect the captures that will be supplied to the lambda function
        // encoding the constraint
        std::vector<ImplicitConstraintCapture> implicit_constraint_captures;
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

            ImplicitConstraintCapture capture;
            capture.nca_to_predecessor_subtree = nca_to_predecessor_subtree;
            capture.nca_to_successor_subtree = nca_to_successor_subtree;
            capture.predecessor_to_constraint_origin_transform = constraint->predecessor_to_constraint_origin_transform;
            capture.successor_to_constraint_origin_transform = constraint->successor_to_constraint_origin_transform;
            implicit_constraint_captures.push_back(capture);
        }

        // TODO(@MatthewChignoli): Would like to do some more sophisticated detection and specialization here. For example, if the jacobian of phi is constant, then we can use an explicit constraint
        // Create constraints and add clusters
        std::function<DVec<SX>(const JointCoordinate<SX> &)> phi;
        std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint;
        if (constraint_type == urdf::Constraint::POSITION)
        {
            phi = implicitPositionConstraint(implicit_constraint_captures,
                                             joints_sx, constraint_axis);

            using LoopConstraintType = LoopConstraint::GenericImplicit<Scalar>;
            loop_constraint = std::make_shared<LoopConstraintType>(independent_coordinates, phi);
        }
        else if (constraint_type == urdf::Constraint::ROLLING)
        {
            // TODO(@MatthewChignoli): This seems like a very long way to do this. I think it can be streamlined. Yeah this desparately needs to be refactored
            phi = implicitRotationConstraint(implicit_constraint_captures,
                                             joints_sx, constraint_axis);

            // TODO(@MatthewChignoli): Assumes all joints are 1 DOF
            SX cs_q_sym = SX::sym("cs_q_sym", joints_sx.size());
            DVec<SX> q_sym = DVec<SX>(joints_sx.size());
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
        std::vector<UrdfLinkPtr> unregistered_links = cluster->links;
        while (unregistered_links.size() > 0)
        {
            std::vector<UrdfLinkPtr> unregistered_links_next;
            for (UrdfLinkPtr link : unregistered_links)
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
        std::vector<ImplicitConstraintCapture> &captures,
        std::map<std::string, JointPtr<SX>> joints_sx,
        urdf::Vector3 constraint_axis)
    {
        return [captures, joints_sx, constraint_axis](const JointCoordinate<SX> &q)
        {
            int jidx = 0;
            using Xform = spatial::Transform<SX>;

            // TODO(@MatthewChignoli): A lot of duplicate code here. Maybe make a helper function that we can call once for parent and once for child
            DVec<SX> phi_out = DVec<SX>(2 * captures.size());
            for (size_t i = 0; i < captures.size(); i++)
            {
                const ImplicitConstraintCapture &capture = captures[i];

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

                // TODO(@MatthewChignoli): Make sure unit test covers all of these cases
                // TODO(@MatthewChignoli): Maybe a helper function can clean up this code and the code for computing the radii in the rotation constraint
                if (constraint_axis.x == 1)
                {
                    phi_out.segment(2 * i, 2) = DVec<SX>(r_constraint.tail<2>());
                }
                else if (constraint_axis.y == 1)
                {
                    phi_out.segment(2 * i, 2) = DVec<SX>(r_constraint({0, 2}));
                }
                else if (constraint_axis.z == 1)
                {
                    phi_out.segment(2 * i, 2) = DVec<SX>(r_constraint.head<2>());
                }
                else
                {
                    throw std::runtime_error("Constraint axis must be one of the standard axes");
                }
            }

            return phi_out;
        };
    }

    template <typename Scalar>
    std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
    ClusterTreeModel<Scalar>::implicitRotationConstraint(
        std::vector<ImplicitConstraintCapture> &captures,
        std::map<std::string, JointPtr<SX>> joints_sx,
        urdf::Vector3 constraint_axis)
    {
        return [captures, joints_sx, constraint_axis](const JointCoordinate<SX> &q)
        {
            // TODO(@MatthewChignoli): A lot of duplicate code here. Maybe make a helper function that we can call once for parent and once for child
            DVec<SX> phi_out = DVec<SX>(captures.size());
            for (size_t i = 0; i < captures.size(); i++)
            {
                const ImplicitConstraintCapture &capture = captures[i];

                // Compute radii
                using Vector3 = urdf::Vector3;
                Vector3 predecessor_offset = capture.predecessor_to_constraint_origin_transform.position;
                Vector3 successor_offset = capture.successor_to_constraint_origin_transform.position;
                SX predecessor_radius, successor_radius;
                // TODO(@MatthewChignoli): Another instance of it not being allowed that the axis is negative?
                if (constraint_axis.x == 1)
                {
                    predecessor_radius = sqrt(predecessor_offset.y * predecessor_offset.y +
                                              predecessor_offset.z * predecessor_offset.z);
                    successor_radius = sqrt(successor_offset.y * successor_offset.y +
                                            successor_offset.z * successor_offset.z);
                }
                else if (constraint_axis.y == 1)
                {
                    predecessor_radius = sqrt(predecessor_offset.x * predecessor_offset.x +
                                              predecessor_offset.z * predecessor_offset.z);
                    successor_radius = sqrt(successor_offset.x * successor_offset.x +
                                            successor_offset.z * successor_offset.z);
                }
                else if (constraint_axis.z == 1)
                {
                    predecessor_radius = sqrt(predecessor_offset.x * predecessor_offset.x +
                                              predecessor_offset.y * predecessor_offset.y);
                    successor_radius = sqrt(successor_offset.x * successor_offset.x +
                                            successor_offset.y * successor_offset.y);
                }
                else
                {
                    throw std::runtime_error("Constraint axis must be one of the standard axes");
                }

                // Through predecessor
                int jidx = 0;
                SX predecessor_angle_rel_nca = 0;
                for (const Body<SX> &body : capture.nca_to_predecessor_subtree)
                {
                    predecessor_angle_rel_nca += q(jidx);
                    JointPtr<SX> joint = joints_sx.at(body.name_);
                    jidx += joint->numPositions();
                }

                // Through successor
                SX successor_angle_rel_nca = 0;
                for (const Body<SX> &body : capture.nca_to_successor_subtree)
                {
                    successor_angle_rel_nca += q(jidx);
                    JointPtr<SX> joint = joints_sx.at(body.name_);
                    jidx += joint->numPositions();
                }

                phi_out(i) = predecessor_radius * predecessor_angle_rel_nca -
                             successor_radius * successor_angle_rel_nca;
            }
            return phi_out;
        };
    }

    template class ClusterTreeModel<double>;
    template class ClusterTreeModel<float>;
    template class ClusterTreeModel<casadi::SX>;

} // namespace grbda
