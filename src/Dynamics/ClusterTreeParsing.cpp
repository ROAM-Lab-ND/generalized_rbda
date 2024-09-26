#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{
    template <typename Scalar, typename OriTpl>
    void ClusterTreeModel<Scalar, OriTpl>::buildFromUrdfModelInterface(
        const urdf::ModelInterfaceSharedPtr model)
    {
        if (model == nullptr)
            throw std::runtime_error("Could not parse URDF file");

        // Ensure valid root
        urdf::LinkConstSharedPtr root = model->getRoot();
        body_name_to_body_index_[root->name] = -1;
        urdf::ClusterConstSharedPtr root_cluster = model->getContainingCluster(root->name);
        if (root_cluster->size() != 1)
        {
            throw std::runtime_error("The root cluster may only contain one body");
        }

        // Add remaining bodies
        std::map<urdf::ClusterSharedPtr, bool> visited;
        for (urdf::ClusterSharedPtr child : root_cluster->child_clusters)
        {
            appendClustersViaDFS(visited, child);
        }
    }

    template <typename Scalar, typename OriTpl>
    void ClusterTreeModel<Scalar, OriTpl>::appendClustersViaDFS(
        std::map<urdf::ClusterSharedPtr, bool> &visited, urdf::ClusterSharedPtr cluster)
    {
        visited[cluster] = true;
        appendClusterFromUrdfCluster(cluster);

        for (const urdf::ClusterSharedPtr &child : cluster->child_clusters)
        {
            if (!visited[child])
            {
                appendClustersViaDFS(visited, child);
            }
        }
    }

    // TODO(@MatthewChignoli): Eventually add some specialization and detection for common clusters so that we can use sparsity exploiting classes
    template <typename Scalar, typename OriTpl>
    void
    ClusterTreeModel<Scalar, OriTpl>::appendClusterFromUrdfCluster(urdf::ClusterSharedPtr cluster)
    {
        // Error if cluster is empty or null
        if (cluster->size() == 0)
        {
            throw std::runtime_error("Cluster is empty");
        }

        // Special case for a cluster with one link and (necessarily) no constraints
        if (cluster->size() == 1)
        {
            urdf::LinkSharedPtr link = cluster->front();
            urdf::JointSharedPtr joint = link->parent_joint;

            if (joint->type == urdf::Joint::CONTINUOUS || joint->type == urdf::Joint::REVOLUTE)
            {
                appendSimpleRevoluteJointFromUrdfCluster(link);
            }
            else if (joint->type == urdf::Joint::FLOATING)
            {
                appendSimpleFloatingJointFromUrdfCluster(link);
            }
            else
            {
                throw std::runtime_error("The only joint in a cluster with one link must be revolute or floating");
            }

            return;
        } 

        // Register bodies in cluster. Symbolic bodies and joints are needed to define
        // the constraint symbolically
        std::vector<JointPtr<Scalar>> joints_in_current_cluster;
        std::vector<bool> independent_coordinates;
        std::vector<Body<SX>> bodies_sx;
        std::map<std::string, JointPtr<SX>> joints_sx;
        registerBodiesInUrdfCluster(cluster, joints_in_current_cluster, independent_coordinates,
                                    bodies_sx, joints_sx);

        // Verify that all constraints in the cluster are of the same type
        std::vector<urdf::ConstraintSharedPtr> cluster_constraints;
        for (urdf::LinkSharedPtr link : *cluster)
        {
            for (urdf::ConstraintSharedPtr constraint : link->constraints)
            {
                cluster_constraints.push_back(constraint);
            }
        }

        // Error if there are no constraints
        if (cluster_constraints.size() == 0)
        {
            throw std::runtime_error("Cluster must have at least one constraint");
        }

        // Verify that all constraints in the cluster are of the same type
        auto constraint_class_type = cluster_constraints.front()->class_type;
        for (urdf::ConstraintSharedPtr constraint : cluster_constraints)
        {
            if (constraint->class_type != constraint_class_type)
                throw std::runtime_error("All constraints in cluster must be of same class type");
        }

        // And so here is where we do the hard split on the constraint class type
        if (constraint_class_type == urdf::Constraint::LOOP)
        {
            // TODO(@MatthewChignoli): Now check that all of the loop constraints come from the same joint type

            // For each constraint, collect the captures that will be supplied to the lambda 
            // function encoding the constraint
            std::vector<LoopConstraintCapture> constraint_captures;
            for (urdf::ConstraintSharedPtr constraint : cluster_constraints)
            {
                urdf::LoopConstraintSharedPtr loop_constraint =
                    std::dynamic_pointer_cast<urdf::LoopConstraint>(constraint);

                std::vector<Body<SX>> nca_to_predecessor_subchain, nca_to_successor_subchain;

                // TODO(@MatthewChignoli): Helper function for this?
                std::string link_name = constraint->predecessor_link_name;
                while (link_name != constraint->nearest_common_ancestor_name)
                {
                    const Body<Scalar> &body_i = body(link_name);
                    const int sub_index = body_i.sub_index_within_cluster_;
                    nca_to_predecessor_subchain.insert(nca_to_predecessor_subchain.begin(),
                                                       bodies_sx[sub_index]);

                    const int& parent_index = body_i.parent_index_;
                    if (parent_index == -1)
                        break;
                    link_name = body(parent_index).name_;
                }

                link_name = constraint->successor_link_name;
                while (link_name != constraint->nearest_common_ancestor_name)
                {
                    const Body<Scalar> &body_i = body(link_name);
                    const int sub_index = body_i.sub_index_within_cluster_;
                    nca_to_successor_subchain.insert(nca_to_successor_subchain.begin(),
                                                    bodies_sx[sub_index]);
                    
                    const int& parent_index = body_i.parent_index_;
                    if (parent_index == -1)
                        break;
                    link_name = body(parent_index).name_;
                }

                LoopConstraintCapture capture;
                capture.nca_to_predecessor_subchain = nca_to_predecessor_subchain;
                capture.nca_to_successor_subchain = nca_to_successor_subchain;
                capture.predecessor_to_joint_origin_transform = loop_constraint->predecessor_to_joint_origin_transform;
                capture.successor_to_joint_origin_transform = loop_constraint->successor_to_joint_origin_transform;
                constraint_captures.push_back(capture);
            }

            // TODO(@MatthewChignoli): Would like to do some more sophisticated detection and specialization here. For example, if the jacobian of phi is constant, then we can use an explicit constraint. The best solution might just be codegen
            // Create constraints and add clusters
            std::string cluster_name = "cluster-" + std::to_string(cluster_nodes_.size());
            std::function<DVec<SX>(const JointCoordinate<SX> &)> phi;
            phi = implicitPositionConstraint(constraint_captures, joints_sx);

            using LoopConstraintType = LoopConstraint::GenericImplicit<Scalar>;
            std::shared_ptr<LoopConstraintType> generic_implicit;
            generic_implicit = std::make_shared<LoopConstraintType>(independent_coordinates, phi);
            appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
                cluster_name, bodies_in_current_cluster_,
                joints_in_current_cluster, generic_implicit);
        }
        else if (constraint_class_type == urdf::Constraint::COUPLING)
        {

            // For each constraint, collect the captures that will be supplied to the lambda 
            // function encoding the constraint
            std::vector<CouplingConstraintCapture> constraint_captures;
            for (urdf::ConstraintSharedPtr constraint : cluster_constraints)
            {
                urdf::CouplingConstraintSharedPtr coupling_constraint =
                    std::dynamic_pointer_cast<urdf::CouplingConstraint>(constraint);

                std::vector<Body<SX>> nca_to_predecessor_subchain, nca_to_successor_subchain;
                
                std::string link_name = constraint->predecessor_link_name;
                while (link_name != constraint->nearest_common_ancestor_name)
                {
                    const Body<Scalar> &body_i = body(link_name);
                    const int sub_index = body_i.sub_index_within_cluster_;
                    nca_to_predecessor_subchain.insert(nca_to_predecessor_subchain.begin(),
                                                       bodies_sx[sub_index]);

                    const int &parent_index = body_i.parent_index_;
                    if (parent_index == -1)
                        break;
                    link_name = body(parent_index).name_;
                }

                link_name = constraint->successor_link_name;
                while (link_name != constraint->nearest_common_ancestor_name)
                {
                    const Body<Scalar> &body_i = body(link_name);
                    const int sub_index = body_i.sub_index_within_cluster_;
                    nca_to_successor_subchain.insert(nca_to_successor_subchain.begin(),
                                                     bodies_sx[sub_index]);

                    const int &parent_index = body_i.parent_index_;
                    if (parent_index == -1)
                        break;
                    link_name = body(parent_index).name_;
                }

                CouplingConstraintCapture capture;
                capture.nca_to_predecessor_subchain = nca_to_predecessor_subchain;
                capture.nca_to_successor_subchain = nca_to_successor_subchain;
                capture.ratio = coupling_constraint->ratio;
                constraint_captures.push_back(capture);
            }

            // TODO(@MatthewChignoli): Would like to do some more sophisticated detection and specialization here. For example, if the jacobian of phi is constant, then we can use an explicit constraint. The best solution might just be codegen
            // Create constraints and add clusters
            std::string cluster_name = "cluster-" + std::to_string(cluster_nodes_.size());
            using KGPair = std::pair<DMat<Scalar>, DMat<Scalar>>;
            KGPair K_G = explicitRollingConstraint(constraint_captures,
                                                   independent_coordinates);

            std::shared_ptr<LoopConstraint::Static<Scalar>> static_constraint;
            static_constraint = std::make_shared<LoopConstraint::Static<Scalar>>(K_G.second,
                                                                                 K_G.first);
            appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
                cluster_name, bodies_in_current_cluster_,
                joints_in_current_cluster, static_constraint);
        }
        else
        {
            throw std::runtime_error("Constraint class type not supported");
        }
    }

    template <typename Scalar, typename OriTpl>
    void ClusterTreeModel<Scalar, OriTpl>::appendSimpleRevoluteJointFromUrdfCluster(
        urdf::LinkSharedPtr link)
    {
        std::string name = link->name;
        std::string parent_name = link->getParent()->name;
        SpatialInertia<Scalar> inertia(link->inertial);
        spatial::Transform<Scalar> xtree(link->parent_joint->parent_to_joint_origin_transform);
        ori::CoordinateAxis axis = ori::urdfAxisToCoordinateAxis(link->parent_joint->axis);
        using Revolute = ClusterJoints::Revolute<Scalar>;
        appendBody<Revolute>(name, inertia, parent_name, xtree, axis, link->parent_joint->name);
    }

    template <typename Scalar, typename OriTpl>
    void ClusterTreeModel<Scalar, OriTpl>::appendSimpleFloatingJointFromUrdfCluster(
        urdf::LinkSharedPtr link)
    {
        if (cluster_nodes_.size() > 0)
            throw std::runtime_error("Floating joint must be the first joint in the system");

        std::string name = link->name;
        std::string parent_name = link->getParent()->name;
        SpatialInertia<Scalar> inertia(link->inertial);
        spatial::Transform<Scalar> xtree(link->parent_joint->parent_to_joint_origin_transform);
        using Free = ClusterJoints::Free<Scalar, OriTpl>;
        appendBody<Free>(name, inertia, parent_name, xtree, link->parent_joint->name);
    }

    template <typename Scalar, typename OriTpl>
    void ClusterTreeModel<Scalar, OriTpl>::registerBodiesInUrdfCluster(
        urdf::ClusterSharedPtr cluster,
        std::vector<JointPtr<Scalar>> &joints,
        std::vector<bool> &independent_coordinates,
        std::vector<Body<SX>> &bodies_sx,
        std::map<std::string, JointPtr<SX>> &joints_sx)
    {
        std::vector<urdf::LinkSharedPtr> unregistered_links = *cluster;
        while (unregistered_links.size() > 0)
        {
            std::vector<urdf::LinkSharedPtr> unregistered_links_next;
            for (urdf::LinkSharedPtr link : unregistered_links)
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
                std::string joint_name = link->parent_joint->name;
                joints.push_back(std::make_shared<Joints::Revolute<Scalar>>(axis, joint_name));
                joints_sx.insert({name, std::make_shared<Joints::Revolute<SX>>(axis, joint_name)});

                // Extract independent coordinates from joint
                independent_coordinates.push_back(link->parent_joint->independent);
            }

            unregistered_links = unregistered_links_next;
        }
    }

    // TODO(@MatthewChignoli): This needs to depend on the joint type
    template <typename Scalar, typename OriTpl>
    std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
    ClusterTreeModel<Scalar, OriTpl>::implicitPositionConstraint(
        std::vector<LoopConstraintCapture> &captures,
        std::map<std::string, JointPtr<SX>> joints_sx)
    {
        return [captures, joints_sx](const JointCoordinate<SX> &q)
        {
            // TODO(@MatthewChignoli): There is a bug with the joint idx. It assumes that joints are ordered so that pred sub tree comes first, in order
            int jidx = 0;
            using Xform = spatial::Transform<SX>;

            DVec<SX> phi_out = DVec<SX>(0);
            for (size_t i = 0; i < captures.size(); i++)
            {
                const LoopConstraintCapture &capture = captures[i];

                // Through predecessor
                Xform X_via_predecessor;
                for (const Body<SX> &body : capture.nca_to_predecessor_subchain)
                {
                    JointPtr<SX> joint = joints_sx.at(body.name_);
                    joint->updateKinematics(q.segment(jidx, joint->numPositions()),
                                            DVec<SX>::Zero(joint->numVelocities()));
                    jidx += joint->numPositions();
                    X_via_predecessor = joint->XJ() * body.Xtree_ * X_via_predecessor;
                }
                X_via_predecessor = Xform(capture.predecessor_to_joint_origin_transform) *
                                    X_via_predecessor;
                Vec3<SX> r_nca_to_constraint_via_predecessor = X_via_predecessor.getTranslation();

                // Through successor
                Xform X_via_successor;
                for (const Body<SX> &body : capture.nca_to_successor_subchain)
                {
                    JointPtr<SX> joint = joints_sx.at(body.name_);
                    joint->updateKinematics(q.segment(jidx, joint->numPositions()),
                                            DVec<SX>::Zero(joint->numVelocities()));
                    jidx += joint->numPositions();
                    X_via_successor = joint->XJ() * body.Xtree_ * X_via_successor;
                }
                X_via_successor = Xform(capture.successor_to_joint_origin_transform) *
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

    template <typename Scalar, typename OriTpl>
    std::pair<DMat<Scalar>, DMat<Scalar>>
    ClusterTreeModel<Scalar, OriTpl>::explicitRollingConstraint(
        std::vector<CouplingConstraintCapture> &captures,
        std::vector<bool> independent_coordinates)
    {
        // TODO(@MatthewChignoli): Assumes all joints are 1 DOF?
        DMat<Scalar> K = DMat<Scalar>::Zero(captures.size(), independent_coordinates.size());
        for (size_t i = 0; i < captures.size(); i++)
        {
            const CouplingConstraintCapture &capture = captures[i];

            // Through predecessor
            for (const Body<SX> &body : capture.nca_to_predecessor_subchain)
            {
                K(i, body.sub_index_within_cluster_) = capture.ratio;
            }

            // Through successor
            for (const Body<SX> &body : capture.nca_to_successor_subchain)
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
