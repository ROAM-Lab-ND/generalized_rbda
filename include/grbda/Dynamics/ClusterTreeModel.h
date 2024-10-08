#ifndef GRBDA_CLUSTER_TREE_MODEL_H
#define GRBDA_CLUSTER_TREE_MODEL_H

#include <string>
#include <vector>
#include <stdio.h>
#include <stdexcept>
#include <optional>

#include "grbda/Dynamics/TreeModel.h"
#include "grbda/Dynamics/ClusterJoints/LoopConstraintCaptures.h"
#include "grbda/Dynamics/Nodes/ClusterTreeNode.h"
#include "grbda/Urdf/urdf_parser.h"

namespace grbda
{
    template <typename Scalar>
    using ClusterTreeNodePtr = std::shared_ptr<ClusterTreeNode<Scalar>>;

    using UrdfModelPtr = std::shared_ptr<urdf::ModelInterface>;
    using UrdfClusterPtr = std::shared_ptr<urdf::Cluster>;
    using UrdfLinkPtr = std::shared_ptr<urdf::Link>;
    using UrdfConstraintPtr = std::shared_ptr<urdf::Constraint>;

    /*!
     * Class to represent a floating base rigid body model with rotors and ground contacts.
     */
    template <typename Scalar = double>
    class ClusterTreeModel : public TreeModel<Scalar>
    {
    public:
        ClusterTreeModel()
        {
            body_name_to_body_index_["ground"] = -1;
            body_index_to_cluster_index_[-1] = -1;
        }
        ~ClusterTreeModel() {}

        
        template <typename OrientationRepresentation = ori_representation::Quaternion>
        void buildModelFromURDF(
            const std::string &urdf_filename, bool floating_base)
        {
            std::shared_ptr<urdf::ModelInterface> model;
            model = urdf::parseURDFFile(urdf_filename, false);
            buildFromUrdfModelInterface<OrientationRepresentation>(model, floating_base);
        }
        
        template <typename OrientationRepresentation = ori_representation::Quaternion>
        void buildModelFromURDF(
            const std::vector<std::string> &urdf_filenames, bool floating_base)
        {
            std::shared_ptr<urdf::ModelInterface> model;
            model = urdf::parseURDFFiles(urdf_filenames, false);
            buildFromUrdfModelInterface<OrientationRepresentation>(model, floating_base);
        }

        // The standard process for appending a cluster to the tree is to register all the bodies
        // in  given cluster and then append them as a cluster by specifying the type of cluster
        // joint that connects them
        Body<Scalar> registerBody(const std::string name, const SpatialInertia<Scalar> inertia,
                                  const std::string parent, const spatial::Transform<Scalar> Xtree);

        template <typename ClusterJointType, typename... Args>
        void appendRegisteredBodiesAsCluster(const std::string name, Args &&...args)
        {
            auto cluster_joint = std::make_shared<ClusterJointType>(args...);
            appendRegisteredBodiesAsCluster(name, cluster_joint);
        }

        // Alternatively, this function can be used when appending individual bodies to the model
        template <typename ClusterJointType, typename... Args>
        void appendBody(const std::string name, const SpatialInertia<Scalar> inertia,
                        const std::string parent_name, const spatial::Transform<Scalar> Xtree,
                        Args &&...args)
        {
            Body<Scalar> body = registerBody(name, inertia, parent_name, Xtree);
            std::shared_ptr<ClusterJointType> joint = std::make_shared<ClusterJointType>(body,
                                                                                         args...);
            appendRegisteredBodiesAsCluster(name, joint);
        }

        void appendContactPoint(const std::string body_name, const Vec3<Scalar> &local_offset,
                                const std::string contact_name, const bool is_end_eff = false);
        void appendContactBox(const std::string body_name, const Vec3<Scalar> &box_dimensions);

        // Current implementation every operational space has size 6x1
        void appendEndEffector(const std::string body_name, const Vec3<Scalar> &local_offset,
                               const std::string end_effector_name);

        void print() const;

        typedef std::pair<DVec<Scalar>, DVec<Scalar>> StatePair;
        void setState(const ModelState<Scalar> &model_state);
        void setState(const StatePair &q_qd_pair);
        void setState(const DVec<Scalar>& q_qd_vec);
        ModelState<Scalar> stateVectorToModelState(const StatePair& q_qd_pair);

        int getNumBodies() const override { return (int)bodies_.size(); }

        // NOTE: A body's "cluster ancestor" is the nearest member in the body's supporting tree
        // that belongs to a different cluster. This function is only intended to be run when
        // registering bodies. At all other times, a bodies cluster ancestor should be accesses via
        // body.cluster_ancestor_index_
        int getClusterAncestorIndexFromParent(const int body_index);

        int getSubIndexWithinClusterForBody(const Body<Scalar> &body) const;
        int getSubIndexWithinClusterForBody(const int body_index) const;
        int getSubIndexWithinClusterForBody(const std::string &body_name) const;

        int getNumBodiesInCluster(const ClusterTreeNodePtr<Scalar> cluster) const;
        int getNumBodiesInCluster(const int cluster_index) const;
        int getNumBodiesInCluster(const std::string &cluster_name) const;

        int getIndexOfClusterContainingBody(const Body<Scalar> &body);
        int getIndexOfClusterContainingBody(const int body_index);
        int getIndexOfClusterContainingBody(const std::string &body_name);

        ClusterTreeNodePtr<Scalar> getClusterContainingBody(const Body<Scalar> &body);
        ClusterTreeNodePtr<Scalar> getClusterContainingBody(const int body_index);
        ClusterTreeNodePtr<Scalar> getClusterContainingBody(const std::string &body_name);

        int getIndexOfParentClusterFromBodies(const std::vector<Body<Scalar>> &bodies);

        const Body<Scalar> &getBody(int index) const override { return bodies_[index]; }
        const TreeNodePtr<Scalar> getNodeContainingBody(int index) override
        {
            return this->nodes_[getIndexOfClusterContainingBody(index)];
        }

        const std::vector<Body<Scalar>> &bodies() const { return bodies_; }
        const std::vector<ClusterTreeNodePtr<Scalar>> &clusters() const { return cluster_nodes_; }

        const Body<Scalar> &body(const int body_index) const { return bodies_[body_index]; }
        const Body<Scalar> &body(const std::string& body_name) const override
        {
            return bodies_[body_name_to_body_index_.at(body_name)];
        }

        const ClusterTreeNodePtr<Scalar> cluster(const int cluster_index) const
        {
            return cluster_nodes_[cluster_index];
        }

        Vec3<Scalar> getPosition(const std::string &body_name,
                                 const Vec3<Scalar> &offset = Vec3<Scalar>::Zero()) override;
        Mat3<Scalar> getOrientation(const std::string &body_name) override;
        Vec3<Scalar> getLinearVelocity(const std::string &body_name,
                                       const Vec3<Scalar> &offset = Vec3<Scalar>::Zero()) override;
        Vec3<Scalar> getAngularVelocity(const std::string &body_name) override;
        Vec3<Scalar> getLinearAcceleration(const DVec<Scalar> &qdd,
                                           const std::string &body_name,
                                           const Vec3<Scalar> &offset = Vec3<Scalar>::Zero())override;
        Vec3<Scalar> getAngularAcceleration(const DVec<Scalar> &qdd,
                                            const std::string &body_name) override;

        D6Mat<Scalar> contactJacobianBodyFrame(const std::string &cp_name) override;
        const D6Mat<Scalar> &contactJacobianWorldFrame(const std::string &cp_name) override;

        DVec<Scalar> inverseDynamics(const DVec<Scalar> &qdd) override;
        DVec<Scalar> forwardDynamics(const DVec<Scalar> &tau) override;
        DMat<Scalar> inverseOperationalSpaceInertiaMatrix() override;
        Scalar applyTestForce(const std::string &contact_point_name,
                              const Vec3<Scalar> &force, DVec<Scalar> &dstate_out) override;

        DMat<Scalar> getMassMatrix() override;
        DVec<Scalar> getBiasForceVector() override;

    protected:
        using SX = casadi::SX;

        template <typename OrientationRepresentation = ori_representation::Quaternion>
        void buildFromUrdfModelInterface(const UrdfModelPtr model, bool floating_base);
        
        void appendClustersViaDFS(std::map<UrdfClusterPtr, bool> &visited, UrdfClusterPtr cluster);
        void appendClusterFromUrdfCluster(UrdfClusterPtr cluster);
        void appendSimpleRevoluteJointFromUrdfCluster(UrdfLinkPtr link);
        void registerBodiesInUrdfCluster(UrdfClusterPtr cluster,
                                         std::vector<JointPtr<Scalar>>& joints,
                                         std::vector<bool>& independent_coordinates,
                                         std::vector<Body<SX>>& bodies_sx,
                                         std::map<std::string, JointPtr<SX>>& joints_sx);

        std::function<DVec<SX>(const JointCoordinate<SX> &)> implicitPositionConstraint(
            std::vector<PositionConstraintCapture> &captures,
            std::map<std::string, JointPtr<SX>> joints_sx);

        std::pair<DMat<Scalar>, DMat<Scalar>> explicitRollingConstraint(
            std::vector<RollingConstraintCapture> &captures,
            std::vector<bool> independent_coordinates);

        void appendRegisteredBodiesAsCluster(const std::string name,
                                             std::shared_ptr<ClusterJoints::Base<Scalar>> joint);

        void checkValidParentClusterForBodiesInCluster(const ClusterTreeNodePtr<Scalar> cluster);
        void checkValidParentClusterForBodiesInCluster(const int cluster_index);
        void checkValidParentClusterForBodiesInCluster(const std::string &cluster_nam);
        std::optional<int> searchClustersForBody(const int body_index);

        void resizeSystemMatrices();
        void resetCache() override;

        void updateArticulatedBodies();
        void updateForcePropagators();
        void updateQddEffects();

        DVec<Scalar> localCartesianForceAtPointToWorldPluckerForceOnCluster(
            const Vec3<Scalar> &force, const ContactPoint<Scalar> &contact_point);

        std::vector<Body<Scalar>> bodies_;
        std::vector<ClusterTreeNodePtr<Scalar>> cluster_nodes_;

        std::vector<Body<Scalar>> bodies_in_current_cluster_;

        std::unordered_map<std::string, int> body_name_to_body_index_;
        std::unordered_map<std::string, int> cluster_name_to_cluster_index_;
        std::unordered_map<int, int> body_index_to_cluster_index_;

        bool articulated_bodies_updated_ = false;
        bool force_propagators_updated_ = false;
        bool qdd_effects_updated_ = false;

        template <typename Scalar2>
        friend class RigidBodyTreeModel;

        template <typename Scalar2>
        friend class ReflectedInertiaTreeModel;
    };

} // namespace grbda

#endif // GRBDA_CLUSTER_TREE_MODEL_H
