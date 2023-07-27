#pragma once

#include <string>
#include <vector>
#include <stdio.h>
#include <stdexcept>

#include "TreeModel.h"
#include "Dynamics/Nodes/ClusterTreeNode.h"
#include "Dynamics/Nodes/ClusterTreeNodeVisitors.h"
#ifdef TIMING_STATS
#include "Utils/Utilities/Timer.h"
#endif

namespace grbda
{

    using namespace std;
    using namespace ori;
    using namespace spatial;

#ifdef TIMING_STATS
    struct ClusterTreeTimingStatistics
    {
        double forward_kinematics_time = 0.0;
        double update_articulated_bodies_time = 0.0;
        double forward_pass1_time = 0.0;
        double external_force_time = 0.0;
        double backward_pass_time = 0.0;
        double forward_pass2_time = 0.0;
        double invert_xform_spatial_inertia_time = 0.0;
        double update_and_solve_D_time = 0.0;
        double reset_IA_time = 0.0;

        void zero()
        {
            forward_kinematics_time = 0.0;
            update_articulated_bodies_time = 0.0;
            forward_pass1_time = 0.0;
            external_force_time = 0.0;
            backward_pass_time = 0.0;
            forward_pass2_time = 0.0;
            invert_xform_spatial_inertia_time = 0.0;
            update_and_solve_D_time = 0.0;
            reset_IA_time = 0.0;
        }

        ClusterTreeTimingStatistics &operator+=(const ClusterTreeTimingStatistics &other)
        {
            forward_kinematics_time += other.forward_kinematics_time;
            update_articulated_bodies_time += other.update_articulated_bodies_time;
            forward_pass1_time += other.forward_pass1_time;
            external_force_time += other.external_force_time;
            backward_pass_time += other.backward_pass_time;
            forward_pass2_time += other.forward_pass2_time;
            invert_xform_spatial_inertia_time += other.invert_xform_spatial_inertia_time;
            update_and_solve_D_time += other.update_and_solve_D_time;
            reset_IA_time += other.reset_IA_time;
            return *this;
        }

        ClusterTreeTimingStatistics &operator/=(const double &scalar)
        {
            forward_kinematics_time /= scalar;
            update_articulated_bodies_time /= scalar;
            forward_pass1_time /= scalar;
            external_force_time /= scalar;
            backward_pass_time /= scalar;
            forward_pass2_time /= scalar;
            invert_xform_spatial_inertia_time /= scalar;
            update_and_solve_D_time /= scalar;
            reset_IA_time /= scalar;
            return *this;
        }
    };
#endif

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class ClusterTreeModel : public TreeModel<ClusterTreeModel>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef TreeModel<ClusterTreeModel>::NodeTypeVariants NodeTypeVariants;

        ClusterTreeModel()
        {
            body_name_to_body_index_["ground"] = -1;
            body_index_to_cluster_index_[-1] = -1;
        }
        ~ClusterTreeModel() {}

        // TODO(@MatthewChignoli): These are functions and members shared with FloatingBaseModel. Not sure how I want to deal with them moving forward. It's unclear which parts of Robot-Software need to change for compatiblity with GRBDA and which parts of GRBDA need to change for compatibility with Robot-Software.
        Vec3<double> getPosition(const string &body_name);
        Mat3<double> getOrientation(const string &body_name);
        Vec3<double> getLinearVelocity(const string &body_name);
        Vec3<double> getAngularVelocity(const string &body_name);

        void setState(const DVec<double> &state);
        void setExternalForces(const string &body_name, const SVec<double> &force);
        void setExternalForces(const unordered_map<string, SVec<double>> &ext_forces = {});
        void resetExternalForces();
        void resetCalculationFlags() { resetCache(); }

        void contactJacobians();
        const std::unordered_map<std::string, int> &contacts() const;
        const Vec3<double> &pGC(const string &cp_name) const;
        const Vec3<double> &vGC(const string &cp_name) const;
        const string &gcParent(const string &cp_name) const;
        D3Mat<double> Jc(const string &cp_name) const;

        DMat<double> massMatrix() { return getMassMatrix(); }
        double applyTestForce(const string &cp_name,
                              const Vec3<double> &force_ics_at_contact,
                              DVec<double> &dstate_out);

        void addJointLim(size_t jointID, double joint_lim_value_lower,
                         double joint_lim_value_upper);
        size_t _nJointLim = 0;
        vector<size_t> _JointLimID;
        vector<double> _JointLimValueLower;
        vector<double> _JointLimValueUpper;

        /////////////////////////////////////

        Body registerBody(const string name, const SpatialInertia<double> inertia,
                          const string parent_name, const SpatialTransform Xtree);
        void appendRegisteredBodiesAsCluster(const string name,
                                             shared_ptr<GeneralizedJoints::Base> joint);
        void appendContactPoint(const string body_name,
                                const Vec3<double> &local_offset,
                                const string contact_point_name);
        void appendContactBox(const string body_name, const Vec3<double> &box_dimensions);

        void print() const;

        void initializeState(const ModelState &model_state);

        int getNumBodies() const { return (int)bodies_.size(); }

        // NOTE: A body's "cluster ancestor" is the nearest member in the body's supporting tree
        // that belongs to a different cluster. This function is only intended to be run when
        // registering bodies. At all other times, a bodies cluster ancestor should be accesses via
        // body.cluster_ancestor_index_
        int getClusterAncestorIndexFromParent(const int body_index);

        int getSubIndexWithinClusterForBody(const int body_index) const;

        int getNumBodiesInCluster(const int cluster_index) const;
        int getNumBodiesInCluster(const string &cluster_name) const;

        int getIndexOfClusterContainingBody(const Body &body);
        int getIndexOfClusterContainingBody(const int body_index);
        int getIndexOfClusterContainingBody(const string &body_name);

        // TODO(@MatthewChignoli): Aren't these the same as get node containing? Should clean that up later
        NodeTypeVariants &getClusterVarContainingBody(const Body &body);
        NodeTypeVariants &getClusterVarContainingBody(const int body_index);
        NodeTypeVariants &getClusterVarContainingBody(const string &body_name);

        int getIndexOfParentClusterFromBodies(const vector<Body> &bodies);

        const Body &getBody(int index) const { return bodies_[index]; }
        NodeTypeVariants &getNodeVariantContainingBody(int index)
        {
            return nodes_variants_[getIndexOfClusterContainingBody(index)];
        }

        const vector<Body> &bodies() const { return bodies_; }
        // TODO(@MatthewChignoli): delete
        // const vector<NodeType> &clusters() const { return nodes_; }
        const vector<NodeTypeVariants> &clusterVariants() const { return nodes_variants_; }

        const Body &body(const int body_index) const { return bodies_[body_index]; }
        const Body &body(const string body_name) const
        {
            return bodies_[body_name_to_body_index_.at(body_name)];
        }

        const NodeTypeVariants &clusterVariant(const int cluster_index) const
        {
            return nodes_variants_[cluster_index];
        }
        const NodeTypeVariants &clusterVariant(const string &cluster_name) const
        {
            return nodes_variants_[cluster_name_to_cluster_index_.at(cluster_name)];
        }

        DVec<double> inverseDynamics(const DVec<double> &qdd);
        DVec<double> forwardDynamics(const DVec<double> &tau);
        double applyLocalFrameTestForceAtContactPoint(const Vec3<double> &force,
                                                      const string &contact_point_name,
                                                      DVec<double> &dstate_out);

        DMat<double> getMassMatrix();
        DVec<double> getBiasForceVector();

#ifdef TIMING_STATS
        const ClusterTreeTimingStatistics &getTimingStatistics() const
        {
            return timing_statistics_;
        }
#endif

    private:
        // void checkValidParentClusterForBodiesInCluster(const NodeType &cluster);
        void checkValidParentClusterForBodiesInCluster(const NodeTypeVariants &cluster);

        optional<int> searchClustersForBody(const int body_index);

        void resizeSystemMatrices();
        void resetCache();

        void updateArticulatedBodies();
        void updateForcePropagators();
        void updateQddEffects();

        DVec<double> localCartesianForceAtPointToWorldPluckerForceOnCluster(
            const Vec3<double> &force, const ContactPoint &contact_point);

        vector<Body> bodies_;
        vector<Body> bodies_in_current_cluster_;
        std::unordered_map<string, int> body_name_to_body_index_;
        std::unordered_map<string, int> cluster_name_to_cluster_index_;
        std::unordered_map<int, int> body_index_to_cluster_index_;

        bool articulated_bodies_updated_ = false;
        bool force_propagators_updated_ = false;
        bool qdd_effects_updated_ = false;

#ifdef TIMING_STATS
        Timer timer_;
        ClusterTreeTimingStatistics timing_statistics_;
#endif

        friend class TreeModel<ClusterTreeModel>;
        friend class RigidBodyTreeModel;
        friend class ReflectedInertiaTreeModel;
    };

} // namespace grbda
