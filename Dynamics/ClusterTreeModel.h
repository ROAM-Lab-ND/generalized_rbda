#pragma once

#include <string>
#include <vector>
#include <stdio.h>
#include <stdexcept>

#include "TreeModel.h"
#include "Dynamics/Nodes/ClusterTreeNode.h"

namespace grbda
{

    using namespace std;
    using namespace ori;
    using namespace spatial;

    using ClusterTreeNodePtr = std::shared_ptr<ClusterTreeNode>;

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class ClusterTreeModel : public TreeModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ClusterTreeModel()
        {
            body_name_to_body_index_["ground"] = -1;
            body_index_to_cluster_index_[-1] = -1;
        }
        ~ClusterTreeModel() {}

        // TODO(@MatthewChignoli): These are functions and members shared with FloatingBaseModel. Not sure how I want to deal with them moving forward. It's unclear which parts of Robot-Software need to change for compatiblity with GRBDA and which parts of GRBDA need to change for compatibility with Robot-Software. Some of this should be moved to TreeModel base class?

        Vec3<double> getPosition(const string &body_name);
        Mat3<double> getOrientation(const string &body_name);
        Vec3<double> getLinearVelocity(const string &body_name);
        Vec3<double> getAngularVelocity(const string &body_name);

        void resetExternalForces()
        {
            for (const int index : indices_of_nodes_experiencing_external_forces_)
                nodes_[index]->f_ext_.setZero();
            indices_of_nodes_experiencing_external_forces_.clear();

            resetCache();
        }

        // TODO(@MatthewChignoli): Want to delete this eventually
        vectorAligned<SVec<double>> _externalForces;

        void setExternalForces(const string &body_name, const SVec<double> &force)
        {
            const auto &body_i = body(body_name);
            const auto node = getNodeContainingBody(body_i.index_);
            node->applyForceToBody(force, body_i);

            // Add index to vector if vector does not already contain this cluster
            if (!vectorContainsIndex(indices_of_nodes_experiencing_external_forces_, node->index_))
                indices_of_nodes_experiencing_external_forces_.push_back(node->index_);
        }

        void setExternalForces(const unordered_map<string, SVec<double>> &ext_forces = {})
        {
            for (const auto &ext_force : ext_forces)
            {
                const string &body_name = ext_force.first;
                const SVec<double> &force = ext_force.second;
                setExternalForces(body_name, force);
            }
        }

        // TODO(@MatthewChignoli): For now we will use this, but I think maybe we want to create a contact point struct that holds this information
        const std::unordered_map<std::string, int> &contacts() const { return contact_name_to_contact_index_; }

        const Vec3<double> &pGC(const string &cp_name) const
        {
            return contactPoint(cp_name).position_;
        }

        const Vec3<double> &vGC(const string &cp_name) const
        {
            return contactPoint(cp_name).velocity_;
        }

        const string &gcParent(const string &cp_name) const
        {
            const int& body_index = contactPoint(cp_name).body_index_;
            return bodies_.at(body_index).name_;
        }

        // TODO(@MatthewChignoli): So all of this stuff get's deprecated

        void resetCalculationFlags() { resetCache(); }

        void setState(const DVec<double> &state)
        {
            const int nq = getNumPositions();
            const int nv = getNumDegreesOfFreedom();
            initializeIndependentStates(state.head(nq), state.tail(nv));
        }

        void contactJacobians() {}

        double applyTestForce(const int gc_index, const Vec3<double> &force_ics_at_contact,
                              DVec<double> &dstate_out) { return 0.; }

        DMat<double> massMatrix() { return DMat<double>::Zero(0, 0); }

        // TODO(@MatthewChignoli): Things to delete
        size_t _nGroundContact = 0;
        vector<size_t> _gcParent;
        vector<Vec3<double>> _pGC;
        vector<Vec3<double>> _vGC;
        vector<D3Mat<double>> _Jc;

        size_t _nJointLim = 0;
        vector<size_t> _JointLimID;         // hold the joint nb ID as defined in humanoid.h
        vector<double> _JointLimValueLower; // hold the joint nb ID as defined in humanoid.h
        vector<double> _JointLimValueUpper; // hold the joint nb ID as defined in humanoid.h

        /////////////////////////////////////

        Body registerBody(const string name, const SpatialInertia<double> inertia,
                          const string parent_name, const SpatialTransform Xtree);
        // TODO(@MatthewChignoli): need to clean this up, should have to make all of these vectors we we are only appending one body. Right now the append body function is silly. Because That function creates a body (by registering it), but it also requires a generalized joint... which requires a body...
        void appendBody(const string name, const SpatialInertia<double> inertia,
                        const string parent_name, const SpatialTransform Xtree,
                        shared_ptr<GeneralizedJoints::Base> joint);
        void appendRegisteredBodiesAsCluster(const string name,
                                             shared_ptr<GeneralizedJoints::Base> joint);
        void appendContactPoint(const string body_name,
                                const Vec3<double> &local_offset,
                                const string contact_point_name);
        void appendContactBox(const string body_name, const Vec3<double> &box_dimensions);

        void print() const;

        void initializeIndependentStates(const DVec<double> &y, const DVec<double> &yd) override;
	void initializeTelloIndependentStates(const DVec<double> &q, const DVec<double> &y_dot);

        int getNumBodies() const override { return (int)bodies_.size(); }

        // NOTE: A body's "cluster ancestor" is the nearest member in the body's supporting tree that belongs to a different cluster
        int getClusterAncestorIndexFromParent(const int body_index);

        int getSubIndexWithinClusterForBody(const Body &body) const;
        int getSubIndexWithinClusterForBody(const int body_index) const;
        int getSubIndexWithinClusterForBody(const string &body_name) const;

        int getNumBodiesInCluster(const ClusterTreeNodePtr cluster) const;
        int getNumBodiesInCluster(const int cluster_index) const;
        int getNumBodiesInCluster(const string &cluster_name) const;

        int getIndexOfClusterContainingBody(const Body &body);
        int getIndexOfClusterContainingBody(const int body_index);
        int getIndexOfClusterContainingBody(const string &body_name);

        ClusterTreeNodePtr getClusterContainingBody(const Body &body);
        ClusterTreeNodePtr getClusterContainingBody(const int body_index);
        ClusterTreeNodePtr getClusterContainingBody(const string &body_name);

        int getIndexOfParentClusterFromBodies(const vector<Body> &bodies);

        const Body &getBody(int index) const override { return bodies_[index]; }
        const TreeNodePtr getNodeContainingBody(int index) override
        {
            return nodes_[getIndexOfClusterContainingBody(index)];
        }

        const vector<Body> &bodies() const { return bodies_; }
        const vector<ClusterTreeNodePtr> &clusters() const { return cluster_nodes_; }

        const Body &body(const int body_index) const { return bodies_[body_index]; }
        const Body &body(const string body_name) const
        {
            return bodies_[body_name_to_body_index_.at(body_name)];
        }

        const ClusterTreeNodePtr cluster(const int cluster_index) const { return cluster_nodes_[cluster_index]; }
        const ClusterTreeNodePtr cluster(const string &cluster_name) const
        {
            return cluster_nodes_[cluster_name_to_cluster_index_.at(cluster_name)];
        }

        DVec<double> inverseDyamics(const DVec<double> &qdd);
        DVec<double> forwardDynamics(const DVec<double> &tau) override;
        double applyLocalFrameTestForceAtConactPoint(const Vec3<double> &force,
                                                     const string &contact_point_name,
                                                     DVec<double> &dstate_out);

        DMat<double> getMassMatrix() override;
        DVec<double> getBiasForceVector() override;

    private:
        void checkValidParentClusterForBodiesInCluster(const ClusterTreeNodePtr cluster);
        void checkValidParentClusterForBodiesInCluster(const int cluster_index);
        void checkValidParentClusterForBodiesInCluster(const string &cluster_nam);
        optional<int> searchClustersForBody(const int body_index);

        void resizeSystemMatrices();
        void resetCache() override;

        void updateArticulatedBodies();
        void updateForcePropagators();
        void updateQddEffects();

        DVec<double> localCartesianForceAtPointToWorldPluckerForceOnCluster(
            const Vec3<double> &force, const ContactPoint &contact_point);

        vector<Body> bodies_;
        vector<ClusterTreeNodePtr> cluster_nodes_;

        vector<Body> bodies_in_current_cluster_;

        UnorderedMap<string, int> body_name_to_body_index_;
        UnorderedMap<string, int> cluster_name_to_cluster_index_;
        UnorderedMap<int, int> body_index_to_cluster_index_;

        bool articulated_bodies_updated_ = false;
        bool force_propagators_updated_ = false;
        bool qdd_effects_updated_ = false;

        friend class RigidBodyTreeModel;
        friend class ReflectedInertiaTreeModel;
    };

} // namespace grbda
