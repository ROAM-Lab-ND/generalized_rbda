#pragma once

#include <string>
#include <vector>
#include <stdio.h>
#include <stdexcept>

#include "TreeModel.h"
#include "Dynamics/Nodes/ClusterTreeNode.h"

namespace grbda
{

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

        // TODO(@MatthewChignoli): These are functions and members shared with FloatingBaseModel. Not sure how I want to deal with them moving forward. It's unclear which parts of Robot-Software need to change for compatiblity with GRBDA and which parts of GRBDA need to change for compatibility with Robot-Software
        Vec3<double> getPosition(const int link_idx) { return Vec3<double>::Zero(); }
        Mat3<double> getOrientation(const int link_idx) { return Mat3<double>::Zero(); }
        Vec3<double> getLinearVelocity(const int link_idx) { return Vec3<double>::Zero(); }
        Vec3<double> getAngularVelocity(const int link_idx) { return Vec3<double>::Zero(); }

        void resetExternalForces()
        {
            for (int i = 0; i < getNumDegreesOfFreedom(); i++)
            {
                _externalForces[i] = SVec<double>::Zero();
            }
        }
        vectorAligned<SVec<double>> _externalForces;

        size_t _nGroundContact = 0;
        std::vector<size_t> _gcParent;
        std::vector<Vec3<double>> _pGC;
        std::vector<Vec3<double>> _vGC;
        std::vector<D3Mat<double>> _Jc;

        size_t _nJointLim = 0;
        vector<size_t> _JointLimID;    // hold the joint nb ID as defined in humanoid.h
        vector<double> _JointLimValueLower; // hold the joint nb ID as defined in humanoid.h
        vector<double> _JointLimValueUpper; // hold the joint nb ID as defined in humanoid.h

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

        /////////////////////////////////////

        Body registerBody(const std::string name, const SpatialInertia<double> inertia,
                          const std::string parent_name, const SpatialTransform Xtree);
        // TODO(@MatthewChignoli): need to clean this up, should have to make all of these std::vectors we we are only appending one body. Right now the append body function is silly. Because That function creates a body (by registering it), but it also requires a generalized joint... which requires a body...
        void appendBody(const std::string name, const SpatialInertia<double> inertia,
                        const std::string parent_name, const SpatialTransform Xtree,
                        std::shared_ptr<GeneralizedJoints::Base> joint);
        void appendRegisteredBodiesAsCluster(const std::string name,
                                             std::shared_ptr<GeneralizedJoints::Base> joint);
        void appendContactPoint(const std::string body_name,
                                const Vec3<double> &local_offset,
                                const std::string contact_point_name);

        void print() const;

        void initializeIndependentStates(const DVec<double> &y, const DVec<double> &yd) override;
	void initializeTelloIndependentStates(const DVec<double> &q, const DVec<double> &y_dot);

        int getNumBodies() const override { return (int)bodies_.size(); }

        // NOTE: A body's "cluster ancestor" is the nearest member in the body's supporting tree that belongs to a different cluster
        int getClusterAncestorIndexFromParent(const int body_index);

        int getSubIndexWithinClusterForBody(const Body &body) const;
        int getSubIndexWithinClusterForBody(const int body_index) const;
        int getSubIndexWithinClusterForBody(const std::string &body_name) const;

        int getNumBodiesInCluster(const ClusterTreeNodePtr cluster) const;
        int getNumBodiesInCluster(const int cluster_index) const;
        int getNumBodiesInCluster(const std::string &cluster_name) const;

        int getIndexOfClusterContainingBody(const Body &body);
        int getIndexOfClusterContainingBody(const int body_index);
        int getIndexOfClusterContainingBody(const std::string &body_name);

        ClusterTreeNodePtr getClusterContainingBody(const Body &body);
        ClusterTreeNodePtr getClusterContainingBody(const int body_index);
        ClusterTreeNodePtr getClusterContainingBody(const std::string &body_name);

        int getIndexOfParentClusterFromBodies(const std::vector<Body> &bodies);

        const Body &getBody(int index) const override { return bodies_[index]; }
        const TreeNodePtr getNodeContainingBody(int index) override
        {
            return nodes_[getIndexOfClusterContainingBody(index)];
        }

        const std::vector<Body> &bodies() const { return bodies_; }
        const std::vector<ClusterTreeNodePtr> &clusters() const { return cluster_nodes_; }

        const Body &body(const int body_index) const { return bodies_[body_index]; }
        const Body &body(const std::string body_name) const
        {
            return bodies_[body_name_to_body_index_.at(body_name)];
        }

        const ClusterTreeNodePtr cluster(const int cluster_index) const { return cluster_nodes_[cluster_index]; }
        const ClusterTreeNodePtr cluster(const std::string &cluster_name) const
        {
            return cluster_nodes_[cluster_name_to_cluster_index_.at(cluster_name)];
        }

        DVec<double> inverseDyamics(const DVec<double> &qdd);
        DVec<double> forwardDynamics(const DVec<double> &tau) override;
        double applyLocalFrameTestForceAtConactPoint(const Vec3<double> &force,
                                                     const std::string &contact_point_name,
                                                     DVec<double> &dstate_out);

        DMat<double> getMassMatrix() override;
        DVec<double> getBiasForceVector() override;

    private:
        void checkValidParentClusterForBodiesInCluster(const ClusterTreeNodePtr cluster);
        void checkValidParentClusterForBodiesInCluster(const int cluster_index);
        void checkValidParentClusterForBodiesInCluster(const std::string &cluster_nam);
        std::optional<int> searchClustersForBody(const int body_index);

        void resizeSystemMatrices();
        void resetCache() override;

        void updateArticulatedBodies();
        void updateForcePropagators();
        void updateQddEffects();

        DVec<double> localCartesianForceAtPointToWorldPluckerForceOnCluster(
            const Vec3<double> &force, const ContactPoint &contact_point);

        std::vector<Body> bodies_;
        std::vector<ClusterTreeNodePtr> cluster_nodes_;

        std::vector<Body> bodies_in_current_cluster_;

        UnorderedMap<std::string, int> body_name_to_body_index_;
        UnorderedMap<std::string, int> cluster_name_to_cluster_index_;
        UnorderedMap<int, int> body_index_to_cluster_index_;

        bool articulated_bodies_updated_ = false;
        bool force_propagators_updated_ = false;
        bool qdd_effects_updated_ = false;

        friend class RigidBodyTreeModel;
        friend class ReflectedInertiaTreeModel;
    };

} // namespace grbda