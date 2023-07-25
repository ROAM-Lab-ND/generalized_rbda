#pragma once

#include "ClusterTreeModel.h"
#include "Dynamics/Nodes/ReflectedInertiaTreeNode.h"

namespace grbda
{

    using namespace ori;
    using namespace spatial;

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class ReflectedInertiaTreeModel : public TreeModel<ReflectedInertiaTreeModel, ReflectedInertiaTreeNode>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReflectedInertiaTreeModel(const ClusterTreeModel &cluster_tree_model,
                                  bool use_off_diagonal_terms = true);

        // TODO(@MatthewChignoli): These are functions and members shared with FloatingBaseModel. Not sure how I want to deal with them moving forward. It's unclear which parts of Robot-Software need to change for compatiblity with GRBDA and which parts of GRBDA need to change for compatibility with Robot-Software. Should these functions be abstraced to TreeModel since ClusterTreeModel also uses them?
        Vec3<double> getPosition(const string &body_name);
        Mat3<double> getOrientation(const string &body_name);
        Vec3<double> getLinearVelocity(const string &body_name);
        Vec3<double> getAngularVelocity(const string &body_name);

        // TODO(@MatthewChignoli): We currently assume that the state is given as independent coordinates.
        void setState(const DVec<double> &state)
        {
            const int &nq = ceil(state.size() / 2.0);
            const int &nv = floor(state.size() / 2.0);
            initializeIndependentStates(state.head(nq), state.tail(nv));
        }

        /////////////////////////////////////

        int getNumBodies() const { return (int)nodes_.size(); }

        const Body &getBody(int spanning_tree_index) const;
        ReflectedInertiaTreeNode& getNodeContainingBody(int spanning_tree_index);
        int getIndexOfParentNodeForBody(const int spanning_tree_index);

        const DVec<int> &getIndependentCoordinateIndices() const { return independent_coord_indices_; }

        void initializeIndependentStates(const DVec<double> &y, const DVec<double> &yd);

        void contactJacobians();

        DVec<double> forwardDynamics(const DVec<double> &tau);
        DVec<double> inverseDynamics(const DVec<double> &ydd);

        double applyLocalFrameTestForceAtContactPoint(const Vec3<double> &force,
                                                      const string &contact_point_name,
                                                      DVec<double> &dstate_out);

        DMat<double> getMassMatrix();
        DVec<double> getBiasForceVector();

    private:
        void extractRigidBodiesAndJointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractIndependentCoordinatesFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractContactPointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);

        void resetCache();

        DVec<double> forwardDynamicsWithoutOffDiag(const DVec<double> &tau);
        DVec<double> forwardDynamicsWithOffDiag(const DVec<double> &tau);
        void updateArticulatedBodies();

        DMat<double> reflected_inertia_;

        DMat<double> spanning_tree_to_independent_coords_conversion_;
        DVec<int> independent_coord_indices_;

        bool articulated_bodies_updated_ = false;

        const bool use_off_diagonal_terms_;

        friend class TreeModel<ReflectedInertiaTreeModel, ReflectedInertiaTreeNode>;
    };

} // namespace grbda
