#ifndef GRBDA_REFLECTED_INERTIA_TREE_MODEL_H
#define GRBDA_REFLECTED_INERTIA_TREE_MODEL_H

#include "ClusterTreeModel.h"
#include "Dynamics/Nodes/ReflectedInertiaTreeNode.h"

namespace grbda
{

    using ReflectedInertiaTreeNodePtr = std::shared_ptr<ReflectedInertiaTreeNode<>>;

    enum class RotorInertiaApproximation
    {
        NONE,
        DIAGONAL,
        BLOCK_DIAGONAL,
    };

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class ReflectedInertiaTreeModel : public TreeModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReflectedInertiaTreeModel(const ClusterTreeModel &cluster_tree_model,
                                  RotorInertiaApproximation rotor_inertia_approximation =
                                      RotorInertiaApproximation::NONE);

        int getNumBodies() const override { return (int)reflected_inertia_nodes_.size(); }

        const Body<> &getBody(int spanning_tree_index) const override;
        const TreeNodePtr getNodeContainingBody(int spanning_tree_index) override;
        int getIndexOfParentNodeForBody(const int spanning_tree_index);

        const DVec<int> &getIndependentCoordinateIndices() const { return independent_coord_indices_; }

        void setIndependentStates(const DVec<double> &y, const DVec<double> &yd);

        Vec3<double> getPosition(const std::string &body_name) override
        {
            throw std::runtime_error("Not implemented");
        }
        Mat3<double> getOrientation(const std::string &body_name) override
        {
            throw std::runtime_error("Not implemented");
        }
        Vec3<double> getLinearVelocity(const std::string &body_name) override
        {
            throw std::runtime_error("Not implemented");
        }
        Vec3<double> getAngularVelocity(const std::string &body_name) override
        {
            throw std::runtime_error("Not implemented");
        }

        D6Mat<double> contactJacobianBodyFrame(const std::string &cp_name) override;
        const D6Mat<double>& contactJacobianWorldFrame(const std::string &cp_name) override;

        DVec<double> forwardDynamics(const DVec<double> &tau) override;
        DVec<double> inverseDynamics(const DVec<double> &ydd) override;
        DMat<double> inverseOperationalSpaceInertiaMatrix() override;

        double applyTestForce(const std::string &contact_point_name,
                              const Vec3<double> &force, DVec<double> &dstate_out) override;

        DMat<double> getMassMatrix() override;
        DVec<double> getBiasForceVector() override;

    private:
        void extractRigidBodiesAndJointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractIndependentCoordinatesFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractContactPointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);

        void resetCache() override;

        DVec<double> forwardDynamicsABA(const DVec<double> &tau, bool use_reflected_inertia);
        DVec<double> forwardDynamicsHinv(const DVec<double> &tau);
        void updateArticulatedBodies(bool use_reflected_inertia);

        DMat<double> inverseOpSpaceInertiaEFPA(bool use_reflected_inertia);
        DMat<double> inverseOpSpaceInertiaHinv();

        double applyTestForceEFPA(const Vec3<double> &force, const std::string &contact_point_name,
                                  DVec<double> &dstate_out, bool use_reflected_inertia);
        double applyTestForceHinv(const Vec3<double> &force, const std::string &contact_point_name,
                                  DVec<double> &dstate_out);

        void updateForcePropagators(bool use_reflected_inertia);
        void updateQddEffects(bool use_reflected_inertia);
        SVec<double> localCartesianForceAtPointToWorldPluckerForceOnCluster(
            const Vec3<double> &force, const ContactPoint &contact_point);

        std::vector<ReflectedInertiaTreeNodePtr> reflected_inertia_nodes_;

        DMat<double> reflected_inertia_;

        DMat<double> spanning_tree_to_independent_coords_conversion_;
        DVec<int> independent_coord_indices_;

        bool articulated_bodies_updated_ = false;
        bool force_propagators_updated_ = false;
        bool qdd_effects_updated_ = false;

        const RotorInertiaApproximation rotor_inertia_approximation_;
    };

} // namespace grbda

#endif // GRBDA_REFLECTED_INERTIA_TREE_MODEL_H
