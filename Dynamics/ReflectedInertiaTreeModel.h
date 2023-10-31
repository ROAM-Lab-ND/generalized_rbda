#ifndef GRBDA_REFLECTED_INERTIA_TREE_MODEL_H
#define GRBDA_REFLECTED_INERTIA_TREE_MODEL_H

#include "ClusterTreeModel.h"
#include "Dynamics/Nodes/ReflectedInertiaTreeNode.h"

namespace grbda
{
    template <typename Scalar>
    using ReflectedInertiaTreeNodePtr = std::shared_ptr<ReflectedInertiaTreeNode<Scalar>>;

    enum class RotorInertiaApproximation
    {
        NONE,
        DIAGONAL,
        BLOCK_DIAGONAL,
    };

    /*!
     * Class to represent a floating base rigid body model with rotors and ground contacts.
     */
    template <typename Scalar = double>
    class ReflectedInertiaTreeModel : public TreeModel<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReflectedInertiaTreeModel(const ClusterTreeModel<Scalar> &cluster_tree_model,
                                  RotorInertiaApproximation rotor_inertia_approximation =
                                      RotorInertiaApproximation::NONE);

        int getNumBodies() const override { return (int)reflected_inertia_nodes_.size(); }

        const Body<Scalar> &getBody(int spanning_tree_index) const override;
        const TreeNodePtr<Scalar> getNodeContainingBody(int spanning_tree_index) override;
        int getIndexOfParentNodeForBody(const int spanning_tree_index);

        const DVec<int> &getIndependentCoordinateIndices() const
        {
            return independent_coord_indices_;
        }

        void setIndependentStates(const DVec<Scalar> &y, const DVec<Scalar> &yd);

        Vec3<Scalar> getPosition(const std::string &body_name) override
        {
            throw std::runtime_error("Not implemented");
        }
        Mat3<Scalar> getOrientation(const std::string &body_name) override
        {
            throw std::runtime_error("Not implemented");
        }
        Vec3<Scalar> getLinearVelocity(const std::string &body_name) override
        {
            throw std::runtime_error("Not implemented");
        }
        Vec3<Scalar> getAngularVelocity(const std::string &body_name) override
        {
            throw std::runtime_error("Not implemented");
        }

        D6Mat<Scalar> contactJacobianBodyFrame(const std::string &cp_name) override;
        const D6Mat<Scalar>& contactJacobianWorldFrame(const std::string &cp_name) override;

        DVec<Scalar> forwardDynamics(const DVec<Scalar> &tau) override;
        DVec<Scalar> inverseDynamics(const DVec<Scalar> &ydd) override;
        DMat<Scalar> inverseOperationalSpaceInertiaMatrix() override;

        Scalar applyTestForce(const std::string &contact_point_name,
                              const Vec3<Scalar> &force, DVec<Scalar> &dstate_out) override;

        DMat<Scalar> getMassMatrix() override;
        DVec<Scalar> getBiasForceVector() override;

    protected:
        void extractRigidBodiesAndJointsFromClusterModel(
            const ClusterTreeModel<Scalar> &cluster_tree_model);
        void extractIndependentCoordinatesFromClusterModel(
            const ClusterTreeModel<Scalar> &cluster_tree_model);
        void extractContactPointsFromClusterModel(
            const ClusterTreeModel<Scalar> &cluster_tree_model);

        void resetCache() override;

        DVec<Scalar> forwardDynamicsABA(const DVec<Scalar> &tau, bool use_reflected_inertia);
        DVec<Scalar> forwardDynamicsHinv(const DVec<Scalar> &tau);
        void updateArticulatedBodies(bool use_reflected_inertia);

        DMat<Scalar> inverseOpSpaceInertiaEFPA(bool use_reflected_inertia);
        DMat<Scalar> inverseOpSpaceInertiaHinv();

        Scalar applyTestForceEFPA(const Vec3<Scalar> &force, const std::string &contact_point_name,
                                  DVec<Scalar> &dstate_out, bool use_reflected_inertia);
        Scalar applyTestForceHinv(const Vec3<Scalar> &force, const std::string &contact_point_name,
                                  DVec<Scalar> &dstate_out);

        void updateForcePropagators(bool use_reflected_inertia);
        void updateQddEffects(bool use_reflected_inertia);
        SVec<Scalar> localCartesianForceAtPointToWorldPluckerForceOnCluster(
            const Vec3<Scalar> &force, const ContactPoint<Scalar> &contact_point);

        std::vector<ReflectedInertiaTreeNodePtr<Scalar>> reflected_inertia_nodes_;

        DMat<Scalar> reflected_inertia_;

        DMat<int> spanning_tree_to_independent_coords_conversion_;
        DVec<int> independent_coord_indices_;

        bool articulated_bodies_updated_ = false;
        bool force_propagators_updated_ = false;
        bool qdd_effects_updated_ = false;

        const RotorInertiaApproximation rotor_inertia_approximation_;
    };

} // namespace grbda

#endif // GRBDA_REFLECTED_INERTIA_TREE_MODEL_H
