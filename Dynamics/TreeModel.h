#pragma once

#include <variant>
#include "Body.h"
#include "DynamicsUtilities.h"
// #include "Nodes/TreeNode.h"
#include "Utils/Utilities/spatial.h"
#include "Utils/Utilities/SpatialTransforms.h"
#include "Utils/Utilities/utilities.h"

#include "Nodes/ClusterTreeNode.h"
#include "Nodes/ReflectedInertiaTreeNode.h"
#include "Nodes/RigidBodyTreeNode.h"

namespace grbda
{

    class ClusterTreeModel;
    class RigidBodyTreeModel;
    class ReflectedInertiaTreeModel;

    template <typename Derived>
    struct BaseTraits;

    template <>
    struct BaseTraits<ClusterTreeModel>
    {
        typedef ClusterTreeNode<> NodeType;
    };

    template <>
    struct BaseTraits<RigidBodyTreeModel>
    {
        typedef RigidBodyTreeNode NodeType;
    };

    template <>
    struct BaseTraits<ReflectedInertiaTreeModel>
    {
        typedef ReflectedInertiaTreeNode NodeType;
    };

    // TODO(@MatthewChignoli): I should not need to pass both templates. I should somehow be able to get the NodeType from Derived (Pinocchio knows how to do this)
    template <typename Derived>
    class TreeModel
    {
    public:
        typedef typename BaseTraits<Derived>::NodeType NodeType;

        const Derived &derived() const { return *static_cast<const Derived *>(this); }
        Derived &derived() { return *static_cast<Derived *>(this); }

        const int &getNumPositions() const { return position_index_; }
        const int &getNumDegreesOfFreedom() const { return velocity_index_; }
        int getNumActuatedDegreesOfFreedom() const { return velocity_index_ - unactuated_dofs_; }

        DMat<double> getMassMatrix() { return derived().getMassMatrix(); }
        DVec<double> getBiasForceVector() { return derived().getBiasForceVector(); }

        int getNumBodies() const { return derived().getNumBodies(); }

        const Body &getBody(int index) const { return derived().getBody(index); }

        NodeType &getNodeContainingBody(int index)
        {
            return derived().getNodeContainingBody(index);
        }

        void setGravity(const Vec3<double> &g) { gravity_.tail<3>() = g; }
        SVec<double> getGravity() const { return gravity_; }

        void initializeExternalForces(
            const std::vector<ExternalForceAndBodyIndexPair> &force_and_body_index_pairs = {});

        void forwardKinematics();
        void contactJacobians() { derived().contactJacobians(); }

        DVec<double> forwardDynamics(const DVec<double> &tau) { return derived().forwardDynamics(tau); }
        DVec<double> inverseDynamics(const DVec<double> &qdd) { return derived().inverseDynamics(qdd); }

        double applyLocalFrameTestForceAtContactPoint(const Vec3<double> &force,
                                                      const string &contact_point_name,
                                                      DVec<double> &dstate_out)
        {
            return derived().applyLocalFrameTestForceAtContactPoint(force, contact_point_name, dstate_out);
        }

        NodeType &node(const int index) { return nodes_[index]; }
        const NodeType &node(const int index) const { return nodes_[index]; }
        const std::vector<NodeType> &nodes() const { return nodes_; }

        const std::vector<ContactPoint> &contactPoints() const { return contact_points_; }
        const ContactPoint &contactPoint(const int index) const { return contact_points_[index]; }
        const ContactPoint &contactPoint(const std::string &name) const
        {
            return contact_points_[contact_name_to_contact_index_.at(name)];
        }

    protected:
        void contactPointForwardKinematics();
        void compositeRigidBodyAlgorithm();
        void updateBiasForceVector();
        DVec<double> recursiveNewtonEulerAlgorithm(const DVec<double> &qdd);

        void resetCache() { derived().resetCache(); }

        bool vectorContainsIndex(const std::vector<int> vec, const int index);

        SVec<double> gravity_;

        DMat<double> H_;
        DVec<double> C_;

        int position_index_ = 0;
        int velocity_index_ = 0;
        int unactuated_dofs_ = 0;

        std::vector<NodeType> nodes_;
        std::vector<int> indices_of_nodes_experiencing_external_forces_;

        std::vector<ContactPoint> contact_points_;
        std::unordered_map<std::string, int> contact_name_to_contact_index_;

        bool kinematics_updated_ = false;
        bool mass_matrix_updated_ = false;
        bool bias_force_updated_ = false;

    private:
        TreeModel()
        {
            gravity_ << 0., 0., 0., 0., 0., -9.81;
        }
        friend Derived;
    };


} // namespace grbda
