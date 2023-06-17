#pragma once

#include "Body.h"
#include "DynamicsUtilities.h"
#include "Nodes/TreeNode.h"
#include "Utils/Utilities/spatial.h"
#include "Utils/Utilities/SpatialTransforms.h"
#include "Utils/Utilities/utilities.h"

namespace grbda
{

    using TreeNodePtr = std::shared_ptr<TreeNode<>>;

    class TreeModel
    {
    public:
        TreeModel()
        {
            gravity_ << 0., 0., 0., 0., 0., -9.81;
        }
        virtual ~TreeModel() {}

        int getNumPositions() const { return position_index_; }
        int getNumDegreesOfFreedom() const { return velocity_index_; }
        int getNumActuatedDegreesOfFreedom() const { return velocity_index_ - unactuated_dofs_; }

        virtual DMat<double> getMassMatrix() = 0;
        virtual DVec<double> getBiasForceVector() = 0;

        virtual int getNumBodies() const = 0;

        virtual const Body &getBody(int index) const = 0;
        virtual const TreeNodePtr getNodeContainingBody(int index) = 0;

        void setGravity(const Vec3<double> &g) { gravity_.tail<3>() = g; }
        SVec<double> getGravity() const { return gravity_; }

        virtual void initializeState(const State<double> &joint_pos, const State<double> &joint_vel)
        {
            throw std::runtime_error("Not implemented");
        }

        virtual void initializeIndependentStates(const DVec<double> &y, const DVec<double> &yd) = 0;
        virtual void initializeExternalForces(
            const std::vector<ExternalForceAndBodyIndexPair> &force_and_body_index_pairs = {});

        void forwardKinematics();
        virtual DVec<double> forwardDynamics(const DVec<double> &tau) = 0;

        const TreeNodePtr node(const int index) const { return nodes_[index]; }
        const std::vector<TreeNodePtr> &nodes() const { return nodes_; }

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

        virtual void resetCache();

        bool vectorContainsIndex(const std::vector<int> vec, const int index);

        SVec<double> gravity_;

        DMat<double> H_;
        DVec<double> C_;

        int position_index_ = 0;
        int velocity_index_ = 0;
        int unactuated_dofs_ = 0;

        std::vector<TreeNodePtr> nodes_;
        std::vector<int> indices_of_nodes_experiencing_external_forces_;

        std::vector<ContactPoint> contact_points_;
        UnorderedMap<std::string, int> contact_name_to_contact_index_;

        bool kinematics_updated_ = false;
        bool mass_matrix_updated_ = false;
        bool bias_force_updated_ = false;
    };

} // namespace grbda
