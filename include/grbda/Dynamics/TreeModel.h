#ifndef GRBDA_TREEMODEL_H

#include "grbda/Dynamics/Body.h"
#include "grbda/Dynamics/Nodes/TreeNode.h"
#include "grbda/Utils/Spatial.h"
#include "grbda/Utils/SpatialTransforms.h"
#include "grbda/Utils/Utilities.h"

namespace grbda
{

    template <typename Scalar>
    using TreeNodePtr = std::shared_ptr<TreeNode<Scalar>>;

    template <typename Scalar = double>
    class TreeModel
    {
    public:
        TreeModel()
        {
            gravity_ << 0., 0., 0., 0., 0., -9.81;
        }
        virtual ~TreeModel() {}

        const int& getNumPositions() const { return position_index_; }
        const int& getNumDegreesOfFreedom() const { return velocity_index_; }
        int getNumActuatedDegreesOfFreedom() const { return velocity_index_ - unactuated_dofs_; }
        const int& getNumEndEffectors() const { return num_end_effectors_; }

        virtual Vec3<Scalar> getPosition(const std::string &body_name) = 0;
        virtual Mat3<Scalar> getOrientation(const std::string &body_name) = 0;
        virtual Vec3<Scalar> getLinearVelocity(const std::string &body_name) = 0;
        virtual Vec3<Scalar> getAngularVelocity(const std::string &body_name) = 0;

        virtual DMat<Scalar> getMassMatrix() = 0;
        virtual DVec<Scalar> getBiasForceVector() = 0;

        virtual int getNumBodies() const = 0;

        virtual const Body<Scalar> &getBody(int index) const = 0;
        virtual const TreeNodePtr<Scalar> getNodeContainingBody(int index) = 0;

        void setGravity(const Vec3<Scalar> &g) { gravity_.template tail<3>() = g; }
        SVec<Scalar> getGravity() const { return gravity_; }

        void setExternalForces(
            const std::vector<ExternalForceAndBodyIndexPair<Scalar>> &force_and_body_pairs = {});

        void forwardKinematics();
        void forwardKinematicsIncludingContactPoints()
        {
            forwardKinematics();
            contactPointForwardKinematics();
        }

        void updateContactPointJacobians();
        virtual D6Mat<Scalar> contactJacobianBodyFrame(const std::string &cp_name) = 0;
        virtual const D6Mat<Scalar>& contactJacobianWorldFrame(const std::string &cp_name) = 0;

        // Returns independent (non-spanning) joint accelerations
        virtual DVec<Scalar> forwardDynamics(const DVec<Scalar> &tau) = 0;
        
        // Takes as input independent (non-spanning) joint accelerations
        virtual DVec<Scalar> inverseDynamics(const DVec<Scalar> &qdd) = 0;
        
        virtual DMat<Scalar> inverseOperationalSpaceInertiaMatrix() = 0;

        // The test force is expressed in the local frame
        virtual Scalar applyTestForce(const std::string &contact_point_name,
                                      const Vec3<Scalar> &force, DVec<Scalar> &dstate_out) = 0;

        const TreeNodePtr<Scalar> node(const int index) const { return nodes_[index]; }
        const std::vector<TreeNodePtr<Scalar>> &nodes() const { return nodes_; }

        const std::vector<ContactPoint<Scalar>> &contactPoints() const { return contact_points_; }
        const ContactPoint<Scalar>  &contactPoint(const int index) const
        {
            return contact_points_[index];
        }
        const ContactPoint<Scalar> &contactPoint(const std::string &name) const
        {
            return contact_points_[contact_name_to_contact_index_.at(name)];
        }

    protected:
        void contactPointForwardKinematics();
        void compositeRigidBodyAlgorithm();
        void updateBiasForceVector();

        // Takes as input independent (non-spanning) joint accelerations
        DVec<Scalar> recursiveNewtonEulerAlgorithm(const DVec<Scalar> &qdd);

        virtual void resetCache();

        int getNearestSharedSupportingNode(const std::pair<int, int> &contact_pt_indices);
        bool vectorContainsIndex(const std::vector<int> vec, const int index);

        SVec<Scalar> gravity_;

        DMat<Scalar> H_;
        DVec<Scalar> C_;

        int position_index_ = 0;
        int velocity_index_ = 0;
        int motion_subspace_index_ = 0;
        int unactuated_dofs_ = 0;
        int num_end_effectors_ = 0;

        std::vector<TreeNodePtr<Scalar>> nodes_;
        std::vector<int> indices_of_nodes_experiencing_external_forces_;

        std::vector<ContactPoint<Scalar> > contact_points_;
        std::unordered_map<std::string, int> contact_name_to_contact_index_;

        bool kinematics_updated_ = false;
        bool contact_point_kinematics_updated_ = false;
        bool mass_matrix_updated_ = false;
        bool bias_force_updated_ = false;
        bool contact_jacobians_updated_ = false;
    };

} // namespace grbda

#endif // GRBDA_TREEMODEL_H
