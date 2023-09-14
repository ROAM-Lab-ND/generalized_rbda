#ifndef GRBDA_DYNAMICS_RIGID_BODY_TREE_MODEL_H
#define GRBDA_DYNAMICS_RIGID_BODY_TREE_MODEL_H

#include "ClusterTreeModel.h"
#include "Factorization.h"
#ifdef TIMING_STATS
#include "Utils/Utilities/Timer.h"
#endif

namespace grbda
{

    using namespace ori;
    using namespace spatial;

    enum class FwdDynMethod
    {
        Projection,
        LagrangeMultiplierCustom,
        LagrangeMultiplierEigen
    };

#ifdef TIMING_STATS
    struct RigidBodyTreeTimingStatistics
    {
        double ltl_factorization_time = 0.0;
        double tau_prime_calc_time = 0.0;
        double Y_and_z_calc_time = 0.0;
        double A_and_b_time = 0.0;
        double lambda_solve_time = 0.0;
        double qdd_solve_time = 0.0;

        RigidBodyTreeTimingStatistics &operator+=(const RigidBodyTreeTimingStatistics &other)
        {
            ltl_factorization_time += other.ltl_factorization_time;
            tau_prime_calc_time += other.tau_prime_calc_time;
            Y_and_z_calc_time += other.Y_and_z_calc_time;
            A_and_b_time += other.A_and_b_time;
            lambda_solve_time += other.lambda_solve_time;
            qdd_solve_time += other.qdd_solve_time;
            return *this;
        }

        RigidBodyTreeTimingStatistics &operator/=(const double &scalar)
        {
            ltl_factorization_time /= scalar;
            tau_prime_calc_time /= scalar;
            Y_and_z_calc_time /= scalar;
            A_and_b_time /= scalar;
            lambda_solve_time /= scalar;
            qdd_solve_time /= scalar;
            return *this;
        }
    };
#endif

    using RigidBodyTreeNodePtr = std::shared_ptr<RigidBodyTreeNode>;

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class RigidBodyTreeModel : public TreeModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RigidBodyTreeModel(const ClusterTreeModel &cluster_tree_model,
                           const FwdDynMethod fd_method = FwdDynMethod::Projection);
        ~RigidBodyTreeModel() {}

        void setForwardDynamicsMethod(FwdDynMethod fd_method)
        {
            forward_dynamics_method_ = fd_method;
        }

        int getNumBodies() const override { return (int)rigid_body_nodes_.size(); }

        // TOOD(@MatthewChignoli): I don't really like these functions...
        const Body &getBody(int index) const override { return rigid_body_nodes_[index]->body_; }
        const TreeNodePtr getNodeContainingBody(int index) override { return rigid_body_nodes_[index]; }

        void initializeState(const DVec<double> &q, const DVec<double> &qd);

        void updateLoopConstraints();

        Vec3<double> getPosition(const string &body_name) override;
        Mat3<double> getOrientation(const string &body_name) override;
        Vec3<double> getLinearVelocity(const string &body_name) override;
        Vec3<double> getAngularVelocity(const string &body_name) override;

        D6Mat<double> bodyJacobian(const std::string &cp_name) override;
        const D6Mat<double> &contactJacobian(const std::string &cp_name) override;

        DVec<double> forwardDynamics(const DVec<double> &tau) override;
        DVec<double> inverseDynamics(const DVec<double> &ydd) override;
        DMat<double> inverseOperationalSpaceInertiaMatrix() override;

        double applyTestForce(const string &contact_point_name,
                              const Vec3<double> &force, DVec<double> &dstate_out) override;

        DMat<double> getMassMatrix() override;
        DVec<double> getBiasForceVector() override;

        DVec<double> qddToYdd(DVec<double> qdd) const
        {
            return loop_constraints_.G_pinv() * (qdd - loop_constraints_.g());
        }

        DVec<double> yddToQdd(DVec<double> ydd) const
        {
            return loop_constraints_.G() * ydd + loop_constraints_.g();
        }

#ifdef TIMING_STATS
        const RigidBodyTreeTimingStatistics &getTimingStatistics() const
        {
            return timing_statistics_;
        }
#endif

    private:
        void extractRigidBodiesAndJointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractLoopClosureFunctionsFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractContactPointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractExpandedTreeConnectivity();

        void resetCache() override
        {
            TreeModel::resetCache();
            loop_constraints_updated_ = false;
        }

        FwdDynMethod forward_dynamics_method_;

        LoopConstraint::Collection loop_constraints_;
        bool loop_constraints_updated_ = false;

        std::vector<RigidBodyTreeNodePtr> rigid_body_nodes_;
        std::unordered_map<string, int> body_name_to_body_index_;

        // NOTE: The expanded tree parent indices represent the parent indices for the connectivty 
        // graph resulting from treating multi-dof joints as multiple single-dof joints.
        std::vector<int> expanded_tree_parent_indices_;

        DVec<double> q_;
        DVec<double> qd_;

#ifdef TIMING_STATS
        Timer timer_;
        RigidBodyTreeTimingStatistics timing_statistics_;
#endif
    };

} // namespace grbda

#endif // GRBDA_DYNAMICS_RIGID_BODY_TREE_MODEL_H
