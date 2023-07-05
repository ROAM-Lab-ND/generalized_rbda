#pragma once

#include "ClusterTreeModel.h"
#include "Factorization.h"
#include "Utils/Utilities/Timer.h"

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

        const Body &getBody(int index) const override { return rigid_body_nodes_[index]->body_; }
        const TreeNodePtr getNodeContainingBody(int index) override { return rigid_body_nodes_[index]; }

        void initializeState(const DVec<double> &q, const DVec<double> &qd);

        DVec<double> forwardDynamics(const DVec<double> &tau) override;

        DMat<double> getMassMatrix() override;
        DVec<double> getBiasForceVector() override;

        DVec<double> qddToYdd(DVec<double> qdd) const { return G_pinv_ * (qdd - g_); }
        DVec<double> yddToQdd(DVec<double> ydd) const { return G_ * ydd + g_; }

        void extractLoopClosureFunctionsFromClusterModel(const ClusterTreeModel &cluster_tree_model);

#ifdef TIMING_STATS
        const RigidBodyTreeTimingStatistics &getTimingStatistics() const
        {
            return timing_statistics_;
        }
#endif

    private:
        void extractRigidBodiesAndJointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractContactPointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);

        FwdDynMethod forward_dynamics_method_;

        DVecFcn<double> gamma_;
        DMat<double> G_;
        DMat<double> G_pinv_;
        DMat<double> G_tranpose_pinv_;
        DVec<double> g_;

        DVecFcn<double> phi_;
        DMat<double> K_;
        DVec<double> k_;

        std::vector<RigidBodyTreeNodePtr> rigid_body_nodes_;

#ifdef TIMING_STATS
        Timer timer_;
        RigidBodyTreeTimingStatistics timing_statistics_;
#endif
    };

} // namespace grbda
