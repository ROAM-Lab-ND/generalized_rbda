#pragma once

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

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class RigidBodyTreeModel : public TreeModel<RigidBodyTreeModel, RigidBodyTreeNode>
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

            const DVec<double> q = loop_constraints_.gamma(state.head(nq));
            const DVec<double> qd = loop_constraints_.G() * state.tail(nv);

            initializeState(q, qd);
        }

        /////////////////////////////////////

        int getNumBodies() const { return (int)nodes_.size(); }

        // TOOD(@MatthewChignoli): I don't really like these functions...
        const Body &getBody(int index) const { return nodes_[index].body_; }
        RigidBodyTreeNode &getNodeContainingBody(int index) { return nodes_[index]; }

        void initializeState(const DVec<double> &q, const DVec<double> &qd);

        void updateLoopConstraints();

        void contactJacobians();

        DVec<double> forwardDynamics(const DVec<double> &tau);

        DMat<double> getMassMatrix();
        DVec<double> getBiasForceVector();

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

        void resetCache()
        {
            kinematics_updated_ = false;
            mass_matrix_updated_ = false;
            bias_force_updated_ = false;
            loop_constraints_updated_ = false;
        }

        FwdDynMethod forward_dynamics_method_;

        LoopConstraint::Collection loop_constraints_;
        bool loop_constraints_updated_ = false;

        std::unordered_map<string, int> body_name_to_body_index_;

        DVec<double> q_;
        DVec<double> qd_;

#ifdef TIMING_STATS
        Timer timer_;
        RigidBodyTreeTimingStatistics timing_statistics_;
#endif

        friend class TreeModel<RigidBodyTreeModel, RigidBodyTreeNode>;
    };

} // namespace grbda
