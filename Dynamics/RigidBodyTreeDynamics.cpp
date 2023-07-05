#include "RigidBodyTreeModel.h"

namespace grbda
{

    DVec<double> RigidBodyTreeModel::forwardDynamics(const DVec<double> &tau)
    {
        forwardKinematics();
        compositeRigidBodyAlgorithm();
        updateBiasForceVector();

        switch (forward_dynamics_method_)
        {
        case FwdDynMethod::Projection:
        {
            // Based on Method 3 in Featherstone Ch 8.5
            DMat<double> A = G_.transpose() * H_ * G_;
            DVec<double> b = tau - G_.transpose() * (C_ + H_ * g_);
            DVec<double> ydd = A.llt().solve(b);
            return G_ * ydd + g_;
        }

        case FwdDynMethod::LagrangeMultiplierCustom:
        {
            // TODO(@MatthewChignoli): In a future issue, we will implement this in a more efficient way since in theory it should be faster than the projection method

            // Based on Method 2 in Featherstone Ch 8.5 (using custom sparse factorization)

            // Factorize H into L^T*L
#ifdef TIMING_STATS
            timer_.start();
#endif
            factorization::LTL L(H_, rigid_body_nodes_);
#ifdef TIMING_STATS
            timing_statistics_.ltl_factorization_time = timer_.getMs();
#endif

            // Calculate tau_prime
#ifdef TIMING_STATS
            timer_.start();
#endif
            DVec<double> tau_full = G_tranpose_pinv_ * tau;
            DVec<double> tau_prime = tau_full - C_;
#ifdef TIMING_STATS
            timing_statistics_.tau_prime_calc_time = timer_.getMs();
#endif

            // Calculate Y and z via back-subsition
#ifdef TIMING_STATS
            timer_.start();
#endif
            DMat<double> Y = L.inverseTransposeMatrixProduct(K_.transpose());
            DVec<double> z = L.inverseTransposeProduct(tau_prime);
#ifdef TIMING_STATS
            timing_statistics_.Y_and_z_calc_time = timer_.getMs();
#endif

            // Calculate A and b
#ifdef TIMING_STATS
            timer_.start();
#endif
            DMat<double> A = Y.transpose() * Y;
            DVec<double> b = k_ - Y.transpose() * z;
#ifdef TIMING_STATS
            timing_statistics_.A_and_b_time = timer_.getMs();
#endif

            // Solve Linear System A*lambda = b
#ifdef TIMING_STATS
            timer_.start();
#endif
            DVec<double> lambda;
            if (A.size() > 0)
                lambda = A.colPivHouseholderQr().solve(b);
            else
                lambda = DVec<double>::Zero(0);
#ifdef TIMING_STATS
            timing_statistics_.lambda_solve_time = timer_.getMs();
#endif

            // Solve for qdd using the factors from step 1
#ifdef TIMING_STATS
            timer_.start();
            DVec<double> qdd = L.solve(tau_prime + K_.transpose() * lambda);
            timing_statistics_.qdd_solve_time = timer_.getMs();
            return qdd;
#else
            return L.solve(tau_prime + K_.transpose() * lambda);
#endif
        }

        case FwdDynMethod::LagrangeMultiplierEigen:
        {
            // Based on Method 2 in Featherstone Ch 8.5 (using Eigen factorization)

            auto lltOfH = H_.llt();
            DMat<double> KT = K_.transpose();
            DMat<double> A = K_ * (lltOfH.solve(KT));
            DVec<double> tau_full = G_tranpose_pinv_ * tau;
            DVec<double> tau_prime = tau_full - C_;
            DVec<double> b = k_ - K_ * lltOfH.solve(tau_prime);
            DVec<double> lambda = A.size() > 0 ? DVec<double>(A.colPivHouseholderQr().solve(b))
                                               : DVec<double>::Zero(0);
            return lltOfH.solve(tau_prime + KT * lambda);
        }

        default:
            throw std::runtime_error("Invalid forward dynamics method specified.");
        }
    }

} // namespace grbda
