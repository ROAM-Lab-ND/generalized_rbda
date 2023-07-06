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
        case ForwardDynamicsMethod::Projection:
        {
            // Based on Method 3 in Featherstone Ch 8.5
            DMat<double> A = G_.transpose() * H_ * G_;
            DVec<double> b = tau - G_.transpose() * (C_ + H_ * g_);
            DVec<double> ydd = A.llt().solve(b);
            return G_ * ydd + g_;
        }

        case ForwardDynamicsMethod::LagrangeMultiplier:
        {
            // TODO(@MatthewChignoli): In a future issue, we will implement this in a more efficient way since in theory it should be faster than the projection method

            // Based on Method 2 in Featherstone Ch 8.5

            // Factorize H into L^T*L
            factorization::LTL L(H_, rigid_body_nodes_);

            // ISSUE #14
            if (L.size() == 0)
            {
                DMat<double> H_inv = H_.inverse();
                DMat<double> A = K_ * H_inv * K_.transpose();
                DVec<double> tau_full = G_tranpose_pinv_ * tau;
                DVec<double> tau_prime = tau_full - C_;
                DVec<double> b = k_ - K_ * H_inv * tau_prime;
                DVec<double> lambda = A.size() > 0 ? DVec<double>(A.colPivHouseholderQr().solve(b))
                                                   : DVec<double>::Zero(0);
                return H_inv * (tau_prime + K_.transpose() * lambda);
            }

            // Calculate tau_prime
            DVec<double> tau_full = G_tranpose_pinv_ * tau;
            DVec<double> tau_prime = tau_full - C_;

            // Calculate Y and z via back-subsition
            DMat<double> Y = L.inverseTransposeMatrixProduct(K_.transpose());
            DVec<double> z = L.inverseTransposeProduct(tau_prime);

            // Calculate A and b
            DMat<double> A = Y.transpose() * Y;
            DVec<double> b = k_ - Y.transpose() * z;

            // Solve Linear System A*lambda = b
            DVec<double> lambda = A.size() > 0 ? DVec<double>(A.colPivHouseholderQr().solve(b))
                                               : DVec<double>::Zero(0);

            // Solve for qdd using the factors from step 1
            return L.solve(tau_prime + K_.transpose() * lambda);
        }

        default:
            throw std::runtime_error("Invalid forward dynamics method specified.");
        }
    }

} // namespace grbda
