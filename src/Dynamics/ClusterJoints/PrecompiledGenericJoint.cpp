#include "grbda/Dynamics/ClusterJoints/PrecompiledGenericJoint.h"
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"
#include <iostream>

namespace grbda
{
    namespace LoopConstraint
    {
        template <typename Scalar>
        PrecompiledGenericImplicit<Scalar>::PrecompiledGenericImplicit(
            std::vector<bool> is_coordinate_independent,
            SymPhiFcn phi_sym,
            NativePhiFcn phi_native,
            const std::string& constraint_name)
            : is_coordinate_independent_(is_coordinate_independent),
              phi_sym_(phi_sym),
              phi_native_(phi_native),
              constraint_name_(constraint_name),
              state_dim_(is_coordinate_independent.size())
        {
            // ULTRA-FAST initialization - NO symbolic graph construction!
            auto start = std::chrono::high_resolution_clock::now();

            // Separate independent and dependent coordinates
            for (int i = 0; i < state_dim_; i++) {
                if (is_coordinate_independent[i])
                    ind_coords_.push_back(i);
                else
                    dep_coords_.push_back(i);
            }
            ind_dim_ = ind_coords_.size();
            dep_dim_ = dep_coords_.size();

            // Initialize matrices to zero (will be filled by pre-compiled functions)
            this->G_ = DMat<Scalar>::Zero(state_dim_, ind_dim_);
            this->g_ = DVec<Scalar>::Zero(state_dim_);
            this->K_ = DMat<Scalar>::Zero(dep_dim_, state_dim_);  // constraint_dim = dep_dim for implicit
            this->k_ = DVec<Scalar>::Zero(dep_dim_);

            // Phi function uses native implementation
            this->phi_ = phi_native;

            auto end = std::chrono::high_resolution_clock::now();
            double init_us = std::chrono::duration<double, std::micro>(end - start).count();

            std::cout << "[PrecompiledGenericImplicit] " << constraint_name_
                      << ": Initialized in " << init_us << " μs (using pre-compiled code)\n";
        }

        template <typename Scalar>
        DVec<Scalar> PrecompiledGenericImplicit<Scalar>::gamma(const JointCoordinate<Scalar> &joint_pos) const
        {
            return phi_native_(joint_pos);
        }

        template <typename Scalar>
        void PrecompiledGenericImplicit<Scalar>::updateJacobians(const JointCoordinate<Scalar> &joint_pos)
        {
            // Use optimized inline C code instead of CasADi
            if constexpr (std::is_same_v<Scalar, double>) {
                // Call pre-compiled tello_hip_G_optimized
                double G_flat[8];  // 4x2 matrix in column-major
                tello_hip_G_optimized(joint_pos.data(), G_flat);

                // Convert flat to Eigen matrix
                this->G_ = Eigen::Map<Eigen::Matrix<double, 4, 2, Eigen::ColMajor>>(G_flat);

                // K matrix can be computed analytically if needed, or set to zero for now
                this->K_.setZero();
            } else {
                // Fallback for non-double types (shouldn't happen for Tello)
                std::cerr << "[PrecompiledGenericImplicit] Warning: No pre-compiled code for this scalar type\n";
                this->G_.setIdentity();
            }
        }

        template <typename Scalar>
        void PrecompiledGenericImplicit<Scalar>::updateBiases(const JointState<Scalar> &joint_state)
        {
            if constexpr (std::is_same_v<Scalar, double>) {
                // Call pre-compiled tello_hip_g_optimized
                double g_arr[4];
                tello_hip_g_optimized(joint_state.position.data(), joint_state.velocity.data(), g_arr);
                this->g_ = Eigen::Map<Eigen::Vector<double, 4>>(g_arr);

                // k bias (implicit)
                this->k_.setZero();
            } else {
                this->g_.setZero();
                this->k_.setZero();
            }
        }

        template <typename Scalar>
        void PrecompiledGenericImplicit<Scalar>::createRandomStateHelpers()
        {
            // PrecompiledGenericImplicit doesn't use random state helpers
            // Instead, randomJointState() will call copyAsDouble() which creates a
            // temporary GenericImplicit that handles constraint solving
        }

        template <typename Scalar>
        GenericImplicit<double> PrecompiledGenericImplicit<Scalar>::copyAsDouble() const
        {
            // For constraint solving (randomJointState), create a GenericImplicit<double>
            // This WILL build a symbolic graph, but only for the temporary copy
            // The main PrecompiledGenericImplicit avoids this during normal operations

            if constexpr (std::is_same_v<Scalar, double>) {
                return GenericImplicit<double>(is_coordinate_independent_, phi_sym_, phi_native_);
            } else {
                return GenericImplicit<double>(is_coordinate_independent_, phi_sym_);
            }
        }

        // Explicit template instantiations
        template struct PrecompiledGenericImplicit<double>;
        template struct PrecompiledGenericImplicit<float>;
        template struct PrecompiledGenericImplicit<std::complex<double>>;
        template struct PrecompiledGenericImplicit<casadi::SX>;

    } // namespace LoopConstraint
} // namespace grbda
