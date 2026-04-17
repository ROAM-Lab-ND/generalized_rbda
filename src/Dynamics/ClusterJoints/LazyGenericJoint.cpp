#include "grbda/Dynamics/ClusterJoints/LazyGenericJoint.h"

namespace grbda
{
    namespace LoopConstraint
    {
        template <typename Scalar>
        LazyGenericImplicit<Scalar>::LazyGenericImplicit(
            std::vector<bool> is_coordinate_independent,
            SymPhiFcn phi_fcn,
            NativePhiFcn phi_native)
            : is_coordinate_independent_(is_coordinate_independent),
              phi_sym_(phi_fcn),
              phi_native_(phi_native),
              initialized_(false)
        {
            // Constructor does NOTHING expensive!
            // Just save parameters for later lazy initialization

            // Initialize base class members to zero
            int state_dim = is_coordinate_independent.size();
            int ind_dim = 0;
            for (bool is_ind : is_coordinate_independent) {
                if (is_ind) ind_dim++;
            }

            this->G_ = DMat<Scalar>::Zero(state_dim, ind_dim);
            this->g_ = DVec<Scalar>::Zero(state_dim);
            this->K_ = DMat<Scalar>::Zero(0, 0);  // Don't know constraint_dim yet
            this->k_ = DVec<Scalar>::Zero(0);

            // phi_ will be set during initialization
            this->phi_ = [this](const JointCoordinate<Scalar> &joint_pos) {
                ensureInitialized();
                return impl_->phi(joint_pos);
            };
        }

        template <typename Scalar>
        void LazyGenericImplicit<Scalar>::ensureInitialized() const
        {
            if (initialized_) return;

            std::lock_guard<std::mutex> lock(init_mutex_);

            // Double-check after acquiring lock
            if (initialized_) return;

            // NOW build the actual constraint (expensive!)
            auto start = std::chrono::high_resolution_clock::now();

            if (phi_native_) {
                impl_ = std::make_unique<GenericImplicit<Scalar>>(
                    is_coordinate_independent_, phi_sym_, phi_native_);
            } else {
                impl_ = std::make_unique<GenericImplicit<Scalar>>(
                    is_coordinate_independent_, phi_sym_);
            }

            auto end = std::chrono::high_resolution_clock::now();
            double init_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

            std::cout << "[LazyGenericImplicit] Initialized constraint in "
                      << init_time_ms << " ms (deferred from cold-start)\n";

            initialized_ = true;
        }

        template <typename Scalar>
        DVec<Scalar> LazyGenericImplicit<Scalar>::gamma(const JointCoordinate<Scalar> &joint_pos) const
        {
            ensureInitialized();
            return impl_->gamma(joint_pos);
        }

        template <typename Scalar>
        void LazyGenericImplicit<Scalar>::updateJacobians(const JointCoordinate<Scalar> &joint_pos)
        {
            ensureInitialized();
            impl_->updateJacobians(joint_pos);

            // Copy results to base class members
            this->K_ = impl_->K();
            this->G_ = impl_->G();
        }

        template <typename Scalar>
        void LazyGenericImplicit<Scalar>::updateBiases(const JointState<Scalar> &joint_state)
        {
            ensureInitialized();
            impl_->updateBiases(joint_state);

            // Copy results to base class members
            this->k_ = impl_->k();
            this->g_ = impl_->g();
        }

        template <typename Scalar>
        void LazyGenericImplicit<Scalar>::createRandomStateHelpers()
        {
            ensureInitialized();
            impl_->createRandomStateHelpers();

            // Copy random state helpers to base class
            this->random_state_helpers_ = impl_->random_state_helpers_;
        }

        // Explicit template instantiations
        template struct LazyGenericImplicit<double>;
        template struct LazyGenericImplicit<casadi::SX>;
        template struct LazyGenericImplicit<std::complex<double>>;

    } // namespace LoopConstraint
} // namespace grbda
