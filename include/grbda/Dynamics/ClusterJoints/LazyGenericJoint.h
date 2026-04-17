#ifndef GRBDA_GENERALIZED_JOINT_LAZY_GENERIC_H
#define GRBDA_GENERALIZED_JOINT_LAZY_GENERIC_H

#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"
#include <memory>
#include <mutex>

namespace grbda
{
    namespace LoopConstraint
    {
        /**
         * Lazy-initialized GenericImplicit constraint
         *
         * Defers CasADi symbolic graph construction until first use,
         * dramatically reducing cold-start time from ~5.5ms to <0.1ms per constraint.
         *
         * The symbolic functions are built on the first call to updateJacobians()
         * or updateBiases(), amortizing the cost over the first control loop iteration.
         *
         * Thread-safe: Uses mutex to ensure initialization happens exactly once.
         */
        template <typename Scalar = double>
        struct LazyGenericImplicit : Base<Scalar>
        {
            using SX = casadi::SX;
            using SymPhiFcn = std::function<DVec<SX>(const JointCoordinate<SX> &)>;
            using NativePhiFcn = std::function<DVec<Scalar>(const JointCoordinate<Scalar> &)>;

            // Constructor - only stores parameters, doesn't build symbolic functions
            LazyGenericImplicit(std::vector<bool> is_coordinate_independent,
                               SymPhiFcn phi_fcn,
                               NativePhiFcn phi_native = nullptr);

            // Overload that accepts CasadiHelperFunctions (ignores them for lazy init)
            LazyGenericImplicit(std::vector<bool> is_coordinate_independent,
                               SymPhiFcn phi_fcn,
                               NativePhiFcn phi_native,
                               const CasadiHelperFunctions<Scalar>& kg_dGdq_codegen)
                : LazyGenericImplicit(is_coordinate_independent, phi_fcn, phi_native)
            {
                // Ignore codegen helpers - we'll build symbolically on first use
            }

            // Delete copy constructor (mutex/unique_ptr not copyable)
            LazyGenericImplicit(const LazyGenericImplicit&) = delete;
            LazyGenericImplicit& operator=(const LazyGenericImplicit&) = delete;

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                // Create new instance instead of copying
                return std::make_shared<LazyGenericImplicit<Scalar>>(
                    is_coordinate_independent_, phi_sym_, phi_native_);
            }

            // Core interface - triggers lazy initialization if needed
            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override;
            DVec<Scalar> solveConstraintsComplex(const DVec<Scalar>& y_independent,
                                                const DVec<Scalar>& q_dep_init,
                                                int max_iters = 20,
                                                double tol = 1e-12) const
            {
                ensureInitialized();
                return impl_->solveConstraintsComplex(y_independent, q_dep_init, max_iters, tol);
            }
            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;
            void updateBiases(const JointState<Scalar> &joint_state) override;

            void createRandomStateHelpers() override;

            // Check if native phi is available (without triggering initialization)
            bool hasNativePhi() const { return phi_native_ != nullptr; }

            // Get native phi function (without triggering initialization)
            const NativePhiFcn& nativePhi() const { return phi_native_; }

            // Get coordinate independence (without triggering initialization)
            const std::vector<bool>& isCoordinateIndependent() const { return is_coordinate_independent_; }

            // Copy as double (for constraint solving) - creates a new GenericImplicit<double>
            // This WILL trigger CasADi symbolic construction, but only for the copy
            GenericImplicit<double> copyAsDouble() const
            {
                if constexpr (std::is_same_v<Scalar, double>) {
                    // When Scalar=double, phi_native can be passed directly
                    return GenericImplicit<double>(is_coordinate_independent_, phi_sym_, phi_native_);
                } else {
                    // When Scalar!=double, phi_native is templated on wrong type, so omit it
                    return GenericImplicit<double>(is_coordinate_independent_, phi_sym_);
                }
            }

            // Copy as symbolic (for CasADi operations)
            GenericImplicit<casadi::SX> copyAsSymbolic() const
            {
                return GenericImplicit<casadi::SX>(is_coordinate_independent_, phi_sym_);
            }

        private:
            // Build the actual GenericImplicit constraint (expensive, done once)
            void ensureInitialized() const;

            // Parameters saved from constructor
            std::vector<bool> is_coordinate_independent_;
            SymPhiFcn phi_sym_;
            NativePhiFcn phi_native_;

            // Lazy-initialized actual constraint
            mutable std::unique_ptr<GenericImplicit<Scalar>> impl_;
            mutable std::mutex init_mutex_;
            mutable bool initialized_ = false;
        };

    } // namespace LoopConstraint
} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_LAZY_GENERIC_H
