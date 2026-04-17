#ifndef GRBDA_PRECOMPILED_GENERIC_JOINT_H
#define GRBDA_PRECOMPILED_GENERIC_JOINT_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"
#include "grbda/Codegen/CasadiGen.h"
#include "grbda/Codegen/tello_constraints_optimized.h"

namespace grbda
{
    namespace LoopConstraint
    {
        // Forward declaration
        template <typename Scalar> struct GenericImplicit;
        /**
         * FAST GenericImplicit using ONLY pre-compiled C code (no CasADi symbolic graph).
         *
         * This bypasses the expensive symbolic construction entirely,
         * reducing initialization from ~900μs to ~5μs per constraint.
         *
         * For Tello with 6 constraints: 5.4ms -> 0.03ms (180x faster!)
         */
        template <typename Scalar = double>
        struct PrecompiledGenericImplicit : Base<Scalar>
        {
            using SX = casadi::SX;
            using SymPhiFcn = std::function<DVec<SX>(const JointCoordinate<SX> &)>;
            using NativePhiFcn = std::function<DVec<Scalar>(const JointCoordinate<Scalar> &)>;

            // Constructor - uses pre-compiled functions for main operations
            // Stores symbolic phi for rare operations like copyAsDouble()
            PrecompiledGenericImplicit(
                std::vector<bool> is_coordinate_independent,
                SymPhiFcn phi_sym,
                NativePhiFcn phi_native,
                const std::string& constraint_name = "hip");

            // Delete copy constructor
            PrecompiledGenericImplicit(const PrecompiledGenericImplicit&) = delete;
            PrecompiledGenericImplicit& operator=(const PrecompiledGenericImplicit&) = delete;

            std::shared_ptr<Base<Scalar>> clone() const override {
                // Create new instance
                return std::make_shared<PrecompiledGenericImplicit<Scalar>>(
                    is_coordinate_independent_, phi_sym_, phi_native_, constraint_name_);
            }

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override;
            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;
            void updateBiases(const JointState<Scalar> &joint_state) override;

            const std::vector<bool>& isCoordinateIndependent() const { return is_coordinate_independent_; }
            void createRandomStateHelpers() override;

            // Methods needed for randomJointState()
            bool hasNativePhi() const { return phi_native_ != nullptr; }
            const NativePhiFcn& nativePhi() const { return phi_native_; }

            // Copy as double - for PrecompiledGenericImplicit<double>, just return a copy
            // For other Scalar types, create a GenericImplicit (shouldn't happen in practice)
            GenericImplicit<double> copyAsDouble() const;

        private:
            std::vector<bool> is_coordinate_independent_;
            SymPhiFcn phi_sym_;  // Only used for copyAsDouble(), not for main operations
            NativePhiFcn phi_native_;
            std::string constraint_name_;

            int state_dim_;
            int ind_dim_;
            int dep_dim_;
            std::vector<int> ind_coords_, dep_coords_;
        };

    } // namespace LoopConstraint
} // namespace grbda

#endif // GRBDA_PRECOMPILED_GENERIC_JOINT_H
