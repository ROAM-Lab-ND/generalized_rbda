#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <complex>
#include <vector>
#include <string>
#include <cmath>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "config.h"

using namespace grbda;

// ============================================================================
// Serial Chain (30 links) Error Matrix Benchmark
// ============================================================================
// This benchmark computes the per-element error matrix for dtau/dq and dtau/dqdot
// for a 30-link serial chain with revolute joints with rotors.
// Data is averaged over 1000 random configurations.
//
// Output: CSV files with 30x30 error matrices
// ============================================================================

int main()
{
    std::string output_dir = "/tmp/";

    std::cout << "\n";
    std::cout << "===========================================================================\n";
    std::cout << "Serial Chain (30 links) Error Matrix Benchmark - Complex-Step Validation\n";
    std::cout << "===========================================================================\n\n";

    typedef RevoluteChainWithRotor<30, double> SerialChain30;
    typedef RevoluteChainWithRotor<30, std::complex<double>> SerialChain30Complex;

    // Use uniform parameters (not random) so both models are identical
    SerialChain30 robot_real(false);
    SerialChain30Complex robot_complex(false);

    ClusterTreeModel<double> model_real = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    const int nDOF = model_real.getNumDegreesOfFreedom();
    const int NUM_TRIALS = 1000;
    const double h = 1e-20;  // Complex-step step size
    const bool floating_base = false;  // Serial chain has fixed base

    std::cout << "Serial Chain DOF: " << nDOF << "\n";
    std::cout << "Running " << NUM_TRIALS << " trials...\n\n";

    // Accumulate error matrices
    DMat<double> error_dq_sum = DMat<double>::Zero(nDOF, nDOF);
    DMat<double> error_dqdot_sum = DMat<double>::Zero(nDOF, nDOF);
    DMat<double> error_dq_max = DMat<double>::Zero(nDOF, nDOF);
    DMat<double> error_dqdot_max = DMat<double>::Zero(nDOF, nDOF);

    for (int trial = 0; trial < NUM_TRIALS; ++trial) {
        if ((trial + 1) % 100 == 0) {
            std::cout << "  Trial " << (trial + 1) << "/" << NUM_TRIALS << "\n";
        }

        // Set random state on real model
        ModelState<double> model_state_real;
        for (const auto& cluster : model_real.clusters()) {
            model_state_real.push_back(cluster->joint_->randomJointState());
        }
        model_real.setState(model_state_real);

        // Random acceleration
        const DVec<double> ydd_real = DVec<double>::Random(nDOF);

        // Get analytical derivatives
        auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd_real);

        // Get real state
        std::pair<DVec<double>, DVec<double>> state_real = model_real.getState();
        const DVec<double>& q0 = state_real.first;
        const DVec<double>& qd0 = state_real.second;

        // Set the same state on complex model
        DVec<std::complex<double>> q_complex = q0.cast<std::complex<double>>();
        DVec<std::complex<double>> qd_complex = qd0.cast<std::complex<double>>();

        ModelState<std::complex<double>> model_state_complex;
        int pos_idx = 0, vel_idx = 0;
        for (const auto& cluster : model_complex.clusters()) {
            JointState<std::complex<double>> joint_state;
            joint_state.position = q_complex.segment(pos_idx, cluster->num_positions_);
            joint_state.velocity = qd_complex.segment(vel_idx, cluster->num_velocities_);
            model_state_complex.push_back(joint_state);
            pos_idx += cluster->num_positions_;
            vel_idx += cluster->num_velocities_;
        }
        model_complex.setState(model_state_complex);

        DVec<std::complex<double>> ydd_complex = ydd_real.cast<std::complex<double>>();

        // Compute dtau_dq using complex-step and compare element-wise
        DMat<double> dtau_dq_cs(nDOF, nDOF);
        for (int j = 0; j < nDOF; ++j) {
            DVec<std::complex<double>> dq_complex = DVec<std::complex<double>>::Zero(nDOF);
            dq_complex(j) = std::complex<double>(0.0, h);

            // For fixed base, configuration addition is simple
            DVec<std::complex<double>> q_perturbed = q_complex + dq_complex;

            ModelState<std::complex<double>> state_perturbed;
            pos_idx = 0; vel_idx = 0;
            for (const auto& cluster : model_complex.clusters()) {
                JointState<std::complex<double>> joint_state;
                joint_state.position = q_perturbed.segment(pos_idx, cluster->num_positions_);
                joint_state.velocity = qd_complex.segment(vel_idx, cluster->num_velocities_);
                state_perturbed.push_back(joint_state);
                pos_idx += cluster->num_positions_;
                vel_idx += cluster->num_velocities_;
            }
            model_complex.setState(state_perturbed);

            DVec<std::complex<double>> tau_perturbed = model_complex.inverseDynamics(ydd_complex);
            dtau_dq_cs.col(j) = tau_perturbed.imag() / h;
        }

        // Compute dtau_dqdot using complex-step
        DMat<double> dtau_dqdot_cs(nDOF, nDOF);
        for (int j = 0; j < nDOF; ++j) {
            DVec<std::complex<double>> dqd_complex = DVec<std::complex<double>>::Zero(nDOF);
            dqd_complex(j) = std::complex<double>(0.0, h);

            DVec<std::complex<double>> qd_perturbed = qd_complex + dqd_complex;

            ModelState<std::complex<double>> state_perturbed;
            pos_idx = 0; vel_idx = 0;
            for (const auto& cluster : model_complex.clusters()) {
                JointState<std::complex<double>> joint_state;
                joint_state.position = q_complex.segment(pos_idx, cluster->num_positions_);
                joint_state.velocity = qd_perturbed.segment(vel_idx, cluster->num_velocities_);
                state_perturbed.push_back(joint_state);
                pos_idx += cluster->num_positions_;
                vel_idx += cluster->num_velocities_;
            }
            model_complex.setState(state_perturbed);

            DVec<std::complex<double>> tau_perturbed = model_complex.inverseDynamics(ydd_complex);
            dtau_dqdot_cs.col(j) = tau_perturbed.imag() / h;
        }

        // Compute errors and update sums and maxima
        DMat<double> error_dq = (dtau_dq_cs - dtau_dq).cwiseAbs();
        DMat<double> error_dqdot = (dtau_dqdot_cs - dtau_dqdot).cwiseAbs();

        error_dq_sum += error_dq;
        error_dqdot_sum += error_dqdot;
        error_dq_max = error_dq_max.cwiseMax(error_dq);
        error_dqdot_max = error_dqdot_max.cwiseMax(error_dqdot);
    }

    // Compute averages
    DMat<double> error_dq_avg = error_dq_sum / NUM_TRIALS;
    DMat<double> error_dqdot_avg = error_dqdot_sum / NUM_TRIALS;

    // Output results
    std::cout << "\nResults:\n";
    std::cout << "  dtau/dq   - Max error: " << std::scientific << error_dq_max.maxCoeff() << "\n";
    std::cout << "  dtau/dq   - Avg error: " << error_dq_avg.mean() << "\n";
    std::cout << "  dtau/dqdot - Max error: " << error_dqdot_max.maxCoeff() << "\n";
    std::cout << "  dtau/dqdot - Avg error: " << error_dqdot_avg.mean() << "\n";

    // Save to CSV files
    std::cout << "\nExporting error matrices to CSV...\n";

    // Save average error matrices
    {
        std::ofstream file(output_dir + "serialchain30_error_dq_avg.csv");
        file << std::scientific << std::setprecision(6);
        for (int i = 0; i < nDOF; ++i) {
            for (int j = 0; j < nDOF; ++j) {
                file << error_dq_avg(i, j);
                if (j < nDOF - 1) file << ",";
            }
            file << "\n";
        }
        std::cout << "  Saved: " << output_dir << "serialchain30_error_dq_avg.csv\n";
    }

    {
        std::ofstream file(output_dir + "serialchain30_error_dqdot_avg.csv");
        file << std::scientific << std::setprecision(6);
        for (int i = 0; i < nDOF; ++i) {
            for (int j = 0; j < nDOF; ++j) {
                file << error_dqdot_avg(i, j);
                if (j < nDOF - 1) file << ",";
            }
            file << "\n";
        }
        std::cout << "  Saved: " << output_dir << "serialchain30_error_dqdot_avg.csv\n";
    }

    // Save max error matrices
    {
        std::ofstream file(output_dir + "serialchain30_error_dq_max.csv");
        file << std::scientific << std::setprecision(6);
        for (int i = 0; i < nDOF; ++i) {
            for (int j = 0; j < nDOF; ++j) {
                file << error_dq_max(i, j);
                if (j < nDOF - 1) file << ",";
            }
            file << "\n";
        }
        std::cout << "  Saved: " << output_dir << "serialchain30_error_dq_max.csv\n";
    }

    {
        std::ofstream file(output_dir + "serialchain30_error_dqdot_max.csv");
        file << std::scientific << std::setprecision(6);
        for (int i = 0; i < nDOF; ++i) {
            for (int j = 0; j < nDOF; ++j) {
                file << error_dqdot_max(i, j);
                if (j < nDOF - 1) file << ",";
            }
            file << "\n";
        }
        std::cout << "  Saved: " << output_dir << "serialchain30_error_dqdot_max.csv\n";
    }

    std::cout << "\n===========================================================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "===========================================================================\n\n";

    return 0;
}
