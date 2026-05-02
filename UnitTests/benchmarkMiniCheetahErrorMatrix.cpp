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
// Mini Cheetah Error Matrix Benchmark
// ============================================================================
// This benchmark computes the per-element error matrix for dtau/dq and dtau/dqdot
// averaged over 1000 random configurations, outputting data for heatmap visualization.
//
// Output: CSV files with 18x18 error matrices
// ============================================================================

// Lie group configuration addition for complex-valued states
template<typename T>
DVec<T> lieGroupConfigurationAddition(const DVec<T>& q0, const DVec<T>& dq, bool floating_base) {
    if (!floating_base) {
        return q0 + dq;
    } else {
        const int n_q = q0.size();
        const int n_v = dq.size();
        const int nj = n_v - 6;

        DVec<T> q_new = q0;
        q_new.tail(nj) += dq.tail(nj);

        Eigen::Matrix<T, 3, 1> p = q0.head(3);
        Eigen::Matrix<T, 4, 1> quat_vec = q0.segment(3, 4);
        Eigen::Matrix<T, 3, 1> omega_body = dq.head(3);

        Eigen::Matrix<T, 4, 1> delta_quat;

        bool has_imag = false;
        if constexpr (!std::is_arithmetic<T>::value) {
            has_imag = (std::abs(std::imag(omega_body[0])) > 1e-30 ||
                       std::abs(std::imag(omega_body[1])) > 1e-30 ||
                       std::abs(std::imag(omega_body[2])) > 1e-30);
        }

        if (has_imag) {
            delta_quat[0] = T(0.0);
            delta_quat.template tail<3>() = omega_body / T(2.0);
        } else {
            T theta = omega_body.norm();
            if (std::abs(theta) < 1e-10) {
                delta_quat[0] = T(1.0);
                delta_quat.template tail<3>() = omega_body / T(2.0);
            } else {
                T half_theta = theta / T(2.0);
                delta_quat[0] = std::cos(half_theta);
                delta_quat.template tail<3>() = std::sin(half_theta) * omega_body / theta;
            }
        }

        Eigen::Matrix<T, 4, 1> quat_new;

        if (has_imag) {
            T w = quat_vec[0], x = quat_vec[1], y = quat_vec[2], z = quat_vec[3];
            T dw = delta_quat[0], dx = delta_quat[1], dy = delta_quat[2], dz = delta_quat[3];

            quat_new[0] = w*dw - x*dx - y*dy - z*dz;
            quat_new[1] = w*dx + x*dw + y*dz - z*dy;
            quat_new[2] = w*dy - x*dz + y*dw + z*dx;
            quat_new[3] = w*dz + x*dy - y*dx + z*dw;

            quat_new = quat_vec + quat_new;
        } else {
            T w = quat_vec[0], x = quat_vec[1], y = quat_vec[2], z = quat_vec[3];
            T dw = delta_quat[0], dx = delta_quat[1], dy = delta_quat[2], dz = delta_quat[3];

            quat_new[0] = w*dw - x*dx - y*dy - z*dz;
            quat_new[1] = w*dx + x*dw + y*dz - z*dy;
            quat_new[2] = w*dy - x*dz + y*dw + z*dx;
            quat_new[3] = w*dz + x*dy - y*dx + z*dw;
        }

        q_new.segment(3, 4) = quat_new;

        Eigen::Matrix<T, 3, 1> v_body = dq.segment(3, 3);

        T w = quat_vec[0], x = quat_vec[1], y = quat_vec[2], z = quat_vec[3];
        Eigen::Matrix<T, 3, 3> R;
        R(0,0) = T(1) - T(2)*(y*y + z*z);  R(0,1) = T(2)*(x*y - w*z);        R(0,2) = T(2)*(x*z + w*y);
        R(1,0) = T(2)*(x*y + w*z);         R(1,1) = T(1) - T(2)*(x*x + z*z); R(1,2) = T(2)*(y*z - w*x);
        R(2,0) = T(2)*(x*z - w*y);         R(2,1) = T(2)*(y*z + w*x);        R(2,2) = T(1) - T(2)*(x*x + y*y);

        Eigen::Matrix<T, 3, 1> p_new = p + R.transpose() * v_body;
        q_new.head(3) = p_new;

        return q_new;
    }
}

int main() {
    std::cout << "\n" << std::string(80, '=') << "\n";
    std::cout << "Mini Cheetah Error Matrix Benchmark\n";
    std::cout << "Computing per-element error matrices averaged over 1000 trials\n";
    std::cout << std::string(80, '=') << "\n\n";

    const int NUM_TRIALS = 1000;
    const double h = 1e-20;  // Complex-step size
    const std::complex<double> ih(0.0, h);

    // Build both real and complex models
    MiniCheetah<double, ori_representation::Quaternion> robot_real;
    MiniCheetah<std::complex<double>, ori_representation::Quaternion> robot_complex;

    ClusterTreeModel<double> model_real = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    const int nDOF = model_real.getNumDegreesOfFreedom();
    const int nQ = model_real.getNumPositions();
    const bool floating_base = true;  // Mini Cheetah has floating base

    std::cout << "Mini Cheetah DOF: " << nDOF << "\n";
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
            dq_complex(j) = ih;

            DVec<std::complex<double>> q_perturbed = lieGroupConfigurationAddition(
                q_complex, dq_complex, floating_base);

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
            DVec<std::complex<double>> qd_perturbed = qd_complex;
            qd_perturbed(j) += ih;

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

        // Compute element-wise errors
        DMat<double> error_dq = (dtau_dq - dtau_dq_cs).cwiseAbs();
        DMat<double> error_dqdot = (dtau_dqdot - dtau_dqdot_cs).cwiseAbs();

        // Accumulate
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

    // Save to CSV files - write inside the Docker-mounted source tree
    std::string output_dir = std::string(SOURCE_DIRECTORY) + "/Benchmarking/data/";

    // Save average error matrices
    {
        std::ofstream file(output_dir + "minicheetah_error_dq_avg.csv");
        file << std::scientific << std::setprecision(6);
        for (int i = 0; i < nDOF; ++i) {
            for (int j = 0; j < nDOF; ++j) {
                file << error_dq_avg(i, j);
                if (j < nDOF - 1) file << ",";
            }
            file << "\n";
        }
        std::cout << "\nSaved: " << output_dir << "minicheetah_error_dq_avg.csv\n";
    }

    {
        std::ofstream file(output_dir + "minicheetah_error_dqdot_avg.csv");
        file << std::scientific << std::setprecision(6);
        for (int i = 0; i < nDOF; ++i) {
            for (int j = 0; j < nDOF; ++j) {
                file << error_dqdot_avg(i, j);
                if (j < nDOF - 1) file << ",";
            }
            file << "\n";
        }
        std::cout << "Saved: " << output_dir << "minicheetah_error_dqdot_avg.csv\n";
    }

    // Save max error matrices
    {
        std::ofstream file(output_dir + "minicheetah_error_dq_max.csv");
        file << std::scientific << std::setprecision(6);
        for (int i = 0; i < nDOF; ++i) {
            for (int j = 0; j < nDOF; ++j) {
                file << error_dq_max(i, j);
                if (j < nDOF - 1) file << ",";
            }
            file << "\n";
        }
        std::cout << "Saved: " << output_dir << "minicheetah_error_dq_max.csv\n";
    }

    {
        std::ofstream file(output_dir + "minicheetah_error_dqdot_max.csv");
        file << std::scientific << std::setprecision(6);
        for (int i = 0; i < nDOF; ++i) {
            for (int j = 0; j < nDOF; ++j) {
                file << error_dqdot_max(i, j);
                if (j < nDOF - 1) file << ",";
            }
            file << "\n";
        }
        std::cout << "Saved: " << output_dir << "minicheetah_error_dqdot_max.csv\n";
    }

    std::cout << "\n" << std::string(80, '=') << "\n";
    std::cout << "Benchmark Complete\n";
    std::cout << std::string(80, '=') << "\n";

    return 0;
}
