#include <iostream>
#include <fstream>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"
#include "Utils/Timer.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;
using namespace grbda::ori_representation;

template <typename RobotType>
void runForwardDynamicsBenchmark(std::ofstream &file)
{
    Timer timer;
    double t_cluster = 0.;
    double t_lg_custom = 0.;
    double t_lg_eigen = 0.;
    double t_projection = 0.;
    double t_reflected_inertia = 0.;

    RobotType robot;

    ClusterTreeModel<> cluster_model = robot.buildClusterTreeModel();

    RigidBodyTreePtr lg_custom_mult_model =
        std::make_shared<RigidBodyTreeModel<>>(cluster_model, FwdDynMethod::LagrangeMultiplierCustom);
    RigidBodyTreePtr lg_eigen_mult_model =
        std::make_shared<RigidBodyTreeModel<>>(cluster_model, FwdDynMethod::LagrangeMultiplierEigen);
    RigidBodyTreePtr projection_model =
        std::make_shared<RigidBodyTreeModel<>>(cluster_model, FwdDynMethod::Projection);

    ReflectedInertiaTreePtr reflected_inertia_model =
        std::make_shared<ReflectedInertiaTreeModel<>>(cluster_model, RotorInertiaApproximation::DIAGONAL);

    std::vector<RigidBodyTreePtr> rigid_body_models{lg_custom_mult_model,
                                                    lg_eigen_mult_model,
                                                    projection_model};
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{reflected_inertia_model};

    const int num_state_samples = 1000;
    const int nv = cluster_model.getNumDegreesOfFreedom();
    for (int j = 0; j < num_state_samples; j++)
    {
        bool nan_detected = setRandomStates(cluster_model, rigid_body_models, ref_inertia_models);
        if (nan_detected)
        {
            j--;
            continue;
        }

        // Forward Dynamics
        DVec<double> tau = DVec<double>::Random(nv);
        timer.start();
        cluster_model.forwardDynamics(tau);
        t_cluster += timer.getMs();

        timer.start();
        lg_custom_mult_model->forwardDynamics(tau);
        t_lg_custom += timer.getMs();

        timer.start();
        lg_eigen_mult_model->forwardDynamics(tau);
        t_lg_eigen += timer.getMs();

        timer.start();
        projection_model->forwardDynamics(tau);
        t_projection += timer.getMs();

        timer.start();
        reflected_inertia_model->forwardDynamics(tau);
        t_reflected_inertia += timer.getMs();
    }

    file << t_cluster / num_state_samples << ","
         << t_lg_custom / num_state_samples << ","
         << t_lg_eigen / num_state_samples << ","
         << t_projection / num_state_samples << ","
         << t_reflected_inertia / num_state_samples << std::endl;

    std::cout << "Finished benchmark for robot" << std::endl;
}

template <typename RobotType>
void runInverseDynamicsBenchmark(std::ofstream &file)
{
    Timer timer;
    double t_cluster = 0.;
    double t_projection = 0.;
    double t_reflected_inertia = 0.;

    RobotType robot;
    ClusterTreeModel<> cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreePtr projection_model =
        std::make_shared<RigidBodyTreeModel<>>(cluster_model, FwdDynMethod::Projection);
    ReflectedInertiaTreePtr reflected_inertia_model =
        std::make_shared<ReflectedInertiaTreeModel<>>(cluster_model, RotorInertiaApproximation::DIAGONAL);

    std::vector<RigidBodyTreePtr> rigid_body_models{projection_model};
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{reflected_inertia_model};

    const int num_state_samples = 1000;
    const int nv = cluster_model.getNumDegreesOfFreedom();
    for (int j = 0; j < num_state_samples; j++)
    {
        bool nan_detected = setRandomStates(cluster_model, rigid_body_models, ref_inertia_models);
        if (nan_detected)
        {
            j--;
            continue;
        }

        // Inverse Dynamics
        DVec<double> ydd = DVec<double>::Random(nv);
        timer.start();
        cluster_model.inverseDynamics(ydd);
        t_cluster += timer.getMs();

        timer.start();
        projection_model->inverseDynamics(ydd);
        t_projection += timer.getMs();

        timer.start();
        reflected_inertia_model->inverseDynamics(ydd);
        t_reflected_inertia += timer.getMs();
    }

    file << t_cluster / num_state_samples << ","
         << t_projection / num_state_samples << ","
         << t_reflected_inertia / num_state_samples << std::endl;

    std::cout << "Finished benchmark for robot" << std::endl;
}

template <typename RobotType>
void runInverseOperationalSpaceInertiaBenchmark(std::ofstream &file)
{
    Timer timer;
    double t_cluster = 0.;
    double t_projection = 0.;
    double t_reflected_inertia = 0.;

    RobotType robot;
    ClusterTreeModel<> cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreePtr projection_model =
        std::make_shared<RigidBodyTreeModel<>>(cluster_model, FwdDynMethod::Projection);
    ReflectedInertiaTreePtr reflected_inertia_model =
        std::make_shared<ReflectedInertiaTreeModel<>>(cluster_model, RotorInertiaApproximation::DIAGONAL);
    std::vector<RigidBodyTreePtr> rigid_body_models{projection_model};
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{reflected_inertia_model};

    const int num_state_samples = 1000;
    const int nv = cluster_model.getNumDegreesOfFreedom();
    for (int j = 0; j < num_state_samples; j++)
    {
        bool nan_detected = setRandomStates(cluster_model, rigid_body_models, ref_inertia_models);
        if (nan_detected)
        {
            j--;
            continue;
        }

        timer.start();
        cluster_model.inverseOperationalSpaceInertiaMatrix();
        t_cluster += timer.getMs();

        timer.start();
        projection_model->inverseOperationalSpaceInertiaMatrix();
        t_projection += timer.getMs();

        timer.start();
        reflected_inertia_model->inverseOperationalSpaceInertiaMatrix();
        t_reflected_inertia += timer.getMs();
    }

    file << t_cluster / num_state_samples << ","
         << t_projection / num_state_samples << ","
         << t_reflected_inertia / num_state_samples << std::endl;

    std::cout << "Finished benchmark for robot" << std::endl;
}

template <typename RobotType>
void runApplyTestForceBenchmark(std::ofstream &file, const std::string &contact_point)
{
    Timer timer;
    double t_cluster = 0.;
    double t_projection = 0.;
    double t_reflected_inertia = 0.;

    RobotType robot;
    ClusterTreeModel<> cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreePtr projection_model =
        std::make_shared<RigidBodyTreeModel<>>(cluster_model, FwdDynMethod::Projection);
    ReflectedInertiaTreePtr ref_inertia_model =
        std::make_shared<ReflectedInertiaTreeModel<>>(cluster_model, RotorInertiaApproximation::DIAGONAL);
    std::vector<RigidBodyTreePtr> rigid_body_models{projection_model};
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{ref_inertia_model};

    const int num_state_samples = 1000;
    const int nv = cluster_model.getNumDegreesOfFreedom();
    for (int j = 0; j < num_state_samples; j++)
    {
        bool nan_detected = setRandomStates(cluster_model, rigid_body_models, ref_inertia_models);
        if (nan_detected)
        {
            j--;
            continue;
        }

        // Forward Dynamics
        DVec<double> force = DVec<double>::Random(nv);
        DVec<double> dstate;
        timer.start();
        cluster_model.applyTestForce(contact_point, force, dstate);
        t_cluster += timer.getMs();

        timer.start();
        projection_model->applyTestForce(contact_point, force, dstate);
        t_projection += timer.getMs();

        timer.start();
        ref_inertia_model->applyTestForce(contact_point, force, dstate);
        t_reflected_inertia += timer.getMs();
    }

    file << t_cluster / num_state_samples << ","
         << t_projection / num_state_samples << ","
         << t_reflected_inertia / num_state_samples << std::endl;

    std::cout << "Finished benchmark for robot" << std::endl;
}

#define N_CHAIN 12

int main()
{
    std::cout << "\n\n**Starting Forward Dynamics Timing Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/TimingFD_";
    std::ofstream fd_file;
    fd_file.open(path_to_data + "Robots.csv");
    runForwardDynamicsBenchmark<RevoluteChainWithRotor<N_CHAIN>>(fd_file);
    runForwardDynamicsBenchmark<RevolutePairChainWithRotor<N_CHAIN>>(fd_file);
    runForwardDynamicsBenchmark<MiniCheetah<double, QuaternionRepresentation< Quat<double> >>>(fd_file);
    runForwardDynamicsBenchmark<MIT_Humanoid<double, QuaternionRepresentation< Quat<double> >>>(fd_file);
    runForwardDynamicsBenchmark<TelloWithArms>(fd_file);
    fd_file.close();

    std::cout << "\n\n**Starting Inverse Dynamics Timing Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/TimingID_";
    std::ofstream id_file;
    id_file.open(path_to_data + "Robots.csv");
    runInverseDynamicsBenchmark<RevoluteChainWithRotor<N_CHAIN>>(id_file);
    runInverseDynamicsBenchmark<RevolutePairChainWithRotor<N_CHAIN>>(id_file);
    runInverseDynamicsBenchmark<MiniCheetah<double, QuaternionRepresentation< Quat<double> >>>(id_file);
    runInverseDynamicsBenchmark<MIT_Humanoid<double, QuaternionRepresentation< Quat<double> >>>(id_file);
    runInverseDynamicsBenchmark<TelloWithArms>(id_file);
    id_file.close();

    std::cout << "\n\n**Starting Inv OSIM Timing Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/TimingIOSIM_";
    std::ofstream iosim_file;
    iosim_file.open(path_to_data + "Robots.csv");
    runInverseOperationalSpaceInertiaBenchmark<RevoluteChainWithRotor<N_CHAIN>>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<RevolutePairChainWithRotor<N_CHAIN>>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<MiniCheetah<double, QuaternionRepresentation< Quat<double> >>>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<MIT_Humanoid<double, QuaternionRepresentation< Quat<double> >>>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<TelloWithArms>(iosim_file);
    iosim_file.close();

    std::cout << "\n\n**Starting Apply Test Force Timing Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/TimingATF_";
    std::ofstream atf_file;
    atf_file.open(path_to_data + "Robots.csv");
    std::string rev_chain_cp = "cp-" + std::to_string(N_CHAIN - 1);
    runApplyTestForceBenchmark<RevoluteChainWithRotor<N_CHAIN>>(atf_file, rev_chain_cp);
    std::string rev_pair_chain_cp = "cp-B-" + std::to_string(N_CHAIN / 2 - 1);
    runApplyTestForceBenchmark<RevolutePairChainWithRotor<N_CHAIN>>(atf_file, rev_pair_chain_cp);
    runApplyTestForceBenchmark<MiniCheetah<double, QuaternionRepresentation< Quat<double> >>>(atf_file, "FL_foot_contact");
    runApplyTestForceBenchmark<MIT_Humanoid<double, QuaternionRepresentation< Quat<double> >>>(atf_file, "left_toe_contact");
    runApplyTestForceBenchmark<TelloWithArms>(atf_file, "left-toe_contact");
    atf_file.close();
}
