#include <iostream>
#include <fstream>

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;
using RigidBodyTreePtr = std::shared_ptr<RigidBodyTreeModel>;
using ReflectedInertiaTreePtr = std::shared_ptr<ReflectedInertiaTreeModel>;

// TODO(@MatthewChignoli): This should be a helper function shared with accuracy benchmarks

// TODO(@MatthewChignoli): We could also just make a benchmark helper function that takes a lambda function as a parameter. And then we could heavily re-use that

bool setRandomStates(ClusterTreeModel &cluster_model,
                     std::vector<RigidBodyTreePtr> &rigid_body_models,
                     std::vector<ReflectedInertiaTreePtr> &reflected_inertia_models)
{

    ModelState model_state;
    DVec<double> independent_joint_pos = DVec<double>::Zero(0);
    DVec<double> independent_joint_vel = DVec<double>::Zero(0);
    DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
    DVec<double> spanning_joint_vel = DVec<double>::Zero(0);

    for (const auto &cluster : cluster_model.clusters())
    {
        JointState joint_state = cluster->joint_->randomJointState();
        if (joint_state.position.hasNaN())
        {
            return true;
        }

        JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

        DVec<double> independent_joint_pos_i;
        DVec<double> independent_joint_vel_i;
        if (cluster->joint_->type() == GeneralizedJointTypes::TelloHipDifferential ||
            cluster->joint_->type() == GeneralizedJointTypes::TelloKneeAnkleDifferential)
        {
            independent_joint_pos_i = spanning_joint_state.position.tail<2>();
            independent_joint_vel_i = spanning_joint_state.velocity.tail<2>();
        }
        else
        {
            if (joint_state.position.isSpanning() || joint_state.velocity.isSpanning())
                throw std::runtime_error("Initializing reflected inertia model requires all independent coordinates");
            independent_joint_pos_i = joint_state.position;
            independent_joint_vel_i = joint_state.velocity;
        }

        independent_joint_pos = appendEigenVector(independent_joint_pos,
                                                  independent_joint_pos_i);
        independent_joint_vel = appendEigenVector(independent_joint_vel,
                                                  independent_joint_vel_i);

        spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                               spanning_joint_state.position);
        spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                               spanning_joint_state.velocity);

        model_state.push_back(joint_state);
    }

    cluster_model.initializeState(model_state);

    for (RigidBodyTreePtr model : rigid_body_models)
        model->initializeState(spanning_joint_pos, spanning_joint_vel);

    for (ReflectedInertiaTreePtr model : reflected_inertia_models)
        model->initializeIndependentStates(independent_joint_pos, independent_joint_vel);

    return false;
}

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
    ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreePtr lg_custom_mult_model =
        std::make_shared<RigidBodyTreeModel>(cluster_model, FwdDynMethod::LagrangeMultiplierCustom);
    RigidBodyTreePtr lg_eigen_mult_model =
        std::make_shared<RigidBodyTreeModel>(cluster_model, FwdDynMethod::LagrangeMultiplierEigen);
    RigidBodyTreePtr projection_model =
        std::make_shared<RigidBodyTreeModel>(cluster_model, FwdDynMethod::Projection);
    ReflectedInertiaTreePtr reflected_inertia_model =
        std::make_shared<ReflectedInertiaTreeModel>(cluster_model,
                                                    RotorInertiaApproximation::DIAGONAL);
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
    ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreePtr projection_model =
        std::make_shared<RigidBodyTreeModel>(cluster_model, FwdDynMethod::Projection);
    ReflectedInertiaTreePtr reflected_inertia_model =
        std::make_shared<ReflectedInertiaTreeModel>(cluster_model,
                                                    RotorInertiaApproximation::DIAGONAL);

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
    ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreePtr projection_model =
        std::make_shared<RigidBodyTreeModel>(cluster_model, FwdDynMethod::Projection);
    ReflectedInertiaTreePtr reflected_inertia_model =
        std::make_shared<ReflectedInertiaTreeModel>(cluster_model,
                                                    RotorInertiaApproximation::DIAGONAL);
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
void runApplyTestForceBenchmark(std::ofstream &file, const std::string& contact_point)
{
    Timer timer;
    double t_cluster = 0.;
    double t_projection = 0.;
    double t_reflected_inertia = 0.;

    RobotType robot;
    ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreePtr projection_model =
        std::make_shared<RigidBodyTreeModel>(cluster_model, FwdDynMethod::Projection);
    ReflectedInertiaTreePtr ref_inertia_model =
        std::make_shared<ReflectedInertiaTreeModel>(cluster_model,
                                                    RotorInertiaApproximation::DIAGONAL);
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
        cluster_model.applyLocalFrameTestForceAtContactPoint(force, contact_point, dstate);
        t_cluster += timer.getMs();

        timer.start();
        projection_model->applyLocalFrameTestForceAtContactPoint(force, contact_point, dstate);
        t_projection += timer.getMs();

        timer.start();
        ref_inertia_model->applyLocalFrameTestForceAtContactPoint(force, contact_point, dstate);
        t_reflected_inertia += timer.getMs();
    }

    file << t_cluster / num_state_samples << ","
         << t_projection / num_state_samples << ","
         << t_reflected_inertia / num_state_samples << std::endl;

    std::cout << "Finished benchmark for robot" << std::endl;
}

int main()
{
    std::cout << "\n\n**Starting Forward Dynamics Timing Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/TimingFD_";
    std::ofstream fd_file;
    fd_file.open(path_to_data + "Robots.csv");
    runForwardDynamicsBenchmark<MiniCheetah>(fd_file);
    runForwardDynamicsBenchmark<Tello>(fd_file);
    runForwardDynamicsBenchmark<MIT_Humanoid>(fd_file);
    runForwardDynamicsBenchmark<TelloWithArms>(fd_file);
    fd_file.close();

    std::cout << "\n\n**Starting Inverse Dynamics Timing Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/TimingID_";
    std::ofstream id_file;
    id_file.open(path_to_data + "Robots.csv");
    runInverseDynamicsBenchmark<MiniCheetah>(id_file);
    runInverseDynamicsBenchmark<Tello>(id_file);
    runInverseDynamicsBenchmark<MIT_Humanoid>(id_file);
    runInverseDynamicsBenchmark<TelloWithArms>(id_file);
    id_file.close();

    std::cout << "\n\n**Starting Inv OSIM Timing Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/TimingIOSIM_";
    std::ofstream iosim_file;
    iosim_file.open(path_to_data + "Robots.csv");
    runInverseOperationalSpaceInertiaBenchmark<MiniCheetah>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<Tello>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<MIT_Humanoid>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<TelloWithArms>(iosim_file);
    iosim_file.close();

    std::cout << "\n\n**Starting Apply Test Force Timing Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/TimingATF_";
    std::ofstream atf_file;
    atf_file.open(path_to_data + "Robots.csv");
    runApplyTestForceBenchmark<MiniCheetah>(atf_file, "FL_foot_contact");
    runApplyTestForceBenchmark<Tello>(atf_file, "left-toe_contact");
    runApplyTestForceBenchmark<MIT_Humanoid>(atf_file, "left_toe_contact");
    runApplyTestForceBenchmark<TelloWithArms>(atf_file, "left-toe_contact");
    atf_file.close();
}
