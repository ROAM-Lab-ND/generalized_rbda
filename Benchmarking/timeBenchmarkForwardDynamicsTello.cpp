#include <iostream>
#include <fstream>

#include "Dynamics/RigidBodyTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

void runTelloBenchmark(std::ofstream &file, const bool include_forces)
{
    Timer timer;
    double t_cluster = 0.;
    double t_lg_custom = 0.;
    double t_lg_eigen = 0.;
    double t_projection = 0.;

    const int num_state_samples = 1000;

    Tello robot = Tello();
    ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreeModel lg_custom_mult_model{cluster_model,
                                            FwdDynMethod::LagrangeMultiplierCustom};
    RigidBodyTreeModel lg_eigen_mult_model{cluster_model,
                                           FwdDynMethod::LagrangeMultiplierEigen};
    RigidBodyTreeModel projection_model{cluster_model,
                                        FwdDynMethod::Projection};

    const int nq = cluster_model.getNumPositions();
    const int nv = cluster_model.getNumDegreesOfFreedom();

    for (int i = 0; i < num_state_samples; i++)
    {
        // Set random state
        ModelState model_state;
        DVec<double> independent_joint_pos = DVec<double>::Zero(0);
        DVec<double> independent_joint_vel = DVec<double>::Zero(0);
        DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
        DVec<double> spanning_joint_vel = DVec<double>::Zero(0);

	bool nan_detected = false;
        for (const auto &cluster : cluster_model.clusters())
        {
            JointState joint_state = cluster->joint_->randomJointState();
	    if (joint_state.position.hasNaN())
	    {
		nan_detected = true;
		break;
	    }

            JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);
            spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                   spanning_joint_state.position);
            spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                   spanning_joint_state.velocity);

            model_state.push_back(joint_state);
        }
	if (nan_detected)
	{
	    i--;
	    continue;
	}

        cluster_model.initializeState(model_state);
        lg_custom_mult_model.initializeState(spanning_joint_pos, spanning_joint_vel);
        lg_eigen_mult_model.initializeState(spanning_joint_pos, spanning_joint_vel);
        projection_model.initializeState(spanning_joint_pos, spanning_joint_vel);

        // TODO(@MatthewChignoli): Seems like including forces does not slow it down? That's strange
        if (include_forces)
        {
            // Set random spatial forces on bodies
            std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs;
            for (const auto &body : cluster_model.bodies())
                force_and_index_pairs.emplace_back(body.index_, SVec<double>::Random());
            cluster_model.initializeExternalForces(force_and_index_pairs);
            lg_custom_mult_model.initializeExternalForces(force_and_index_pairs);
            lg_eigen_mult_model.initializeExternalForces(force_and_index_pairs);
            projection_model.initializeExternalForces(force_and_index_pairs);
        }

        // Forward Dynamics
        DVec<double> tau = DVec<double>::Random(nv);
        timer.start();
        cluster_model.forwardDynamics(tau);
        t_cluster += timer.getMs();

        timer.start();
        lg_custom_mult_model.extractLoopClosureFunctionsFromClusterModel(cluster_model);
	lg_custom_mult_model.forwardDynamics(tau);
        t_lg_custom += timer.getMs();

        timer.start();
        lg_eigen_mult_model.extractLoopClosureFunctionsFromClusterModel(cluster_model);
        lg_eigen_mult_model.forwardDynamics(tau);
        t_lg_eigen += timer.getMs();

        timer.start();
        projection_model.extractLoopClosureFunctionsFromClusterModel(cluster_model);
        projection_model.forwardDynamics(tau);
        t_projection += timer.getMs();
    }

    file << t_cluster / num_state_samples << ","
         << t_lg_custom / num_state_samples << ","
         << t_lg_eigen / num_state_samples << ","
         << t_projection / num_state_samples << std::endl;

    std::cout << "Finished benchmark for robot" << std::endl;
}

int main()
{
    const bool include_forces = false;
    const std::string path_to_data = "../Benchmarking/data/";

    std::cout << "** Tello Benchmark **" << std::endl;

    std::ofstream tello_file;
    tello_file.open(path_to_data + "Timing_Tello.csv");

    runTelloBenchmark(tello_file, include_forces);

    tello_file.close();
}
