#include <iostream>
#include <fstream>

#include "Dynamics/ClusterTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

void runTelloBenchmark(std::ofstream &file)
{
    const int num_state_samples = 1000;
    ClusterTreeTimingStatistics timing_stats;

    Tello robot = Tello();
    ClusterTreeModel cluster_model = robot.buildClusterTreeModel();

    const int nq = cluster_model.getNumPositions();
    const int nv = cluster_model.getNumDegreesOfFreedom();

    for (int i = 0; i < num_state_samples; i++)
    {

        // Set random state
        ModelState model_state;
	bool nan_detected = false;
        for (const auto &cluster : cluster_model.clusters())
        {
	    JointState joint_state = cluster->joint_->randomJointState();
	    if (joint_state.position.hasNaN())
	    {
		nan_detected = true;
		break;
	    }
            model_state.push_back(joint_state);
        }
	if (nan_detected)
	{
	    i--;
	    continue;
	}

        cluster_model.initializeState(model_state);

        // Forward Dynamics
        DVec<double> tau = DVec<double>::Random(nv);
        cluster_model.forwardDynamics(tau);

        timing_stats += cluster_model.getTimingStatistics();
    }

    timing_stats /= num_state_samples;

    file << timing_stats.forward_kinematics_time << ","
         << timing_stats.update_articulated_bodies_time << ","
         << timing_stats.forward_pass1_time << ","
         << timing_stats.external_force_time << ","
         << timing_stats.backward_pass_time << ","
         << timing_stats.forward_pass2_time << std::endl;

    std::cout << "Finished benchmark for robot" << std::endl;
}

int main()
{
    const std::string path_to_data = "../Benchmarking/data/CL_";

    std::cout << "** Tello Benchmark **" << std::endl;

    std::ofstream tello_file;
    tello_file.open(path_to_data + "Tello.csv");

    runTelloBenchmark(tello_file);

    tello_file.close();

    return 0;
}
