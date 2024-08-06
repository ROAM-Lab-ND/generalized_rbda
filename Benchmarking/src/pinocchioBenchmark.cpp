#include "grbda/Dynamics/ClusterTreeModel.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/rnea.hpp"

// TODO(@MatthewChignoli): Maybe we make a helper function for compatibility between ModelState and pinocchio state

int main()
{
    const std::string urdf_filename = "../robot-models/double_pendulum.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename, false);

    for (int i = 0; i < 10; i++)
    {

        // Eigen::VectorXd q = pinocchio::neutral(model);
        Eigen::VectorXd q = pinocchio::randomConfiguration(model);
        Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
        Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
        grbda::ClusterTreeModel<double>::StatePair state = {q, v};

        const Eigen::VectorXd &tau = pinocchio::rnea(model, data, q, v, a);
        std::cout << "tau = " << tau.transpose() << std::endl;

        cluster_tree.setState(state);
        Eigen::VectorXd tau_grbda = cluster_tree.inverseDynamics(a);
        std::cout << "tau_grbda = " << tau_grbda.transpose() << std::endl;

        // Check if the results are the same
        if ((tau - tau_grbda).norm() > 1e-6)
        {
            std::cerr << "Error: The results of the inverse dynamics are not the same" << std::endl;
            return 1;
        }
    }

    return 0;
}

// TODO(@MatthewChignoli): Now we should try to create casadi functions for the dynamics and compare the results
