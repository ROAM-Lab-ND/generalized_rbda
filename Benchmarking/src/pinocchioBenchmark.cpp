#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/rnea.hpp"

int main()
{
    const std::string urdf_filename = "../robot-models/four_bar.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    Eigen::VectorXd q = pinocchio::neutral(model);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    const Eigen::VectorXd &tau = pinocchio::rnea(model, data, q, v, a);
    std::cout << "tau = " << tau.transpose() << std::endl;

    return 0;
}
