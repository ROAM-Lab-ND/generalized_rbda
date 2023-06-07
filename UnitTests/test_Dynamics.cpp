#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <URDF/urdf_parser.h>
#include <URDF/model.h>
#include <Configuration.h>
#include <TestEngine.hpp>

using namespace dynacore::urdf;

TEST(URDF, urdf_openchain){
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    model = parseURDFFile(THIS_COM"Systems/Manipulator/robot_models/Openchain.urdf", true);
    //model = parseURDFFile(THIS_COM"Systems/MC-Vision/robot_models/mini-cheetah-vision.urdf", true);
}

TEST(URDF, urdf_dynamics){
  TestEngine<double>("Systems/Manipulator/robot_models/Openchain.urdf");
}
