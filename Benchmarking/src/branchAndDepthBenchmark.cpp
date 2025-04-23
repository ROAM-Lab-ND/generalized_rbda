#include "branchAndDepthHelpers.hpp"

TEST(BranchAndDepthBenchmark, NumericalValidation)
{
    ConstraintModelVector<double> test_constraints;

    std::string urdf_filename = main_urdf_directory + "four_bar.urdf";

    // Build models
    PinModel<double> model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // PinocchioADModel ad_model = model.cast<ADScalar>();
    // PinocchioADModel::Data ad_data(ad_model);
    // PinocchioADModel::Data ad_data_cd(ad_model);

    parseURDFFileForLoopConstraints(urdf_filename, model);
    printf("Branch and Depth Benchmark\n");
}
