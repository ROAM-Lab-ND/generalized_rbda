#include "gtest/gtest.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "Utils/SpatialInertia.h"
#include "Utils/SpatialTransforms.h"

namespace internal_helpers
{

    using namespace grbda;
    using namespace grbda::spatial;

    Mat3<double> randomRotationMatrix()
    {
        const Vec3<double> rpy = Vec3<double>::Random();
        Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[0], Vec3<double>::UnitZ()) *
                               Eigen::AngleAxisd(rpy[1], Vec3<double>::UnitY()) *
                               Eigen::AngleAxisd(rpy[2], Vec3<double>::UnitZ());
        return q.matrix();
    }

    Mat6<double> randomSpatialInertia()
    {
        Mat4<double> P = 0.5 * Mat4<double>::Random();
        P = 0.5 * (P + P.transpose());
        P = P.exp();
        SpatialInertia<double> I = SpatialInertia(P);
        return I.getMatrix();
    }

    DVec<int> toBase(int number, int base, int digits)
    {
        DVec<int> out = DVec<int>::Zero(digits);
        for (int i = 0; i < digits; i++)
            out[i] = (number % (int)pow(base, i + 1)) / pow(base, i);
        return out;
    }

}

using namespace internal_helpers;

GTEST_TEST(Spatial, SpatialMotionTransform)
{
    // Verify that transforming a spatial motion vector using a 6x6 Eigen matrix and a
    // SpatialTransform object gives the same result

    const Mat3<double> E = randomRotationMatrix();
    const Vec3<double> r = Vec3<double>::Random();

    Mat6<double> X_eigen_matrix = createSXform(E, r);
    SpatialTransform X_spatial_transform = SpatialTransform(E, r);

    SVec<double> v = SVec<double>::Random();

    SVec<double> v_eigen_matrix = X_eigen_matrix * v;
    SVec<double> v_spatial_transform = X_spatial_transform.transformMotionVector(v);

    GTEST_ASSERT_LT((v_eigen_matrix - v_spatial_transform).norm(), 1e-8);

    SVec<double> v_original = X_spatial_transform.inverseTransformMotionVector(v_spatial_transform);
    GTEST_ASSERT_LT((v - v_original).norm(), 1e-8);
}

GTEST_TEST(Spatial, SpatialInertiaTransform)
{
    // Verify that transforming a spatial inertia vector using a 6x6 Eigen matrix and a
    // SpatialTransform object gives the same result

    const Mat3<double> E = randomRotationMatrix();
    const Vec3<double> r = Vec3<double>::Random();

    Mat6<double> X_eigen_matrix = createSXform(E, r);
    SpatialTransform X_spatial_transform = SpatialTransform(E, r);

    Mat6<double> I = randomSpatialInertia();

    Mat6<double> I_eigen_matrix = X_eigen_matrix.transpose() * I * X_eigen_matrix;
    Mat6<double> I_spatial_transform = X_spatial_transform.inverseTransformSpatialInertia(I);

    GTEST_ASSERT_LT((I_eigen_matrix - I_spatial_transform).norm(), 1e-8);
}

GTEST_TEST(Spatial, SpatialTransformMultiplication)
{
    // Verify that multiplying two spatial transforms gives the same result as multiplying two
    // 6x6 Eigen matrices

    const Mat3<double> E1 = randomRotationMatrix();
    const Vec3<double> r1 = Vec3<double>::Random();

    const Mat3<double> E2 = randomRotationMatrix();
    const Vec3<double> r2 = Vec3<double>::Random();

    Mat6<double> X1_eigen_matrix = createSXform(E1, r1);
    Mat6<double> X2_eigen_matrix = createSXform(E2, r2);

    SpatialTransform X1_spatial_transform = SpatialTransform(E1, r1);
    SpatialTransform X2_spatial_transform = SpatialTransform(E2, r2);

    Mat6<double> X3_eigen_matrix = X1_eigen_matrix * X2_eigen_matrix;
    SpatialTransform X3_spatial_transform = X1_spatial_transform * X2_spatial_transform;

    GTEST_ASSERT_LT((rotationFromSXform(X3_eigen_matrix) - X3_spatial_transform.getRotation()).norm(), 1e-8);
    GTEST_ASSERT_LT((translationFromSXform(X3_eigen_matrix) - X3_spatial_transform.getTranslation()).norm(), 1e-8);
    GTEST_ASSERT_LT((X3_eigen_matrix - X3_spatial_transform.toMatrix()).norm(), 1e-8);
}

GTEST_TEST(Spatial, GeneralizedSpatialTransforms)
{
    // Verify that transforming a spatial motion vector, spatial force vector, and spatial inertia
    // using a 6Nx6N Eigen matrix and a GeneralizedSpatialTransform object gives the same result

    const int max_num_output_bodies = 3;
    const int max_num_parent_bodies = 3;
    std::vector<std::pair<int, int>> num_output_and_parent_bodies;
    for (int i = 1; i <= max_num_output_bodies; i++)
        for (int j = 1; j <= max_num_parent_bodies; j++)
            num_output_and_parent_bodies.push_back(std::make_pair(i, j));

    for (const auto &pair : num_output_and_parent_bodies)
    {
        const int num_output_bodies = pair.first;
        const int num_parent_bodies = pair.second;

        for (int i = 0; i < pow(num_parent_bodies, num_output_bodies); i++)
        {
            DVec<int> parent_subindices = toBase(i, num_parent_bodies, num_output_bodies);

            DMat<double> X_eigen_matrix = DMat<double>::Zero(6 * num_output_bodies, 6 * num_parent_bodies);
            GeneralizedSpatialTransform X_generalized = GeneralizedSpatialTransform(num_parent_bodies);

            for (int j = 0; j < num_output_bodies; j++)
            {
                const Mat3<double> E = randomRotationMatrix();
                const Vec3<double> r = Vec3<double>::Random();

                X_eigen_matrix.block<6, 6>(6 * j, 6 * parent_subindices[j]) = createSXform(E, r);

                SpatialTransform X_spatial_transform = SpatialTransform(E, r);
                X_generalized.appendSpatialTransformWithClusterAncestorSubIndex(
                    X_spatial_transform, parent_subindices[j]);
            }

            // Conversion to matrix
            GTEST_ASSERT_LT((X_eigen_matrix - X_generalized.toMatrix()).norm(), 1e-8);

            // Motion transform
            DVec<double> v = DVec<double>::Random(6 * num_parent_bodies);
            DVec<double> v_eigen_matrix = X_eigen_matrix * v;
            DVec<double> v_generalized = X_generalized.transformMotionVector(v);

            GTEST_ASSERT_EQ(v_generalized.rows(), 6 * num_output_bodies);
            GTEST_ASSERT_LT((v_eigen_matrix - v_generalized).norm(), 1e-8);

            // Inverse force transform
            DVec<double> f = DVec<double>::Random(6 * num_output_bodies);
            DVec<double> f_eigen_matrix = X_eigen_matrix.transpose() * f;
            DVec<double> f_generalized = X_generalized.inverseTransformForceVector(f);

            GTEST_ASSERT_EQ(f_generalized.rows(), 6 * num_parent_bodies);
            GTEST_ASSERT_LT((f_eigen_matrix - f_generalized).norm(), 1e-8);

            // Inertia transforms
            DMat<double> I = DMat<double>::Zero(6 * num_output_bodies, 6 * num_output_bodies);
            for (int j = 0; j < num_output_bodies; j++)
            {
                I.block<6, 6>(6 * j, 6 * j) = randomSpatialInertia();
            }
            DMat<double> I_eigen_matrix = X_eigen_matrix.transpose() * I * X_eigen_matrix;
            DMat<double> I_generalized = X_generalized.inverseTransformSpatialInertia(I);

            GTEST_ASSERT_EQ(I_generalized.rows(), 6 * num_parent_bodies);
            GTEST_ASSERT_EQ(I_generalized.cols(), 6 * num_parent_bodies);
            GTEST_ASSERT_LT((I_eigen_matrix - I_generalized).norm(), 1e-8);
        }
    }
}

GTEST_TEST(Spatial, GeneralizedSpatialTransformMultiplication)
{
    // Verify that transforming a spatial motion vector using a 6Nx6N Eigen matrix and a
    // GeneralizedSpatialTransform object gives the same result
    //
    // Matrix multiplication X3 = X1 * X2 where
    // X1 is 6*num_outputs x 6*num_intermediates
    // X2 is 6*num_intermediates x 6*num_parents
    // X3 is 6*num_outputs x 6*num_parents

    const int max_num_output_bodies = 3;
    const int max_num_intermediate_bodies = 3;
    const int max_num_parent_bodies = 3;
    std::vector<std::tuple<int, int, int>> num_outputs_intermediates_and_parents;
    for (int i = 1; i <= max_num_output_bodies; i++)
        for (int j = 1; j <= max_num_intermediate_bodies; j++)
            for (int k = 1; k <= max_num_parent_bodies; k++)
                num_outputs_intermediates_and_parents.push_back(std::make_tuple(i, j, k));

    for (const auto &tuple : num_outputs_intermediates_and_parents)
    {
        const int num_output_bodies = std::get<0>(tuple);
        const int num_intermediate_bodies = std::get<1>(tuple);
        const int num_parent_bodies = std::get<2>(tuple);

        for (int i = 0; i < pow(num_intermediate_bodies, num_output_bodies); i++)
        {
            DVec<int> intermediate_subindices =
                toBase(i, num_intermediate_bodies, num_output_bodies);

            DMat<double> X1_eigen_matrix =
                DMat<double>::Zero(6 * num_output_bodies, 6 * num_intermediate_bodies);

            GeneralizedSpatialTransform X1_generalized =
                GeneralizedSpatialTransform(num_intermediate_bodies);

            for (int j = 0; j < num_output_bodies; j++)
            {
                const Mat3<double> E = randomRotationMatrix();
                const Vec3<double> r = Vec3<double>::Random();

                X1_eigen_matrix.block<6, 6>(6 * j, 6 * intermediate_subindices[j]) =
                    createSXform(E, r);

                SpatialTransform X_spatial_transform = SpatialTransform(E, r);
                X1_generalized.appendSpatialTransformWithClusterAncestorSubIndex(
                    X_spatial_transform, intermediate_subindices[j]);
            }

            for (int k = 0; k < pow(num_parent_bodies, num_intermediate_bodies); k++)
            {
                DVec<int> parent_subindices =
                    toBase(k, num_parent_bodies, num_intermediate_bodies);

                DMat<double> X2_eigen_matrix =
                    DMat<double>::Zero(6 * num_intermediate_bodies, 6 * num_parent_bodies);

                GeneralizedSpatialTransform X2_generalized =
                    GeneralizedSpatialTransform(num_parent_bodies);

                for (int j = 0; j < num_intermediate_bodies; j++)
                {
                    const Mat3<double> E = randomRotationMatrix();
                    const Vec3<double> r = Vec3<double>::Random();

                    X2_eigen_matrix.block<6, 6>(6 * j, 6 * parent_subindices[j]) =
                        createSXform(E, r);

                    SpatialTransform X_spatial_transform = SpatialTransform(E, r);
                    X2_generalized.appendSpatialTransformWithClusterAncestorSubIndex(
                        X_spatial_transform, parent_subindices[j]);
                }

                DMat<double> X3_eigen_matrix = X1_eigen_matrix * X2_eigen_matrix;
                GeneralizedSpatialTransform X3_generalized = X1_generalized * X2_generalized;
                GTEST_ASSERT_LT((X3_eigen_matrix - X3_generalized.toMatrix()).norm(), 1e-8);
            }
        }
    }
}
