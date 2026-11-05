#include <Eigen/Core>

#include <gtest/gtest.h>

TEST(EigenTest, ConstructsVectorAndAccessesComponents) {
    const Eigen::Vector3d vector(0.25, -1.5, 2.75);

    EXPECT_DOUBLE_EQ(vector.x(), 0.25);
    EXPECT_DOUBLE_EQ(vector.y(), -1.5);
    EXPECT_DOUBLE_EQ(vector.z(), 2.75);
    EXPECT_DOUBLE_EQ(vector[0], vector.x());
    EXPECT_DOUBLE_EQ(vector[1], vector.y());
    EXPECT_DOUBLE_EQ(vector[2], vector.z());
}

TEST(EigenTest, AddsVectors) {
    const Eigen::Vector3d left(0.1, -2.0, 3.0);
    const Eigen::Vector3d right(0.2, 4.0, -1.0);
    const Eigen::Vector3d result = left + right;

    EXPECT_NEAR(result.x(), 0.3, 1e-12);
    EXPECT_NEAR(result.y(), 2.0, 1e-12);
    EXPECT_NEAR(result.z(), 2.0, 1e-12);
}

TEST(EigenTest, ComputesDotProduct) {
    const Eigen::Vector3d left(1.0, -2.0, 3.0);
    const Eigen::Vector3d right(4.0, 5.0, -6.0);

    EXPECT_NEAR(left.dot(right), -24.0, 1e-12);
}

TEST(EigenTest, MultipliesMatrixByVector) {
    Eigen::Matrix3d matrix;
    matrix << 1.0, 2.0, 3.0,
              4.0, 5.0, 6.0,
              7.0, 8.0, 10.0;
    const Eigen::Vector3d vector(2.0, -1.0, 0.5);
    const Eigen::Vector3d result = matrix * vector;

    EXPECT_NEAR(result.x(), 1.5, 1e-12);
    EXPECT_NEAR(result.y(), 6.0, 1e-12);
    EXPECT_NEAR(result.z(), 11.0, 1e-12);
}

TEST(EigenTest, IdentityMatrixPreservesVector) {
    const Eigen::Vector3d vector(0.1, -2.5, 3.75);
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d result = identity * vector;

    EXPECT_NEAR(result.x(), vector.x(), 1e-12);
    EXPECT_NEAR(result.y(), vector.y(), 1e-12);
    EXPECT_NEAR(result.z(), vector.z(), 1e-12);
}
