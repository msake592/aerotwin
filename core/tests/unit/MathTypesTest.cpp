#include <aerotwin/math/Types.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

static_assert(std::is_same_v<aerotwin::Vector3, Eigen::Vector3d>);
static_assert(std::is_same_v<aerotwin::Quaternion, Eigen::Quaterniond>);

TEST(MathTypesTest, VectorUsesDoubleXYZAndExplicitInitialization) {
    const aerotwin::Vector3 position_world_m(1.0, 2.0, 3.0);
    const aerotwin::Vector3 displacement_world_m(-1.0, 4.0, 0.0);
    const aerotwin::Vector3 moved_position_world_m = position_world_m + displacement_world_m;
    EXPECT_DOUBLE_EQ(moved_position_world_m.x(), 0.0);
    EXPECT_DOUBLE_EQ(moved_position_world_m.y(), 6.0);
    EXPECT_DOUBLE_EQ(moved_position_world_m.z(), 3.0);
    EXPECT_TRUE(aerotwin::isFinite(aerotwin::Vector3::Zero()));
    EXPECT_TRUE(aerotwin::isFinite(position_world_m));
}

TEST(MathTypesTest, RejectsNonFiniteVectorComponents) {
    for (int axis = 0; axis < 3; ++axis) {
        for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
            aerotwin::Vector3 vector = aerotwin::Vector3::Zero();
            vector[axis] = invalid;
            EXPECT_FALSE(aerotwin::isFinite(vector));
        }
    }
}

TEST(MathTypesTest, QuaternionConstructorIsWXYZButStorageIsXYZW) {
    const aerotwin::Quaternion quaternion(1.0, 2.0, 3.0, 4.0);
    EXPECT_DOUBLE_EQ(quaternion.w(), 1.0);
    EXPECT_DOUBLE_EQ(quaternion.x(), 2.0);
    EXPECT_DOUBLE_EQ(quaternion.y(), 3.0);
    EXPECT_DOUBLE_EQ(quaternion.z(), 4.0);
    EXPECT_DOUBLE_EQ(quaternion.coeffs()[0], quaternion.x());
    EXPECT_DOUBLE_EQ(quaternion.coeffs()[3], quaternion.w());
}

TEST(MathTypesTest, AcceptsBothSignsOfUnitQuaternion) {
    EXPECT_TRUE(aerotwin::isUnitQuaternion(aerotwin::Quaternion::Identity()));
    EXPECT_TRUE(aerotwin::isUnitQuaternion(aerotwin::Quaternion(-1.0, 0.0, 0.0, 0.0)));
    EXPECT_TRUE(aerotwin::isUnitQuaternion(aerotwin::Quaternion(0.5, 0.5, 0.5, 0.5)));
}

TEST(MathTypesTest, RejectsInvalidQuaternionWithoutRepair) {
    const aerotwin::Quaternion scaled(2.0, 0.0, 0.0, 0.0);
    EXPECT_FALSE(aerotwin::isUnitQuaternion(scaled));
    EXPECT_DOUBLE_EQ(scaled.w(), 2.0);
    EXPECT_FALSE(aerotwin::isUnitQuaternion(aerotwin::Quaternion(0.0, 0.0, 0.0, 0.0)));
    EXPECT_FALSE(aerotwin::isUnitQuaternion(aerotwin::Quaternion(0.0, 0.0, 0.0, 0.0), 10.0));
    for (int axis = 0; axis < 4; ++axis) {
        for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::max()}) {
            aerotwin::Quaternion quaternion = aerotwin::Quaternion::Identity();
            quaternion.coeffs()[axis] = invalid;
            EXPECT_FALSE(aerotwin::isUnitQuaternion(quaternion));
        }
    }
}

TEST(MathTypesTest, ChecksNormToleranceAndRejectsInvalidTolerance) {
    const aerotwin::Quaternion near_unit(1.0 + 5e-13, 0.0, 0.0, 0.0);
    EXPECT_TRUE(aerotwin::isUnitQuaternion(near_unit));
    EXPECT_FALSE(aerotwin::isUnitQuaternion(near_unit, 0.0));
    EXPECT_FALSE(aerotwin::isUnitQuaternion(aerotwin::Quaternion(1.0 + 2e-12, 0.0, 0.0, 0.0)));
    EXPECT_TRUE(aerotwin::isUnitQuaternion(aerotwin::Quaternion(1.0 - 5e-13, 0.0, 0.0, 0.0)));
    EXPECT_FALSE(aerotwin::isUnitQuaternion(aerotwin::Quaternion(1.0 - 2e-12, 0.0, 0.0, 0.0)));
    EXPECT_TRUE(aerotwin::isUnitQuaternion(aerotwin::Quaternion(1.125, 0.0, 0.0, 0.0), 0.125));
    const auto identity = aerotwin::Quaternion::Identity();
    EXPECT_FALSE(aerotwin::isUnitQuaternion(identity, -1.0));
    EXPECT_FALSE(aerotwin::isUnitQuaternion(identity, std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(aerotwin::isUnitQuaternion(identity, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_TRUE(aerotwin::isUnitQuaternion(identity, 0.0));
}
