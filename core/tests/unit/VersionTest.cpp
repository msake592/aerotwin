#include "aerotwin/common/Version.hpp"

#include <gtest/gtest.h>

TEST(VersionTest, ReturnsNonNullPointer) {
    EXPECT_NE(aerotwin::version(), nullptr);
}

TEST(VersionTest, ReturnsNonEmptyString) {
    const char* version = aerotwin::version();
    ASSERT_NE(version, nullptr);
    EXPECT_NE(version[0], '\0');
}

TEST(VersionTest, MatchesDevelopmentVersion) {
    EXPECT_STREQ(aerotwin::version(), "0.1.0-dev");
}
