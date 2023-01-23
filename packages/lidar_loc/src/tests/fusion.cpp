#include <gtest/gtest.h>
#include <Fusion.hpp>

namespace
{
    class FusionTest : public ::testing::Test
    {
    protected:
        void SetUp() override{};
        lidar_loc::Fusion fusion = lidar_loc::Fusion(
            0.9,                     // odometry weight
            0.1,                     // lidar weight
            Eigen::Vector2d::Zero(), // landmark position (x, y)
            {}                       // optional starting pose (set None)
        );
    };

    TEST_F(FusionTest, YawToQuaternionToYawStaysEqual)
    {
        const double yaw_original = 0.123;
        // create quaternion from angle axis yaw
        const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw_original, Eigen::Vector3d::UnitZ()));
        // convert a pose to a twist to obtain yaw
        const Eigen::Vector3d twist_out = fusion.pose2twist(Eigen::Vector3d::Zero(), q_yaw);
        const double yaw_out = twist_out.z();
        // make sure the two yaw angles are equal
        ASSERT_DOUBLE_EQ(yaw_original, yaw_out);
    }
}