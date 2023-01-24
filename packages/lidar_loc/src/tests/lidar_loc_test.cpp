#define _USE_MATH_DEFINES
#include <gtest/gtest.h>
#include <Fusion.hpp>
#include <CircleDetection.hpp>

namespace
{

    /**
     * @brief the Text fixture class for the `CircleDetection` utility.
     *
     * @details It is used to provide common functionality and brackground information
     * for simulated scan Tests.
     * All tests have the same center point and radius for the circle.
     * The function `point_on_circle(angle)` generates a point that lies on the circle.
     *
     * TODO: include noise in simulated points and check for a tolerance.
     */
    class CircleDetectionTest : public ::testing::Test
    {
    protected:
        void SetUp() override{};
        // return a 2D point that lies on the circle at the origin
        Eigen::Vector2d point_on_circle(const double &angle) const
        {
            return origin + radius * Eigen::Vector2d(std::cos(angle), std::sin(angle));
        }
        const Eigen::Vector2d origin = Eigen::Vector2d(-0.2, 0.5);
        const double radius = 0.125; // [m]
        lidar_loc::CircleDetection circle_detection = lidar_loc::CircleDetection(radius);
    };

    TEST_F(CircleDetectionTest, NoScansNoResult)
    {
        // simulate a scenario where no scans could be obtained
        const lidar_loc::MaybeVector2d result = circle_detection.compute_center({});
        // expect no result
        ASSERT_FALSE(result.has_value());
    }

    TEST_F(CircleDetectionTest, SingleScanPairCenterIsCorrect)
    {
        // the angles on the circle on which the points lie
        const double angle1 = -M_PI / 4.0, angle2 = -2 * M_PI / 4.0; // [rad]
        // create a pair of points in a circle with the common radius
        const Eigen::Vector2d point1 = point_on_circle(angle1), point2 = point_on_circle(angle2);
        // simulate a list of scans, that is a single pair
        const lidar_loc::Scans simulated_scans = {point1, point2};
        // compute the center
        const lidar_loc::MaybeVector2d result = circle_detection.compute_center(simulated_scans);
        // existence check
        ASSERT_TRUE(result.has_value()) << "result is None";
        // check center equality
        ASSERT_DOUBLE_EQ(result->x(), origin.x()) << "result->x() " << result->x() << " != " << origin.x();
        ASSERT_DOUBLE_EQ(result->y(), origin.y()) << "result->y() " << result->y() << " != " << origin.y();
    }

    TEST_F(CircleDetectionTest, EvenNumberOfScansCenterIsCorrect)
    {
        // create an even number of multiple scans
        const size_t SCAN_COUNT = 6;
        // define the angle range
        const double ANGLE_MIN = -M_PI / 4, ANGLE_MAX = -3 * M_PI / 4;
        // create that many scans
        lidar_loc::Scans simulated_scans = {};
        for (int i = 0; i < SCAN_COUNT; i++)
        {
            const double angle = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) / SCAN_COUNT * i;
            auto const scan_point = point_on_circle(angle);
            simulated_scans.push_back(scan_point);
        }
        // check that the amount of scans is correct
        ASSERT_EQ(simulated_scans.size(), SCAN_COUNT);
        // compute a result
        const lidar_loc::MaybeVector2d result = circle_detection.compute_center(simulated_scans);
        // check existence of result
        ASSERT_TRUE(result.has_value());
        // check correctness of result
        ASSERT_DOUBLE_EQ(result->x(), origin.x()) << "result->x() " << result->x() << " != " << origin.x();
        ASSERT_DOUBLE_EQ(result->y(), origin.y()) << "result->y() " << result->y() << " != " << origin.y();
    }

    TEST_F(CircleDetectionTest, OddNumberOfScansCenterIsCorrect)
    {
        // the same test as above, but with an odd number of scan points...

        // create an even number of multiple scans
        const size_t SCAN_COUNT = 9;
        // define the angle range
        const double ANGLE_MIN = -M_PI / 4, ANGLE_MAX = -3 * M_PI / 4;
        // create that many scans
        lidar_loc::Scans simulated_scans = {};
        for (int i = 0; i < SCAN_COUNT; i++)
        {
            const double angle = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) / SCAN_COUNT * i;
            auto const scan_point = point_on_circle(angle);
            simulated_scans.push_back(scan_point);
        }
        // check that the amount of scans is correct
        ASSERT_EQ(simulated_scans.size(), SCAN_COUNT);
        // compute a result
        const lidar_loc::MaybeVector2d result = circle_detection.compute_center(simulated_scans);
        // check existence of result
        ASSERT_TRUE(result.has_value());
        // check correctness of result
        ASSERT_DOUBLE_EQ(result->x(), origin.x()) << "result->x() " << result->x() << " != " << origin.x();
        ASSERT_DOUBLE_EQ(result->y(), origin.y()) << "result->y() " << result->y() << " != " << origin.y();
    }

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
        const double THRESHOLD = 0.05; // accepted floating point rounding error
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
        ASSERT_NEAR(yaw_original, yaw_out, THRESHOLD);
    }

    TEST_F(FusionTest, YawToQuaternionToYawStaysEqualWhenNegative)
    {
        const double yaw_original = -0.123;
        // create quaternion from angle axis yaw
        const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw_original, Eigen::Vector3d::UnitZ()));
        // convert a pose to a twist to obtain yaw
        const Eigen::Vector3d twist_out = fusion.pose2twist(Eigen::Vector3d::Zero(), q_yaw);
        const double yaw_out = twist_out.z();
        // make sure the two yaw angles are equal
        ASSERT_NEAR(yaw_original, yaw_out, THRESHOLD);
    }

    TEST_F(FusionTest, YawToQuaternionToYawFlipsWhenLargerThanPiRadians)
    {
        const double yaw_original = M_PI + 0.123;
        // now the expected should be a negative angle [0, -pi] because we are leaivng positive y
        const double yaw_expected = yaw_original - 2 * M_PI;
        // create quaternion from angle axis yaw
        const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw_original, Eigen::Vector3d::UnitZ()));
        // convert a pose to a twist to obtain yaw
        const Eigen::Vector3d twist_out = fusion.pose2twist(Eigen::Vector3d::Zero(), q_yaw);
        const double yaw_out = twist_out.z();
        // make sure the two yaw angles are equal
        ASSERT_NEAR(yaw_expected, yaw_out, THRESHOLD);
    }

}