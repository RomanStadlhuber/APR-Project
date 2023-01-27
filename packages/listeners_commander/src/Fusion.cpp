#include <Fusion.hpp>

namespace lidar_loc
{
    Fusion::Fusion(
        const double &weight_odom,
        const double &weight_lidar,
        const Eigen::Vector2d &landmark_position,
        const std::optional<Eigen::Vector3d> &starting_pose)
        : weight_odom(weight_odom / (weight_odom + weight_lidar)),
          weight_lidar(weight_lidar / (weight_odom + weight_lidar)),
          landmark_position(landmark_position)
    {
        if (starting_pose.has_value())
        {
            this->set_reference(*starting_pose);
        }
    }

    Eigen::Vector3d Fusion::fuse_to_pose(
        const Eigen::Vector3d &twist_odom, const std::optional<Eigen::Vector2d> &lidar_observation)
    {
        // at first, convert the odometry twist to the reference frame
        const Eigen::Vector3d twist_est_odom = get_reference_twist(twist_odom);

        if (!lidar_observation.has_value())
        {
            this->pose = twist_est_odom;
            return this->pose;
        }
        /**
         * position estimate from lidar = landmark position - scan delta in global position
         * fused position = odom weight * odom estimate + lidar weight * lidar estimate
         */
        // the last fused heading value
        const double last_heading = this->pose.z();
        // the position estimate from odometry
        const Eigen::Vector2d pos_est_odom(twist_est_odom.x(), twist_est_odom.y());
        // the heading from odometry
        const double heading_est_odom = twist_est_odom.z();
        // the assumed frame orientation at the fused position
        const Eigen::Rotation2D R_last(last_heading);
        // the relative landmark position in the world frame
        const Eigen::Vector2d delta_robot_landmark = R_last.toRotationMatrix() * (*lidar_observation);
        // the pose estimate of the lidar data
        const Eigen::Vector2d pos_est_lidar = this->landmark_position - delta_robot_landmark;
        // the fused position
        const Eigen::Vector2d fused_pos_est =
            this->weight_odom * pos_est_odom + this->weight_lidar * pos_est_lidar;

        const double heading_lidar =
            std::atan2(delta_robot_landmark.y(), delta_robot_landmark.x()) - // the relative angle in the world frame
            std::atan2(lidar_observation->y(), lidar_observation->x());      // the relative angle in the robot frame
        const double heading_odom = twist_odom.z();
        const double fused_heading = this->weight_odom * heading_odom + this->weight_lidar * heading_lidar;
        // combine fusion data to a twist in reference frame
        const Eigen::Vector3d fused_twist(fused_pos_est.x(), fused_pos_est.y(), fused_heading);
        this->pose = fused_twist;
        return this->pose;
    }

    Eigen::Vector3d Fusion::pose2twist(const Eigen::Vector3d &tvec, const Eigen::Quaterniond &quat) const
    {
        // rotate the unit x vector and obtain the new yaw angle
        const Eigen::Vector3d x_rot = quat._transformVector(Eigen::Vector3d::UnitX());
        const double yaw = std::atan2(x_rot.y(), x_rot.x());

        // const double yaw = quat.toRotationMatrix().eulerAngles(0, 1, 2).z();
        // const double cos_half_angle = Eigen::Quaterniond::Identity().dot(quat);
        // const double yaw = 2 * std::acos(cos_half_angle);

        return Eigen::Vector3d(tvec.x(), tvec.y(), yaw);
    }

    Eigen::Vector3d Fusion::get_reference_twist(const Eigen::Vector3d &twist) const
    {
        Eigen::Vector3d relative_twist = twist - this->reference_frame;
        // normalize the angle
        const Eigen::Rotation2D R(relative_twist.z());
        relative_twist.z() = R.angle();
        return relative_twist;
    }

    void Fusion::set_reference(const Eigen::Vector3d &reference_twist)
    {
        this->reference_frame = reference_twist;
    }
}