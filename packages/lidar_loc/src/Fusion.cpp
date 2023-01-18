#include <Fusion.hpp>

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
    const Eigen::Vector3d &twist_odom, const std::optional<Eigen::Vector2d> &lidar_obersvation)
{
    // at first, convert the odometry twist to the reference frame
    const Eigen::Vector3d twist_est_odom = get_reference_twist(twist_odom);

    if (!lidar_obersvation.has_value())
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
    const Eigen::Vector2d pos_est_odom(twist_est_odom.x(), twist_est_odom.y());
    const double heading_est_odom = twist_est_odom.z();
    const Eigen::Rotation2D R_last(last_heading);
    // the pose estimate of the lidar data
    const Eigen::Vector2d pos_est_lidar =
        this->landmark_position - R_last.inverse().toRotationMatrix() * (*lidar_obersvation);
    // fuse the position estimates
    const Eigen::Vector2d fused_pos_est =
        this->weight_odom * pos_est_odom + this->weight_lidar * pos_est_lidar;

    /**
     * -- then estimate the landmark angle using the fused position --
     * -- fuse the estimated and observed landmark angle --
     * compute the distance between landmark and fused position
     * transfer that distance vector to the robot frame, rotating by the odom heading
     * compute the observed landmark angle
     * fuse the relative angle using the same weights
     */
    const Eigen::Rotation2D R_odom(heading_est_odom);
    // model the observed lidar delta vector based on the odometry estimate
    const Eigen::Vector2d lidar_est_odom = R_odom * (this->landmark_position - pos_est_odom);
    // an estimated observation angle based on odometry data
    const double beta_est_odom = std::atan2(lidar_est_odom.y(), lidar_est_odom.x());
    // the real observation angle from lidar data
    const double beta_est_lidar = std::atan2(lidar_obersvation->y(), lidar_obersvation->x());
    // fuse the relative angle to the landmark
    const double beta_fused =
        this->weight_odom * beta_est_odom + this->weight_lidar * beta_est_lidar;
    // the fused relative position to the landmark
    const Eigen::Vector2d fused_pos_relative = this->landmark_position - fused_pos_est;
    // obtain the heading from the fused angle relations

    /**
     * compute the fused heading as the difference:
     * global frame angle estimate using fused position - robot frame landmark angle
     */
    const double fused_heading = std::atan2(fused_pos_relative.y(), fused_pos_relative.x()) - beta_fused;
    // combine fusion data to a twist in reference frame
    const Eigen::Vector3d fused_twist(fused_pos_est.x(), fused_pos_est.y(), fused_heading);
    this->pose = fused_twist;
    return this->pose;
}

Eigen::Vector3d Fusion::pose2twist(const Eigen::Vector3d &tvec, const Eigen::Quaterniond &quat) const
{
    const double yaw = quat.toRotationMatrix().eulerAngles(0, 1, 2).z();
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