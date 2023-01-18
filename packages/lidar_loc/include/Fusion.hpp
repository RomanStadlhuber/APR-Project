#include <eigen3/Eigen/Dense>
#include <optional>
#include <cmath>

#ifndef FUSION_HPP_
#define FUSION_HPP_

class Fusion
{
private:
    /// @brief the weights used when fusing the poses
    const double weight_odom, weight_lidar;
    const Eigen::Vector2d landmark_position;
    /// @brief the internal storage of the pose and reference frames
    Eigen::Vector3d pose, reference_frame;

public:
    Fusion(
        const double &weight_odom,
        const double &weight_lidar,
        const Eigen::Vector2d &landmark_position,
        const std::optional<Eigen::Vector3d> &starting_pose);
    ~Fusion(){};
    /// @brief set the reference pose
    void set_reference(const Eigen::Vector3d &reference_twist);
    /// @brief fuse the 2D twists of lidar and odometry using the supplied weights
    /// @param twist_odom
    /// @param twist_lidar
    /// @return the fused pose
    Eigen::Vector3d fuse_to_pose(
        const Eigen::Vector3d &twist_odom,
        const std::optional<Eigen::Vector2d> &lidar_observation);
    /// @brief convert a ROS-Pose to a twist representation
    /// @param tvec - the translation vector
    /// @param quat - the orientation quaternion
    /// @return the 2D twist representation of that pose
    Eigen::Vector3d pose2twist(const Eigen::Vector3d &tvec, const Eigen::Quaterniond &quat) const;
    /// @brief return an odometry twist in the reference frame (the starting pose)
    Eigen::Vector3d get_reference_twist(const Eigen::Vector3d &twist) const;
};

#endif // FUSIION_HPP_
