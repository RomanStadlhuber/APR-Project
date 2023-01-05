/**
 * Author: Roman Stadlhuber
 */

#ifndef CIRCLE_DETECTION_HPP_
#define CIRCLE_DETECTION_HPP_

#include <eigen3/Eigen/Dense>
#include <vector>
#include <array>
#include <optional>
#include <utility>
#include <numeric>
#include <cmath>
#include <random>

namespace lidar_loc
{
    typedef std::optional<Eigen::Vector2d> MaybeVector2d;
    typedef std::pair<Eigen::Vector2d, Eigen::Vector2d> ScanPair;
    typedef std::vector<Eigen::Vector2d> Scans;

    class CircleDetection
    {
    private:
        /// the constant radius of the circle to be detected
        const double radius;
        /// get a random selection of (noisy) laser scan pairs that lie on the circle
        std::vector<ScanPair> select_random_pairs(const Scans &scans) const;
        /// compute the _presumed_ center for a single scan pair
        Eigen::Vector2d compute_center_from_pair(const ScanPair &scan_pair) const;

    protected:
    public:
        CircleDetection(const double &radius);
        ~CircleDetection(){};
        // CircleDetection &operator=(const CircleDetection &other);

        /// @brief  compute the presumed center of the detected circle in the local frame
        /// @param scans the 2D vectors of the laserscans in an (x,y) shape
        /// @return an `optional` container of a 2D vector (x,y) _if_ a center can be computed
        MaybeVector2d compute_center(const Scans &scans) const;
    };
}

#endif