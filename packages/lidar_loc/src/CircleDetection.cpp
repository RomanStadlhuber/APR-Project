/**
 * Author: Roman Stadlhuber
 */

#include <CircleDetection.hpp>

namespace lidar_loc
{
    CircleDetection::CircleDetection(const double &radius)
        : radius(radius)
    {
    }

    std::vector<ScanPair> CircleDetection::select_random_pairs(const Scans &scans) const
    {
        auto selected_pairs = std::vector<ScanPair>();

        if (scans.size() > 0)
        {
            const int pair_count = scans.size() / 2;
            for (int i = 0; i < pair_count; i++)
            {
                selected_pairs.push_back(ScanPair(
                    scans.at(i), scans.at(scans.size() - (i + 1))));
            }
        }

        return selected_pairs;
    }

    MaybeVector2d CircleDetection::compute_center_from_pair(const ScanPair &scan_pair) const
    {
        // Remark: the vector is drawn from the first to the second point
        auto const diff = scan_pair.second - scan_pair.first;
        const double discriminant = 4 * std::pow(radius, 2) - std::pow(diff.norm(), 2);
        if (discriminant <= 0.0) // fail the computation if the sqrt result would be imaginary
            return {};
        const double perp_height = 0.5 * std::sqrt(discriminant);
        // the anchor point for placing the perpendicular vector
        const Eigen::Vector2d perp_anchor(scan_pair.first + 0.5 * diff);
        // perpendicular vector candidates A and B, find out which is closer
        auto const perp_a = Eigen::Vector2d(-diff.y(), diff.x()).normalized() * perp_height,
                   perp_b = Eigen::Vector2d(diff.y(), -diff.x()).normalized() * perp_height;
        // the distances of the perpendicular vectors to the robot frame origin
        const Eigen::Vector2d candidate_a = perp_anchor + perp_a,
                              candidate_b = perp_anchor + perp_b;
        // return the center candidate that is farther away as only this can be the real center
        if (candidate_a.norm() > candidate_b.norm())
            return candidate_a;
        else
            return candidate_b;
    }

    MaybeVector2d CircleDetection::compute_center(const Scans &scans) const
    {
        auto const scan_pairs = select_random_pairs(scans);

        if (scan_pairs.size() == 0) // return None if no scans are provided
            return {};

        auto center_candidates = std::vector<Eigen::Vector2d>();
        // obtain all center candidates
        for (auto const scan_pair : scan_pairs)
        {
            auto const pair_center_candidate = compute_center_from_pair(scan_pair);
            if (pair_center_candidate.has_value())
                center_candidates.push_back(*pair_center_candidate);
        }

        const Eigen::Vector2d averaged_center = std::accumulate(
                                                    center_candidates.begin(),
                                                    center_candidates.end(),
                                                    Eigen::Vector2d(0, 0),
                                                    [](Eigen::Vector2d acc, Eigen::Vector2d candidate)
                                                    {
                                                        return acc + candidate;
                                                    }) /
                                                (double)center_candidates.size();

        return averaged_center;
    }
}