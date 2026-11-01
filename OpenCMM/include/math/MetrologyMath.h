#pragma once
#include <Eigen/Dense>
#include <vector>

namespace OpenCMM {

    struct SphereFitResult {
        bool success = false;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        double radius = 0.0;
        double formError = 0.0;
        double rmsError = 0.0;
    };

    struct PlaneFitResult {
        bool success = false;
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        double flatness = 0.0;
        double rmsError = 0.0;
    };

    class MetrologyMath {
    public:
        static SphereFitResult fitSphereLeastSquares(const std::vector<Eigen::Vector3d>& points);
        static PlaneFitResult fitPlaneLeastSquares(const std::vector<Eigen::Vector3d>& points);
    };

} // namespace OpenCMM