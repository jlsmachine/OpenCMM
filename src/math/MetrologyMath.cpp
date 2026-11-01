#include "math/MetrologyMath.h"
#include <cmath>
#include <algorithm>

namespace OpenCMM {

    SphereFitResult MetrologyMath::fitSphereLeastSquares(const std::vector<Eigen::Vector3d>& points) {
        SphereFitResult res;
        const size_t n = points.size();
        if (n < 4) return res;

        Eigen::MatrixXd A(n, 4);
        Eigen::VectorXd b(n);

        for (size_t i = 0; i < n; ++i) {
            A(i, 0) = 2.0 * points[i].x();
            A(i, 1) = 2.0 * points[i].y();
            A(i, 2) = 2.0 * points[i].z();
            A(i, 3) = 1.0;
            b(i) = points[i].squaredNorm();
        }

        Eigen::Vector4d x = A.colPivHouseholderQr().solve(b);
        Eigen::Vector3d center(x(0), x(1), x(2));
        double rSquared = x(3) + center.squaredNorm();
        if (rSquared <= 0.0) return res;

        res.center = center;
        res.radius = std::sqrt(rSquared);

        double minR = 1e30, maxR = -1e30, sumSqErr = 0.0;
        for (const auto& pt : points) {
            double dist = (pt - res.center).norm();
            minR = std::min(minR, dist);
            maxR = std::max(maxR, dist);
            double err = dist - res.radius;
            sumSqErr += err * err;
        }

        res.formError = maxR - minR;
        res.rmsError = std::sqrt(sumSqErr / static_cast<double>(n));
        res.success = true;
        return res;
    }

    PlaneFitResult MetrologyMath::fitPlaneLeastSquares(const std::vector<Eigen::Vector3d>& points) {
        PlaneFitResult res;
        const size_t n = points.size();
        if (n < 3) return res;

        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& pt : points) centroid += pt;
        centroid /= static_cast<double>(n);

        Eigen::MatrixXd centered(n, 3);
        for (size_t i = 0; i < n; ++i) {
            centered.row(i) = (points[i] - centroid).transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
        Eigen::Vector3d normal = svd.matrixV().col(2).normalized();
        if (normal.z() < 0.0) normal = -normal;

        double minDist = 1e30, maxDist = -1e30, sumSq = 0.0;
        for (const auto& pt : points) {
            double d = (pt - centroid).dot(normal);
            minDist = std::min(minDist, d);
            maxDist = std::max(maxDist, d);
            sumSq += d * d;
        }

        res.centroid = centroid;
        res.normal = normal;
        res.flatness = maxDist - minDist;
        res.rmsError = std::sqrt(sumSq / static_cast<double>(n));
        res.success = true;
        return res;
    }

} // namespace OpenCMM