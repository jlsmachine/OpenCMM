#include "math/MetrologyMath.h"
#include <cmath>
#include <algorithm>

namespace OpenCMM {

    namespace {
        constexpr double PI_VAL = 3.14159265358979323846;
    }

    WorkplaneFrame WorkplaneFrame::fromStandard(StandardWorkplane wp) {
        WorkplaneFrame frame;
        frame.origin = Eigen::Vector3d::Zero();
        switch (wp) {
        case StandardWorkplane::YZ_PlusX:
            frame.uAxis = Eigen::Vector3d::UnitY();
            frame.vAxis = Eigen::Vector3d::UnitZ();
            frame.normal = Eigen::Vector3d::UnitX();
            break;
        case StandardWorkplane::ZX_PlusY:
            frame.uAxis = Eigen::Vector3d::UnitZ();
            frame.vAxis = Eigen::Vector3d::UnitX();
            frame.normal = Eigen::Vector3d::UnitY();
            break;
        case StandardWorkplane::XY_PlusZ:
        default:
            frame.uAxis = Eigen::Vector3d::UnitX();
            frame.vAxis = Eigen::Vector3d::UnitY();
            frame.normal = Eigen::Vector3d::UnitZ();
            break;
        }
        return frame;
    }

    WorkplaneFrame WorkplaneFrame::fromPlane(const Eigen::Vector3d& planeCentroid, const Eigen::Vector3d& planeNormal) {
        WorkplaneFrame frame;
        frame.origin = planeCentroid;
        frame.normal = planeNormal.normalized();

        // Pick a stable reference axis not parallel to normal to build orthonormal U, V basis
        Eigen::Vector3d refAxis = (std::abs(frame.normal.x()) < 0.9)
            ? Eigen::Vector3d::UnitX()
            : Eigen::Vector3d::UnitY();

        frame.uAxis = (refAxis - refAxis.dot(frame.normal) * frame.normal).normalized();
        frame.vAxis = frame.normal.cross(frame.uAxis).normalized();
        return frame;
    }

    Eigen::Vector2d WorkplaneFrame::projectTo2D(const Eigen::Vector3d& pt3d, double* outHeight) const {
        Eigen::Vector3d rel = pt3d - origin;
        if (outHeight) {
            *outHeight = rel.dot(normal);
        }
        return Eigen::Vector2d(rel.dot(uAxis), rel.dot(vAxis));
    }

    Eigen::Vector3d WorkplaneFrame::to3D(const Eigen::Vector2d& pt2d, double height) const {
        return origin + pt2d.x() * uAxis + pt2d.y() * vAxis + height * normal;
    }

    ProbeCalibrationResult MetrologyMath::calibrateProbeOnMasterBall(
        const std::vector<Eigen::Vector3d>& points,
        double certifiedBallDiameterMm) {

        ProbeCalibrationResult cal;
        cal.certifiedDiameter = certifiedBallDiameterMm;

        if (points.size() < 4 || certifiedBallDiameterMm <= 0.0) {
            return cal;
        }

        SphereFitResult rawFit = fitSphereLeastSquares(points, 0.0, ProbeCompMode::Uncompensated);
        if (!rawFit.success) {
            return cal;
        }

        const double certifiedRadius = certifiedBallDiameterMm * 0.5;
        const double effRadius = rawFit.rawRadius - certifiedRadius;

        if (effRadius <= 0.1) {
            return cal;
        }

        cal.masterBallCenter = rawFit.center;
        cal.rawFitRadius = rawFit.rawRadius;
        cal.effectiveRadius = effRadius;
        cal.formError = rawFit.formError;
        cal.rmsError = rawFit.rmsError;
        cal.success = true;
        return cal;
    }

    SphereFitResult MetrologyMath::fitSphereLeastSquares(
        const std::vector<Eigen::Vector3d>& points,
        double probeRadiusMm,
        ProbeCompMode compMode) {

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
        res.rawRadius = std::sqrt(rSquared);

        if (compMode == ProbeCompMode::External) {
            res.appliedComp = -probeRadiusMm;
        }
        else if (compMode == ProbeCompMode::Internal) {
            res.appliedComp = +probeRadiusMm;
        }
        else {
            res.appliedComp = 0.0;
        }

        res.radius = std::max(0.0, res.rawRadius + res.appliedComp);

        double minR = 1e30, maxR = -1e30, sumSqErr = 0.0;
        for (const auto& pt : points) {
            double dist = (pt - res.center).norm();
            minR = std::min(minR, dist);
            maxR = std::max(maxR, dist);
            double err = dist - res.rawRadius;
            sumSqErr += err * err;
        }

        res.formError = maxR - minR;
        res.rmsError = std::sqrt(sumSqErr / static_cast<double>(n));
        res.success = true;
        return res;
    }

    PlaneFitResult MetrologyMath::fitPlaneLeastSquares(
        const std::vector<Eigen::Vector3d>& points,
        double probeRadiusMm,
        ProbeCompMode compMode,
        const Eigen::Vector3d* clearancePoint) {

        PlaneFitResult res;
        const size_t n = points.size();
        if (n < 3) return res;

        Eigen::Vector3d rawCentroid = Eigen::Vector3d::Zero();
        for (const auto& pt : points) rawCentroid += pt;
        rawCentroid /= static_cast<double>(n);

        Eigen::MatrixXd centered(n, 3);
        for (size_t i = 0; i < n; ++i) {
            centered.row(i) = (points[i] - rawCentroid).transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
        Eigen::Vector3d normal = svd.matrixV().col(2).normalized();

        if (clearancePoint != nullptr) {
            double signedClearanceDist = (*clearancePoint - rawCentroid).dot(normal);
            if (std::abs(signedClearanceDist) > 0.5) {
                if (signedClearanceDist < 0.0) normal = -normal;
            }
            else if (normal.z() < 0.0) {
                normal = -normal;
            }
        }
        else if (normal.z() < 0.0) {
            normal = -normal;
        }

        double minDist = 1e30, maxDist = -1e30, sumSq = 0.0;
        for (const auto& pt : points) {
            double d = (pt - rawCentroid).dot(normal);
            minDist = std::min(minDist, d);
            maxDist = std::max(maxDist, d);
            sumSq += d * d;
        }

        res.rawCentroid = rawCentroid;
        res.normal = normal;

        if (compMode != ProbeCompMode::Uncompensated) {
            res.appliedComp = -probeRadiusMm;
            res.centroid = rawCentroid - probeRadiusMm * normal;
        }
        else {
            res.appliedComp = 0.0;
            res.centroid = rawCentroid;
        }

        res.flatness = maxDist - minDist;
        res.rmsError = std::sqrt(sumSq / static_cast<double>(n));
        res.success = true;
        return res;
    }

    Circle2DFitResult MetrologyMath::fitCircle2DLeastSquares(
        const std::vector<Eigen::Vector3d>& points,
        const WorkplaneFrame& workplane,
        double probeRadiusMm,
        ProbeCompMode compMode) {

        Circle2DFitResult res;
        const size_t n = points.size();
        if (n < 3) return res;

        std::vector<Eigen::Vector2d> pts2d(n);
        double meanHeight = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double h = 0.0;
            pts2d[i] = workplane.projectTo2D(points[i], &h);
            meanHeight += h;
        }
        meanHeight /= static_cast<double>(n);

        // Center coordinates before solving to maximize numerical conditioning
        Eigen::Vector2d mean2d = Eigen::Vector2d::Zero();
        for (const auto& p : pts2d) mean2d += p;
        mean2d /= static_cast<double>(n);

        Eigen::MatrixXd A(n, 3);
        Eigen::VectorXd b(n);
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector2d q = pts2d[i] - mean2d;
            A(i, 0) = 2.0 * q.x();
            A(i, 1) = 2.0 * q.y();
            A(i, 2) = 1.0;
            b(i) = q.squaredNorm();
        }

        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
        Eigen::Vector2d localCenter(x(0), x(1));
        double rSquared = x(2) + localCenter.squaredNorm();
        if (rSquared <= 1e-12) return res;

        Eigen::Vector2d center2d = mean2d + localCenter;
        res.rawRadius = std::sqrt(rSquared);
        res.center = workplane.to3D(center2d, meanHeight);
        res.planeNormal = workplane.normal;

        if (compMode == ProbeCompMode::Internal) {
            res.appliedComp = +probeRadiusMm;
        }
        else if (compMode == ProbeCompMode::External) {
            res.appliedComp = -probeRadiusMm;
        }
        else {
            res.appliedComp = 0.0;
        }

        res.radius = std::max(0.0, res.rawRadius + res.appliedComp);

        double minR = 1e30, maxR = -1e30, sumSqErr = 0.0;
        for (const auto& p : pts2d) {
            double dist = (p - center2d).norm();
            minR = std::min(minR, dist);
            maxR = std::max(maxR, dist);
            double err = dist - res.rawRadius;
            sumSqErr += err * err;
        }

        res.circularity = maxR - minR;
        res.rmsError = std::sqrt(sumSqErr / static_cast<double>(n));
        res.success = true;
        return res;
    }

    Line2DFitResult MetrologyMath::fitLine2DLeastSquares(
        const std::vector<Eigen::Vector3d>& points,
        const WorkplaneFrame& workplane,
        double probeRadiusMm,
        ProbeCompMode compMode,
        const Eigen::Vector3d* clearancePoint) {

        Line2DFitResult res;
        const size_t n = points.size();
        if (n < 2) return res;

        std::vector<Eigen::Vector2d> pts2d(n);
        double meanHeight = 0.0;
        Eigen::Vector2d rawCentroid2d = Eigen::Vector2d::Zero();

        for (size_t i = 0; i < n; ++i) {
            double h = 0.0;
            pts2d[i] = workplane.projectTo2D(points[i], &h);
            rawCentroid2d += pts2d[i];
            meanHeight += h;
        }
        rawCentroid2d /= static_cast<double>(n);
        meanHeight /= static_cast<double>(n);

        Eigen::MatrixXd centered(n, 2);
        for (size_t i = 0; i < n; ++i) {
            centered.row(i) = (pts2d[i] - rawCentroid2d).transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
        if (svd.singularValues()(0) < 1e-9) return res;

        Eigen::Vector2d dir2d = svd.matrixV().col(0).normalized();
        Eigen::Vector2d norm2d = svd.matrixV().col(1).normalized();

        // Orient tangent positively along dominant axis in workplane
        if (std::abs(dir2d.x()) >= std::abs(dir2d.y())) {
            if (dir2d.x() < 0.0) dir2d = -dir2d;
        }
        else {
            if (dir2d.y() < 0.0) dir2d = -dir2d;
        }

        // Orient in-plane edge normal toward clearance point (if backed off > 0.25mm),
        // otherwise default toward +V / +U
        if (clearancePoint != nullptr) {
            Eigen::Vector2d clear2d = workplane.projectTo2D(*clearancePoint);
            double signedClearDist = (clear2d - rawCentroid2d).dot(norm2d);
            if (std::abs(signedClearDist) > 0.25) {
                if (signedClearDist < 0.0) norm2d = -norm2d;
            }
            else if (norm2d.y() < 0.0) {
                norm2d = -norm2d;
            }
        }
        else if (norm2d.y() < 0.0) {
            norm2d = -norm2d;
        }

        double minDist = 1e30, maxDist = -1e30, sumSq = 0.0;
        for (const auto& p : pts2d) {
            double d = (p - rawCentroid2d).dot(norm2d);
            minDist = std::min(minDist, d);
            maxDist = std::max(maxDist, d);
            sumSq += d * d;
        }

        Eigen::Vector2d compCentroid2d = rawCentroid2d;
        if (compMode != ProbeCompMode::Uncompensated) {
            res.appliedComp = -probeRadiusMm;
            compCentroid2d = rawCentroid2d - probeRadiusMm * norm2d;
        }
        else {
            res.appliedComp = 0.0;
        }

        res.rawCentroid = workplane.to3D(rawCentroid2d, meanHeight);
        res.centroid = workplane.to3D(compCentroid2d, meanHeight);
        res.direction = (dir2d.x() * workplane.uAxis + dir2d.y() * workplane.vAxis).normalized();
        res.edgeNormal = (norm2d.x() * workplane.uAxis + norm2d.y() * workplane.vAxis).normalized();
        res.straightness = maxDist - minDist;
        res.rmsError = std::sqrt(sumSq / static_cast<double>(n));
        res.success = true;
        return res;
    }

    FeatureRelationResult MetrologyMath::computeFeatureRelation(
        const MeasuredFeature& featA,
        const MeasuredFeature& featB,
        const WorkplaneFrame& workplane) {

        FeatureRelationResult rel;
        rel.deltaXYZ = featB.position - featA.position;
        rel.distance3D = rel.deltaXYZ.norm();

        Eigen::Vector2d posA2D = workplane.projectTo2D(featA.position);
        Eigen::Vector2d posB2D = workplane.projectTo2D(featB.position);
        rel.distance2D = (posB2D - posA2D).norm();

        // Perpendicular offset along Feature A's normal/edge normal
        if (featA.kind == FeatureKind::Plane3D) {
            rel.normalDistance = std::abs(rel.deltaXYZ.dot(featA.direction.normalized()));
        }
        else if (featA.kind == FeatureKind::Line2D) {
            rel.normalDistance = std::abs(rel.deltaXYZ.dot(featA.edgeNormal.normalized()));
        }
        else if (featB.kind == FeatureKind::Plane3D) {
            rel.normalDistance = std::abs(rel.deltaXYZ.dot(featB.direction.normalized()));
        }
        else {
            rel.normalDistance = std::abs(rel.deltaXYZ.dot(workplane.normal));
        }

        // Angle between directional vectors (0 to 90 degrees acute/orthogonal reference)
        double cosTheta = std::clamp(std::abs(featA.direction.normalized().dot(featB.direction.normalized())), 0.0, 1.0);
        rel.angleDegrees = std::acos(cosTheta) * (180.0 / PI_VAL);

        // If both features are 2D Lines, solve their 2D intersection point in the workplane
        if (featA.kind == FeatureKind::Line2D && featB.kind == FeatureKind::Line2D) {
            Eigen::Vector2d dA(featA.direction.dot(workplane.uAxis), featA.direction.dot(workplane.vAxis));
            Eigen::Vector2d dB(featB.direction.dot(workplane.uAxis), featB.direction.dot(workplane.vAxis));
            double det = dA.x() * (-dB.y()) - dA.y() * (-dB.x());
            if (std::abs(det) > 1e-6) {
                Eigen::Vector2d rhs = posB2D - posA2D;
                double t = (rhs.x() * (-dB.y()) - rhs.y() * (-dB.x())) / det;
                Eigen::Vector2d int2D = posA2D + t * dA;
                double hA = (featA.position - workplane.origin).dot(workplane.normal);
                rel.intersectionPoint = workplane.to3D(int2D, hA);
                rel.hasIntersection = true;
            }
        }

        rel.success = true;
        return rel;
    }

} // namespace OpenCMM