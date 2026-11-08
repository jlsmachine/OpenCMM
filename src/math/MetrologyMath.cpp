// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#include "math/MetrologyMath.h"

#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
// Add these includes at the top of src/math/MetrologyMath.cpp:
#include <BRepGProp_Face.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <gp_Vec.hxx>

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

namespace OpenCMM {

    namespace {
        constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
    }

    WorkplaneFrame WorkplaneFrame::fromStandard(StandardWorkplane wp) {
        WorkplaneFrame frame;
        frame.origin = Eigen::Vector3d::Zero();
        switch (wp) {
        case StandardWorkplane::YZ_PlusX:
            frame.uAxis = Eigen::Vector3d::UnitY();
            frame.vAxis = Eigen::Vector3d::UnitZ();
            frame.wAxis = Eigen::Vector3d::UnitX();
            break;
        case StandardWorkplane::ZX_PlusY:
            frame.uAxis = Eigen::Vector3d::UnitZ();
            frame.vAxis = Eigen::Vector3d::UnitX();
            frame.wAxis = Eigen::Vector3d::UnitY();
            break;
        case StandardWorkplane::XY_PlusZ:
        case StandardWorkplane::CustomPlane:
        default:
            frame.uAxis = Eigen::Vector3d::UnitX();
            frame.vAxis = Eigen::Vector3d::UnitY();
            frame.wAxis = Eigen::Vector3d::UnitZ();
            break;
        }
        return frame;
    }

    WorkplaneFrame WorkplaneFrame::fromPlane(const Eigen::Vector3d& centroid, const Eigen::Vector3d& normal) {
        WorkplaneFrame frame;
        frame.origin = centroid;
        frame.wAxis = normal.normalized();

        Eigen::Vector3d ref = (std::abs(frame.wAxis.z()) < 0.9)
            ? Eigen::Vector3d::UnitZ()
            : Eigen::Vector3d::UnitX();

        frame.uAxis = ref.cross(frame.wAxis).normalized();
        frame.vAxis = frame.wAxis.cross(frame.uAxis).normalized();
        return frame;
    }

    SphereFitResult MetrologyMath::fitSphereLeastSquares(
        const std::vector<Eigen::Vector3d>& hits,
        double probeRadiusMm,
        ProbeCompMode compMode) {

        SphereFitResult res;
        const size_t n = hits.size();
        if (n < 4) return res;

        Eigen::MatrixXd A(n, 4);
        Eigen::VectorXd b(n);

        for (size_t i = 0; i < n; ++i) {
            const auto& p = hits[i];
            A(i, 0) = 2.0 * p.x();
            A(i, 1) = 2.0 * p.y();
            A(i, 2) = 2.0 * p.z();
            A(i, 3) = 1.0;
            b(i) = p.squaredNorm();
        }

        Eigen::Vector4d x = A.colPivHouseholderQr().solve(b);
        Eigen::Vector3d center(x(0), x(1), x(2));
        double rSquared = x(3) + center.squaredNorm();
        if (rSquared <= 1e-9) return res;

        double rawRadius = std::sqrt(rSquared);

        // Gauss-Newton algebraic refinement
        for (int iter = 0; iter < 15; ++iter) {
            Eigen::MatrixXd J(n, 4);
            Eigen::VectorXd r(n);
            for (size_t i = 0; i < n; ++i) {
                Eigen::Vector3d diff = hits[i] - center;
                double dist = diff.norm();
                if (dist < 1e-9) return res;
                r(i) = dist - rawRadius;
                J(i, 0) = -diff.x() / dist;
                J(i, 1) = -diff.y() / dist;
                J(i, 2) = -diff.z() / dist;
                J(i, 3) = -1.0;
            }
            Eigen::Vector4d delta = J.colPivHouseholderQr().solve(-r);
            center += delta.head<3>();
            rawRadius += delta(3);
            if (delta.norm() < 1e-9) break;
        }

        double minR = std::numeric_limits<double>::max();
        double maxR = std::numeric_limits<double>::lowest();
        for (const auto& p : hits) {
            double d = (p - center).norm();
            minR = std::min(minR, d);
            maxR = std::max(maxR, d);
        }

        double finalRadius = rawRadius;
        if (compMode == ProbeCompMode::External) {
            finalRadius = rawRadius - probeRadiusMm;
        }
        else if (compMode == ProbeCompMode::Internal) {
            finalRadius = rawRadius + probeRadiusMm;
        }

        if (finalRadius <= 0.0) return res;

        res.center = center;
        res.radius = finalRadius;
        res.formError = maxR - minR;
        res.success = true;
        return res;
    }

    ProbeCalibrationResult MetrologyMath::calibrateProbeOnMasterBall(
        const std::vector<Eigen::Vector3d>& hits,
        double certifiedBallDiameterMm) {

        ProbeCalibrationResult cal;
        if (hits.size() < 5 || certifiedBallDiameterMm <= 0.0) return cal;

        SphereFitResult centerPathSphere = fitSphereLeastSquares(hits, 0.0, ProbeCompMode::Uncompensated);
        if (!centerPathSphere.success) return cal;

        const double certifiedRadius = 0.5 * certifiedBallDiameterMm;
        const double effRadius = centerPathSphere.radius - certifiedRadius;
        if (effRadius <= 0.05) return cal;

        cal.certifiedDiameter = certifiedBallDiameterMm;
        cal.effectiveRadius = effRadius;
        cal.formError = centerPathSphere.formError;
        cal.masterBallCenter = centerPathSphere.center;
        cal.success = true;
        return cal;
    }

    PlaneFitResult MetrologyMath::fitPlaneLeastSquares(
        const std::vector<Eigen::Vector3d>& hits,
        double probeRadiusMm,
        ProbeCompMode compMode,
        const Eigen::Vector3d* approachRefPoint) {

        PlaneFitResult res;
        const size_t n = hits.size();
        if (n < 3) return res;

        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : hits) centroid += p;
        centroid /= static_cast<double>(n);

        Eigen::MatrixXd centered(n, 3);
        for (size_t i = 0; i < n; ++i) {
            centered.row(i) = (hits[i] - centroid).transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
        if (svd.singularValues()(1) < 1e-7) return res;

        Eigen::Vector3d normal = svd.matrixV().col(2).normalized();

        if (approachRefPoint) {
            Eigen::Vector3d toRef = *approachRefPoint - centroid;
            if (toRef.norm() > 1e-3 && normal.dot(toRef) < 0.0) {
                normal = -normal;
            }
        }
        else if (normal.z() < 0.0) {
            normal = -normal;
        }

        double minDist = std::numeric_limits<double>::max();
        double maxDist = std::numeric_limits<double>::lowest();
        for (const auto& p : hits) {
            double d = (p - centroid).dot(normal);
            minDist = std::min(minDist, d);
            maxDist = std::max(maxDist, d);
        }

        if (compMode == ProbeCompMode::External) {
            centroid -= normal * probeRadiusMm;
        }
        else if (compMode == ProbeCompMode::Internal) {
            centroid += normal * probeRadiusMm;
        }

        res.centroid = centroid;
        res.normal = normal;
        res.flatness = maxDist - minDist;
        res.success = true;
        return res;
    }

    Circle2DFitResult MetrologyMath::fitCircle2DLeastSquares(
        const std::vector<Eigen::Vector3d>& hits,
        const WorkplaneFrame& workplane,
        double probeRadiusMm,
        ProbeCompMode compMode) {

        Circle2DFitResult res;
        const size_t n = hits.size();
        if (n < 3) return res;

        Eigen::MatrixXd A(n, 3);
        Eigen::VectorXd b(n);
        double wSum = 0.0;

        std::vector<Eigen::Vector2d> pts2D(n);
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector3d rel = hits[i] - workplane.origin;
            double u = rel.dot(workplane.uAxis);
            double v = rel.dot(workplane.vAxis);
            wSum += rel.dot(workplane.wAxis);
            pts2D[i] = Eigen::Vector2d(u, v);

            A(i, 0) = 2.0 * u;
            A(i, 1) = 2.0 * v;
            A(i, 2) = 1.0;
            b(i) = u * u + v * v;
        }

        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
        Eigen::Vector2d c2d(x(0), x(1));
        double rSq = x(2) + c2d.squaredNorm();
        if (rSq <= 1e-9) return res;

        double rawRadius = std::sqrt(rSq);

        for (int iter = 0; iter < 15; ++iter) {
            Eigen::MatrixXd J(n, 3);
            Eigen::VectorXd r(n);
            for (size_t i = 0; i < n; ++i) {
                Eigen::Vector2d diff = pts2D[i] - c2d;
                double dist = diff.norm();
                if (dist < 1e-9) return res;
                r(i) = dist - rawRadius;
                J(i, 0) = -diff.x() / dist;
                J(i, 1) = -diff.y() / dist;
                J(i, 2) = -1.0;
            }
            Eigen::Vector3d delta = J.colPivHouseholderQr().solve(-r);
            c2d += delta.head<2>();
            rawRadius += delta(2);
            if (delta.norm() < 1e-9) break;
        }

        double minR = std::numeric_limits<double>::max();
        double maxR = std::numeric_limits<double>::lowest();
        for (const auto& p : pts2D) {
            double d = (p - c2d).norm();
            minR = std::min(minR, d);
            maxR = std::max(maxR, d);
        }

        double finalRadius = rawRadius;
        if (compMode == ProbeCompMode::External) {
            finalRadius = rawRadius - probeRadiusMm;
        }
        else if (compMode == ProbeCompMode::Internal) {
            finalRadius = rawRadius + probeRadiusMm;
        }
        if (finalRadius <= 0.0) return res;

        double wMean = wSum / static_cast<double>(n);
        res.center = workplane.origin
            + c2d.x() * workplane.uAxis
            + c2d.y() * workplane.vAxis
            + wMean * workplane.wAxis;
        res.planeNormal = workplane.wAxis;
        res.radius = finalRadius;
        res.circularity = maxR - minR;
        res.success = true;
        return res;
    }

    Line2DFitResult MetrologyMath::fitLine2DLeastSquares(
        const std::vector<Eigen::Vector3d>& hits,
        const WorkplaneFrame& workplane,
        double probeRadiusMm,
        ProbeCompMode compMode,
        const Eigen::Vector3d* approachRefPoint) {

        Line2DFitResult res;
        const size_t n = hits.size();
        if (n < 2) return res;

        std::vector<Eigen::Vector2d> pts2D(n);
        Eigen::Vector2d mean2D = Eigen::Vector2d::Zero();
        double wSum = 0.0;

        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector3d rel = hits[i] - workplane.origin;
            pts2D[i] = Eigen::Vector2d(rel.dot(workplane.uAxis), rel.dot(workplane.vAxis));
            mean2D += pts2D[i];
            wSum += rel.dot(workplane.wAxis);
        }
        mean2D /= static_cast<double>(n);
        double wMean = wSum / static_cast<double>(n);

        Eigen::MatrixXd centered(n, 2);
        for (size_t i = 0; i < n; ++i) {
            centered.row(i) = (pts2D[i] - mean2D).transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
        if (svd.singularValues()(0) < 1e-7) return res;

        Eigen::Vector2d dir2D = svd.matrixV().col(0).normalized();
        Eigen::Vector2d firstToLast = pts2D.back() - pts2D.front();
        if (firstToLast.dot(dir2D) < 0.0) {
            dir2D = -dir2D;
        }

        Eigen::Vector2d norm2D(-dir2D.y(), dir2D.x());
        if (approachRefPoint) {
            Eigen::Vector3d relRef = *approachRefPoint - workplane.origin;
            Eigen::Vector2d ref2D(relRef.dot(workplane.uAxis), relRef.dot(workplane.vAxis));
            if ((ref2D - mean2D).dot(norm2D) < 0.0) {
                norm2D = -norm2D;
            }
        }

        double minD = std::numeric_limits<double>::max();
        double maxD = std::numeric_limits<double>::lowest();
        for (const auto& p : pts2D) {
            double d = (p - mean2D).dot(norm2D);
            minD = std::min(minD, d);
            maxD = std::max(maxD, d);
        }

        if (compMode == ProbeCompMode::External) {
            mean2D -= norm2D * probeRadiusMm;
        }
        else if (compMode == ProbeCompMode::Internal) {
            mean2D += norm2D * probeRadiusMm;
        }

        res.centroid = workplane.origin
            + mean2D.x() * workplane.uAxis
            + mean2D.y() * workplane.vAxis
            + wMean * workplane.wAxis;
        res.direction = (dir2D.x() * workplane.uAxis + dir2D.y() * workplane.vAxis).normalized();
        res.edgeNormal = (norm2D.x() * workplane.uAxis + norm2D.y() * workplane.vAxis).normalized();
        res.straightness = maxD - minD;
        res.success = true;
        return res;
    }

    FeatureRelationResult MetrologyMath::computeFeatureRelation(
        const MeasuredFeature& featA,
        const MeasuredFeature& featB,
        const WorkplaneFrame& workplane) {

        FeatureRelationResult rel;
        rel.deltaXYZ = featB.location - featA.location;
        rel.distance3D = rel.deltaXYZ.norm();

        double du = rel.deltaXYZ.dot(workplane.uAxis);
        double dv = rel.deltaXYZ.dot(workplane.vAxis);
        rel.distance2D = std::hypot(du, dv);

        if (featA.kind == FeatureKind::Plane3D) {
            rel.normalDistance = std::abs(rel.deltaXYZ.dot(featA.primaryVector.normalized()));
        }
        else if (featA.kind == FeatureKind::Line2D) {
            rel.normalDistance = std::abs(rel.deltaXYZ.dot(featA.secondaryVector.normalized()));
        }
        else {
            rel.normalDistance = rel.distance2D;
        }

        double dotVal = std::clamp(
            featA.primaryVector.normalized().dot(featB.primaryVector.normalized()),
            -1.0, 1.0
        );
        rel.angleDegrees = std::acos(std::abs(dotVal)) * RAD_TO_DEG;

        if (featA.kind == FeatureKind::Line2D && featB.kind == FeatureKind::Line2D) {
            Eigen::Vector2d pA(
                (featA.location - workplane.origin).dot(workplane.uAxis),
                (featA.location - workplane.origin).dot(workplane.vAxis)
            );
            Eigen::Vector2d dA(
                featA.primaryVector.dot(workplane.uAxis),
                featA.primaryVector.dot(workplane.vAxis)
            );
            Eigen::Vector2d pB(
                (featB.location - workplane.origin).dot(workplane.uAxis),
                (featB.location - workplane.origin).dot(workplane.vAxis)
            );
            Eigen::Vector2d dB(
                featB.primaryVector.dot(workplane.uAxis),
                featB.primaryVector.dot(workplane.vAxis)
            );

            double det = dA.x() * (-dB.y()) - dA.y() * (-dB.x());
            if (std::abs(det) > 1e-6) {
                Eigen::Vector2d rhs = pB - pA;
                double t = (rhs.x() * (-dB.y()) - rhs.y() * (-dB.x())) / det;
                Eigen::Vector2d inter2D = pA + t * dA;
                double wAvg = 0.5 * (
                    (featA.location - workplane.origin).dot(workplane.wAxis) +
                    (featB.location - workplane.origin).dot(workplane.wAxis)
                    );
                rel.hasIntersection = true;
                rel.intersectionPoint = workplane.origin
                    + inter2D.x() * workplane.uAxis
                    + inter2D.y() * workplane.vAxis
                    + wAvg * workplane.wAxis;
            }
        }

        rel.success = true;
        return rel;
    }

    Alignment321Result MetrologyMath::compute321Alignment(
        const MeasuredFeature& levelPlane,
        const MeasuredFeature& rotateFeat,
        const MeasuredFeature& originFeat) {

        Alignment321Result res;
        if (levelPlane.kind != FeatureKind::Plane3D) {
            return res;
        }

        Eigen::Vector3d zAxis = levelPlane.primaryVector.normalized();
        Eigen::Vector3d rawX = Eigen::Vector3d::UnitX();

        if (rotateFeat.kind == FeatureKind::Line2D) {
            rawX = rotateFeat.primaryVector;
        }
        else if (rotateFeat.kind == FeatureKind::Plane3D) {
            // Secondary Datum is a side plane (e.g., front or side wall):
            // The intersection line of Primary Plane (Z) and Secondary Plane runs along zAxis.cross(nB)
            Eigen::Vector3d nB = rotateFeat.primaryVector.normalized();
            Eigen::Vector3d edgeDir = nB.cross(zAxis);
            if (edgeDir.norm() > 1e-4) {
                // Keep +X pointing in the positive world X/Y hemisphere by default
                if (edgeDir.x() < -1e-4 || (std::abs(edgeDir.x()) <= 1e-4 && edgeDir.y() < 0.0)) {
                    edgeDir = -edgeDir;
                }
                rawX = edgeDir;
            }
        }
        else {
            Eigen::Vector3d diff = rotateFeat.location - originFeat.location;
            if (diff.norm() > 1e-4) {
                rawX = diff;
            }
        }

        // Project rawX onto the primary plane
        rawX = rawX - (rawX.dot(zAxis)) * zAxis;
        if (rawX.norm() < 1e-6) {
            return res;
        }
        Eigen::Vector3d xAxis = rawX.normalized();
        Eigen::Vector3d yAxis = zAxis.cross(xAxis).normalized();

        // Compute true 3-2-1 datum origin:
        // Start from originFeat.location, project Z onto Primary Datum Plane (levelPlane),
        // and if rotateFeat/originFeat are Planes, intersect the three datum planes!
        Eigen::Vector3d origin = originFeat.location;
        if (rotateFeat.kind == FeatureKind::Plane3D && originFeat.kind == FeatureKind::Plane3D) {
            // Solve 3-plane intersection:
            // nA . P = nA . pA
            // nB . P = nB . pB
            // nC . P = nC . pC
            Eigen::Matrix3d M;
            M.row(0) = zAxis.transpose();
            M.row(1) = rotateFeat.primaryVector.normalized().transpose();
            M.row(2) = originFeat.primaryVector.normalized().transpose();
            if (std::abs(M.determinant()) > 1e-5) {
                Eigen::Vector3d rhs(
                    zAxis.dot(levelPlane.location),
                    M.row(1).dot(rotateFeat.location),
                    M.row(2).dot(originFeat.location)
                );
                origin = M.colPivHouseholderQr().solve(rhs);
            }
            else {
                origin = origin - ((origin - levelPlane.location).dot(zAxis)) * zAxis;
            }
        }
        else {
            // Project originFeat (e.g. Bore center or point) onto Primary Datum Plane (Z = 0 on Datum A)
            origin = origin - ((origin - levelPlane.location).dot(zAxis)) * zAxis;
        }

        Eigen::Matrix3d R;
        R.row(0) = xAxis.transpose();
        R.row(1) = yAxis.transpose();
        R.row(2) = zAxis.transpose();

        res.transformationMatrix.setIdentity();
        res.transformationMatrix.block<3, 3>(0, 0) = R;
        res.transformationMatrix.block<3, 1>(0, 3) = -R * origin;
        res.success = true;
        return res;
    }
    CylinderFitResult MetrologyMath::fitCylinderLeastSquares(
        const std::vector<Eigen::Vector3d>& hits,
        double probeRadius,
        ProbeCompMode compMode,
        const Eigen::Vector3d* initialAxisHint
    ) {
        CylinderFitResult res;
        if (hits.size() < 5) return res;

        // 1. Apply stylus probe compensation
        std::vector<Eigen::Vector3d> compHits;
        compHits.reserve(hits.size());
        for (size_t i = 0; i < hits.size(); ++i) {
            Eigen::Vector3d pt = hits[i];
            if (compMode != ProbeCompMode::Uncompensated && probeRadius > 0.0) {
                // Estimate local outward normal from centroid
                Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
                for (const auto& h : hits) centroid += h;
                centroid /= static_cast<double>(hits.size());
                Eigen::Vector3d radialDir = (pt - centroid);
                radialDir.z() = 0.0; // Assume mostly vertical or horizontal cylinder axis
                if (radialDir.norm() > 1e-6) {
                    radialDir.normalize();
                    double sign = (compMode == ProbeCompMode::Internal) ? 1.0 : -1.0;
                    pt += sign * probeRadius * radialDir;
                }
            }
            compHits.push_back(pt);
        }

        // 2. Compute centroid and initial axis via PCA (Principal Component Analysis)
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : compHits) centroid += p;
        centroid /= static_cast<double>(compHits.size());

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& p : compHits) {
            Eigen::Vector3d diff = p - centroid;
            cov += diff * diff.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
        Eigen::Vector3d axis = eig.eigenvectors().col(2).normalized(); // eigenvector with largest eigenvalue
        if (initialAxisHint && initialAxisHint->norm() > 1e-6) {
            axis = initialAxisHint->normalized();
            if (axis.dot(eig.eigenvectors().col(2)) < 0.0) axis = -axis;
        }

        // Initial radius estimate
        double sumR = 0.0;
        for (const auto& p : compHits) {
            Eigen::Vector3d d = (p - centroid) - ((p - centroid).dot(axis)) * axis;
            sumR += d.norm();
        }
        double radius = sumR / static_cast<double>(compHits.size());

        // 3. Gauss-Newton Iterative Refinement for Cylinder (Axis point P0, direction V, radius R)
        // Parameter vector: [P0x, P0y, P0z, Vx, Vy, R] (6 degrees of freedom, V is unit constrained)
        Eigen::Vector3d p0 = centroid;

        for (int iter = 0; iter < 15; ++iter) {
            Eigen::MatrixXd J(compHits.size(), 6);
            Eigen::VectorXd F(compHits.size());

            for (size_t i = 0; i < compHits.size(); ++i) {
                const Eigen::Vector3d& p = compHits[i];
                Eigen::Vector3d d = p - p0;
                Eigen::Vector3d cross = d.cross(axis);
                double distToAxis = cross.norm();
                double r_i = distToAxis;

                F(i) = r_i - radius;

                if (distToAxis < 1e-9) continue;

                // Partial derivatives w.r.t p0 (dx, dy, dz), axis (vx, vy), and radius (R)
                // d(dist)/dp0 = - (axis x cross) / distToAxis
                Eigen::Vector3d d_p0 = -(axis.cross(cross)) / distToAxis;
                // d(dist)/d(axis) = (d x cross) / distToAxis
                Eigen::Vector3d d_axis = (d.cross(cross)) / distToAxis;

                J(i, 0) = d_p0.x();
                J(i, 1) = d_p0.y();
                J(i, 2) = d_p0.z();
                J(i, 3) = d_axis.x();
                J(i, 4) = d_axis.y();
                J(i, 5) = -1.0; // w.r.t radius R
            }

            Eigen::VectorXd delta = (J.transpose() * J).ldlt().solve(-J.transpose() * F);
            p0 += Eigen::Vector3d(delta(0), delta(1), delta(2));
            axis += Eigen::Vector3d(delta(3), delta(4), 0.0);
            axis.normalize();
            radius += delta(5);

            if (delta.norm() < 1e-7) break;
        }

        // 4. Compute Form Error (Cylindricity: max_r - min_r)
        double minR = std::numeric_limits<double>::max();
        double maxR = std::numeric_limits<double>::lowest();
        double sqSum = 0.0;

        for (const auto& p : compHits) {
            Eigen::Vector3d d = (p - p0) - ((p - p0).dot(axis)) * axis;
            double r = d.norm();
            minR = std::min(minR, r);
            maxR = std::max(maxR, r);
            double err = r - radius;
            sqSum += err * err;
        }

        res.success = true;
        res.center = p0;
        res.axis = axis;
        res.radius = radius;
        res.diameter = radius * 2.0;
        res.cylindricity = maxR - minR;
        res.rmsError = std::sqrt(sqSum / static_cast<double>(compHits.size()));
        return res;
    }


    Eigen::Vector3d MetrologyMath::transformPoint(
        const Eigen::Vector3d& pt,
        const Eigen::Matrix4d& trsfMatrix) {
        Eigen::Vector4d h(pt.x(), pt.y(), pt.z(), 1.0);
        return (trsfMatrix * h).head<3>();
    }

    DeviationResult MetrologyMath::computePointToCadDeviation(
        const Eigen::Vector3d& point,
        const TopoDS_Shape& cadShape,
        double probeRadiusMm,
        ProbeCompMode compMode) {

        DeviationResult res;
        if (cadShape.IsNull()) return res;

        gp_Pnt pIn(point.x(), point.y(), point.z());
        BRepBuilderAPI_MakeVertex mkVert(pIn);

        BRepExtrema_DistShapeShape dister(mkVert.Vertex(), cadShape);
        if (!dister.IsDone() || dister.NbSolution() == 0) {
            return res;
        }

        gp_Pnt pCad = dister.PointOnShape2(1);
        res.nearestCadPoint = Eigen::Vector3d(pCad.X(), pCad.Y(), pCad.Z());
        res.measuredPoint = point;

        Eigen::Vector3d vDelta = res.measuredPoint - res.nearestCadPoint;
        double rawDist = vDelta.norm();

        Eigen::Vector3d normal(0.0, 0.0, 1.0);
        if (rawDist > 1e-9) {
            normal = vDelta / rawDist;
        }

        // Extract true outward CAD face normal if the nearest support is a Face
        TopoDS_Shape support = dister.SupportOnShape2(1);
        if (!support.IsNull() && support.ShapeType() == TopAbs_FACE) {
            Standard_Real u = 0.0, v = 0.0;
            dister.ParOnFaceS2(1, u, v);
            TopoDS_Face face = TopoDS::Face(support);
            BRepGProp_Face prop(face);
            gp_Pnt surfPt;
            gp_Vec surfNorm;
            prop.Normal(u, v, surfPt, surfNorm);
            if (surfNorm.Magnitude() > 1e-9) {
                surfNorm.Normalize();
                normal = Eigen::Vector3d(surfNorm.X(), surfNorm.Y(), surfNorm.Z());
            }
        }

        res.surfaceNormal = normal;

        // Project delta onto outward surface normal
        double signedCenterDist = vDelta.dot(normal);

        // Compensate for probe tip radius along the surface normal
        double compOffset = 0.0;
        if (compMode == ProbeCompMode::External) {
            compOffset = probeRadiusMm;
        }
        else if (compMode == ProbeCompMode::Internal) {
            compOffset = -probeRadiusMm;
        }

        // If probing approached from the opposite side of a single-sided sheet, flip sign
        if (signedCenterDist < 0.0 && compMode == ProbeCompMode::External && std::abs(signedCenterDist) >= 0.5 * probeRadiusMm) {
            signedCenterDist = std::abs(signedCenterDist);
        }

        res.signedDeviationMm = signedCenterDist - compOffset;
        res.success = true;
        return res;
    }

} // namespace OpenCMM