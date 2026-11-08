// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <vector>
#include <Eigen/Dense>
#include <TopoDS_Shape.hxx>
#include "core/Units.h"

namespace OpenCMM {

    enum class FeatureKind {
        Plane3D,
        Circle2D,
        Sphere3D,
        Line2D
    };

    struct MeasuredFeature {
        int id = 0;
        std::string name;
        FeatureKind kind = FeatureKind::Circle2D;
        Eigen::Vector3d location = Eigen::Vector3d::Zero();
        Eigen::Vector3d primaryVector = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d secondaryVector = Eigen::Vector3d::UnitX();
        double diameter = 0.0;
        double formError = 0.0;
    };

    struct SphereFitResult {
        bool success = false;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        double radius = 0.0;
        double formError = 0.0;
    };

    struct CylinderFitResult {
        bool success = false;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        double radius = 0.0;
        double diameter = 0.0;
        double cylindricity = 0.0;
        double rmsError = 0.0;
    };

    struct PlaneFitResult {
        bool success = false;
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        double flatness = 0.0;
    };

    struct Circle2DFitResult {
        bool success = false;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d planeNormal = Eigen::Vector3d::UnitZ();
        double radius = 0.0;
        double circularity = 0.0;
    };

    struct Line2DFitResult {
        bool success = false;
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
        Eigen::Vector3d edgeNormal = Eigen::Vector3d::UnitY();
        double straightness = 0.0;
    };

    struct ProbeCalibrationResult {
        bool success = false;
        double certifiedDiameter = 25.4000;
        double effectiveRadius = 1.5000;
        double formError = 0.0;
        Eigen::Vector3d masterBallCenter = Eigen::Vector3d::Zero();
    };

    struct FeatureRelationResult {
        bool success = false;
        double distance3D = 0.0;
        double distance2D = 0.0;
        double normalDistance = 0.0;
        double angleDegrees = 0.0;
        Eigen::Vector3d deltaXYZ = Eigen::Vector3d::Zero();
        bool hasIntersection = false;
        Eigen::Vector3d intersectionPoint = Eigen::Vector3d::Zero();
    };

    struct Alignment321Result {
        bool success = false;
        Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    };

    struct DeviationResult {
        bool success = false;
        Eigen::Vector3d measuredPoint = Eigen::Vector3d::Zero();
        Eigen::Vector3d nearestCadPoint = Eigen::Vector3d::Zero();
        Eigen::Vector3d surfaceNormal = Eigen::Vector3d::UnitZ();
        double signedDeviationMm = 0.0;
    };

    class MetrologyMath {
    public:
        static SphereFitResult fitSphereLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::External
        );

        static CylinderFitResult fitCylinderLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            double probeRadius = 0.0,
            ProbeCompMode compMode = ProbeCompMode::External,
            const Eigen::Vector3d* initialAxisHint = nullptr
        );

        static ProbeCalibrationResult calibrateProbeOnMasterBall(
            const std::vector<Eigen::Vector3d>& hits,
            double certifiedBallDiameterMm = 25.4000
        );

        static PlaneFitResult fitPlaneLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::External,
            const Eigen::Vector3d* approachRefPoint = nullptr
        );

        static Circle2DFitResult fitCircle2DLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            const WorkplaneFrame& workplane,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::External
        );

        static Line2DFitResult fitLine2DLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            const WorkplaneFrame& workplane,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::External,
            const Eigen::Vector3d* approachRefPoint = nullptr
        );

        static FeatureRelationResult computeFeatureRelation(
            const MeasuredFeature& featA,
            const MeasuredFeature& featB,
            const WorkplaneFrame& workplane
        );

        static Alignment321Result compute321Alignment(
            const MeasuredFeature& levelPlane,
            const MeasuredFeature& rotateFeat,
            const MeasuredFeature& originFeat
        );

        static Eigen::Vector3d transformPoint(
            const Eigen::Vector3d& pt,
            const Eigen::Matrix4d& trsfMatrix
        );

        static DeviationResult computePointToCadDeviation(
            const Eigen::Vector3d& point,
            const TopoDS_Shape& cadShape,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::External
        );
    };

} // namespace OpenCMM