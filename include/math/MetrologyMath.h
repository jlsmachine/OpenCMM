#pragma once
#include <Eigen/Dense>
#include <TopoDS_Shape.hxx>
#include <vector>
#include <string>

namespace OpenCMM {

    enum class ProbeCompMode {
        External = 0,
        Internal = 1,
        Uncompensated = 2
    };

    enum class StandardWorkplane {
        XY_PlusZ = 0,
        YZ_PlusX = 1,
        ZX_PlusY = 2,
        CustomPlane = 3
    };

    enum class FeatureKind {
        Sphere3D = 0,
        Plane3D = 1,
        Circle2D = 2,
        Line2D = 3
    };

    struct WorkplaneFrame {
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        Eigen::Vector3d uAxis = Eigen::Vector3d::UnitX();
        Eigen::Vector3d vAxis = Eigen::Vector3d::UnitY();
        Eigen::Vector3d wAxis = Eigen::Vector3d::UnitZ();

        static WorkplaneFrame fromStandard(StandardWorkplane wp);
        static WorkplaneFrame fromPlane(const Eigen::Vector3d& centroid, const Eigen::Vector3d& normal);
    };

    struct MeasuredFeature {
        int id = 0;
        std::string name;
        FeatureKind kind = FeatureKind::Sphere3D;
        Eigen::Vector3d location = Eigen::Vector3d::Zero();
        Eigen::Vector3d primaryVector = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d secondaryVector = Eigen::Vector3d::UnitY();
        double diameter = 0.0;
        double formError = 0.0;
    };

    struct ProbeCalibrationResult {
        bool success = false;
        double certifiedDiameter = 25.4;
        double effectiveRadius = 1.5;
        double formError = 0.0;
        Eigen::Vector3d masterBallCenter = Eigen::Vector3d::Zero();
    };

    struct SphereFitResult {
        bool success = false;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        double radius = 0.0;
        double formError = 0.0;
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
        double signedDeviationMm = 0.0;
        Eigen::Vector3d measuredPoint = Eigen::Vector3d::Zero();
        Eigen::Vector3d nearestCadPoint = Eigen::Vector3d::Zero();
        Eigen::Vector3d surfaceNormal = Eigen::Vector3d::UnitZ();
    };

    class MetrologyMath {
    public:
        static ProbeCalibrationResult calibrateProbeOnMasterBall(
            const std::vector<Eigen::Vector3d>& hits,
            double certifiedBallDiameterMm);

        static SphereFitResult fitSphereLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            double probeRadiusMm,
            ProbeCompMode compMode);

        static PlaneFitResult fitPlaneLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            double probeRadiusMm,
            ProbeCompMode compMode,
            const Eigen::Vector3d* approachRefPoint = nullptr);

        static Circle2DFitResult fitCircle2DLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            const WorkplaneFrame& workplane,
            double probeRadiusMm,
            ProbeCompMode compMode);

        static Line2DFitResult fitLine2DLeastSquares(
            const std::vector<Eigen::Vector3d>& hits,
            const WorkplaneFrame& workplane,
            double probeRadiusMm,
            ProbeCompMode compMode,
            const Eigen::Vector3d* approachRefPoint = nullptr);

        static FeatureRelationResult computeFeatureRelation(
            const MeasuredFeature& featA,
            const MeasuredFeature& featB,
            const WorkplaneFrame& workplane);

        static Alignment321Result compute321Alignment(
            const MeasuredFeature& levelPlane,
            const MeasuredFeature& rotateFeat,
            const MeasuredFeature& originFeat);

        static Eigen::Vector3d transformPoint(
            const Eigen::Vector3d& pt,
            const Eigen::Matrix4d& trsfMatrix);

        static DeviationResult computePointToCadDeviation(
            const Eigen::Vector3d& point,
            const TopoDS_Shape& cadShape);
    };

} // namespace OpenCMM