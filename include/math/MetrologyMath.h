#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace OpenCMM {

    enum class ProbeCompMode {
        External,      // Boss / Outer Edge (-R_eff)
        Internal,      // Bore / Cavity (+R_eff)
        Uncompensated  // Raw Ruby Center (0)
    };

    enum class StandardWorkplane {
        XY_PlusZ,      // Origin (0,0,0), Normal +Z, U=+X, V=+Y
        YZ_PlusX,      // Origin (0,0,0), Normal +X, U=+Y, V=+Z
        ZX_PlusY,      // Origin (0,0,0), Normal +Y, U=+Z, V=+X
        CustomPlane    // Constructed from last fitted 3D Plane
    };

    struct WorkplaneFrame {
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        Eigen::Vector3d uAxis = Eigen::Vector3d::UnitX();
        Eigen::Vector3d vAxis = Eigen::Vector3d::UnitY();
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

        static WorkplaneFrame fromStandard(StandardWorkplane wp);
        static WorkplaneFrame fromPlane(const Eigen::Vector3d& planeCentroid, const Eigen::Vector3d& planeNormal);

        Eigen::Vector2d projectTo2D(const Eigen::Vector3d& pt3d, double* outHeight = nullptr) const;
        Eigen::Vector3d to3D(const Eigen::Vector2d& pt2d, double height = 0.0) const;
    };

    struct ProbeCalibrationResult {
        bool success = false;
        Eigen::Vector3d masterBallCenter = Eigen::Vector3d::Zero();
        double certifiedDiameter = 25.4;
        double rawFitRadius = 0.0;
        double effectiveRadius = 0.0;
        double formError = 0.0;
        double rmsError = 0.0;
    };

    struct SphereFitResult {
        bool success = false;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        double rawRadius = 0.0;
        double radius = 0.0;
        double appliedComp = 0.0;
        double formError = 0.0;
        double rmsError = 0.0;
    };

    struct PlaneFitResult {
        bool success = false;
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d rawCentroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        double appliedComp = 0.0;
        double flatness = 0.0;
        double rmsError = 0.0;
    };

    struct Circle2DFitResult {
        bool success = false;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d planeNormal = Eigen::Vector3d::UnitZ();
        double rawRadius = 0.0;
        double radius = 0.0;
        double appliedComp = 0.0;
        double circularity = 0.0; // Roundness (Max R - Min R in workplane)
        double rmsError = 0.0;
    };

    struct Line2DFitResult {
        bool success = false;
        Eigen::Vector3d rawCentroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d direction = Eigen::Vector3d::UnitX();  // Unit tangent along edge
        Eigen::Vector3d edgeNormal = Eigen::Vector3d::UnitY(); // In-plane normal pointing toward probe
        double appliedComp = 0.0;
        double straightness = 0.0; // Max - Min orthogonal distance
        double rmsError = 0.0;
    };

    enum class FeatureKind {
        Sphere3D,
        Plane3D,
        Circle2D,
        Line2D
    };

    struct MeasuredFeature {
        int id = 0;
        std::string name;
        FeatureKind kind = FeatureKind::Sphere3D;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();   // Center or Centroid
        Eigen::Vector3d direction = Eigen::Vector3d::UnitZ(); // Normal, Axis, or Line Tangent
        Eigen::Vector3d edgeNormal = Eigen::Vector3d::UnitY();
        double diameter = 0.0;
        double formError = 0.0;
    };

    struct FeatureRelationResult {
        bool success = false;
        Eigen::Vector3d deltaXYZ = Eigen::Vector3d::Zero();
        double distance3D = 0.0;
        double distance2D = 0.0;       // Projected distance in active workplane
        double normalDistance = 0.0;   // Perpendicular offset along Feature A's normal
        double angleDegrees = 0.0;     // Angle between feature axes/normals/directions
        bool hasIntersection = false;
        Eigen::Vector3d intersectionPoint = Eigen::Vector3d::Zero();
    };

    class MetrologyMath {
    public:
        static ProbeCalibrationResult calibrateProbeOnMasterBall(
            const std::vector<Eigen::Vector3d>& points,
            double certifiedBallDiameterMm);

        static SphereFitResult fitSphereLeastSquares(
            const std::vector<Eigen::Vector3d>& points,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::Uncompensated);

        static PlaneFitResult fitPlaneLeastSquares(
            const std::vector<Eigen::Vector3d>& points,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::Uncompensated,
            const Eigen::Vector3d* clearancePoint = nullptr);

        static Circle2DFitResult fitCircle2DLeastSquares(
            const std::vector<Eigen::Vector3d>& points,
            const WorkplaneFrame& workplane,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::Internal);

        static Line2DFitResult fitLine2DLeastSquares(
            const std::vector<Eigen::Vector3d>& points,
            const WorkplaneFrame& workplane,
            double probeRadiusMm = 0.0,
            ProbeCompMode compMode = ProbeCompMode::External,
            const Eigen::Vector3d* clearancePoint = nullptr);

        static FeatureRelationResult computeFeatureRelation(
            const MeasuredFeature& featA,
            const MeasuredFeature& featB,
            const WorkplaneFrame& workplane);
    };

} // namespace OpenCMM