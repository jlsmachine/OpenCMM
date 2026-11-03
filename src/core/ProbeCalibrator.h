#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <QVector3D>

namespace OpenCMM {

    enum class FeatureBoundaryMode {
        Uncompensated = 0,
        InternalBore,     // ID: Add R_eff to fitted radius
        ExternalBoss,     // OD: Subtract R_eff from fitted radius
        PlaneSurface      // Shift origin along normal by R_eff
    };

    struct CalibrationResult {
        bool success = false;
        QVector3D sphereCenter{ 0.0f, 0.0f, 0.0f };
        double measuredCenterRadius = 0.0; // R_meas (mm)
        double referenceRadius = 0.0;      // R_ref (mm)
        double effectiveStylusRadius = 0.0;// R_eff = R_meas - R_ref (mm)
        double sphericityError = 0.0;      // Max radial residual spread (mm)
        double rmsResidual = 0.0;          // RMS fit error (mm)
    };

    class ProbeCalibrator {
    public:
        // Solve sphere from >= 4 non-coplanar ruby center hits
        static CalibrationResult calibrateOnSphere(
            const std::vector<QVector3D>& hits,
            double referenceSphereDiameterMm)
        {
            CalibrationResult res;
            const size_t N = hits.size();
            if (N < 4 || referenceSphereDiameterMm <= 0.0) {
                return res;
            }

            res.referenceRadius = referenceSphereDiameterMm * 0.5;

            // Build normal equations (A^T * A) * c = (A^T * b) in double precision
            // to avoid floating-point catastrophic cancellation
            double ATA[4][4] = { 0.0 };
            double ATb[4] = { 0.0 };

            // Centroid shift for numerical stability
            double cx = 0.0, cy = 0.0, cz = 0.0;
            for (const auto& p : hits) {
                cx += p.x(); cy += p.y(); cz += p.z();
            }
            cx /= N; cy /= N; cz /= N;

            for (const auto& p : hits) {
                double x = p.x() - cx;
                double y = p.y() - cy;
                double z = p.z() - cz;
                double r2 = x * x + y * y + z * z;

                double row[4] = { x, y, z, 1.0 };
                for (int i = 0; i < 4; ++i) {
                    ATb[i] += row[i] * r2;
                    for (int j = 0; j < 4; ++j) {
                        ATA[i][j] += row[i] * row[j];
                    }
                }
            }

            // Solve 4x4 linear system via Gaussian elimination with partial pivoting
            double c[4] = { 0.0 };
            if (!solve4x4(ATA, ATb, c)) {
                return res;
            }

            double x0 = 0.5 * c[0];
            double y0 = 0.5 * c[1];
            double z0 = 0.5 * c[2];
            double rMeasSq = c[3] + x0 * x0 + y0 * y0 + z0 * z0;
            if (rMeasSq <= 0.0) {
                return res;
            }

            res.sphereCenter = QVector3D(
                static_cast<float>(x0 + cx),
                static_cast<float>(y0 + cy),
                static_cast<float>(z0 + cz)
            );
            res.measuredCenterRadius = std::sqrt(rMeasSq);
            res.effectiveStylusRadius = res.measuredCenterRadius - res.referenceRadius;

            // Compute residuals and sphericity (lobing/form error)
            double minR = 1e9, maxR = -1e9, sumSqErr = 0.0;
            for (const auto& p : hits) {
                double dx = p.x() - res.sphereCenter.x();
                double dy = p.y() - res.sphereCenter.y();
                double dz = p.z() - res.sphereCenter.z();
                double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                minR = std::min(minR, dist);
                maxR = std::max(maxR, dist);
                double err = dist - res.measuredCenterRadius;
                sumSqErr += err * err;
            }

            res.sphericityError = maxR - minR;
            res.rmsResidual = std::sqrt(sumSqErr / N);
            res.success = (res.effectiveStylusRadius > 0.1 && res.effectiveStylusRadius < 15.0);
            return res;
        }

        // Compensate a single point using the approach unit vector (from pre-hit DRO buffer)
        static QVector3D compensateHitByApproach(
            const QVector3D& rawHit,
            const QVector3D& preHitDroPosition,
            double effectiveStylusRadiusMm)
        {
            QVector3D delta = rawHit - preHitDroPosition;
            if (delta.lengthSquared() < 1e-6f) {
                return rawHit; // Fallback if stationary
            }
            QVector3D approachDir = delta.normalized();
            return rawHit + approachDir * static_cast<float>(effectiveStylusRadiusMm);
        }

        // Compensate a solved 2D/3D Circle, Cylinder, or Sphere radius
        static double compensateRadius(
            double rawFittedRadiusMm,
            double effectiveStylusRadiusMm,
            FeatureBoundaryMode mode)
        {
            switch (mode) {
            case FeatureBoundaryMode::InternalBore:
                return rawFittedRadiusMm + effectiveStylusRadiusMm;
            case FeatureBoundaryMode::ExternalBoss:
                return std::max(0.0, rawFittedRadiusMm - effectiveStylusRadiusMm);
            default:
                return rawFittedRadiusMm;
            }
        }

    private:
        static bool solve4x4(double A[4][4], double b[4], double x[4]) {
            for (int i = 0; i < 4; ++i) {
                int pivot = i;
                for (int j = i + 1; j < 4; ++j) {
                    if (std::abs(A[j][i]) > std::abs(A[pivot][i])) pivot = j;
                }
                if (std::abs(A[pivot][i]) < 1e-12) return false;
                for (int k = 0; k < 4; ++k) std::swap(A[i][k], A[pivot][k]);
                std::swap(b[i], b[pivot]);

                for (int j = i + 1; j < 4; ++j) {
                    double factor = A[j][i] / A[i][i];
                    b[j] -= factor * b[i];
                    for (int k = i; k < 4; ++k) A[j][k] -= factor * A[i][k];
                }
            }
            for (int i = 3; i >= 0; --i) {
                double sum = b[i];
                for (int j = i + 1; j < 4; ++j) sum -= A[i][j] * x[j];
                x[i] = sum / A[i][i];
            }
            return true;
        }
    };

} // namespace OpenCMM