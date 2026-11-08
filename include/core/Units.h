// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <string>

namespace OpenCMM {

    struct Position3D {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    enum class UnitSystem {
        Millimeters,
        Inches
    };

    enum class ProbeCompMode {
        External,
        Internal,
        Uncompensated
    };

    enum class StandardWorkplane {
        XY_PlusZ,
        YZ_PlusX,
        ZX_PlusY,
        CustomPlane
    };

    struct WorkplaneFrame {
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        Eigen::Vector3d uAxis = Eigen::Vector3d::UnitX();
        Eigen::Vector3d vAxis = Eigen::Vector3d::UnitY();
        Eigen::Vector3d wAxis = Eigen::Vector3d::UnitZ();

        static WorkplaneFrame fromStandard(StandardWorkplane wp);
        static WorkplaneFrame fromPlane(const Eigen::Vector3d& centroid, const Eigen::Vector3d& normal);
    };

    inline double toDisplayUnit(double valueMm, UnitSystem sys) {
        return (sys == UnitSystem::Millimeters) ? valueMm : (valueMm / 25.4);
    }

    inline double fromDisplayUnit(double valueDisplay, UnitSystem sys) {
        return (sys == UnitSystem::Millimeters) ? valueDisplay : (valueDisplay * 25.4);
    }

} // namespace OpenCMM