// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace OpenCMM {

    enum class UnitSystem {
        Millimeters,
        Inches
    };

    struct Position3D {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    constexpr double MM_PER_INCH = 25.4;
    constexpr double ENCODER_SCALE_MM = 0.0005;

    inline double toDisplayUnit(double valueMm, UnitSystem units) {
        return (units == UnitSystem::Inches) ? (valueMm / MM_PER_INCH) : valueMm;
    }

} // namespace OpenCMM