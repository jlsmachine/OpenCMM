// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <QString>
#include <Eigen/Core>
#include "ui/SettingsDialog.h" // Ensures UnitSystem is known

namespace OpenCMM {

    class AppSettings {
    public:
        static void loadSettings(
            QString& machinePath,
            QString& port,
            int& baud,
            Eigen::Vector3d& scale,
            Eigen::Vector3i& sign,
            UnitSystem& units,
            double& defaultTol,
            bool& audioEnabled,
            bool& isDarkMode
        );

        static void saveSettings(
            const QString& machinePath,
            const QString& port,
            int baud,
            const Eigen::Vector3d& scale,
            const Eigen::Vector3i& sign,
            UnitSystem units,
            double defaultTol,
            bool audioEnabled,
            bool isDarkMode
        );
        static void resetToDefaults();
    };

} // namespace OpenCMM