// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <QString>
#include <Eigen/Core>
#include "ui/SettingsDialog.h" // Ensures UnitSystem is known

namespace OpenCMM {

    class AppSettings {
    public:
        static void loadSettings(QString& machinePath, QString& serialPort, int& baudRate,
            Eigen::Vector3d& axisScale, Eigen::Vector3i& axisSign,
            UnitSystem& units, double& defaultTol, bool& audioEnabled);

        static void saveSettings(const QString& machinePath, const QString& serialPort, int baudRate,
            const Eigen::Vector3d& axisScale, const Eigen::Vector3i& axisSign,
            UnitSystem units, double defaultTol, bool audioEnabled);
    };

} // namespace OpenCMM