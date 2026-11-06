// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/AppSettings.h"
#include <QSettings>

namespace OpenCMM {

    void AppSettings::loadSettings(
        QString& machinePath, QString& port, int& baud,
        Eigen::Vector3d& scale, Eigen::Vector3i& sign,
        UnitSystem& units, double& defaultTol, bool& audioEnabled,
        bool& isDarkMode
    ) {
        QSettings settings("JLS Machine", "OpenCMM");
        machinePath = settings.value("machinePath", "").toString();
        port = settings.value("port", "").toString();
        baud = settings.value("baud", 115200).toInt();
        // ... load scale and sign ...
        units = static_cast<UnitSystem>(settings.value("unitSystem", static_cast<int>(UnitSystem::Millimeters)).toInt());
        defaultTol = settings.value("defaultTolerance", 0.025).toDouble();
        audioEnabled = settings.value("audioEnabled", true).toBool();
        isDarkMode = settings.value("isDarkMode", true).toBool(); // Default to dark if unset
    }

    void AppSettings::saveSettings(
        const QString& machinePath, const QString& port, int baud,
        const Eigen::Vector3d& scale, const Eigen::Vector3i& sign,
        UnitSystem units, double defaultTol, bool audioEnabled,
        bool isDarkMode
    ) {
        QSettings settings("JLS Machine", "OpenCMM");
        settings.setValue("machinePath", machinePath);
        settings.setValue("port", port);
        settings.setValue("baud", baud);
        // ... save scale and sign ...
        settings.setValue("unitSystem", static_cast<int>(units));
        settings.setValue("defaultTolerance", defaultTol);
        settings.setValue("audioEnabled", audioEnabled);
        settings.setValue("isDarkMode", isDarkMode);
    }

    void AppSettings::resetToDefaults() {
        QSettings settings("JLS Machine", "OpenCMM");
        settings.clear();
        settings.sync();
    }

} // namespace OpenCMM