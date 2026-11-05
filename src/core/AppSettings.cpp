// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/AppSettings.h"
#include <QSettings>

namespace OpenCMM {

    void AppSettings::loadSettings(QString& machinePath, QString& serialPort, int& baudRate,
        Eigen::Vector3d& axisScale, Eigen::Vector3i& axisSign,
        UnitSystem& units, double& defaultTol, bool& audioEnabled) {
        QSettings settings("JLS Machine LLC", "OpenCMM");

        machinePath = settings.value("machine/packagePath", "").toString();
        serialPort = settings.value("serial/portName", "").toString();
        baudRate = settings.value("serial/baudRate", 115200).toInt();

        axisScale.x() = settings.value("hardware/scaleX", 0.001).toDouble();
        axisScale.y() = settings.value("hardware/scaleY", 0.001).toDouble();
        axisScale.z() = settings.value("hardware/scaleZ", 0.001).toDouble();

        axisSign.x() = settings.value("hardware/signX", 1).toInt();
        axisSign.y() = settings.value("hardware/signY", 1).toInt();
        axisSign.z() = settings.value("hardware/signZ", 1).toInt();

        units = static_cast<UnitSystem>(settings.value("prefs/unitSystem", 0).toInt());
        defaultTol = settings.value("prefs/defaultTolerance", 0.025).toDouble();
        audioEnabled = settings.value("prefs/audioChimes", true).toBool();
    }

    void AppSettings::saveSettings(const QString& machinePath, const QString& serialPort, int baudRate,
        const Eigen::Vector3d& axisScale, const Eigen::Vector3i& axisSign,
        UnitSystem units, double defaultTol, bool audioEnabled) {
        QSettings settings("JLS Machine LLC", "OpenCMM");

        settings.setValue("machine/packagePath", machinePath);
        settings.setValue("serial/portName", serialPort);
        settings.setValue("serial/baudRate", baudRate);

        settings.setValue("hardware/scaleX", axisScale.x());
        settings.setValue("hardware/scaleY", axisScale.y());
        settings.setValue("hardware/scaleZ", axisScale.z());

        settings.setValue("hardware/signX", axisSign.x());
        settings.setValue("hardware/signY", axisSign.y());
        settings.setValue("hardware/signZ", axisSign.z());

        settings.setValue("prefs/unitSystem", static_cast<int>(units));
        settings.setValue("prefs/defaultTolerance", defaultTol);
        settings.setValue("prefs/audioChimes", audioEnabled);
    }

} // namespace OpenCMM