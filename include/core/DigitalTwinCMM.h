// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QString>
#include <QStringList>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "cad/CmmDigitalTwin.h"

#ifndef VCPKG_TOOLCHAIN
// Forward declare if OCC headers aren't explicitly needed in this translation unit
class AIS_InteractiveContext;
#else
#include <AIS_InteractiveContext.hxx>
#endif

namespace OpenCMM {

    struct MachineConfig {
        std::string machineName = "Helmel Checkmaster / Moving Bridge CMM";
        std::string manufacturer = "Helmel / JLS Machine";
        Eigen::Vector3d scaleMm{ 0.001, 0.001, 0.001 };
        Eigen::Vector3i sign{ 1, 1, 1 };
        Eigen::Vector3d probeMountOffsetMm{ 0.0, 0.0, 0.0 };
    };

    class DigitalTwinCMM {
    public:
        DigitalTwinCMM() = default;
        ~DigitalTwinCMM() = default;

        template<typename T>
        void setContext(const T& context) { (void)context; }

        bool loadMachinePackage(const QString& path) { (void)path; return true; }
        void buildDefaultBridgeMachine() {}
        void updateMachinePosition(double x, double y, double z) { (void)x; (void)y; (void)z; }

        QStringList getLinkIds() const { return { "granite_base", "bridge_gantry", "z_quill" }; }
        bool isLinkVisible(const QString& id) const { (void)id; return true; }
        void setLinkVisible(const QString& id, bool visible) { (void)id; (void)visible; }
        void setMachineVisible(bool visible) { (void)visible; }

        const MachineConfig& config() const { return m_config; }

    private:
        MachineConfig m_config;
    };

} // namespace OpenCMM