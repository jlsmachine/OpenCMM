// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QString>
#include <QStringList>
#include <QByteArray>
#include <Eigen/Dense>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <TopoDS_Shape.hxx>
#include <string>
#include <vector>
#include <map>

namespace OpenCMM {

    struct KinematicPrimitive {
        std::string type = "box";
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        Eigen::Vector3d size = Eigen::Vector3d::Zero();
    };

    struct KinematicLinkDef {
        std::string id;
        std::string parentId;
        std::string axis = "NONE";
        std::string stepFile;
        Eigen::Vector3d colorRgb{ 0.75, 0.77, 0.80 };
        std::map<std::string, Eigen::Vector3d> partColors;
        std::vector<KinematicPrimitive> primitives;
    };

    struct MachinePostConfig {
        std::string machineName = "Default Moving-Bridge CMM";
        std::string manufacturer = "OpenCMM";
        std::string controllerType = "PicoSerial";
        Eigen::Vector3d scaleMm{ 0.001, 0.001, 0.001 };
        Eigen::Vector3i sign{ 1, 1, 1 };
        Eigen::Vector3d travelMinMm{ 0.0, 0.0, -300.0 };
        Eigen::Vector3d travelMaxMm{ 500.0, 400.0, 0.0 };
        Eigen::Vector3d probeMountOffsetMm{ 0.0, 0.0, 0.0 };
        std::vector<KinematicLinkDef> links;
    };

    class CmmDigitalTwin {
    public:
        CmmDigitalTwin() = default;
        ~CmmDigitalTwin() = default;

        void setContext(const Handle(AIS_InteractiveContext)& context) { m_context = context; }
        void clearScene();
        void buildDefaultBridgeMachine();
        bool loadMachinePackage(const QString& packagePath);
        void updateMachinePosition(double xMm, double yMm, double zMm);

        void setMachineVisible(bool visible);
        QStringList getLinkIds() const;
        bool isLinkVisible(const QString& linkId) const;
        void setLinkVisible(const QString& linkId, bool visible);

        const MachinePostConfig& config() const { return m_config; }

    private:
        struct RuntimeNode {
            KinematicLinkDef def;
            int parentIndex = -1;
            bool visible = true;
            std::vector<Handle(AIS_Shape)> aisShapes;
        };

        bool parseManifestJson(const QByteArray& jsonBytes, const QString& baseDir);
        TopoDS_Shape buildLinkShape(
            const KinematicLinkDef& def,
            const QString& baseDir,
            std::vector<std::string>& outSolidNames
        );
        void rebuildAisNodes(const QString& baseDir);

        Handle(AIS_InteractiveContext) m_context;
        MachinePostConfig m_config;
        std::vector<RuntimeNode> m_nodes;
        Eigen::Vector3d m_lastPosMm = Eigen::Vector3d::Zero();
        bool m_visible = true;
    };

    using DigitalTwinCMM = CmmDigitalTwin;
    using MachineConfig = MachinePostConfig;

} // namespace OpenCMM