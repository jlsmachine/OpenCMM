#pragma once
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <QString>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace OpenCMM {

    enum class KinematicAxis {
        None,
        AxisX,
        AxisY,
        AxisZ
    };

    struct KinematicLink {
        std::string id;
        std::string parentId;
        int parentIndex = -1;
        KinematicAxis axis = KinematicAxis::None;
        Handle(AIS_Shape) aisShape;
        gp_Trsf worldTrsf;
    };

    struct MachinePostConfig {
        QString machineName = "Default Moving-Bridge CMM";
        QString manufacturer = "OpenCMM";
        Eigen::Vector3d encoderScaleMm{ 0.0005, 0.0005, 0.0005 };
        Eigen::Vector3i axisSign{ 1, 1, 1 };
        Eigen::Vector3d travelMinMm{ -250.0, -150.0, -200.0 };
        Eigen::Vector3d travelMaxMm{ 250.0, 150.0, 200.0 };
    };

    class CmmDigitalTwin {
    public:
        CmmDigitalTwin() = default;

        void setContext(const Handle(AIS_InteractiveContext)& ctx) { m_ctx = ctx; }

        // Load either a packed .cmm zip archive or a .cmm.json manifest file
        bool loadMachinePackage(const QString& packagePath);

        // Build the default procedural 3-axis moving-bridge CMM
        void buildDefaultBridgeMachine();

        // Real-time kinematic chain transform update
        void updateMachinePosition(double xMm, double yMm, double zMm);

        void setMachineVisible(bool visible);
        bool isMachineVisible() const { return m_visible; }

        const MachinePostConfig& config() const { return m_config; }

    private:
        bool parseJsonConfig(const QByteArray& jsonData, const QString& workingDir);
        void clearLinks();
        void resolveParentIndices();

        Handle(AIS_InteractiveContext) m_ctx;
        MachinePostConfig m_config;
        std::vector<KinematicLink> m_links;
        bool m_visible = true;
        double m_lastX = 0.0;
        double m_lastY = 0.0;
        double m_lastZ = 0.0;
    };

} // namespace OpenCMM