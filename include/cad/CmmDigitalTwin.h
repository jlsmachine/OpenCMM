#pragma once
#include <QString>
#include <QStringList>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <TopoDS_Shape.hxx>

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
        Eigen::Vector3d colorRgb{ 0.5, 0.5, 0.5 };
        std::string stepFile;
        std::map<std::string, Eigen::Vector3d> partColors;
        std::vector<KinematicPrimitive> primitives;
    };

    struct MachinePostConfig {
        std::string machineName = "Default Bridge CMM";
        std::string manufacturer = "OpenCMM";
        std::string controllerType = "PicoSerial";
        Eigen::Vector3d scaleMm{ 0.0005, 0.0005, 0.0005 };
        Eigen::Vector3i sign{ 1, 1, 1 };
        Eigen::Vector3d travelMinMm{ -200.0, -200.0, -250.0 };
        Eigen::Vector3d travelMaxMm{ 200.0, 200.0, 0.0 };
        Eigen::Vector3d probeMountOffsetMm{ 0.0, 0.0, 0.0 };
        std::vector<KinematicLinkDef> links;
    };

    class CmmDigitalTwin {
    public:
        CmmDigitalTwin() = default;
        ~CmmDigitalTwin() = default;

        void setContext(const Handle(AIS_InteractiveContext)& ctx) { m_context = ctx; }

        void buildDefaultBridgeMachine();
        bool loadMachinePackage(const QString& packagePath);
        void clearScene();

        void updateMachinePosition(double xMm, double yMm, double zMm);

        void setMachineVisible(bool visible);
        bool isMachineVisible() const { return m_visible; }

        QStringList getLinkIds() const;
        bool isLinkVisible(const QString& linkId) const;
        void setLinkVisible(const QString& linkId, bool visible);

        const MachinePostConfig& config() const { return m_config; }

    private:
        struct RuntimeNode {
            KinematicLinkDef def;
            std::vector<Handle(AIS_Shape)> aisShapes; // Vector of individual solid AIS shapes
            int parentIndex = -1;
            bool visible = true;
        };

        bool parseManifestJson(const QByteArray& jsonBytes, const QString& baseDir);
        TopoDS_Shape buildLinkShape(const KinematicLinkDef& def, const QString& baseDir, std::vector<std::string>& outSolidNames);
        void rebuildAisNodes(const QString& baseDir);

        Handle(AIS_InteractiveContext) m_context;
        MachinePostConfig m_config;
        std::vector<RuntimeNode> m_nodes;
        bool m_visible = true;
        Eigen::Vector3d m_lastPosMm = Eigen::Vector3d::Zero();
    };

} // namespace OpenCMM