#pragma once
#include <QWidget>
#include <QPoint>
#include <vector>
#include <string>
#include <functional>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>

#include "cad/CmmDigitalTwin.h"

class QPushButton;
class QSlider;
class QLabel;

namespace OpenCMM {

    enum class ModelDisplayMode {
        ShadedWithEdges = 0,
        ShadedOnly = 1,
        Wireframe = 2
    };

    struct CadFaceInfo {
        bool success = false;
        std::string surfaceType = "Unknown";
        double nominalDiameter = 0.0;
        double nominalRadius = 0.0;
        bool isInternal = false;
        gp_Pnt centerOrCentroid{ 0.0, 0.0, 0.0 };
        gp_Dir normalOrAxis{ 0.0, 0.0, 1.0 };
    };

    class OcctViewport : public QWidget {
        Q_OBJECT
    public:
        explicit OcctViewport(QWidget* parent = nullptr);
        ~OcctViewport() override = default;

        Handle(AIS_InteractiveContext) getContext() const { return m_context; }

        void loadStepFile(const QString& filepath);
        bool hasCadModel() const { return !m_cadShapes.empty(); }
        TopoDS_Shape getCombinedCadShape() const;

        // Digital Twin Machine Package Controls
        bool loadMachinePackage(const QString& packagePath);
        void resetDefaultMachine();
        const MachinePostConfig& machineConfig() const { return m_digitalTwin.config(); }

        // CAD Model Orientation & Transformation Controls
        bool getCadBoundingBox(double& xmin, double& ymin, double& zmin,
            double& xmax, double& ymax, double& zmax) const;
        void translateCadModels(double dxMm, double dyMm, double dzMm);
        void rotateCadModels(double rxDeg, double ryDeg, double rzDeg, bool aroundPartCenter = true);
        void dropCadModelsToGranite(double targetZMinMm = 0.0);
        void centerCadModelsXY(double targetXMm = 0.0, double targetYMm = 0.0);
        void snapCadMinCornerToOrigin();
        bool alignSelectedFaceToZ(bool pointNormalUp = true);
        void resetCadModelsTransform();

        void updateProbePosition(double x, double y, double z);
        void addHitPointMarker(double x, double y, double z);
        void addDeviationPointMarker(double x, double y, double z, double deviationMm, double plusTolMm);
        void commitActiveHitMarkers();
        void clearHitMarkers();

        void setModelDisplayMode(ModelDisplayMode mode);
        void setModelTransparency(double transparency);
        void setGraniteVisible(bool visible);
        void setMachineVisible(bool visible);

        void setFaceSelectedCallback(std::function<void(const CadFaceInfo&)> cb) { m_faceSelectedCallback = std::move(cb); }
        CadFaceInfo inspectSelectedCadFace() const;
        const CadFaceInfo& lastSelectedFace() const { return m_lastSelectedFace; }

        QPaintEngine* paintEngine() const override { return nullptr; }
         
    protected:
        void paintEvent(QPaintEvent* event) override;
        void resizeEvent(QResizeEvent* event) override;

        void mousePressEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void wheelEvent(QWheelEvent* event) override;

    private:
        void initOcct();
        void setupOverlayHud();
        void createDemoGraniteScene();
        void createProbeStylus();
        void applyVisualStyleToShape(const Handle(AIS_Shape)& shape);
        void applyTransformToCadModels(const gp_Trsf& trsf);

        Handle(Graphic3d_GraphicDriver) m_graphicDriver;
        Handle(V3d_Viewer) m_viewer;
        Handle(V3d_View) m_view;
        Handle(AIS_InteractiveContext) m_context;

        CmmDigitalTwin m_digitalTwin;

        Handle(AIS_Shape) m_graniteShape;
        std::vector<Handle(AIS_Shape)> m_cadShapes;
        std::vector<TopoDS_Shape> m_originalCadShapes;

        Handle(AIS_Shape) m_probeRuby;
        Handle(AIS_Shape) m_probeStem;
        std::vector<Handle(AIS_Shape)> m_activeHitMarkers;
        std::vector<Handle(AIS_Shape)> m_solvedHitMarkers;

        ModelDisplayMode m_displayMode = ModelDisplayMode::ShadedWithEdges;
        double m_transparency = 0.0;
        bool m_graniteVisible = true;

        CadFaceInfo m_lastSelectedFace;
        std::function<void(const CadFaceInfo&)> m_faceSelectedCallback;

        QWidget* m_hudContainer = nullptr;
        QPushButton* m_btnShadedEdges = nullptr;
        QPushButton* m_btnShadedOnly = nullptr;
        QPushButton* m_btnWireframe = nullptr;
        QPushButton* m_btnGranite = nullptr;
        QPushButton* m_btnMachine = nullptr;
        QSlider* m_verticalSlider = nullptr;
        QLabel* m_sliderValLbl = nullptr;

        QPoint m_lastMousePos;
        QPoint m_pressMousePos;
        bool m_isPanning = false;
        bool m_isRotating = false;
    };

} // namespace OpenCMM