// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>
#include <vector>
#include <functional>
#include <QString>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <AIS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <Quantity_Color.hxx>
#include <Eigen/Dense>

#include "cad/CmmDigitalTwin.h"

class QPushButton;
class QSlider;
class QLabel;
class QVBoxLayout;
class QPaintEngine;

namespace OpenCMM {

    struct CadFaceInfo {
        bool success = false;
        std::string surfaceType;
        double nominalRadius = 0.0;
        double nominalDiameter = 0.0;
        gp_Pnt centerOrCentroid;
        gp_Dir normalOrAxis;
        bool isInternal = false;
        TopoDS_Face face;
    };

    struct CadSurfaceHit {
        bool success = false;
        gp_Pnt surfacePoint;
        gp_Dir surfaceNormal;
    };

    enum class ModelDisplayMode {
        ShadedWithEdges,
        ShadedOnly,
        Wireframe
    };

    struct FittedFeatureVisual {
        int featureId = -1;
        Handle(AIS_Shape) shape;
        Quantity_Color baseColor;
    };

    class OcctViewport : public QWidget {
        Q_OBJECT
    public:
        explicit OcctViewport(QWidget* parent = nullptr);
        ~OcctViewport() override = default;

        // Required by Qt when Qt::WA_PaintOnScreen is set to prevent QWidget::paintEngine warnings
        QPaintEngine* paintEngine() const override { return nullptr; }

        const MachinePostConfig& machineConfig() const { return m_digitalTwin.config(); }

        void loadStepFile(const QString& filepath);
        bool loadMachinePackage(const QString& packagePath);
        void resetDefaultMachine();

        void updateProbePosition(double x, double y, double z);
        void addHitPointMarker(double x, double y, double z);
        void addDeviationPointMarker(double x, double y, double z, double deviationMm, double plusTolMm);
        void commitActiveHitMarkers();
        void clearHitMarkers();

        void setProgramTargetMarkers(const std::vector<gp_Pnt>& surfacePts, const std::vector<gp_Dir>& normals, int activeIndex, double probeRadiusMm);
        void clearProgramTargetMarkers();

        void setPcsTriadVisual(const Eigen::Matrix4d& pcsMatrix, double axisLengthMm = 45.0);
        void clearPcsTriadVisual();

        bool hasCadModel() const { return !m_cadShapes.empty(); }
        TopoDS_Shape getCombinedCadShape() const;
        bool getCadBoundingBox(double& xmin, double& ymin, double& zmin, double& xmax, double& ymax, double& zmax) const;

        void translateCadModels(double dxMm, double dyMm, double dzMm);
        void rotateCadModels(double rxDeg, double ryDeg, double rzDeg, bool aroundPartCenter = true);
        void dropCadModelsToGranite(double targetZMinMm = 0.0);
        void centerCadModelsXY(double targetXMm = 0.0, double targetYMm = 0.0);
        void snapCadMinCornerToOrigin();
        bool alignSelectedFaceToZ(bool pointNormalUp = true);
        void resetCadModelsTransform();

        bool hasSelectedCadFace() const { return m_lastSelectedFace.success; }
        const CadFaceInfo& lastSelectedFace() const { return m_lastSelectedFace; }
        std::vector<CadSurfaceHit> sampleSelectedCadFace(int numPointsPerRing = 4);

        void setFaceSelectedCallback(std::function<void(const CadFaceInfo&)> cb) { m_faceSelectedCallback = cb; }
        void setSurfaceHitCallback(std::function<void(const CadSurfaceHit&)> cb) { m_surfaceHitCallback = cb; }

        void setModelDisplayMode(ModelDisplayMode mode);
        void setModelTransparency(double transparency);
        void setGraniteVisible(bool visible);
        void setMachineVisible(bool visible);
        void setProbeStylusVisible(bool visible);

        void addFittedPlaneVisual(int featureId, double cx, double cy, double cz, double nx, double ny, double nz, double sizeMm = 50.0);
        void addFittedSphereVisual(int featureId, double cx, double cy, double cz, double radiusMm);
        void addFittedCircleVisual(int featureId, double cx, double cy, double cz, double nx, double ny, double nz, double radiusMm);
        void addFittedLineVisual(int featureId, double x1, double y1, double z1, double x2, double y2, double z2);
        void highlightFittedFeature(int featureId);
        void setFittedFeaturesWireframe(bool wireframe);
        void clearFittedFeatures();

        void setBackgroundColor(bool dark);
        void setTheme(bool dark);
        void setProbeCollisionState(bool collision, bool warning);

    protected:
        void paintEvent(QPaintEvent* event) override;
        void resizeEvent(QResizeEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void wheelEvent(QWheelEvent* event) override;
        void showEvent(QShowEvent* event) override;

    private:
        void initOcct();
        void createProbeStylus();
        void applyVisualStyleToShape(const Handle(AIS_Shape)& shape);
        void applyTransformToCadModels(const gp_Trsf& trsf);
        CadFaceInfo inspectSelectedCadFace();
        CadSurfaceHit pickSurfacePointOnSelectedFace(int screenX, int screenY) const;
        void setupOverlayHud();
        void rebuildComponentHudButtons();
        void displayFittedFeatureShape(int featureId, const TopoDS_Shape& shape, const Quantity_Color& color);

        Handle(OpenGl_GraphicDriver) m_graphicDriver;
        Handle(V3d_Viewer) m_viewer;
        Handle(V3d_View) m_view;
        Handle(AIS_InteractiveContext) m_context;

        std::vector<Handle(AIS_Shape)> m_cadShapes;
        std::vector<TopoDS_Shape> m_originalCadShapes;
        std::vector<Handle(AIS_Shape)> m_activeHitMarkers;
        std::vector<Handle(AIS_Shape)> m_solvedHitMarkers;
        std::vector<Handle(AIS_Shape)> m_programTargetMarkers;
        std::vector<Handle(AIS_Shape)> m_pcsTriadShapes;
        std::vector<FittedFeatureVisual> m_fittedFeatureShapes;

        Handle(AIS_Shape) m_probeRuby;
        Handle(AIS_Shape) m_probeStem;
        bool m_probeStylusVisible = true;

        CmmDigitalTwin m_digitalTwin;
        CadFaceInfo m_lastSelectedFace;
        TopoDS_Face m_lastSelectedTopoFace;

        std::function<void(const CadFaceInfo&)> m_faceSelectedCallback;
        std::function<void(const CadSurfaceHit&)> m_surfaceHitCallback;

        QPoint m_lastMousePos;
        QPoint m_pressMousePos;
        bool m_isRotating = false;
        bool m_isPanning = false;

        ModelDisplayMode m_displayMode = ModelDisplayMode::ShadedWithEdges;
        double m_transparency = 0.0;
        bool m_fittedFeaturesWireframe = false;
        bool m_isDarkMode = true;

        QWidget* m_hudContainer = nullptr;
        QPushButton* m_btnShadedEdges = nullptr;
        QPushButton* m_btnShadedOnly = nullptr;
        QPushButton* m_btnWireframe = nullptr;
        QPushButton* m_btnProbeStylus = nullptr;
        QSlider* m_verticalSlider = nullptr;
        QLabel* m_sliderValLbl = nullptr;

        QWidget* m_linkButtonsContainer = nullptr;
        QVBoxLayout* m_linkButtonsLayout = nullptr;
        std::vector<QPushButton*> m_linkButtons;
    };

} // namespace OpenCMM