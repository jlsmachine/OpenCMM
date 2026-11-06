// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
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
class QVBoxLayout;

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

    struct CadSurfaceHit {
        bool success = false;
        gp_Pnt surfacePoint{ 0.0, 0.0, 0.0 };
        gp_Dir surfaceNormal{ 0.0, 0.0, 1.0 };
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

        // DMIS 5.3 Guided Target & Approach Vector Visuals
        void setProgramTargetMarkers(const std::vector<gp_Pnt>& surfacePts,
            const std::vector<gp_Dir>& normals,
            int activeIndex,
            double probeRadiusMm = 1.5);
        void clearProgramTargetMarkers();

        void setModelDisplayMode(ModelDisplayMode mode);
        void setModelTransparency(double transparency);
        void setGraniteVisible(bool visible);
        void setMachineVisible(bool visible);
        void setProbeStylusVisible(bool visible);

        void setFaceSelectedCallback(std::function<void(const CadFaceInfo&)> cb) { m_faceSelectedCallback = std::move(cb); }
        void setSurfaceHitCallback(std::function<void(const CadSurfaceHit&)> cb) { m_surfaceHitCallback = std::move(cb); }
        CadFaceInfo inspectSelectedCadFace();
        const CadFaceInfo& lastSelectedFace() const { return m_lastSelectedFace; }
        bool hasSelectedCadFace() const { return m_lastSelectedFace.success && !m_lastSelectedTopoFace.IsNull(); }
        CadSurfaceHit pickSurfacePointOnSelectedFace(int screenX, int screenY) const;
        std::vector<CadSurfaceHit> sampleSelectedCadFace(int numPoints = 0) const;

        QPaintEngine* paintEngine() const override { return nullptr; }

        // Inside public: section of OcctViewport (below clearHitMarkers();)
        void addFittedPlaneVisual(int featureId,
            double cx, double cy, double cz,
            double nx, double ny, double nz,
            double sizeMm = 60.0);
        void addFittedSphereVisual(int featureId,
            double cx, double cy, double cz,
            double radiusMm);
        void addFittedCircleVisual(int featureId,
            double cx, double cy, double cz,
            double nx, double ny, double nz,
            double radiusMm);
        void addFittedLineVisual(int featureId,
            double x1, double y1, double z1,
            double x2, double y2, double z2);
        void highlightFittedFeature(int featureId);
        void setFittedFeaturesWireframe(bool wireframe);
        bool areFittedFeaturesWireframe() const { return m_fittedFeaturesWireframe; }
        void clearFittedFeatures();
        void setBackgroundColor(bool dark);
        void setTheme(bool dark);
        

    protected:
        void paintEvent(QPaintEvent* event) override;
        void resizeEvent(QResizeEvent* event) override;
        void showEvent(QShowEvent* event) override;

        void mousePressEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void wheelEvent(QWheelEvent* event) override;

    private:
        void initOcct();
        void setupOverlayHud();
        void rebuildComponentHudButtons();
        void createProbeStylus();
        void applyVisualStyleToShape(const Handle(AIS_Shape)& shape);
        void applyTransformToCadModels(const gp_Trsf& trsf);

        Handle(Graphic3d_GraphicDriver) m_graphicDriver;
        Handle(V3d_Viewer) m_viewer;
        Handle(V3d_View) m_view;
        Handle(AIS_InteractiveContext) m_context;

        CmmDigitalTwin m_digitalTwin;

        std::vector<Handle(AIS_Shape)> m_cadShapes;
        std::vector<TopoDS_Shape> m_originalCadShapes;

        Handle(AIS_Shape) m_probeRuby;
        Handle(AIS_Shape) m_probeStem;
        bool m_probeStylusVisible = true;

        std::vector<Handle(AIS_Shape)> m_activeHitMarkers;
        std::vector<Handle(AIS_Shape)> m_solvedHitMarkers;
        std::vector<Handle(AIS_Shape)> m_programTargetMarkers;

        ModelDisplayMode m_displayMode = ModelDisplayMode::ShadedWithEdges;
        double m_transparency = 0.0;

        CadFaceInfo m_lastSelectedFace;
        TopoDS_Face m_lastSelectedTopoFace;
        std::function<void(const CadFaceInfo&)> m_faceSelectedCallback;
        std::function<void(const CadSurfaceHit&)> m_surfaceHitCallback;

        QWidget* m_hudContainer = nullptr;
        QPushButton* m_btnShadedEdges = nullptr;
        QPushButton* m_btnShadedOnly = nullptr;
        QPushButton* m_btnWireframe = nullptr;
        QPushButton* m_btnProbeStylus = nullptr;
        QWidget* m_linkButtonsContainer = nullptr;
        QVBoxLayout* m_linkButtonsLayout = nullptr;
        std::vector<QPushButton*> m_linkButtons;
        QSlider* m_verticalSlider = nullptr;
        QLabel* m_sliderValLbl = nullptr;

        QPoint m_lastMousePos;
        QPoint m_pressMousePos;
        bool m_isPanning = false;
        bool m_isRotating = false;

        // Inside private: section of OcctViewport
        struct FittedFeatureEntry {
            int featureId = -1;
            Handle(AIS_Shape) shape;
            Quantity_Color baseColor;

            std::vector<FittedFeatureEntry> m_fittedFeatureShapes;
            bool m_fittedFeaturesWireframe = false;
            bool m_isDarkMode = true;
        
        };

        void displayFittedFeatureShape(int featureId, const TopoDS_Shape& shape, const Quantity_Color& color);

        std::vector<FittedFeatureEntry> m_fittedFeatureShapes;
        bool m_fittedFeaturesWireframe = false;
        bool m_isDarkMode = true;
    };

} // namespace OpenCMM