#pragma once
#include <QWidget>
#include <QPoint>
#include <vector>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <Graphic3d_GraphicDriver.hxx>

class QPushButton;
class QSlider;
class QLabel;

namespace OpenCMM {

    enum class ModelDisplayMode {
        ShadedWithEdges = 0,
        ShadedOnly = 1,
        Wireframe = 2
    };

    class OcctViewport : public QWidget {
        Q_OBJECT
    public:
        explicit OcctViewport(QWidget* parent = nullptr);
        ~OcctViewport() override = default;

        Handle(AIS_InteractiveContext) getContext() const { return m_context; }

        void loadStepFile(const QString& filepath);
        void updateProbePosition(double x, double y, double z);
        void addHitPointMarker(double x, double y, double z);
        void commitActiveHitMarkers();
        void clearHitMarkers();

        void setModelDisplayMode(ModelDisplayMode mode);
        void setModelTransparency(double transparency);
        void setGraniteVisible(bool visible);

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

        Handle(Graphic3d_GraphicDriver) m_graphicDriver;
        Handle(V3d_Viewer) m_viewer;
        Handle(V3d_View) m_view;
        Handle(AIS_InteractiveContext) m_context;

        Handle(AIS_Shape) m_graniteShape;
        std::vector<Handle(AIS_Shape)> m_cadShapes;

        Handle(AIS_Shape) m_probeRuby;
        Handle(AIS_Shape) m_probeStem;
        std::vector<Handle(AIS_Shape)> m_activeHitMarkers;
        std::vector<Handle(AIS_Shape)> m_solvedHitMarkers;

        ModelDisplayMode m_displayMode = ModelDisplayMode::ShadedWithEdges;
        double m_transparency = 0.0;
        bool m_graniteVisible = true;

        // Overlay HUD Widget container
        QWidget* m_hudContainer = nullptr;
        QPushButton* m_btnShadedEdges = nullptr;
        QPushButton* m_btnShadedOnly = nullptr;
        QPushButton* m_btnWireframe = nullptr;
        QPushButton* m_btnGranite = nullptr;
        QSlider* m_verticalSlider = nullptr;
        QLabel* m_sliderValLbl = nullptr;

        QPoint m_lastMousePos;
        QPoint m_pressMousePos;
        bool m_isPanning = false;
        bool m_isRotating = false;
    };

} // namespace OpenCMM