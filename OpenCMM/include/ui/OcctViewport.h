#pragma once
#include <QWidget>
#include <QPoint>
#include <vector>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <Graphic3d_GraphicDriver.hxx>

namespace OpenCMM {

    class OcctViewport : public QWidget {
        Q_OBJECT
    public:
        explicit OcctViewport(QWidget* parent = nullptr);
        ~OcctViewport() override = default;

        Handle(AIS_InteractiveContext) getContext() const { return m_context; }

        void loadStepFile(const QString& filepath);
        void updateProbePosition(double x, double y, double z);
        void addHitPointMarker(double x, double y, double z);
        void clearHitMarkers();

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
        void createDemoGraniteScene();
        void createProbeStylus();

        Handle(Graphic3d_GraphicDriver) m_graphicDriver;
        Handle(V3d_Viewer) m_viewer;
        Handle(V3d_View) m_view;
        Handle(AIS_InteractiveContext) m_context;

        Handle(AIS_Shape) m_probeRuby;
        Handle(AIS_Shape) m_probeStem;
        std::vector<Handle(AIS_Shape)> m_hitMarkers;

        QPoint m_lastMousePos;
        QPoint m_pressMousePos;
        bool m_isPanning = false;
        bool m_isRotating = false;
    };

} // namespace OpenCMM