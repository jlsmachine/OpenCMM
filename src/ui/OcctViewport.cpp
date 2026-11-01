#include "ui/OcctViewport.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <algorithm>

#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>
#include <Quantity_Color.hxx>

#ifdef _WIN32
#include <WNT_Window.hxx>
#endif

namespace OpenCMM {

    OcctViewport::OcctViewport(QWidget* parent) : QWidget(parent) {
        setAttribute(Qt::WA_PaintOnScreen);
        setAttribute(Qt::WA_NoSystemBackground);
        setAttribute(Qt::WA_NativeWindow);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);
    }

    void OcctViewport::initOcct() {
        if (!m_view.IsNull()) {
            return;
        }

        Handle(Aspect_DisplayConnection) dispConn = new Aspect_DisplayConnection();
        m_graphicDriver = new OpenGl_GraphicDriver(dispConn);

        m_viewer = new V3d_Viewer(m_graphicDriver);
        m_viewer->SetDefaultLights();
        m_viewer->SetLightOn();

        m_context = new AIS_InteractiveContext(m_viewer);
        m_context->SetDisplayMode(AIS_Shaded, Standard_True);
        m_context->DefaultDrawer()->SetFaceBoundaryDraw(Standard_True);
        m_context->DefaultDrawer()->SetFaceBoundaryAspect(
            new Prs3d_LineAspect(Quantity_NOC_GRAY10, Aspect_TOL_SOLID, 1.0)
        );

        m_view = m_viewer->CreateView();
        m_view->SetBackgroundColor(Quantity_NOC_GRAY15);

#ifdef _WIN32
        Handle(WNT_Window) wntWindow = new WNT_Window(reinterpret_cast<Aspect_Handle>(winId()));
        m_view->SetWindow(wntWindow);
        if (!wntWindow->IsMapped()) {
            wntWindow->Map();
        }
#endif

        m_view->TriedronDisplay(
            Aspect_TOTP_LEFT_LOWER,
            Quantity_NOC_WHITE,
            0.1,
            V3d_ZBUFFER
        );

        createDemoGraniteScene();
        createProbeStylus();

        m_view->MustBeResized();
        m_view->FitAll();
    }

    void OcctViewport::createDemoGraniteScene() {
        TopoDS_Shape graniteBox = BRepPrimAPI_MakeBox(gp_Pnt(-300.0, -200.0, -80.0), 600.0, 400.0, 80.0).Shape();
        m_graniteShape = new AIS_Shape(graniteBox);
        m_graniteShape->SetColor(Quantity_Color(0.22, 0.24, 0.26, Quantity_TOC_RGB));
        m_graniteShape->Attributes()->SetFaceBoundaryDraw(Standard_True);
        m_context->Display(m_graniteShape, AIS_Shaded, -1, Standard_False);
    }

    void OcctViewport::createProbeStylus() {
        TopoDS_Shape rubySphere = BRepPrimAPI_MakeSphere(gp_Pnt(0.0, 0.0, 0.0), 1.5).Shape();
        m_probeRuby = new AIS_Shape(rubySphere);
        m_probeRuby->SetColor(Quantity_NOC_RED);

        gp_Ax2 stemAxis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
        TopoDS_Shape stemCyl = BRepPrimAPI_MakeCylinder(stemAxis, 0.75, 50.0).Shape();
        m_probeStem = new AIS_Shape(stemCyl);
        m_probeStem->SetColor(Quantity_NOC_GRAY85);

        m_context->Display(m_probeRuby, AIS_Shaded, -1, Standard_False);
        m_context->Display(m_probeStem, AIS_Shaded, -1, Standard_False);
    }

    void OcctViewport::updateProbePosition(double x, double y, double z) {
        if (m_context.IsNull() || m_probeRuby.IsNull() || m_probeStem.IsNull()) {
            return;
        }

        gp_Trsf trsf;
        trsf.SetTranslation(gp_Vec(x, y, z));
        TopLoc_Location loc(trsf);

        m_context->SetLocation(m_probeRuby, loc);
        m_context->SetLocation(m_probeStem, loc);
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::addHitPointMarker(double x, double y, double z) {
        if (m_context.IsNull()) {
            return;
        }

        TopoDS_Shape hitSphere = BRepPrimAPI_MakeSphere(gp_Pnt(x, y, z), 1.3).Shape();
        Handle(AIS_Shape) aisHit = new AIS_Shape(hitSphere);
        aisHit->SetColor(Quantity_NOC_CYAN1);

        m_context->Display(aisHit, AIS_Shaded, -1, Standard_True);
        m_activeHitMarkers.push_back(aisHit);
    }

    void OcctViewport::commitActiveHitMarkers() {
        if (m_context.IsNull()) {
            return;
        }
        for (const auto& marker : m_activeHitMarkers) {
            marker->SetColor(Quantity_NOC_GREEN1);
            m_context->Redisplay(marker, Standard_False);
            m_solvedHitMarkers.push_back(marker);
        }
        m_activeHitMarkers.clear();
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::clearHitMarkers() {
        if (m_context.IsNull()) {
            return;
        }
        for (const auto& marker : m_activeHitMarkers) {
            m_context->Remove(marker, Standard_False);
        }
        for (const auto& marker : m_solvedHitMarkers) {
            m_context->Remove(marker, Standard_False);
        }
        m_activeHitMarkers.clear();
        m_solvedHitMarkers.clear();
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::applyVisualStyleToShape(const Handle(AIS_Shape)& shape) {
        if (m_context.IsNull() || shape.IsNull()) {
            return;
        }

        if (m_displayMode == ModelDisplayMode::Wireframe) {
            m_context->SetDisplayMode(shape, AIS_WireFrame, Standard_False);
        }
        else {
            const bool drawEdges = (m_displayMode == ModelDisplayMode::ShadedWithEdges);
            shape->Attributes()->SetFaceBoundaryDraw(drawEdges ? Standard_True : Standard_False);
            m_context->SetDisplayMode(shape, AIS_Shaded, Standard_False);
        }

        m_context->SetTransparency(shape, m_transparency, Standard_False);
        m_context->Redisplay(shape, Standard_False);
    }

    void OcctViewport::setModelDisplayMode(ModelDisplayMode mode) {
        m_displayMode = mode;
        if (m_context.IsNull()) {
            return;
        }

        for (const auto& cad : m_cadShapes) {
            applyVisualStyleToShape(cad);
        }
        if (m_cadShapes.empty() && !m_graniteShape.IsNull() && m_graniteVisible) {
            applyVisualStyleToShape(m_graniteShape);
        }
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::setModelTransparency(double transparency) {
        m_transparency = std::clamp(transparency, 0.0, 0.90);
        if (m_context.IsNull()) {
            return;
        }

        for (const auto& cad : m_cadShapes) {
            m_context->SetTransparency(cad, m_transparency, Standard_False);
        }
        if (m_cadShapes.empty() && !m_graniteShape.IsNull() && m_graniteVisible) {
            m_context->SetTransparency(m_graniteShape, m_transparency, Standard_False);
        }
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::setGraniteVisible(bool visible) {
        m_graniteVisible = visible;
        if (m_context.IsNull() || m_graniteShape.IsNull()) {
            return;
        }

        if (visible) {
            m_context->Display(m_graniteShape, AIS_Shaded, -1, Standard_True);
        }
        else {
            m_context->Erase(m_graniteShape, Standard_True);
        }
    }

    void OcctViewport::paintEvent(QPaintEvent* /*event*/) {
        if (m_view.IsNull()) {
            initOcct();
        }
        if (!m_view.IsNull()) {
            m_view->Redraw();
        }
    }

    void OcctViewport::resizeEvent(QResizeEvent* /*event*/) {
        if (!m_view.IsNull()) {
            m_view->MustBeResized();
        }
    }

    void OcctViewport::loadStepFile(const QString& filepath) {
        if (m_view.IsNull()) {
            initOcct();
        }

        STEPControl_Reader reader;
        IFSelect_ReturnStatus status = reader.ReadFile(filepath.toUtf8().constData());
        if (status == IFSelect_RetDone) {
            reader.TransferRoots();
            TopoDS_Shape shape = reader.OneShape();
            Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
            aisShape->SetColor(Quantity_NOC_STEELBLUE3);

            m_context->Display(aisShape, AIS_Shaded, 0, Standard_False);
            m_cadShapes.push_back(aisShape);

            applyVisualStyleToShape(aisShape);
            m_context->Activate(aisShape, AIS_Shape::SelectionMode(TopAbs_FACE));
            m_view->FitAll();
        }
    }

    void OcctViewport::mousePressEvent(QMouseEvent* event) {
        m_lastMousePos = event->pos();
        m_pressMousePos = event->pos();

        if (event->button() == Qt::LeftButton && !m_view.IsNull()) {
            m_isRotating = true;
            m_view->StartRotation(event->pos().x(), event->pos().y());
        }
        else if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
            m_isPanning = true;
        }
    }

    void OcctViewport::mouseReleaseEvent(QMouseEvent* event) {
        if (event->button() == Qt::LeftButton) {
            m_isRotating = false;
            if ((event->pos() - m_pressMousePos).manhattanLength() <= 3 && !m_context.IsNull()) {
                m_context->SelectDetected();
                m_view->Redraw();
            }
        }
        if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
            m_isPanning = false;
        }
    }

    void OcctViewport::mouseMoveEvent(QMouseEvent* event) {
        if (m_view.IsNull() || m_context.IsNull()) {
            return;
        }

        const QPoint pos = event->pos();
        const int dx = pos.x() - m_lastMousePos.x();
        const int dy = pos.y() - m_lastMousePos.y();
        m_lastMousePos = pos;

        if (m_isRotating) {
            m_view->Rotation(pos.x(), pos.y());
            m_view->Redraw();
        }
        else if (m_isPanning) {
            m_view->Pan(dx, -dy);
            m_view->Redraw();
        }
        else {
            m_context->MoveTo(pos.x(), pos.y(), m_view, Standard_True);
        }
    }

    void OcctViewport::wheelEvent(QWheelEvent* event) {
        if (!m_view.IsNull()) {
            const double steps = event->angleDelta().y() / 120.0;
            const double zoomFactor = (steps > 0.0) ? (1.0 + steps * 0.15) : (1.0 / (1.0 - steps * 0.15));
            m_view->SetZoom(zoomFactor, Standard_True);
            m_view->Redraw();
        }
    }

} // namespace OpenCMM