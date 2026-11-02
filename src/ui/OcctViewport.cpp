#include "ui/OcctViewport.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <algorithm>
#include <cmath>

#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <Quantity_Color.hxx>

#ifdef _WIN32
#include <WNT_Window.hxx>
#endif

namespace OpenCMM {

    namespace {
        constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    }

    OcctViewport::OcctViewport(QWidget* parent) : QWidget(parent) {
        setAttribute(Qt::WA_PaintOnScreen);
        setAttribute(Qt::WA_NoSystemBackground);
        setAttribute(Qt::WA_NativeWindow);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);

        setupOverlayHud();
    }

    void OcctViewport::setupOverlayHud() {
        m_hudContainer = new QWidget(this);
        m_hudContainer->setStyleSheet(
            "QWidget { background-color: rgba(33, 37, 43, 220); border: 1px solid #3a3f4b; border-radius: 6px; }"
            "QPushButton { background-color: #282c34; color: #abb2bf; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 8px; font-weight: bold; font-size: 11px; min-width: 105px; }"
            "QPushButton:hover { background-color: #3a3f4b; color: white; }"
            "QPushButton:checked { background-color: #2d5a88; color: white; border: 1px solid #61afef; }"
            "QSlider::groove:vertical { background: #1e2227; width: 6px; border-radius: 3px; }"
            "QSlider::handle:vertical { background: #61afef; height: 16px; margin: 0 -5px; border-radius: 3px; }"
            "QLabel { color: #abb2bf; font-size: 10px; font-weight: bold; background: transparent; border: none; }"
        );

        auto* hudLayout = new QVBoxLayout(m_hudContainer);
        hudLayout->setContentsMargins(8, 8, 8, 8);
        hudLayout->setSpacing(6);

        auto* titleLbl = new QLabel("VIEW HUD", m_hudContainer);
        titleLbl->setAlignment(Qt::AlignCenter);
        hudLayout->addWidget(titleLbl);

        m_btnShadedEdges = new QPushButton("Shaded + Edges", m_hudContainer);
        m_btnShadedEdges->setCheckable(true);
        m_btnShadedEdges->setChecked(true);
        connect(m_btnShadedEdges, &QPushButton::clicked, this, [this]() {
            setModelDisplayMode(ModelDisplayMode::ShadedWithEdges);
            });
        hudLayout->addWidget(m_btnShadedEdges);

        m_btnShadedOnly = new QPushButton("Shaded Only", m_hudContainer);
        m_btnShadedOnly->setCheckable(true);
        connect(m_btnShadedOnly, &QPushButton::clicked, this, [this]() {
            setModelDisplayMode(ModelDisplayMode::ShadedOnly);
            });
        hudLayout->addWidget(m_btnShadedOnly);

        m_btnWireframe = new QPushButton("Wireframe", m_hudContainer);
        m_btnWireframe->setCheckable(true);
        connect(m_btnWireframe, &QPushButton::clicked, this, [this]() {
            setModelDisplayMode(ModelDisplayMode::Wireframe);
            });
        hudLayout->addWidget(m_btnWireframe);

        m_btnGranite = new QPushButton("Granite Table", m_hudContainer);
        m_btnGranite->setCheckable(true);
        m_btnGranite->setChecked(true);
        connect(m_btnGranite, &QPushButton::clicked, this, [this]() {
            setGraniteVisible(m_btnGranite->isChecked());
            });
        hudLayout->addWidget(m_btnGranite);

        m_btnMachine = new QPushButton("Machine Bridge", m_hudContainer);
        m_btnMachine->setCheckable(true);
        m_btnMachine->setChecked(true);
        connect(m_btnMachine, &QPushButton::clicked, this, [this]() {
            setMachineVisible(m_btnMachine->isChecked());
            });
        hudLayout->addWidget(m_btnMachine);

        auto* sliderLayout = new QVBoxLayout();
        sliderLayout->setSpacing(3);
        auto* transTitle = new QLabel("Trans: 0%", m_hudContainer);
        transTitle->setAlignment(Qt::AlignCenter);
        m_sliderValLbl = transTitle;

        m_verticalSlider = new QSlider(Qt::Vertical, m_hudContainer);
        m_verticalSlider->setRange(0, 90);
        m_verticalSlider->setValue(0);
        m_verticalSlider->setFixedHeight(90);
        connect(m_verticalSlider, &QSlider::valueChanged, this, [this](int val) {
            m_sliderValLbl->setText(QString("Trans: %1%").arg(val));
            setModelTransparency(static_cast<double>(val) / 100.0);
            });

        sliderLayout->addWidget(transTitle);
        sliderLayout->addWidget(m_verticalSlider, 0, Qt::AlignHCenter);
        hudLayout->addLayout(sliderLayout);

        m_hudContainer->adjustSize();
        m_hudContainer->raise();
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

        m_digitalTwin.setContext(m_context);
        m_digitalTwin.buildDefaultBridgeMachine();

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

    bool OcctViewport::loadMachinePackage(const QString& packagePath) {
        if (m_view.IsNull()) {
            initOcct();
        }
        bool ok = m_digitalTwin.loadMachinePackage(packagePath);
        if (ok && !m_view.IsNull()) {
            m_view->FitAll();
        }
        return ok;
    }

    void OcctViewport::resetDefaultMachine() {
        if (m_view.IsNull()) {
            initOcct();
        }
        m_digitalTwin.buildDefaultBridgeMachine();
        if (!m_view.IsNull()) {
            m_view->FitAll();
        }
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
        m_digitalTwin.updateMachinePosition(x, y, z);
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
        if (m_btnShadedEdges) m_btnShadedEdges->setChecked(mode == ModelDisplayMode::ShadedWithEdges);
        if (m_btnShadedOnly) m_btnShadedOnly->setChecked(mode == ModelDisplayMode::ShadedOnly);
        if (m_btnWireframe) m_btnWireframe->setChecked(mode == ModelDisplayMode::Wireframe);

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
        if (m_btnGranite) m_btnGranite->setChecked(visible);

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

    void OcctViewport::setMachineVisible(bool visible) {
        if (m_btnMachine) m_btnMachine->setChecked(visible);
        m_digitalTwin.setMachineVisible(visible);
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
        if (m_hudContainer) {
            int x = width() - m_hudContainer->width() - 14;
            int y = 14;
            m_hudContainer->move(x, y);
            m_hudContainer->raise();
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
            m_originalCadShapes.push_back(shape);

            applyVisualStyleToShape(aisShape);
            m_context->Activate(aisShape, AIS_Shape::SelectionMode(TopAbs_FACE));
            m_view->FitAll();
        }
    }

    bool OcctViewport::getCadBoundingBox(double& xmin, double& ymin, double& zmin,
        double& xmax, double& ymax, double& zmax) const {
        if (m_cadShapes.empty()) {
            return false;
        }

        Bnd_Box bbox;
        for (const auto& aisShape : m_cadShapes) {
            if (!aisShape.IsNull() && !aisShape->Shape().IsNull()) {
                BRepBndLib::Add(aisShape->Shape(), bbox);
            }
        }
        if (bbox.IsVoid()) {
            return false;
        }

        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        return true;
    }

    void OcctViewport::applyTransformToCadModels(const gp_Trsf& trsf) {
        if (m_context.IsNull() || m_cadShapes.empty()) {
            return;
        }

        m_context->ClearSelected(Standard_False);
        m_lastSelectedFace = CadFaceInfo{};

        for (auto& aisShape : m_cadShapes) {
            if (aisShape.IsNull() || aisShape->Shape().IsNull()) {
                continue;
            }
            BRepBuilderAPI_Transform xform(aisShape->Shape(), trsf, Standard_True);
            if (xform.IsDone()) {
                m_context->Deactivate(aisShape);
                aisShape->Set(xform.Shape());
                applyVisualStyleToShape(aisShape);
                m_context->Activate(aisShape, AIS_Shape::SelectionMode(TopAbs_FACE));
            }
        }

        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::translateCadModels(double dxMm, double dyMm, double dzMm) {
        gp_Trsf trsf;
        trsf.SetTranslation(gp_Vec(dxMm, dyMm, dzMm));
        applyTransformToCadModels(trsf);
    }

    void OcctViewport::rotateCadModels(double rxDeg, double ryDeg, double rzDeg, bool aroundPartCenter) {
        gp_Pnt pivot(0.0, 0.0, 0.0);
        if (aroundPartCenter) {
            double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
            if (getCadBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)) {
                pivot = gp_Pnt(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax));
            }
        }

        gp_Trsf totalTrsf;
        if (std::abs(rxDeg) > 1e-6) {
            gp_Trsf trsfX;
            trsfX.SetRotation(gp_Ax1(pivot, gp_Dir(1.0, 0.0, 0.0)), rxDeg * DEG_TO_RAD);
            totalTrsf.Multiply(trsfX);
        }
        if (std::abs(ryDeg) > 1e-6) {
            gp_Trsf trsfY;
            trsfY.SetRotation(gp_Ax1(pivot, gp_Dir(0.0, 1.0, 0.0)), ryDeg * DEG_TO_RAD);
            totalTrsf.Multiply(trsfY);
        }
        if (std::abs(rzDeg) > 1e-6) {
            gp_Trsf trsfZ;
            trsfZ.SetRotation(gp_Ax1(pivot, gp_Dir(0.0, 0.0, 1.0)), rzDeg * DEG_TO_RAD);
            totalTrsf.Multiply(trsfZ);
        }

        applyTransformToCadModels(totalTrsf);
    }

    void OcctViewport::dropCadModelsToGranite(double targetZMinMm) {
        double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        if (!getCadBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)) {
            return;
        }
        translateCadModels(0.0, 0.0, targetZMinMm - zmin);
    }

    void OcctViewport::centerCadModelsXY(double targetXMm, double targetYMm) {
        double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        if (!getCadBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)) {
            return;
        }
        const double cx = 0.5 * (xmin + xmax);
        const double cy = 0.5 * (ymin + ymax);
        translateCadModels(targetXMm - cx, targetYMm - cy, 0.0);
    }

    void OcctViewport::snapCadMinCornerToOrigin() {
        double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        if (!getCadBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)) {
            return;
        }
        translateCadModels(-xmin, -ymin, -zmin);
    }

    bool OcctViewport::alignSelectedFaceToZ(bool pointNormalUp) {
        if (!m_lastSelectedFace.success) {
            return false;
        }

        gp_Vec u(m_lastSelectedFace.normalOrAxis);
        gp_Vec v(0.0, 0.0, pointNormalUp ? 1.0 : -1.0);
        if (u.Magnitude() < 1e-9) {
            return false;
        }
        u.Normalize();

        gp_Pnt pivot = m_lastSelectedFace.centerOrCentroid;
        double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        if (getCadBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)) {
            pivot = gp_Pnt(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax));
        }

        const double dotVal = std::clamp(u.Dot(v), -1.0, 1.0);
        if (dotVal > 0.999999) {
            return true;
        }

        gp_Trsf trsf;
        if (dotVal < -0.999999) {
            gp_Dir orthoAxis = (std::abs(u.X()) < 0.9) ? gp_Dir(1.0, 0.0, 0.0) : gp_Dir(0.0, 1.0, 0.0);
            trsf.SetRotation(gp_Ax1(pivot, orthoAxis), 3.14159265358979323846);
        }
        else {
            gp_Vec crossVec = u.Crossed(v);
            const double angleRad = std::acos(dotVal);
            trsf.SetRotation(gp_Ax1(pivot, gp_Dir(crossVec)), angleRad);
        }

        applyTransformToCadModels(trsf);
        return true;
    }

    void OcctViewport::resetCadModelsTransform() {
        if (m_context.IsNull() || m_cadShapes.size() != m_originalCadShapes.size()) {
            return;
        }

        m_context->ClearSelected(Standard_False);
        m_lastSelectedFace = CadFaceInfo{};

        for (size_t i = 0; i < m_cadShapes.size(); ++i) {
            auto& aisShape = m_cadShapes[i];
            m_context->Deactivate(aisShape);
            aisShape->Set(m_originalCadShapes[i]);
            applyVisualStyleToShape(aisShape);
            m_context->Activate(aisShape, AIS_Shape::SelectionMode(TopAbs_FACE));
        }

        m_context->UpdateCurrentViewer();
    }

    CadFaceInfo OcctViewport::inspectSelectedCadFace() const {
        CadFaceInfo info;
        if (m_context.IsNull()) {
            return info;
        }

        m_context->InitSelected();
        if (!m_context->MoreSelected() || !m_context->HasSelectedShape()) {
            return info;
        }

        TopoDS_Shape selectedShape = m_context->SelectedShape();
        if (selectedShape.IsNull() || selectedShape.ShapeType() != TopAbs_FACE) {
            return info;
        }

        TopoDS_Face face = TopoDS::Face(selectedShape);
        BRepAdaptor_Surface surfAdaptor(face, Standard_True);
        GeomAbs_SurfaceType surfType = surfAdaptor.GetType();

        info.isInternal = (face.Orientation() == TopAbs_REVERSED);

        switch (surfType) {
        case GeomAbs_Plane: {
            gp_Pln pln = surfAdaptor.Plane();
            info.surfaceType = "Plane";
            info.centerOrCentroid = pln.Location();
            gp_Dir n = pln.Axis().Direction();
            if (face.Orientation() == TopAbs_REVERSED) {
                n.Reverse();
            }
            info.normalOrAxis = n;
            info.isInternal = false;
            info.success = true;
            break;
        }
        case GeomAbs_Cylinder: {
            gp_Cylinder cyl = surfAdaptor.Cylinder();
            info.surfaceType = info.isInternal ? "Internal Bore (Cylinder)" : "External Boss (Cylinder)";
            info.nominalRadius = cyl.Radius();
            info.nominalDiameter = cyl.Radius() * 2.0;
            info.centerOrCentroid = cyl.Location();
            info.normalOrAxis = cyl.Axis().Direction();
            info.success = true;
            break;
        }
        case GeomAbs_Sphere: {
            gp_Sphere sph = surfAdaptor.Sphere();
            info.surfaceType = info.isInternal ? "Spherical Cavity" : "Sphere";
            info.nominalRadius = sph.Radius();
            info.nominalDiameter = sph.Radius() * 2.0;
            info.centerOrCentroid = sph.Location();
            info.success = true;
            break;
        }
        default:
            info.surfaceType = "Freeform / B-Spline Surface";
            info.success = true;
            break;
        }

        return info;
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

                CadFaceInfo faceInfo = inspectSelectedCadFace();
                if (faceInfo.success) {
                    m_lastSelectedFace = faceInfo;
                    if (m_faceSelectedCallback) {
                        m_faceSelectedCallback(faceInfo);
                    }
                }
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