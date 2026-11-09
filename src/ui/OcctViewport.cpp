// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifdef _WIN32
#include <windows.h>
#include <WNT_Window.hxx>
#endif
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
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
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
#include <BRepGProp_Face.hxx>
#include <BRepTools.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <Quantity_Color.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <gp_Circ.hxx>
#include <Prs3d_IsoAspect.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <BRep_Tool.hxx>
#include <GeomLProp_SLProps.hxx>



namespace OpenCMM {

    namespace {
        constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    }

    OcctViewport::OcctViewport(QWidget* parent)
        : QWidget(parent) {
        setAttribute(Qt::WA_PaintOnScreen);
        setAttribute(Qt::WA_NoSystemBackground);
        setAttribute(Qt::WA_NativeWindow);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);

        setupOverlayHud();
    }

    void OcctViewport::setupOverlayHud() {
        m_hudContainer = new QWidget(this);
        // Give the HUD its own native window handle so OpenGL SwapBuffers on the parent HWND never erases it
        m_hudContainer->setAttribute(Qt::WA_NativeWindow, true);
        m_hudContainer->setAttribute(Qt::WA_StyledBackground, true);
        m_hudContainer->setAutoFillBackground(true);
        m_hudContainer->setMinimumWidth(180);
        m_hudContainer->setMinimumHeight(260);
        m_hudContainer->setStyleSheet(
            "QWidget { background-color: #21252b; border: 1px solid #3a3f4b; border-radius: 6px; }"
            "QPushButton { background-color: #282c34; color: #abb2bf; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 10px; min-width: 150px; min-height: 24px; font-weight: bold; font-size: 11px; text-align: left; }"
            "QPushButton:hover { background-color: #3a3f4b; color: white; }"
            "QPushButton:checked { background-color: #2d5a88; color: white; border: 1px solid #61afef; }"
            "QSlider::groove:vertical { background: #1e2227; width: 6px; border-radius: 3px; }"
            "QSlider::handle:vertical { background: #61afef; height: 16px; margin: 0 -5px; border-radius: 3px; }"
            "QLabel { color: #abb2bf; font-size: 10px; font-weight: bold; background: transparent; border: none; }"
        );

        auto* hudLayout = new QVBoxLayout(m_hudContainer);
        hudLayout->setContentsMargins(8, 8, 8, 8);
        hudLayout->setSpacing(5);

        auto* titleLbl = new QLabel("CAD DISPLAY", m_hudContainer);
        titleLbl->setAlignment(Qt::AlignCenter);
        hudLayout->addWidget(titleLbl);

        m_btnShadedEdges = new QPushButton("✓ Shaded + Edges", m_hudContainer);
        m_btnShadedEdges->setCheckable(true);
        m_btnShadedEdges->setChecked(true);
        connect(m_btnShadedEdges, &QPushButton::clicked, this, [this]() {
            setModelDisplayMode(ModelDisplayMode::ShadedWithEdges);
            });
        hudLayout->addWidget(m_btnShadedEdges);

        m_btnShadedOnly = new QPushButton("  Shaded Only", m_hudContainer);
        m_btnShadedOnly->setCheckable(true);
        connect(m_btnShadedOnly, &QPushButton::clicked, this, [this]() {
            setModelDisplayMode(ModelDisplayMode::ShadedOnly);
            });
        hudLayout->addWidget(m_btnShadedOnly);

        m_btnWireframe = new QPushButton("  Wireframe", m_hudContainer);
        m_btnWireframe->setCheckable(true);
        connect(m_btnWireframe, &QPushButton::clicked, this, [this]() {
            setModelDisplayMode(ModelDisplayMode::Wireframe);
            });
        hudLayout->addWidget(m_btnWireframe);

        auto* compTitle = new QLabel("MACHINE LINKS", m_hudContainer);
        compTitle->setAlignment(Qt::AlignCenter);
        hudLayout->addWidget(compTitle);

        m_linkButtonsContainer = new QWidget(m_hudContainer);
        m_linkButtonsContainer->setStyleSheet(
            "QWidget { border: none; background: transparent; }"
            "QPushButton { background-color: #282c34; color: #abb2bf; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 10px; min-width: 150px; min-height: 24px; font-weight: bold; font-size: 11px; text-align: left; }"
            "QPushButton:hover { background-color: #3a3f4b; color: white; }"
            "QPushButton:checked { background-color: #2d5a88; color: white; border: 1px solid #61afef; }"
        );
        m_linkButtonsLayout = new QVBoxLayout(m_linkButtonsContainer);
        m_linkButtonsLayout->setContentsMargins(0, 0, 0, 0);
        m_linkButtonsLayout->setSpacing(4);
        hudLayout->addWidget(m_linkButtonsContainer);

        m_btnProbeStylus = new QPushButton("✓ Built-in Stylus", m_hudContainer);
        m_btnProbeStylus->setCheckable(true);
        m_btnProbeStylus->setChecked(true);
        connect(m_btnProbeStylus, &QPushButton::clicked, this, [this](bool checked) {
            setProbeStylusVisible(checked);
            });
        hudLayout->addWidget(m_btnProbeStylus);

        auto* sliderLayout = new QVBoxLayout();
        sliderLayout->setSpacing(3);
        auto* transTitle = new QLabel("CAD Trans: 0%", m_hudContainer);
        transTitle->setAlignment(Qt::AlignCenter);
        m_sliderValLbl = transTitle;

        m_verticalSlider = new QSlider(Qt::Vertical, m_hudContainer);
        m_verticalSlider->setRange(0, 90);
        m_verticalSlider->setValue(0);
        m_verticalSlider->setFixedHeight(75);
        connect(m_verticalSlider, &QSlider::valueChanged, this, [this](int val) {
            m_sliderValLbl->setText(QString("CAD Trans: %1%").arg(val));
            setModelTransparency(static_cast<double>(val) / 100.0);
            });

        sliderLayout->addWidget(transTitle);
        sliderLayout->addWidget(m_verticalSlider, 0, Qt::AlignHCenter);
        hudLayout->addLayout(sliderLayout);

        rebuildComponentHudButtons();
        m_hudContainer->adjustSize();
        m_hudContainer->raise();
    }

    void OcctViewport::showEvent(QShowEvent* event) {
        QWidget::showEvent(event);
        if (m_hudContainer) {
            m_hudContainer->layout()->invalidate();
            m_hudContainer->adjustSize();
            int x = width() - m_hudContainer->width() - 14;
            int y = 14;
            m_hudContainer->move(x, y);
            m_hudContainer->show();
            m_hudContainer->raise();
        }
    }

    void OcctViewport::rebuildComponentHudButtons() {
        if (!m_linkButtonsLayout) return;

        for (auto* btn : m_linkButtons) {
            m_linkButtonsLayout->removeWidget(btn);
            delete btn;
        }
        m_linkButtons.clear();

        const QStringList linkIds = m_digitalTwin.getLinkIds();
        for (const QString& id : linkIds) {
            const bool vis = m_digitalTwin.isLinkVisible(id);
            auto* btn = new QPushButton((vis ? "✓ " : "  ") + id, m_linkButtonsContainer);

            btn->setCheckable(true);
            btn->setChecked(vis);
            btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            btn->setMinimumHeight(26);

            QFontMetrics fm(btn->font());
            int textWidth = fm.horizontalAdvance(btn->text()) + 28;
            btn->setMinimumWidth(std::max(165, textWidth));

            connect(btn, &QPushButton::clicked, this, [this, id, btn](bool checked) {
                m_digitalTwin.setLinkVisible(id, checked);
                btn->setText((checked ? "✓ " : "  ") + id);

                QFontMetrics fm2(btn->font());
                int newWidth = fm2.horizontalAdvance(btn->text()) + 28;
                btn->setMinimumWidth(std::max(165, newWidth));

                if (m_hudContainer) {
                    m_hudContainer->layout()->invalidate();
                    m_hudContainer->adjustSize();
                    m_hudContainer->raise();
                    m_hudContainer->update();
                }
                });

            m_linkButtonsLayout->addWidget(btn);
            m_linkButtons.push_back(btn);
        }

        if (m_hudContainer) {
            m_hudContainer->layout()->invalidate();
            m_hudContainer->adjustSize();
            int x = width() - m_hudContainer->width() - 14;
            int y = 14;
            m_hudContainer->move(x, y);
            m_hudContainer->show();
            m_hudContainer->raise();
            m_hudContainer->update();
        }
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

#ifdef _WIN32
        HWND hwnd = reinterpret_cast<HWND>(winId());
        // Clip child windows (like m_hudContainer) so OpenGL SwapBuffers never draws over the HUD
        LONG_PTR style = GetWindowLongPtr(hwnd, GWL_STYLE);
        SetWindowLongPtr(hwnd, GWL_STYLE, style | WS_CLIPCHILDREN | WS_CLIPSIBLINGS);

        Handle(WNT_Window) wntWindow = new WNT_Window(reinterpret_cast<Aspect_Handle>(hwnd));
        m_view->SetWindow(wntWindow);
        if (!wntWindow->IsMapped()) {
            wntWindow->Map();
        }
#endif

        m_view->TriedronDisplay(
            Aspect_TOTP_LEFT_LOWER,
            m_isDarkMode ? Quantity_NOC_WHITE : Quantity_NOC_BLACK,
            0.1,
            V3d_ZBUFFER
        );

        createProbeStylus();

        m_digitalTwin.setContext(m_context);
        m_digitalTwin.buildDefaultBridgeMachine();
        rebuildComponentHudButtons();

        setTheme(m_isDarkMode);

        if (m_hudContainer) {
            m_hudContainer->layout()->activate();
            m_hudContainer->adjustSize();
            int x = width() - m_hudContainer->width() - 14;
            int y = 14;
            m_hudContainer->move(x, y);
            m_hudContainer->show();
            m_hudContainer->raise();
            m_hudContainer->update();
        }

        m_view->MustBeResized();
        m_view->FitAll();
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

    void OcctViewport::setProbeStylusVisible(bool visible) {
        m_probeStylusVisible = visible;
        if (m_btnProbeStylus) {
            m_btnProbeStylus->setChecked(visible);
            m_btnProbeStylus->setText(visible ? "✓ Built-in Stylus" : "  Built-in Stylus");
        }
        if (m_context.IsNull() || m_probeRuby.IsNull() || m_probeStem.IsNull()) {
            return;
        }
        if (visible) {
            m_context->Display(m_probeRuby, AIS_Shaded, -1, Standard_False);
            m_context->Display(m_probeStem, AIS_Shaded, -1, Standard_True);
        }
        else {
            m_context->Erase(m_probeRuby, Standard_False);
            m_context->Erase(m_probeStem, Standard_True);
        }
    }

    bool OcctViewport::loadMachinePackage(const QString& packagePath) {
        if (m_view.IsNull()) {
            initOcct();
        }
        bool ok = m_digitalTwin.loadMachinePackage(packagePath);
        if (ok) {
            rebuildComponentHudButtons();
            const bool hasModeledStylus = (m_digitalTwin.config().probeMountOffsetMm.norm() > 1e-3);
            setProbeStylusVisible(!hasModeledStylus);
            if (!m_view.IsNull()) {
                m_view->FitAll();
            }
        }
        return ok;
    }

    void OcctViewport::resetDefaultMachine() {
        if (m_view.IsNull()) {
            initOcct();
        }
        m_digitalTwin.buildDefaultBridgeMachine();
        rebuildComponentHudButtons();
        setProbeStylusVisible(true);
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

    void OcctViewport::addDeviationPointMarker(double x, double y, double z, double deviationMm, double plusTolMm) {
        if (m_context.IsNull()) {
            return;
        }

        TopoDS_Shape hitSphere = BRepPrimAPI_MakeSphere(gp_Pnt(x, y, z), 1.5).Shape();
        Handle(AIS_Shape) aisHit = new AIS_Shape(hitSphere);

        if (std::abs(deviationMm) <= plusTolMm) {
            aisHit->SetColor(Quantity_NOC_GREEN1);
        }
        else if (deviationMm > plusTolMm) {
            aisHit->SetColor(Quantity_NOC_RED);
        }
        else {
            aisHit->SetColor(Quantity_NOC_BLUE1);
        }

        aisHit->Attributes()->SetFaceBoundaryDraw(Standard_False);
        m_context->Display(aisHit, AIS_Shaded, -1, Standard_True);
        m_solvedHitMarkers.push_back(aisHit);
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

    void OcctViewport::clearProgramTargetMarkers() {
        if (m_context.IsNull()) {
            return;
        }
        for (const auto& marker : m_programTargetMarkers) {
            if (!marker.IsNull()) {
                m_context->Remove(marker, Standard_False);
            }
        }
        m_programTargetMarkers.clear();
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::clearPcsTriadVisual() {
        if (m_context.IsNull()) {
            return;
        }
        for (const auto& ais : m_pcsTriadShapes) {
            if (!ais.IsNull()) {
                m_context->Remove(ais, Standard_False);
            }
        }
        m_pcsTriadShapes.clear();
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::setPcsTriadVisual(const Eigen::Matrix4d& pcsMatrix, double axisLengthMm) {
        if (m_context.IsNull()) {
            return;
        }

        for (const auto& ais : m_pcsTriadShapes) {
            if (!ais.IsNull()) {
                m_context->Remove(ais, Standard_False);
            }
        }
        m_pcsTriadShapes.clear();

        const Eigen::Matrix3d R = pcsMatrix.block<3, 3>(0, 0);
        const Eigen::Vector3d t = pcsMatrix.block<3, 1>(0, 3);
        const Eigen::Vector3d originW = -R.transpose() * t;

        const gp_Pnt origPt(originW.x(), originW.y(), originW.z());

        TopoDS_Shape origSph = BRepPrimAPI_MakeSphere(origPt, 2.2).Shape();
        Handle(AIS_Shape) aisOrig = new AIS_Shape(origSph);
        aisOrig->SetColor(Quantity_NOC_GOLD);
        m_context->Display(aisOrig, AIS_Shaded, -1, Standard_False);
        m_pcsTriadShapes.push_back(aisOrig);

        const Quantity_Color axisColors[3] = {
            Quantity_Color(0.88, 0.42, 0.46, Quantity_TOC_RGB),
            Quantity_Color(0.60, 0.76, 0.47, Quantity_TOC_RGB),
            Quantity_Color(0.38, 0.69, 0.94, Quantity_TOC_RGB)
        };

        for (int a = 0; a < 3; ++a) {
            Eigen::Vector3d dirVec = R.row(a).transpose();
            if (dirVec.norm() <= 1e-6) continue;
            dirVec.normalize();

            gp_Dir gDir(dirVec.x(), dirVec.y(), dirVec.z());
            gp_Ax2 ax(origPt, gDir);
            TopoDS_Shape axisCyl = BRepPrimAPI_MakeCylinder(ax, 0.95, axisLengthMm).Shape();
            Handle(AIS_Shape) aisAxis = new AIS_Shape(axisCyl);
            aisAxis->SetColor(axisColors[a]);
            m_context->Display(aisAxis, AIS_Shaded, -1, Standard_False);
            m_pcsTriadShapes.push_back(aisAxis);

            gp_Pnt tipPt = origPt.Translated(gp_Vec(gDir).Multiplied(axisLengthMm));
            TopoDS_Shape tipSph = BRepPrimAPI_MakeSphere(tipPt, 1.8).Shape();
            Handle(AIS_Shape) aisTip = new AIS_Shape(tipSph);
            aisTip->SetColor(axisColors[a]);
            m_context->Display(aisTip, AIS_Shaded, -1, Standard_False);
            m_pcsTriadShapes.push_back(aisTip);
        }

        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::setProgramTargetMarkers(
        const std::vector<gp_Pnt>& surfacePts,
        const std::vector<gp_Dir>& normals,
        int activeIndex,
        double probeRadiusMm) {

        if (m_context.IsNull()) {
            return;
        }

        for (const auto& marker : m_programTargetMarkers) {
            if (!marker.IsNull()) {
                m_context->Remove(marker, Standard_False);
            }
        }
        m_programTargetMarkers.clear();

        const size_t count = surfacePts.size();
        for (size_t i = 0; i < count; ++i) {
            const gp_Pnt& surfPt = surfacePts[i];
            const gp_Dir norm = (i < normals.size()) ? normals[i] : gp_Dir(0.0, 0.0, 1.0);
            const gp_Vec nVec(norm);
            const gp_Pnt rubyTargetPt = surfPt.Translated(nVec.Multiplied(probeRadiusMm));

            if (static_cast<int>(i) < activeIndex) {
                TopoDS_Shape doneSph = BRepPrimAPI_MakeSphere(rubyTargetPt, 1.2).Shape();
                Handle(AIS_Shape) aisDone = new AIS_Shape(doneSph);
                aisDone->SetColor(Quantity_NOC_GREEN1);
                m_context->Display(aisDone, AIS_Shaded, -1, Standard_False);
                m_programTargetMarkers.push_back(aisDone);
            }
            else if (static_cast<int>(i) == activeIndex) {
                TopoDS_Shape activeSph = BRepPrimAPI_MakeSphere(rubyTargetPt, 2.3).Shape();
                Handle(AIS_Shape) aisActive = new AIS_Shape(activeSph);
                aisActive->SetColor(Quantity_NOC_GOLD);
                aisActive->SetTransparency(0.25);
                m_context->Display(aisActive, AIS_Shaded, -1, Standard_False);
                m_programTargetMarkers.push_back(aisActive);

                gp_Ax2 stemAx(surfPt, norm);
                TopoDS_Shape approachRod = BRepPrimAPI_MakeCylinder(stemAx, 0.45, 18.0 + probeRadiusMm).Shape();
                Handle(AIS_Shape) aisRod = new AIS_Shape(approachRod);
                aisRod->SetColor(Quantity_NOC_GOLD);
                m_context->Display(aisRod, AIS_Shaded, -1, Standard_False);
                m_programTargetMarkers.push_back(aisRod);
            }
            else {
                TopoDS_Shape pendingSph = BRepPrimAPI_MakeSphere(rubyTargetPt, 1.4).Shape();
                Handle(AIS_Shape) aisPending = new AIS_Shape(pendingSph);
                aisPending->SetColor(Quantity_NOC_ORANGE);
                aisPending->SetTransparency(0.35);
                m_context->Display(aisPending, AIS_Shaded, -1, Standard_False);
                m_programTargetMarkers.push_back(aisPending);

                gp_Ax2 stemAx(surfPt, norm);
                TopoDS_Shape shortRod = BRepPrimAPI_MakeCylinder(stemAx, 0.25, 9.0 + probeRadiusMm).Shape();
                Handle(AIS_Shape) aisShortRod = new AIS_Shape(shortRod);
                aisShortRod->SetColor(Quantity_NOC_ORANGE);
                m_context->Display(aisShortRod, AIS_Shaded, -1, Standard_False);
                m_programTargetMarkers.push_back(aisShortRod);
            }
        }

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
        if (m_btnShadedEdges) {
            m_btnShadedEdges->setChecked(mode == ModelDisplayMode::ShadedWithEdges);
            m_btnShadedEdges->setText((mode == ModelDisplayMode::ShadedWithEdges ? "✓ " : "  ") + QString("Shaded + Edges"));
        }
        if (m_btnShadedOnly) {
            m_btnShadedOnly->setChecked(mode == ModelDisplayMode::ShadedOnly);
            m_btnShadedOnly->setText((mode == ModelDisplayMode::ShadedOnly ? "✓ " : "  ") + QString("Shaded Only"));
        }
        if (m_btnWireframe) {
            m_btnWireframe->setChecked(mode == ModelDisplayMode::Wireframe);
            m_btnWireframe->setText((mode == ModelDisplayMode::Wireframe ? "✓ " : "  ") + QString("Wireframe"));
        }

        if (m_context.IsNull()) {
            return;
        }

        for (const auto& cad : m_cadShapes) {
            applyVisualStyleToShape(cad);
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
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::setGraniteVisible(bool visible) {
        m_digitalTwin.setLinkVisible("granite_base", visible);
        rebuildComponentHudButtons();
    }

    void OcctViewport::setMachineVisible(bool visible) {
        m_digitalTwin.setMachineVisible(visible);
    }

    void OcctViewport::paintEvent(QPaintEvent* /*event*/) {
        if (m_view.IsNull()) {
            initOcct();
        }

        if (m_hudContainer) {
            m_hudContainer->layout()->activate();
            m_hudContainer->adjustSize();
            int x = width() - m_hudContainer->width() - 14;
            int y = 14;
            if (m_hudContainer->pos() != QPoint(x, y)) {
                m_hudContainer->move(x, y);
            }
            m_hudContainer->raise();
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
            m_hudContainer->update();
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

    TopoDS_Shape OcctViewport::getCombinedCadShape() const {
        if (m_cadShapes.empty()) {
            return TopoDS_Shape();
        }
        if (m_cadShapes.size() == 1 && !m_cadShapes[0].IsNull()) {
            return m_cadShapes[0]->Shape();
        }

        BRep_Builder builder;
        TopoDS_Compound comp;
        builder.MakeCompound(comp);
        for (const auto& ais : m_cadShapes) {
            if (!ais.IsNull() && !ais->Shape().IsNull()) {
                builder.Add(comp, ais->Shape());
            }
        }
        return comp;
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

    CadFaceInfo OcctViewport::inspectSelectedCadFace() {
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
        m_lastSelectedTopoFace = face;
        BRepAdaptor_Surface surfAdaptor(face, Standard_True);
        GeomAbs_SurfaceType surfType = surfAdaptor.GetType();

        info.isInternal = (face.Orientation() == TopAbs_REVERSED);
        info.face = face;

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

    CadSurfaceHit OcctViewport::pickSurfacePointOnSelectedFace(int screenX, int screenY) const {
        CadSurfaceHit hit;
        if (m_view.IsNull() || m_lastSelectedTopoFace.IsNull()) {
            return hit;
        }

        Standard_Real px = 0.0, py = 0.0, pz = 0.0;
        Standard_Real vx = 0.0, vy = 0.0, vz = 0.0;
        m_view->ConvertWithProj(screenX, screenY, px, py, pz, vx, vy, vz);

        gp_Vec rayVec(vx, vy, vz);
        if (rayVec.Magnitude() <= 1e-9) {
            return hit;
        }
        gp_Lin pickRay(gp_Pnt(px, py, pz), gp_Dir(rayVec));

        IntCurvesFace_ShapeIntersector intersector;
        intersector.Load(m_lastSelectedTopoFace, 1e-4);
        intersector.Perform(pickRay, -1e9, 1e9);

        if (intersector.IsDone() && intersector.NbPnt() >= 1) {
            int bestIdx = 1;
            double minW = intersector.WParameter(1);
            for (int i = 2; i <= intersector.NbPnt(); ++i) {
                if (intersector.WParameter(i) < minW) {
                    minW = intersector.WParameter(i);
                    bestIdx = i;
                }
            }

            const Standard_Real u = intersector.UParameter(bestIdx);
            const Standard_Real v = intersector.VParameter(bestIdx);

            BRepGProp_Face prop(m_lastSelectedTopoFace);
            gp_Pnt surfPt;
            gp_Vec surfNorm;
            prop.Normal(u, v, surfPt, surfNorm);
            if (surfNorm.Magnitude() > 1e-9) {
                surfNorm.Normalize();
                hit.surfacePoint = surfPt;
                hit.surfaceNormal = gp_Dir(surfNorm);
                hit.success = true;
            }
        }
        return hit;
    }

    std::vector<CadSurfaceHit> OcctViewport::sampleSelectedCadFace(int numPointsPerRing) {
        std::vector<CadSurfaceHit> hits;
        if (!hasSelectedCadFace()) return hits;

        const TopoDS_Face& face = m_lastSelectedFace.face;
        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

        Handle(Standard_Type) sType = surf->DynamicType();
        bool isCylinder = (sType == STANDARD_TYPE(Geom_CylindricalSurface));

        int numRings = isCylinder ? 3 : 1;
        double uMin, uMax, vMin, vMax;
        BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);

        for (int r = 0; r < numRings; ++r) {
            double v = (numRings > 1)
                ? (vMin + (0.15 + 0.70 * (static_cast<double>(r) / (numRings - 1))) * (vMax - vMin))
                : (0.5 * (vMin + vMax));
            for (int i = 0; i < numPointsPerRing; ++i) {
                double u = uMin + (static_cast<double>(i) / numPointsPerRing) * (uMax - uMin);
                GeomLProp_SLProps props(surf, u, v, 1, 1e-6);
                if (props.IsNormalDefined()) {
                    gp_Pnt pt = props.Value();
                    gp_Dir norm = props.Normal();
                    if (face.Orientation() == TopAbs_REVERSED) {
                        norm.Reverse();
                    }
                    CadSurfaceHit hit;
                    hit.success = true;
                    hit.surfacePoint = pt;
                    hit.surfaceNormal = norm;
                    hits.push_back(hit);
                }
            }
        }
        return hits;
    }

    void OcctViewport::mousePressEvent(QMouseEvent* event) {
        m_lastMousePos = event->pos();
        m_pressMousePos = event->pos();

        if (event->button() == Qt::LeftButton && !m_view.IsNull()) {
            if (!(event->modifiers() & Qt::ControlModifier)) {
                m_isRotating = true;
                m_view->StartRotation(event->pos().x(), event->pos().y());
            }
        }
        else if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
            m_isPanning = true;
        }
    }

    void OcctViewport::mouseReleaseEvent(QMouseEvent* event) {
        if (event->button() == Qt::LeftButton) {
            m_isRotating = false;
            if ((event->pos() - m_pressMousePos).manhattanLength() <= 3 && !m_context.IsNull()) {
                m_context->MoveTo(event->pos().x(), event->pos().y(), m_view, Standard_False);
                m_context->SelectDetected();
                m_view->Redraw();
                if (m_hudContainer) m_hudContainer->update();

                CadFaceInfo faceInfo = inspectSelectedCadFace();
                if (faceInfo.success) {
                    m_lastSelectedFace = faceInfo;
                    if (m_faceSelectedCallback) {
                        m_faceSelectedCallback(faceInfo);
                    }

                    if ((event->modifiers() & Qt::ControlModifier) && m_surfaceHitCallback) {
                        CadSurfaceHit hit = pickSurfacePointOnSelectedFace(event->pos().x(), event->pos().y());
                        if (hit.success) {
                            m_surfaceHitCallback(hit);
                        }
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
            if (m_hudContainer) m_hudContainer->update();
        }
        else if (m_isPanning) {
            m_view->Pan(dx, -dy);
            m_view->Redraw();
            if (m_hudContainer) m_hudContainer->update();
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
            if (m_hudContainer) {
                m_hudContainer->raise();
                m_hudContainer->update();
            }
        }
    }

    void OcctViewport::displayFittedFeatureShape(int featureId, const TopoDS_Shape& shape, const Quantity_Color& color) {
        if (m_context.IsNull() || shape.IsNull()) return;

        Handle(AIS_Shape) ais = new AIS_Shape(shape);
        ais->SetColor(color);
        ais->SetMaterial(Graphic3d_NOM_PLASTIC);

        Handle(Prs3d_Drawer) drawer = ais->Attributes();
        drawer->SetUIsoAspect(new Prs3d_IsoAspect(color, Aspect_TOL_SOLID, 1.0, 6));
        drawer->SetVIsoAspect(new Prs3d_IsoAspect(color, Aspect_TOL_SOLID, 1.0, 6));
        drawer->SetFaceBoundaryDraw(Standard_True);
        drawer->SetFaceBoundaryAspect(new Prs3d_LineAspect(color, Aspect_TOL_SOLID, 2.0));
        drawer->SetWireAspect(new Prs3d_LineAspect(color, Aspect_TOL_SOLID, 2.0));

        const int dispMode = m_fittedFeaturesWireframe ? 0 : 1;
        ais->SetTransparency(m_fittedFeaturesWireframe ? 0.0 : 0.35);

        m_context->Display(ais, dispMode, -1, Standard_False);
        m_fittedFeatureShapes.push_back({ featureId, ais, color });
        m_view->Redraw();
    }

    void OcctViewport::addFittedPlaneVisual(int featureId,
        double cx, double cy, double cz,
        double nx, double ny, double nz,
        double sizeMm) {
        gp_Pnt center(cx, cy, cz);
        gp_Dir normal(nx, ny, nz);
        gp_Pln pln(center, normal);

        const double half = std::max(10.0, sizeMm * 0.5);
        TopoDS_Face face = BRepBuilderAPI_MakeFace(pln, -half, half, -half, half);
        displayFittedFeatureShape(featureId, face, Quantity_Color(0.38, 0.69, 0.94, Quantity_TOC_RGB));
    }

    void OcctViewport::addFittedSphereVisual(int featureId,
        double cx, double cy, double cz,
        double radiusMm) {
        if (radiusMm <= 1e-4) return;
        TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(gp_Pnt(cx, cy, cz), radiusMm).Shape();
        displayFittedFeatureShape(featureId, sphere, Quantity_Color(0.60, 0.76, 0.47, Quantity_TOC_RGB));
    }

    void OcctViewport::addFittedCircleVisual(int featureId,
        double cx, double cy, double cz,
        double nx, double ny, double nz,
        double radiusMm) {
        if (radiusMm <= 1e-4) return;
        gp_Ax2 ax(gp_Pnt(cx, cy, cz), gp_Dir(nx, ny, nz));
        gp_Circ circ(ax, radiusMm);

        TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(circ);
        TopoDS_Wire wire = BRepBuilderAPI_MakeWire(edge);
        TopoDS_Face diskFace = BRepBuilderAPI_MakeFace(wire, Standard_True);
        displayFittedFeatureShape(featureId, diskFace, Quantity_Color(0.90, 0.75, 0.48, Quantity_TOC_RGB));
    }

    void OcctViewport::addFittedLineVisual(int featureId,
        double x1, double y1, double z1,
        double x2, double y2, double z2) {
        gp_Pnt p1(x1, y1, z1);
        gp_Pnt p2(x2, y2, z2);
        const double len = p1.Distance(p2);
        if (len <= 1e-4) return;

        gp_Dir dir(gp_Vec(p1, p2));
        gp_Ax2 ax(p1, dir);

        TopoDS_Shape rod = BRepPrimAPI_MakeCylinder(ax, 0.45, len).Shape();
        displayFittedFeatureShape(featureId, rod, Quantity_Color(0.88, 0.42, 0.46, Quantity_TOC_RGB));
    }

    void OcctViewport::highlightFittedFeature(int featureId) {
        if (m_context.IsNull()) return;

        const Quantity_Color highlightCol(1.0, 0.92, 0.23, Quantity_TOC_RGB);

        for (auto& entry : m_fittedFeatureShapes) {
            if (entry.shape.IsNull()) continue;

            const bool isSelected = (entry.featureId == featureId && featureId > 0);
            const Quantity_Color& col = isSelected ? highlightCol : entry.baseColor;
            const double lineWidth = isSelected ? 3.5 : 2.0;
            const double isoWidth = isSelected ? 2.0 : 1.0;

            entry.shape->SetColor(col);
            entry.shape->SetTransparency(m_fittedFeaturesWireframe ? 0.0 : (isSelected ? 0.10 : 0.35));

            Handle(Prs3d_Drawer) drawer = entry.shape->Attributes();
            drawer->SetUIsoAspect(new Prs3d_IsoAspect(col, Aspect_TOL_SOLID, isoWidth, 6));
            drawer->SetVIsoAspect(new Prs3d_IsoAspect(col, Aspect_TOL_SOLID, isoWidth, 6));
            drawer->SetFaceBoundaryAspect(new Prs3d_LineAspect(col, Aspect_TOL_SOLID, lineWidth));
            drawer->SetWireAspect(new Prs3d_LineAspect(col, Aspect_TOL_SOLID, lineWidth));

            m_context->Redisplay(entry.shape, Standard_False);
        }
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::setFittedFeaturesWireframe(bool wireframe) {
        m_fittedFeaturesWireframe = wireframe;
        if (m_context.IsNull()) return;

        const int dispMode = m_fittedFeaturesWireframe ? 0 : 1;
        for (const auto& entry : m_fittedFeatureShapes) {
            if (!entry.shape.IsNull()) {
                entry.shape->SetTransparency(m_fittedFeaturesWireframe ? 0.0 : 0.35);
                m_context->SetDisplayMode(entry.shape, dispMode, Standard_False);
            }
        }
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::clearFittedFeatures() {
        if (m_context.IsNull()) return;
        for (const auto& entry : m_fittedFeatureShapes) {
            if (!entry.shape.IsNull()) {
                m_context->Remove(entry.shape, Standard_False);
            }
        }
        m_fittedFeatureShapes.clear();
        m_context->UpdateCurrentViewer();
    }

    void OcctViewport::setBackgroundColor(bool dark) {
        m_isDarkMode = dark;
        if (m_view.IsNull()) return;

        if (dark) {
            m_view->SetBackgroundColor(Quantity_Color(0.1176, 0.1333, 0.1529, Quantity_TOC_RGB));
        }
        else {
            m_view->SetBackgroundColor(Quantity_Color(0.9451, 0.9608, 0.9765, Quantity_TOC_RGB));
        }
        m_view->Redraw();
    }

    void OcctViewport::setTheme(bool dark) {
        m_isDarkMode = dark;

        if (m_hudContainer) {
            if (dark) {
                m_hudContainer->setStyleSheet(
                    "QWidget { background-color: #21252b; border: 1px solid #3a3f4b; border-radius: 6px; }"
                    "QPushButton { background-color: #282c34; color: #abb2bf; border: 1px solid #4b5263; border-radius: 3px; padding: 4px 8px; font-weight: bold; font-size: 11px; min-width: 115px; text-align: left; }"
                    "QPushButton:hover { background-color: #3a3f4b; color: white; }"
                    "QPushButton:checked { background-color: #2d5a88; color: white; border: 1px solid #61afef; }"
                    "QSlider::groove:vertical { background: #1e2227; width: 6px; border-radius: 3px; }"
                    "QSlider::handle:vertical { background: #61afef; height: 16px; margin: 0 -5px; border-radius: 3px; }"
                    "QLabel { color: #abb2bf; font-size: 10px; font-weight: bold; background: transparent; border: none; }"
                );
            }
            else {
                m_hudContainer->setStyleSheet(
                    "QWidget { background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 6px; }"
                    "QPushButton { background-color: #f1f5f9; color: #334155; border: 1px solid #94a3b8; border-radius: 3px; padding: 4px 8px; font-weight: bold; font-size: 11px; min-width: 115px; text-align: left; }"
                    "QPushButton:hover { background-color: #e2e8f0; color: #0f172a; }"
                    "QPushButton:checked { background-color: #0284c7; color: white; border: 1px solid #0284c7; }"
                    "QSlider::groove:vertical { background: #cbd5e1; width: 6px; border-radius: 3px; }"
                    "QSlider::handle:vertical { background: #0284c7; height: 16px; margin: 0 -5px; border-radius: 3px; }"
                    "QLabel { color: #475569; font-size: 10px; font-weight: bold; background: transparent; border: none; }"
                );
            }
            m_hudContainer->update();
        }

        if (!m_view.IsNull()) {
            if (dark) {
                m_view->SetBackgroundColor(Quantity_Color(0.1176, 0.1333, 0.1529, Quantity_TOC_RGB));
                m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.1, V3d_ZBUFFER);
            }
            else {
                m_view->SetBackgroundColor(Quantity_Color(0.9451, 0.9608, 0.9765, Quantity_TOC_RGB));
                m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_BLACK, 0.1, V3d_ZBUFFER);
            }
            m_view->Redraw();
        }
    }
    void OcctViewport::setProbeCollisionState(bool collision, bool warning) {
        if (m_context.IsNull() || m_probeRuby.IsNull()) return;

        if (collision) {
            m_probeRuby->SetColor(Quantity_NOC_RED);
        }
        else if (warning) {
            m_probeRuby->SetColor(Quantity_NOC_ORANGE);
        }
        else {
            m_probeRuby->SetColor(Quantity_NOC_RED); // Normal probe ruby red
        }
        m_context->Redisplay(m_probeRuby, Standard_False);
        m_context->UpdateCurrentViewer();
    }
} // namespace OpenCMM