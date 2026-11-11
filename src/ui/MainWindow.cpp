// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifdef _WIN32
#include <windows.h>
#endif

#include "ui/MainWindow.h"
#include "ui/OcctViewport.h"
#include "ui/SettingsDialog.h"
#include "core/AppSettings.h"
#include "ui/CalibrationWidget.h"
#include "ui/RibbonCustomizeDialog.h"
#include "dmis/QifExporter.h"

#include <QToolBar>
#include <QTabWidget>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QStatusBar>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QSerialPortInfo>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QDesktopServices>
#include <QUrl>
#include <QColor>
#include <QBrush>
#include <algorithm>
#include <cmath>
#include <thread>
#include <QMenu>
#include <QKeySequenceEdit>
#include <QSettings>
#include <QShortcut>
#include <QSettings>
#include <QPdfWriter>
#include <QPageLayout>
#include <QPageSize>
#include <QPainter>
#include <QLineEdit>
#include <QTextEdit>
#include <QEvent>
#include <QScreen>
#include <QScrollArea>
#include <QCoreApplication>
#include <QDir>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Cone.hxx>
#include <BRepTools.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRepLProp_SLProps.hxx>
#include <Precision.hxx>
#include <gp_Pnt2d.hxx>

namespace OpenCMM {

    namespace {
        constexpr int ROLE_ACTUAL_MM = Qt::UserRole + 1;
        constexpr int ROLE_HAS_NOMINAL = Qt::UserRole + 2;
        constexpr int ROLE_NOMINAL_MM = Qt::UserRole + 3;
        constexpr int ROLE_PLUS_TOL_MM = Qt::UserRole + 4;
        constexpr int ROLE_MINUS_TOL_MM = Qt::UserRole + 5;
        constexpr int ROLE_FEATURE_ID = Qt::UserRole + 6;
    }

    MainWindow::MainWindow(QWidget* parent)
        : QMainWindow(parent),
        m_controller(new PicoSerialController(this)) {

        setWindowTitle("OpenCMM - Precision Coordinate Metrology Studio");
        resize(1680, 960);

        // --- 1. Load QSettings Persistence FIRST so m_isDarkMode is known ---
        QString savedMachinePath, savedPort;
        int savedBaud = 115200;
        Eigen::Vector3d savedScale(0.001, 0.001, 0.001);
        Eigen::Vector3i savedSign(1, 1, 1);

        AppSettings::loadSettings(
            savedMachinePath, savedPort, savedBaud,
            savedScale, savedSign,
            m_unitSystem, m_defaultTolMm, m_audioEnabled,
            m_isDarkMode
        );

        // --- 2. Setup UI components with the active theme ---
        setupUi();

        if (!savedMachinePath.isEmpty() && m_viewport) {
            if (m_viewport->loadMachinePackage(savedMachinePath)) {
                m_machinePackagePath = savedMachinePath;
                const auto& cfg = m_viewport->machineConfig();
                if (m_controller) {
                    m_controller->setAxisCalibration(cfg.scaleMm, cfg.sign);
                }
            }
        }
        else if (m_controller) {
            m_controller->setAxisCalibration(savedScale, savedSign);
        }

        if (!savedPort.isEmpty()) {
            m_currentPortName = savedPort;
        }

        if (m_unitSystem == UnitSystem::Inches) {
            for (auto* lbl : m_unitLabels) {
                if (lbl) lbl->setText("in");
            }
        }
        updateDroDisplay();
        updateProbeStatusDisplay();

        m_dmisPlaybackTimer = new QTimer(this);
        m_dmisPlaybackTimer->setInterval(35);
        connect(m_dmisPlaybackTimer, &QTimer::timeout, this, &MainWindow::onDmisPlaybackTick);

        connect(m_controller, &PicoSerialController::positionUpdated,
            this, &MainWindow::onPositionUpdated);
        connect(m_controller, &PicoSerialController::probeTriggered,
            this, &MainWindow::onProbeHit);
        connect(m_controller, &PicoSerialController::connectionStatusChanged,
            this, &MainWindow::onConnectionChanged);

        updateProbeStatusDisplay();

        // --- 3. Apply saved theme immediately on startup ---
        onThemeToggled(m_isDarkMode);

        // --- 4. Restore window geometry and dock pin states ---
        QSettings settings("JLS Machine", "OpenCMM");
        if (settings.contains("windowState")) {
            restoreState(settings.value("windowState").toByteArray());
        }
        if (settings.value("maximized", false).toBool()) {
            if (screen()) {
                setGeometry(screen()->availableGeometry());
            }
            showMaximized();
        }
        else if (settings.contains("geometry")) {
            restoreGeometry(settings.value("geometry").toByteArray());
            if (screen()) {
                const QRect avail = screen()->availableGeometry();
                if (height() > avail.height() || width() > avail.width()) {
                    setGeometry(avail);
                }
            }
        }
        else {
            resize(1280, 800);
        }

        if (m_droDock) {
            setDockPinned(m_droDock, settings.value("pinned_LiveDroDock", true).toBool());
        }
        if (m_treeDock) {
            setDockPinned(m_treeDock, settings.value("pinned_FeatureTreeDock", true).toBool());
        }
        if (m_collisionDock) {
            setDockPinned(m_collisionDock, settings.value("pinned_CollisionLogDock", false).toBool());
        }
    }

    void MainWindow::closeEvent(QCloseEvent* event) {
        QSettings settings("JLS Machine", "OpenCMM");
        settings.setValue("geometry", saveGeometry());
        settings.setValue("windowState", saveState());
        settings.setValue("maximized", isMaximized());

        if (m_droDock && m_dockPinned.count(m_droDock)) {
            settings.setValue("pinned_LiveDroDock", m_dockPinned[m_droDock]);
        }
        if (m_treeDock && m_dockPinned.count(m_treeDock)) {
            settings.setValue("pinned_FeatureTreeDock", m_dockPinned[m_treeDock]);
        }
        if (m_collisionDock && m_dockPinned.count(m_collisionDock)) {
            settings.setValue("pinned_CollisionLogDock", m_dockPinned[m_collisionDock]);
        }
        settings.sync();

        QMainWindow::closeEvent(event);
    }

    MainWindow::~MainWindow() = default;

    void MainWindow::setupUi() {
        setStyleSheet(
            "QMainWindow { background-color: #1e2227; color: #e0e0e0; }"
            "QDockWidget { color: #ffffff; font-weight: bold; }"
            "QDockWidget::title { background: #282c34; padding: 6px; border: 1px solid #3a3f4b; }"
            "QGroupBox { border: 1px solid #3a3f4b; border-radius: 4px; margin-top: 10px; padding-top: 12px; color: #abb2bf; font-weight: bold; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
        );

        // Keep the entire left column dedicated to the left docks so the bottom
        // collision dock only sits underneath the central 3D viewport
        setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
        setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);

        setupToolbar();
        setupAutoHideBars();
        setupDroDock();
        setupFeatureTreeDock();
        setupCollisionLogDock();

        m_viewport = new OcctViewport(this);
        m_viewport->setMinimumSize(200, 150);
        m_viewport->installEventFilter(this);
        m_viewport->setFaceSelectedCallback([this](const CadFaceInfo& faceInfo) {
            onCadFaceSelected(faceInfo);
            });
        m_viewport->setSurfaceHitCallback([this](const CadSurfaceHit& hit) {
            onCadSurfaceProbed(hit);
            });
        auto* centralContainer = new QWidget(this);
        auto* centralLayout = new QVBoxLayout(centralContainer);
        centralLayout->setContentsMargins(0, 0, 0, 0);
        centralLayout->setSpacing(0);

        setupManualPromptBanner(centralContainer);
        centralLayout->addWidget(m_manualPromptBanner, 0);

        m_viewport = new OcctViewport(centralContainer);
        m_viewport->setMinimumSize(200, 150);
        m_viewport->installEventFilter(this);
        m_viewport->setFaceSelectedCallback([this](const CadFaceInfo& faceInfo) {
            onCadFaceSelected(faceInfo);
            });
        m_viewport->setSurfaceHitCallback([this](const CadSurfaceHit& hit) {
            onCadSurfaceProbed(hit);
            });

        centralLayout->addWidget(m_viewport, 1);
        setCentralWidget(centralContainer);

        m_viewport->setTheme(m_isDarkMode);
        tryLoadMcpPlugin();
        loadRibbonCustomization();
        applyKeyboardShortcuts();
        statusBar()->setStyleSheet("background-color: #007acc; color: white; font-weight: bold;");
        statusBar()->showMessage("Ready - Click 'Settings...' to configure hardware/units, or probe features to begin inspection.");
    }

    void MainWindow::setupAutoHideBars() {
        const QString barStyle =
            "QToolBar { background-color: #181a1f; border: 1px solid #2c313a; spacing: 4px; padding: 2px; }"
            "QToolButton { background-color: #282c34; color: #abb2bf; border: 1px solid #3a3f4b; "
            "border-radius: 3px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
            "QToolButton:hover { background-color: #3e4451; color: #ffffff; border: 1px solid #61afef; }"
            "QToolButton:checked { background-color: #2d5a88; color: #ffffff; border: 1px solid #61afef; }";

        m_leftAutoHideBar = new QToolBar("Pinned Left Tabs", this);
        m_leftAutoHideBar->setObjectName("LeftAutoHideBar");
        m_leftAutoHideBar->setMovable(false);
        m_leftAutoHideBar->setFloatable(false);
        m_leftAutoHideBar->setStyleSheet(barStyle);
        addToolBar(Qt::LeftToolBarArea, m_leftAutoHideBar);
        m_leftAutoHideBar->hide();

        m_bottomAutoHideBar = new QToolBar("Pinned Bottom Tabs", this);
        m_bottomAutoHideBar->setObjectName("BottomAutoHideBar");
        m_bottomAutoHideBar->setMovable(false);
        m_bottomAutoHideBar->setFloatable(false);
        m_bottomAutoHideBar->setStyleSheet(barStyle);
        addToolBar(Qt::BottomToolBarArea, m_bottomAutoHideBar);
        m_bottomAutoHideBar->hide();
    }

    void MainWindow::attachPinTitleBar(QDockWidget* dock, Qt::DockWidgetArea area, bool startPinned) {
        if (!dock) return;

        auto* titleBar = new QWidget(dock);
        titleBar->setObjectName("DockCustomTitleBar");
        titleBar->setStyleSheet(
            "QWidget#DockCustomTitleBar { background-color: #282c34; border-bottom: 1px solid #3a3f4b; }"
            "QLabel { color: #ffffff; font-size: 11px; font-weight: bold; border: none; background: transparent; }"
            "QPushButton { background-color: transparent; color: #abb2bf; border: 1px solid transparent; "
            "border-radius: 3px; padding: 2px 6px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #3e4451; color: #ffffff; border: 1px solid #61afef; }"
        );

        auto* layout = new QHBoxLayout(titleBar);
        layout->setContentsMargins(8, 4, 6, 4);
        layout->setSpacing(6);

        auto* titleLbl = new QLabel(dock->windowTitle(), titleBar);
        layout->addWidget(titleLbl, 1);

        auto* pinBtn = new QPushButton(startPinned ? "📌" : "📍", titleBar);
        pinBtn->setToolTip(startPinned
            ? "Window is Pinned Open — Click to Unpin (Auto-Hide to Tab)"
            : "Window is Unpinned (Auto-Hide) — Click to Pin Open");
        connect(pinBtn, &QPushButton::clicked, this, [this, dock]() {
            const bool currentlyPinned = m_dockPinned[dock];
            setDockPinned(dock, !currentlyPinned);
            });
        layout->addWidget(pinBtn);

        auto* closeBtn = new QPushButton("✕", titleBar);
        closeBtn->setToolTip("Close Window (Re-open anytime via Right-Click Ribbon -> Windows)");
        connect(closeBtn, &QPushButton::clicked, this, [this, dock]() {
            dock->hide();
            syncAutoHideTabForDock(dock);
            });
        layout->addWidget(closeBtn);

        dock->setTitleBarWidget(titleBar);
        m_dockPinned[dock] = startPinned;
        m_dockPinButtons[dock] = pinBtn;

        QToolBar* targetBar = (area == Qt::BottomDockWidgetArea || area == Qt::TopDockWidgetArea)
            ? m_bottomAutoHideBar
            : m_leftAutoHideBar;

        QAction* tabAct = targetBar->addAction("📍 " + dock->windowTitle());
        tabAct->setCheckable(true);
        tabAct->setToolTip("Click to pop open / collapse " + dock->windowTitle());
        m_dockTabActions[dock] = tabAct;

        connect(tabAct, &QAction::triggered, this, [this, dock](bool checked) {
            if (checked) {
                dock->show();
                dock->raise();
            }
            else {
                dock->hide();
            }
            syncAutoHideTabForDock(dock);
            });

        connect(dock, &QDockWidget::visibilityChanged, this, [this, dock](bool) {
            syncAutoHideTabForDock(dock);
            });

        if (!startPinned) {
            dock->hide();
        }
        syncAutoHideTabForDock(dock);
    }

    void MainWindow::setDockPinned(QDockWidget* dock, bool pinned) {
        if (!dock) return;
        m_dockPinned[dock] = pinned;

        if (m_dockPinButtons.count(dock) && m_dockPinButtons[dock]) {
            m_dockPinButtons[dock]->setText(pinned ? "📌" : "📍");
            m_dockPinButtons[dock]->setToolTip(pinned
                ? "Window is Pinned Open — Click to Unpin (Auto-Hide to Tab)"
                : "Window is Unpinned (Auto-Hide) — Click to Pin Open");
        }

        if (pinned) {
            dock->show();
            dock->raise();
        }
        else {
            dock->hide();
        }
        syncAutoHideTabForDock(dock);
    }

    void MainWindow::syncAutoHideTabForDock(QDockWidget* dock) {
        if (!dock || !m_dockTabActions.count(dock)) return;

        QAction* act = m_dockTabActions[dock];
        const bool isPinned = m_dockPinned[dock];

        act->setVisible(!isPinned);
        act->setChecked(dock->isVisible());

        auto updateBarVisibility = [](QToolBar* bar) {
            if (!bar) return;
            bool anyVisible = false;
            for (QAction* a : bar->actions()) {
                if (a->isVisible()) {
                    anyVisible = true;
                    break;
                }
            }
            bar->setVisible(anyVisible);
            };

        updateBarVisibility(m_leftAutoHideBar);
        updateBarVisibility(m_bottomAutoHideBar);
    }

    bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
        if (watched == m_viewport && event->type() == QEvent::MouseButtonPress) {
            for (const auto& kv : m_dockPinned) {
                QDockWidget* dock = kv.first;
                const bool isPinned = kv.second;
                if (dock && !isPinned && dock->isVisible()) {
                    dock->hide();
                    syncAutoHideTabForDock(dock);
                }
            }
        }
        return QMainWindow::eventFilter(watched, event);
    }

    QFrame* MainWindow::createRibbonGroup(const QString& title, const QList<QWidget*>& widgets) {
        auto* group = new QFrame(this);
        group->setObjectName("RibbonGroup");

        auto* vLayout = new QVBoxLayout(group);
        vLayout->setContentsMargins(10, 5, 10, 3);
        vLayout->setSpacing(3);

        auto* row = new QHBoxLayout();
        row->setContentsMargins(0, 0, 0, 0);
        row->setSpacing(6);
        for (QWidget* w : widgets) {
            row->addWidget(w);
        }

        auto* lbl = new QLabel(title.toUpper(), group);
        lbl->setObjectName("RibbonGroupTitle");
        lbl->setAlignment(Qt::AlignCenter);

        vLayout->addLayout(row, 1);
        vLayout->addWidget(lbl, 0);
        return group;
    }

    QPushButton* MainWindow::createRibbonButton(const QString& iconSymbol,
        const QString& labelText,
        const QString& accentBg,
        const QString& borderCol,
        const QString& textCol) {
        auto* btn = new QPushButton(QString("%1\n%2").arg(iconSymbol, labelText), this);
        btn->setMinimumSize(78, 56);
        btn->setMaximumHeight(60);
        btn->setCursor(Qt::PointingHandCursor);

        if (m_isDarkMode) {
            btn->setStyleSheet(QString(
                "QPushButton { background-color: %1; color: %3; border: 1px solid %2; "
                "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
                "QPushButton:hover { background-color: #3e4451; border: 1px solid #61afef; color: #ffffff; }"
                "QPushButton:pressed { background-color: #181a1f; }"
            ).arg(accentBg, borderCol, textCol));
        }
        else {
            QString lightBg = (accentBg == "#282c34" || accentBg == "#1e2227") ? "#ffffff" : accentBg;
            QString lightText = (textCol == "#abb2bf" || textCol == "#e0e0e0") ? "#334155" : textCol;
            QString lightBorder = (borderCol == "#3a3f4b" || borderCol == "#4b5263") ? "#cbd5e1" : borderCol;

            btn->setStyleSheet(QString(
                "QPushButton { background-color: %1; color: %3; border: 1px solid %2; "
                "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
                "QPushButton:hover { background-color: #e2e8f0; border: 1px solid #0284c7; color: #0f172a; }"
                "QPushButton:pressed { background-color: #cbd5e1; }"
            ).arg(lightBg, lightBorder, lightText));
        }
        return btn;
    }

    void MainWindow::setupToolbar() {
        auto* ribbonTabs = new QTabWidget(this);
        ribbonTabs->setObjectName("RibbonBar");
        ribbonTabs->setFixedHeight(116);

        ribbonTabs->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(ribbonTabs, &QTabWidget::customContextMenuRequested,
            this, &MainWindow::onRibbonContextMenuRequested);

        ribbonTabs->setStyleSheet(
            "QTabWidget#RibbonBar::pane { background-color: #21252b; border-top: 1px solid #3a3f4b; border-bottom: 2px solid #181a1f; }"
            "QTabWidget#RibbonBar QTabBar::tab { background-color: #181a1f; color: #abb2bf; padding: 6px 20px; font-size: 12px; font-weight: bold; border: 1px solid #2c313a; border-bottom: none; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 3px; margin-top: 2px; }"
            "QTabWidget#RibbonBar QTabBar::tab:hover { background-color: #2c313a; color: #ffffff; }"
            "QTabWidget#RibbonBar QTabBar::tab:selected { background-color: #21252b; color: #61afef; border-top: 2px solid #61afef; }"
            "QFrame#RibbonGroup { border-right: 1px solid #3a3f4b; background: transparent; }"
            "QLabel#RibbonGroupTitle { color: #7f848e; font-size: 9px; font-weight: bold; letter-spacing: 1px; }"
            "QComboBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 4px 8px; font-weight: bold; min-width: 145px; }"
            "QComboBox QAbstractItemView { background-color: #181a1f; color: #ffffff; selection-background-color: #2d5a88; }"
        );

        setMenuWidget(ribbonTabs);
        loadRibbonCustomization();
    }

    QWidget* MainWindow::createCommandWidgetForId(const QString& cmdId) {
        auto makeStackedControl = [this](const QString& labelText, QWidget* control) -> QWidget* {
            auto* box = new QWidget(this);
            auto* v = new QVBoxLayout(box);
            v->setContentsMargins(2, 2, 2, 2);
            v->setSpacing(4);
            auto* lbl = new QLabel(labelText, box);
            lbl->setStyleSheet(m_isDarkMode
                ? "color: #abb2bf; font-size: 10px; font-weight: bold;"
                : "color: #475569; font-size: 10px; font-weight: bold;");
            v->addWidget(lbl);
            v->addWidget(control);
            v->addStretch(1);
            return box;
            };

        // --- System & Hardware ---
        if (cmdId == "sys_settings") {
            auto* btn = createRibbonButton("⚙", "Settings...", "#282c34", "#61afef", "#61afef");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onOpenSettingsClicked);
            if (!m_settingsBtn) m_settingsBtn = btn;
            return btn;
        }
        if (cmdId == "sys_mcp_toggle") {
            // If opencmm_mcp.dll is not installed, return nullptr so the Ribbon omits it
            if (!m_mcpPlugin) {
                return nullptr;
            }
            const bool running = m_mcpPlugin->isRunning();
            auto* btn = createRibbonButton(
                "🤖",
                running ? QString("MCP :%1").arg(m_mcpPlugin->serverPort()) : "MCP: OFF",
                running ? "#1e4620" : "#282c34",
                running ? "#98c379" : "#e5c07b",
                running ? "#98c379" : "#e5c07b"
            );
            btn->setCheckable(true);
            btn->setChecked(running);
            btn->setToolTip(running
                ? QString("MCP Server active on 127.0.0.1:%1 — Click to disable external AI connection").arg(m_mcpPlugin->serverPort())
                : "MCP Server stopped — Click to start listening on 127.0.0.1:8765");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onToggleMcpServerClicked);
            if (!m_mcpStatusBtn) m_mcpStatusBtn = btn;
            return btn;
        }
        if (cmdId == "sys_about") {
            auto* btn = createRibbonButton("ℹ", "About", "#282c34", "#abb2bf", "#abb2bf");
            connect(btn, &QPushButton::clicked, this, [this]() {
                QMessageBox::about(
                    this,
                    "About OpenCMM",
                    "<h3>OpenCMM — Precision Coordinate Metrology Studio</h3>"
                    "<p><b>Copyright &copy; 2026 JLS Machine LLC</b></p>"
                    "<p>OpenCMM is free open-source software licensed under the "
                    "<b>GNU General Public License v3.0 (GPL-3.0-or-later)</b>.</p>"
                    "<p>Built with Qt 6 (LGPLv3), OpenCASCADE Technology (LGPLv2.1), "
                    "Eigen 3 (MPL-2.0), and Raspberry Pi Pico 2 MicroPython.</p>"
                    "<p>This program comes with ABSOLUTELY NO WARRANTY.</p>"
                );
                });
            return btn;
        }
        if (cmdId == "sys_customize") {
            auto* btn = createRibbonButton("🛠", "Customize...", "#282c34", "#e5c07b", "#e5c07b");
            btn->setToolTip("Customize Ribbon Tabs and Commands");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onCustomizeRibbonClicked);
            return btn;
        }

        // --- Machine DRO & Point Buffer ---
        if (cmdId == "dro_zero") {
            auto* btn = createRibbonButton("🎯", "Zero XYZ");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onZeroAxesClicked);
            return btn;
        }
        if (cmdId == "dro_units") {
            auto* btn = createRibbonButton("📏", "Units (mm/in)");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onUnitToggled);
            return btn;
        }
        if (cmdId == "buf_clear_points") {
            auto* btn = createRibbonButton("🗑", "Clear Points", "#5c2b29", "#e06c75", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onClearPointsClicked);
            return btn;
        }

        // --- Offline Simulator & CAD Authoring ---
        if (cmdId == "sim_target_combo") {
            auto* combo = new QComboBox(this);
            combo->addItem("Master Ball Ø25.4mm", 0);
            combo->addItem("Bore #1 Ø20.0mm (50,30)", 1);
            combo->addItem("Bore #2 Ø12.0mm (100,30)", 2);
            combo->addItem("Top Datum Plane (Z=20)", 3);
            combo->addItem("Front X-Edge (Y=0)", 4);
            combo->addItem("Left Y-Edge (X=0)", 5);
            if (!m_simTargetCombo) {
                m_simTargetCombo = combo;
            }
            else {
                combo->setCurrentIndex(m_simTargetCombo->currentIndex());
                connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), m_simTargetCombo, &QComboBox::setCurrentIndex);
                connect(m_simTargetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), combo, &QComboBox::setCurrentIndex);
            }
            return makeStackedControl("Virtual Target:", combo);
        }
        if (cmdId == "sim_hit") {
            auto* btn = createRibbonButton("⚡", "Simulate Hit", "#d19a66", "#e5c07b", "#1e2227");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onSimulateHitClicked);
            return btn;
        }
        if (cmdId == "cad_auto_probe") {
            auto* btn = createRibbonButton("🎯", "Auto-Probe CAD", "#2b6cb0", "#61afef", "#ffffff");
            btn->setToolTip("Automatically sample and solve the currently selected 3D CAD face (or Ctrl+Left-Click on CAD to probe individual points)");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onAutoProbeCadFaceClicked);
            return btn;
        }
        if (cmdId == "prog_create_feature") {
            auto* btn = createRibbonButton("➕", "Create Feature", "#1e4620", "#98c379", "#ffffff");
            btn->setToolTip("Create a planned offline inspection feature (Cylinder, Plane, Circle, Sphere, Line) from CAD or print nominals");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onCreateOfflineFeatureClicked);
            return btn;
        }

        // --- DMIS 5.3 Program ---
        if (cmdId == "dmis_save") {
            auto* btn = createRibbonButton("💾", "Save DMIS", "#1e3a8a", "#60a5fa", "#ffffff");
            btn->setToolTip("Export solved features, CAD touch points, and 3-2-1 Datums to a DMIS 5.3 (.dmis) program");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onExportDmisProgramClicked);
            return btn;
        }
        if (cmdId == "dmis_load") {
            auto* btn = createRibbonButton("📂", "Load DMIS", "#1f7a8c", "#61afef", "#ffffff");
            btn->setToolTip("Load a DMIS 5.3 (.dmis) inspection program and display 3D target guide markers");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onLoadDmisProgramClicked);
            return btn;
        }
        if (cmdId == "dmis_run") {
            const bool running = (m_dmisPlaybackTimer && m_dmisPlaybackTimer->isActive());
            auto* btn = createRibbonButton(running ? "⏹" : "▶", running ? "Stop DMIS" : "Run DMIS", "#2f855a", "#98c379", "#ffffff");
            btn->setToolTip("Animate the Digital Twin CMM through all loaded DMIS target points offline");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onToggleAutoRunDmisClicked);
            if (!m_autoRunDmisBtnHome) m_autoRunDmisBtnHome = btn;
            else if (!m_autoRunDmisBtnCad) m_autoRunDmisBtnCad = btn;
            return btn;
        }
        if (cmdId == "dmis_preflight") {
            auto* btn = createRibbonButton("🔍", "Pre-Flight", "#2b4c7e", "#60a5fa", "#ffffff");
            btn->setToolTip("Run automated collision and clearance safety checks on all loaded DMIS inspection features");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onRunDmisPreFlightCheckClicked);
            return btn;
        }

        // --- Probe & Alignment ---
        if (cmdId == "probe_comp_combo") {
            auto* combo = new QComboBox(this);
            combo->addItem("External (-R)", static_cast<int>(ProbeCompMode::External));
            combo->addItem("Internal (+R)", static_cast<int>(ProbeCompMode::Internal));
            combo->addItem("Uncomp (0)", static_cast<int>(ProbeCompMode::Uncompensated));
            if (!m_compModeCombo) {
                m_compModeCombo = combo;
            }
            else {
                combo->setCurrentIndex(m_compModeCombo->currentIndex());
                connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), m_compModeCombo, &QComboBox::setCurrentIndex);
                connect(m_compModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), combo, &QComboBox::setCurrentIndex);
            }
            return makeStackedControl("Tip Compensation:", combo);
        }
        if (cmdId == "probe_calibrate") {
            auto* btn = createRibbonButton("🔴", "Calibrate Tip", "#5a32a3", "#8957e5", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onCalibrateProbeClicked);
            return btn;
        }
        if (cmdId == "datum_set_a") {
            auto* btn = createRibbonButton("🅰", "Set Datum A", "#1e3a8a", "#60a5fa", "#ffffff");
            btn->setToolTip("Probe selected CAD face (or use highlighted feature) as Primary Datum A (Level +Z)");
            connect(btn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("A"); });
            return btn;
        }
        if (cmdId == "datum_set_b") {
            auto* btn = createRibbonButton("🅱", "Set Datum B", "#14532d", "#4ade80", "#ffffff");
            btn->setToolTip("Probe selected CAD face (or use highlighted feature) as Secondary Datum B (Rotate +X)");
            connect(btn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("B"); });
            return btn;
        }
        if (cmdId == "datum_set_c") {
            auto* btn = createRibbonButton("🅲", "Set Datum C", "#78350f", "#fbbf24", "#ffffff");
            btn->setToolTip("Probe selected CAD face (or use highlighted feature) as Tertiary Datum C (Origin 0,0,0)");
            connect(btn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("C"); });
            return btn;
        }
        if (cmdId == "pcs_align_321") {
            auto* btn = createRibbonButton("📐", "3-2-1 Align...", "#9b2c2c", "#e06c75", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onRun321AlignmentClicked);
            return btn;
        }
        if (cmdId == "pcs_coord_toggle") {
            auto* btn = createRibbonButton("🌐", m_isPartAligned ? "PCS (Aligned)" : "Coord: MCS", "#3a3f4b", "#e5c07b", "#e5c07b");
            btn->setCheckable(true);
            btn->setChecked(m_isPartAligned);
            connect(btn, &QPushButton::clicked, this, [this, btn](bool checked) {
                if (!m_isPartAligned && checked) {
                    QMessageBox::warning(this, "No Alignment Active", "Please run a 3-2-1 alignment first.");
                    btn->setChecked(false);
                    return;
                }
                if (checked) {
                    btn->setText("🌐\nPCS (Aligned)");
                    btn->setStyleSheet("QPushButton { background-color: #1e4620; color: #98c379; border: 1px solid #98c379; border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
                }
                else {
                    btn->setText("🌐\nCoord: MCS");
                    btn->setStyleSheet("QPushButton { background-color: #3a3f4b; color: #e5c07b; border: 1px solid #e5c07b; border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
                }
                updateDroDisplay();
                });
            if (!m_alignStatusBtn) m_alignStatusBtn = btn;
            return btn;
        }

        // --- Measure ---
        if (cmdId == "meas_workplane") {
            auto* combo = new QComboBox(this);
            combo->addItem("XY (+Z)", static_cast<int>(StandardWorkplane::XY_PlusZ));
            combo->addItem("YZ (+X)", static_cast<int>(StandardWorkplane::YZ_PlusX));
            combo->addItem("ZX (+Y)", static_cast<int>(StandardWorkplane::ZX_PlusY));
            combo->addItem("Last Fitted Plane", static_cast<int>(StandardWorkplane::CustomPlane));
            if (!m_workplaneCombo) {
                m_workplaneCombo = combo;
            }
            else {
                combo->setCurrentIndex(m_workplaneCombo->currentIndex());
                connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), m_workplaneCombo, &QComboBox::setCurrentIndex);
                connect(m_workplaneCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), combo, &QComboBox::setCurrentIndex);
            }
            return makeStackedControl("2D Workplane:", combo);
        }
        if (cmdId == "meas_style") {
            auto* combo = new QComboBox(this);
            combo->addItem("Fully Rendered (Shaded)", 0);
            combo->addItem("Wireframe", 1);
            connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
                if (m_viewport) {
                    m_viewport->setFittedFeaturesWireframe(index == 1);
                }
                });
            if (!m_featureRenderCombo) m_featureRenderCombo = combo;
            return makeStackedControl("Feature Style:", combo);
        }
        if (cmdId == "meas_fit_plane") {
            auto* btn = createRibbonButton("▱", "Fit Plane");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onFitPlaneClicked);
            return btn;
        }
        if (cmdId == "meas_fit_sphere") {
            auto* btn = createRibbonButton("🌐", "Fit Sphere");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onFitSphereClicked);
            return btn;
        }
        if (cmdId == "meas_fit_cylinder") {
            auto* btn = createRibbonButton("⌭", "Fit Cylinder", "#2b6cb0", "#61afef", "#ffffff");
            btn->setToolTip("Solve a 3D Cylinder (axis, diameter, and cylindricity) from 5+ multi-depth points");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onFitCylinderClicked);
            return btn;
        }
        if (cmdId == "meas_fit_circle") {
            auto* btn = createRibbonButton("⭕", "Fit 2D Circle", "#2b6cb0", "#61afef", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onFitCircleClicked);
            return btn;
        }
        if (cmdId == "meas_fit_line") {
            auto* btn = createRibbonButton("╱", "Fit 2D Line", "#2b6cb0", "#61afef", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onFitLineClicked);
            return btn;
        }
        if (cmdId == "meas_relation") {
            auto* btn = createRibbonButton("↔", "Distance / ∠", "#2f855a", "#98c379", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onMeasureRelationClicked);
            return btn;
        }

        // --- CAD & Deviation ---
        if (cmdId == "cad_load_step") {
            auto* btn = createRibbonButton("📦", "Load STEP", "#2d5a88", "#61afef", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onLoadStepClicked);
            return btn;
        }
        if (cmdId == "cad_orient") {
            auto* btn = createRibbonButton("🧭", "Orient CAD...", "#3b6998", "#61afef", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onTransformCadClicked);
            return btn;
        }
        if (cmdId == "cad_surface_dev") {
            auto* btn = createRibbonButton("🌡", "Surface Dev", "#0d9488", "#4fd1c5", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onEvaluateCadDeviationClicked);
            return btn;
        }

        // --- Reporting ---
        if (cmdId == "rep_edit_tol") {
            auto* btn = createRibbonButton("±", "Edit Tol...", "#2d3748", "#e5c07b", "#e5c07b");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onEditToleranceClicked);
            return btn;
        }
        if (cmdId == "rep_clear_all") {
            auto* btn = createRibbonButton("🧹", "Clear All", "#5c2b29", "#e06c75", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onClearAllFeaturesClicked);
            return btn;
        }
        if (cmdId == "rep_export_report") {
            auto* btn = createRibbonButton("📄", "Export Report", "#1f7a8c", "#61afef", "#ffffff");
            connect(btn, &QPushButton::clicked, this, &MainWindow::onExportReportClicked);
            return btn;
        }

        return nullptr;
    }

    void MainWindow::setupDroDock() {
        m_droDock = new QDockWidget("LIVE CMM DRO & PROBE STATUS", this);
        m_droDock->setObjectName("LiveDroDock");
        // Removed setMinimumHeight(280) so it can size compactly without forcing an 845px minimum window height

        auto* scrollArea = new QScrollArea(m_droDock);
        scrollArea->setWidgetResizable(true);
        scrollArea->setFrameShape(QFrame::NoFrame);
        scrollArea->setStyleSheet("QScrollArea { background: transparent; border: none; }");

        m_droContainer = new QWidget(scrollArea);
        m_droContainer->setStyleSheet(m_isDarkMode ? "background-color: #181a1f;" : "background-color: #f8fafc;");
        auto* layout = new QVBoxLayout(m_droContainer);
        layout->setContentsMargins(4, 4, 4, 4);
        layout->setSpacing(4);

        auto* grid = new QGridLayout();
        grid->setContentsMargins(0, 0, 0, 0);
        grid->setSpacing(2);

        const char* axisNames[3] = { "X", "Y", "Z" };
        const char* axisColors[3] = { "#e06c75", "#98c379", "#61afef" };
        QLabel** droPtrs[3] = { &m_droX, &m_droY, &m_droZ };

        for (int i = 0; i < 3; ++i) {
            auto* axisLbl = new QLabel(axisNames[i], m_droContainer);
            axisLbl->setStyleSheet(QString("color: %1; font-size: 22px; font-weight: bold; font-family: 'Consolas', monospace;").arg(axisColors[i]));

            *droPtrs[i] = new QLabel("   0.0000", m_droContainer);
            (*droPtrs[i])->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            (*droPtrs[i])->setStyleSheet("color: #0f172a; background-color: #ffffff; border: 1px solid #94a3b8; border-radius: 4px; padding: 3px 6px; font-size: 22px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 150px;");

            m_unitLabels[i] = new QLabel("mm", m_droContainer);
            m_unitLabels[i]->setStyleSheet("color: #64748b; font-size: 13px; font-weight: bold;");

            grid->addWidget(axisLbl, i, 0);
            grid->addWidget(*droPtrs[i], i, 1);
            grid->addWidget(m_unitLabels[i], i, 2);
        }

        layout->addLayout(grid);
        m_hitCountLabel = new QLabel("Active Feature Hits: 0", m_droContainer);
        layout->addWidget(m_hitCountLabel);

        layout->addLayout(grid);
        m_hitCountLabel = new QLabel("Active Feature Hits: 0", m_droContainer);
        layout->addWidget(m_hitCountLabel);

        auto* startGuidedBtn = new QPushButton("🧭 Start Guided Part Probe (F6)", m_droContainer);
        startGuidedBtn->setCursor(Qt::PointingHandCursor);
        startGuidedBtn->setToolTip("Walk through all programmed features step-by-step with your manual probe");
        startGuidedBtn->setStyleSheet(
            "QPushButton { background-color: #1e4620; color: #98c379; border: 1px solid #98c379; "
            "border-radius: 4px; padding: 6px 10px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #276749; color: #ffffff; }"
        );
        connect(startGuidedBtn, &QPushButton::clicked, this, &MainWindow::onStartManualGuidedInspectionClicked);
        layout->addWidget(startGuidedBtn);

        auto* probeGroup = new QGroupBox("ACTIVE PROBE CALIBRATION", m_droContainer);
        auto* probeLayout = new QVBoxLayout(probeGroup);
        probeLayout->setContentsMargins(6, 6, 6, 6);
        probeLayout->setSpacing(2);

        m_probeStatusBadge = new QLabel("UNINITIALIZED (Nominal Default)", probeGroup);
        m_probeDiamLabel = new QLabel("Effective Tip Ø: 3.0000 mm", probeGroup);
        m_probeRadLabel = new QLabel("Effective Radius (R_eff): 1.5000 mm", probeGroup);
        m_probeLobingLabel = new QLabel("Sphericity / Lobing: --", probeGroup);

        probeLayout->addWidget(m_probeStatusBadge);
        probeLayout->addWidget(m_probeDiamLabel);
        probeLayout->addWidget(m_probeRadLabel);
        probeLayout->addWidget(m_probeLobingLabel);
        layout->addWidget(probeGroup);
        layout->addStretch();

        scrollArea->setWidget(m_droContainer);
        m_droDock->setWidget(scrollArea);
        addDockWidget(Qt::LeftDockWidgetArea, m_droDock);
        attachPinTitleBar(m_droDock, Qt::LeftDockWidgetArea, true);
    }

    void MainWindow::setupFeatureTreeDock() {
        m_treeDock = new QDockWidget("INSPECTION FEATURE TREE", this);
        m_treeDock->setObjectName("FeatureTreeDock");
        m_treeDock->setMinimumHeight(250); // <--- Prevent vertical squishing
        m_treeContainer = new QWidget(m_treeDock);
        m_treeContainer->setStyleSheet(m_isDarkMode ? "background-color: #181a1f;" : "background-color: #f8fafc;");
        auto* dockLayout = new QVBoxLayout(m_treeContainer);
        dockLayout->setContentsMargins(4, 4, 4, 4);

        m_featureTree = new QTreeWidget(m_treeContainer);
        m_featureTree->setMinimumHeight(180); // <--- Guarantee tree view space
        m_featureTree->setColumnCount(5);
        m_featureTree->setHeaderLabels({ "Feature / Parameter", "Nominal", "Actual", "±Tol", "Dev / Status" });

        m_featureTree->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(m_featureTree, &QTreeWidget::customContextMenuRequested,
            this, &MainWindow::onFeatureTreeContextMenuRequested);

        auto* deleteKeyShortcut = new QShortcut(QKeySequence::Delete, m_featureTree);
        connect(deleteKeyShortcut, &QShortcut::activated, this, &MainWindow::onDeleteSelectedFeature);

        auto* backspaceKeyShortcut = new QShortcut(QKeySequence(Qt::Key_Backspace), m_featureTree);
        connect(backspaceKeyShortcut, &QShortcut::activated, this, &MainWindow::onDeleteSelectedFeature);

        m_activePointsGroup = new QTreeWidgetItem(m_featureTree, { "Active Probe Buffer", "", "0 pts", "", "READY" });
        m_activePointsGroup->setExpanded(true);

        dockLayout->addWidget(m_featureTree);
        m_treeDock->setWidget(m_treeContainer);
        addDockWidget(Qt::LeftDockWidgetArea, m_treeDock);
        attachPinTitleBar(m_treeDock, Qt::LeftDockWidgetArea, true);

        connect(m_featureTree, &QTreeWidget::currentItemChanged, this,
            [this](QTreeWidgetItem* current, QTreeWidgetItem* /*previous*/) {
                if (!m_viewport) return;
                QTreeWidgetItem* top = current;
                while (top && top->parent() != nullptr) {
                    top = top->parent();
                }
                if (top && top->data(0, ROLE_FEATURE_ID).isValid()) {
                    m_viewport->highlightFittedFeature(top->data(0, ROLE_FEATURE_ID).toInt());
                }
                else {
                    m_viewport->highlightFittedFeature(-1);
                }
            });
    }

    void MainWindow::setupCollisionLogDock() {
        m_collisionDock = new QDockWidget("COLLISION & CLEARANCE LOG", this);
        m_collisionDock->setObjectName("CollisionLogDock");
        m_collisionDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
        m_collisionDock->setMaximumHeight(120);

        m_collisionContainer = new QWidget(m_collisionDock);
        m_collisionContainer->setStyleSheet(m_isDarkMode ? "background-color: #181a1f;" : "background-color: #f8fafc;");
        auto* layout = new QVBoxLayout(m_collisionContainer);
        layout->setContentsMargins(4, 2, 4, 2);
        layout->setSpacing(2);

        m_collisionLogText = new QTextEdit(m_collisionContainer);
        m_collisionLogText->setReadOnly(true);
        m_collisionLogText->setMinimumHeight(35); // Reduced from 45
        m_collisionLogText->setStyleSheet(
            "QTextEdit { background-color: #0d0f12; color: #98c379; border: 1px solid #3a3f4b; font-family: 'Consolas', monospace; font-size: 11px; }"
        );
        layout->addWidget(m_collisionLogText);

        auto* btnRow = new QHBoxLayout();
        auto* clearLogBtn = new QPushButton("Clear Log", m_collisionContainer);
        connect(clearLogBtn, &QPushButton::clicked, this, [this]() {
            if (m_collisionLogText) m_collisionLogText->clear();
            });
        btnRow->addStretch(1);
        btnRow->addWidget(clearLogBtn);
        layout->addLayout(btnRow);

        m_collisionDock->setWidget(m_collisionContainer);
        addDockWidget(Qt::BottomDockWidgetArea, m_collisionDock);

        attachPinTitleBar(m_collisionDock, Qt::BottomDockWidgetArea, false);
    }

    void MainWindow::evaluateProbeCollisionSafety(const Eigen::Vector3d& pos) {
        if (!m_viewport || !m_viewport->hasCadModel()) return;

        TopoDS_Shape cadShape = m_viewport->getCombinedCadShape();
        CollisionCheckResult colRes = MetrologyMath::checkProbeCollision(
            pos,
            m_effectiveProbeRadius,
            cadShape,
            15.0,
            1.5
        );

        static bool lastCollisionState = false;
        static bool lastWarningState = false;

        if (colRes.collision) {
            if (m_dmisPlaybackTimer && m_dmisPlaybackTimer->isActive()) {
                stopDmisAutoRun();
            }
            playSolveChime(false);
            m_viewport->setProbeCollisionState(true, false);
            statusBar()->setStyleSheet("background-color: #b91c1c; color: white; font-weight: bold;");
            statusBar()->showMessage(QString::fromStdString(colRes.message));

            if (!lastCollisionState) {
                logCollisionEvent(colRes.message, true, false);
                lastCollisionState = true;
            }
        }
        else if (colRes.warningProximity) {
            lastCollisionState = false;
            m_viewport->setProbeCollisionState(false, true);
            statusBar()->setStyleSheet("background-color: #d97706; color: white; font-weight: bold;");
            statusBar()->showMessage(QString::fromStdString(colRes.message));

            if (!lastWarningState) {
                logCollisionEvent(colRes.message, false, true);
                lastWarningState = true;
            }
        }
        else {
            if (lastCollisionState || lastWarningState) {
                logCollisionEvent("Probe returned to safe clearance zone.", false, false);
            }
            lastCollisionState = false;
            lastWarningState = false;
            m_viewport->setProbeCollisionState(false, false);
            statusBar()->setStyleSheet(m_isDarkMode
                ? "background-color: #007acc; color: white; font-weight: bold;"
                : "background-color: #0284c7; color: white; font-weight: bold;");
        }
    }

    void MainWindow::logCollisionEvent(const std::string& message, bool isCollision, bool isWarning) {
        if (!m_collisionLogText) return;

        if ((isCollision || isWarning) && m_collisionDock) {
            m_collisionDock->show();
            m_collisionDock->raise();
            syncAutoHideTabForDock(m_collisionDock);
        }

        const QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
        QString color = "#98c379";
        QString tag = "[CLEAR]";

        if (isCollision) {
            color = "#e06c75";
            tag = "[COLLISION]";
        }
        else if (isWarning) {
            color = "#e5c07b";
            tag = "[PROXIMITY WARNING]";
        }

        const QString htmlEntry = QString("<span style='color: #7f848e;'>[%1]</span> <span style='color: %2; font-weight: bold;'>%3</span> <span style='color: #abb2bf;'>%4</span>")
            .arg(timestamp, color, tag, QString::fromStdString(message));

        m_collisionLogText->append(htmlEntry);
    }

    void MainWindow::onOpenSettingsClicked() {
        SettingsDialog dlg(this);

        dlg.setCurrentUnitSystem(m_unitSystem);
        dlg.setDefaultToleranceMm(m_defaultTolMm);
        dlg.setAudioChimesEnabled(m_audioEnabled);
        dlg.setDarkModeEnabled(m_isDarkMode);
        dlg.setSelectedMachinePath(m_machinePackagePath);
        connect(&dlg, &SettingsDialog::resetWindowLayoutRequested, this, &MainWindow::resetWindowLayout);

        if (dlg.exec() != QDialog::Accepted) return;

        if (m_controller) {
            dlg.setAxisCalibration(Eigen::Vector3d(0.001, 0.001, 0.001), Eigen::Vector3i(1, 1, 1));
        }

        QStringList ports;
        const auto availablePorts = QSerialPortInfo::availablePorts();
        for (const QSerialPortInfo& info : availablePorts) {
            ports.append(info.portName());
        }
        if (m_currentPortName.isEmpty() && !ports.isEmpty()) {
            m_currentPortName = ports.first();
        }
        dlg.setAvailableSerialPorts(ports, m_currentPortName);

        connect(&dlg, &SettingsDialog::machinePackageLoaded, this, [this, &dlg](const QString& path) {
            if (m_viewport && m_viewport->loadMachinePackage(path)) {
                m_machinePackagePath = path;
                const auto& cfg = m_viewport->machineConfig();

                if (m_controller) {
                    m_controller->setAxisCalibration(cfg.scaleMm, cfg.sign);
                }
                dlg.setAxisCalibration(cfg.scaleMm, cfg.sign);

                const QString mName = QString::fromStdString(cfg.machineName);
                const QString mMfg = QString::fromStdString(cfg.manufacturer);
                setWindowTitle(QString("OpenCMM - %1 (%2)").arg(mName, mMfg));
                statusBar()->showMessage(QString("Loaded Machine Post: %1 (%2)").arg(mName, path));
            }
            else {
                QMessageBox::warning(&dlg, "Machine Package Load Failed", "Could not parse the selected package.");
            }
            });

        connect(&dlg, &SettingsDialog::machinePackageReset, this, [this]() {
            m_machinePackagePath.clear();
            if (m_viewport) {
                m_viewport->resetDefaultMachine();
                if (m_controller) {
                    const auto& cfg = m_viewport->machineConfig();
                    m_controller->setAxisCalibration(cfg.scaleMm, cfg.sign);
                }
            }
            setWindowTitle("OpenCMM - Precision Coordinate Metrology Studio");
            statusBar()->showMessage("Reset Digital Twin to Default Moving-Bridge CMM.");
            });

        connect(&dlg, &SettingsDialog::factoryResetRequested, this, [this]() {
            AppSettings::resetToDefaults();

            m_machinePackagePath.clear();
            m_currentPortName.clear();
            m_defaultTolMm = 0.025;
            m_audioEnabled = true;
            m_isDarkMode = true;

            m_isProbeCalibrated = false;
            m_certifiedBallDiameterMm = 25.4000;
            m_effectiveProbeRadius = 1.5000;
            m_lastProbeLobingMm = 0.0;

            if (m_unitSystem != UnitSystem::Millimeters) {
                onUnitToggled();
            }

            if (m_controller) {
                m_controller->setAxisCalibration(Eigen::Vector3d(0.001, 0.001, 0.001), Eigen::Vector3i(1, 1, 1));
            }

            onThemeToggled(true);
            updateDroDisplay();
            updateProbeStatusDisplay();

            setWindowTitle("OpenCMM - Precision Coordinate Metrology Studio");
            statusBar()->showMessage("Restored all OpenCMM settings to factory defaults.");
            });

        connect(&dlg, &SettingsDialog::serialPortConnectRequested, this, [this](const QString& portName, int baudRate) {
            if (!portName.isEmpty() && m_controller) {
                m_currentPortName = portName;
                m_controller->connectToPort(portName, baudRate);
            }
            });

        if (m_controller) {
            connect(m_controller, &ICmmController::rawLineReceived, &dlg, &SettingsDialog::appendSerialLog);
            connect(m_controller, &ICmmController::rawCountsUpdated, &dlg, &SettingsDialog::updateRawDiagnostics);
            connect(&dlg, &SettingsDialog::sendSerialCommandRequested, m_controller, &ICmmController::sendRawCommand);
        }

        if (dlg.exec() != QDialog::Accepted) return;

        onThemeToggled(dlg.isDarkModeEnabled());

        m_defaultTolMm = dlg.getDefaultToleranceMm();
        m_audioEnabled = dlg.areAudioChimesEnabled();
        m_machinePackagePath = dlg.getSelectedMachinePath();
        m_isDarkMode = dlg.isDarkModeEnabled();

        if (m_controller) {
            m_controller->setAxisCalibration(dlg.getAxisScale(), dlg.getAxisSign());
        }

        UnitSystem newUnits = dlg.getSelectedUnitSystem();
        if (newUnits != m_unitSystem) {
            onUnitToggled();
        }

        AppSettings::saveSettings(
            m_machinePackagePath,
            m_currentPortName,
            115200,
            dlg.getAxisScale(),
            dlg.getAxisSign(),
            m_unitSystem,
            m_defaultTolMm,
            m_audioEnabled,
            m_isDarkMode
        );
        connect(&dlg, &SettingsDialog::resetWindowLayoutRequested, this, &MainWindow::resetWindowLayout);
        statusBar()->showMessage("Settings updated and saved successfully.");
    }

    void MainWindow::playHitChime() const {
        if (!m_audioEnabled) return;
#ifdef _WIN32
        std::thread([]() { Beep(1250, 35); }).detach();
#endif
    }

    void MainWindow::playSolveChime(bool passed) const {
        if (!m_audioEnabled) return;
#ifdef _WIN32
        std::thread([passed]() {
            if (passed) { Beep(980, 40); Beep(1480, 65); }
            else { Beep(420, 90); Beep(340, 120); }
            }).detach();
#endif
    }

    void MainWindow::applyEvaluationToTreeItem(QTreeWidgetItem* item, double actualMm, bool hasNominal, double nominalMm, double plusTolMm, double minusTolMm) {
        if (!item) return;
        item->setData(0, ROLE_ACTUAL_MM, actualMm);
        item->setData(0, ROLE_HAS_NOMINAL, hasNominal);
        item->setData(0, ROLE_NOMINAL_MM, nominalMm);
        item->setData(0, ROLE_PLUS_TOL_MM, plusTolMm);
        item->setData(0, ROLE_MINUS_TOL_MM, minusTolMm);

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        item->setText(2, QString("%1%2").arg(toDisplayUnit(actualMm, m_unitSystem), 0, 'f', decimals).arg(unitStr));

        if (!hasNominal) {
            item->setText(1, "--");
            item->setText(3, "--");
            item->setText(4, "MEASURED");
            item->setForeground(4, QBrush(QColor("#61afef")));
            return;
        }

        const double devMm = actualMm - nominalMm;
        const bool passed = (devMm <= plusTolMm + 1e-7) && (devMm >= -minusTolMm - 1e-7);

        item->setText(1, QString("%1%2").arg(toDisplayUnit(nominalMm, m_unitSystem), 0, 'f', decimals).arg(unitStr));
        item->setText(3, QString("±%1").arg(toDisplayUnit(plusTolMm, m_unitSystem), 0, 'f', decimals));

        const double dispDev = toDisplayUnit(devMm, m_unitSystem);
        const QString signPrefix = (dispDev >= 0.0) ? "+" : "";
        const QString statusTag = passed ? "PASS" : "FAIL";

        item->setText(4, QString("%1%2 (%3)").arg(signPrefix).arg(dispDev, 0, 'f', decimals).arg(statusTag));
        item->setForeground(4, QBrush(QColor(passed ? "#98c379" : "#e06c75")));
    }

    ProbeCompMode MainWindow::currentCompMode() const {
        if (!m_compModeCombo) return ProbeCompMode::External;
        return static_cast<ProbeCompMode>(m_compModeCombo->currentData().toInt());
    }

    WorkplaneFrame MainWindow::currentWorkplane() const {
        if (!m_workplaneCombo) return WorkplaneFrame::fromStandard(StandardWorkplane::XY_PlusZ);
        auto wp = static_cast<StandardWorkplane>(m_workplaneCombo->currentData().toInt());
        if (wp == StandardWorkplane::CustomPlane && m_hasCustomPlane) {
            return m_lastFittedPlaneFrame;
        }
        return WorkplaneFrame::fromStandard(wp);
    }

    void MainWindow::updateProbeStatusDisplay() {
        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        if (m_isProbeCalibrated) {
            m_probeStatusBadge->setText("CALIBRATED ON MASTER BALL");
            m_probeStatusBadge->setStyleSheet("color: #98c379; font-size: 12px; font-weight: bold;");
            m_probeLobingLabel->setText(QString("Sphericity / Lobing: %1%2").arg(toDisplayUnit(m_lastProbeLobingMm, m_unitSystem), 0, 'f', decimals).arg(unitStr));
        }
        else {
            m_probeStatusBadge->setText("DEFAULT NOMINAL (Uncalibrated)");
            m_probeStatusBadge->setStyleSheet("color: #e5c07b; font-size: 12px; font-weight: bold;");
            m_probeLobingLabel->setText("Sphericity / Lobing: --");
        }

        m_probeDiamLabel->setText(QString("Effective Tip Ø: %1%2").arg(toDisplayUnit(m_effectiveProbeRadius * 2.0, m_unitSystem), 0, 'f', decimals).arg(unitStr));
        m_probeRadLabel->setText(QString("Effective Radius (R_eff): %1%2").arg(toDisplayUnit(m_effectiveProbeRadius, m_unitSystem), 0, 'f', decimals).arg(unitStr));
    }

    void MainWindow::onConnectionChanged(bool connected, const QString& message) {
        if (connected) {
            if (m_settingsBtn) {
                m_settingsBtn->setText("⚙\nConnected");
                m_settingsBtn->setStyleSheet("QPushButton { background-color: #1e3a29; color: #98c379; border: 1px solid #98c379; border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
            }
            statusBar()->setStyleSheet("background-color: #28a745; color: white; font-weight: bold;");
        }
        else {
            if (m_settingsBtn) {
                m_settingsBtn->setText("⚙\nSettings...");
                m_settingsBtn->setStyleSheet("QPushButton { background-color: #282c34; color: #61afef; border: 1px solid #61afef; border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
            }
            statusBar()->setStyleSheet("background-color: #007acc; color: white; font-weight: bold;");
        }
        statusBar()->showMessage(message);
    }

    void MainWindow::onZeroAxesClicked() {
        m_controller->zeroAxes();
        m_lastPosition = { 0.0, 0.0, 0.0 };
        updateDroDisplay();
        if (m_viewport) {
            m_viewport->updateProbePosition(0.0, 0.0, 0.0);
        }
        statusBar()->showMessage("Axes zeroed at current machine position.");
    }

    void MainWindow::onUnitToggled() {
        if (m_unitSystem == UnitSystem::Millimeters) {
            m_unitSystem = UnitSystem::Inches;
            for (auto* lbl : m_unitLabels) { if (lbl) lbl->setText("in"); }
        }
        else {
            m_unitSystem = UnitSystem::Millimeters;
            for (auto* lbl : m_unitLabels) { if (lbl) lbl->setText("mm"); }
        }
        updateDroDisplay();
        updateProbeStatusDisplay();

        for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
            QTreeWidgetItem* item = m_featureTree->topLevelItem(i);
            if (item != m_activePointsGroup && item->data(0, ROLE_ACTUAL_MM).isValid()) {
                applyEvaluationToTreeItem(
                    item,
                    item->data(0, ROLE_ACTUAL_MM).toDouble(),
                    item->data(0, ROLE_HAS_NOMINAL).toBool(),
                    item->data(0, ROLE_NOMINAL_MM).toDouble(),
                    item->data(0, ROLE_PLUS_TOL_MM).toDouble(),
                    item->data(0, ROLE_MINUS_TOL_MM).toDouble()
                );
            }
        }
    }

    void MainWindow::onLoadStepClicked() {
        QString filePath = QFileDialog::getOpenFileName(this, "Load STEP CAD Model", QString(), "STEP Files (*.stp *.step *.STP *.STEP)");
        if (!filePath.isEmpty() && m_viewport) {
            m_viewport->loadStepFile(filePath);
            statusBar()->showMessage("Loaded STEP model: " + filePath);
        }
    }

    void MainWindow::onTransformCadClicked() {
        if (!m_viewport || !m_viewport->hasCadModel()) {
            QMessageBox::information(this, "No STEP Model Loaded", "Please load a STEP model first.");
            return;
        }

        QDialog dlg(this);
        dlg.setWindowTitle("Orient & Position CAD Model on Granite");
        auto* layout = new QVBoxLayout(&dlg);
        auto* form = new QFormLayout();

        auto* dxSpin = new QDoubleSpinBox(&dlg); dxSpin->setRange(-5000.0, 5000.0); dxSpin->setDecimals(3); dxSpin->setSuffix(" mm");
        auto* dySpin = new QDoubleSpinBox(&dlg); dySpin->setRange(-5000.0, 5000.0); dySpin->setDecimals(3); dySpin->setSuffix(" mm");
        auto* dzSpin = new QDoubleSpinBox(&dlg); dzSpin->setRange(-5000.0, 5000.0); dzSpin->setDecimals(3); dzSpin->setSuffix(" mm");

        auto* rxSpin = new QDoubleSpinBox(&dlg); rxSpin->setRange(-360.0, 360.0); rxSpin->setDecimals(2); rxSpin->setSuffix("°");
        auto* rySpin = new QDoubleSpinBox(&dlg); rySpin->setRange(-360.0, 360.0); rySpin->setDecimals(2); rySpin->setSuffix("°");
        auto* rzSpin = new QDoubleSpinBox(&dlg); rzSpin->setRange(-360.0, 360.0); rzSpin->setDecimals(2); rzSpin->setSuffix("°");

        form->addRow("Translate dX:", dxSpin);
        form->addRow("Translate dY:", dySpin);
        form->addRow("Translate dZ:", dzSpin);
        form->addRow("Rotate rX:", rxSpin);
        form->addRow("Rotate rY:", rySpin);
        form->addRow("Rotate rZ:", rzSpin);
        layout->addLayout(form);

        auto* quickRow = new QHBoxLayout();
        auto* btnDrop = new QPushButton("Drop to Granite (Z=0)", &dlg);
        auto* btnCenter = new QPushButton("Center XY", &dlg);
        auto* btnOrigin = new QPushButton("Min Corner → (0,0,0)", &dlg);
        auto* btnAlignFace = new QPushButton("Level Selected Face +Z", &dlg);
        auto* btnReset = new QPushButton("Reset Transform", &dlg);

        quickRow->addWidget(btnDrop);
        quickRow->addWidget(btnCenter);
        quickRow->addWidget(btnOrigin);
        quickRow->addWidget(btnAlignFace);
        quickRow->addWidget(btnReset);
        layout->addLayout(quickRow);

        connect(btnDrop, &QPushButton::clicked, this, [this]() { m_viewport->dropCadModelsToGranite(0.0); });
        connect(btnCenter, &QPushButton::clicked, this, [this]() { m_viewport->centerCadModelsXY(0.0, 0.0); });
        connect(btnOrigin, &QPushButton::clicked, this, [this]() { m_viewport->snapCadMinCornerToOrigin(); });
        connect(btnAlignFace, &QPushButton::clicked, this, [this]() { m_viewport->alignSelectedFaceToZ(true); });
        connect(btnReset, &QPushButton::clicked, this, [this]() { m_viewport->resetCadModelsTransform(); });

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Apply | QDialogButtonBox::Close, &dlg);
        connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked, this, [&]() {
            if (rxSpin->value() != 0.0 || rySpin->value() != 0.0 || rzSpin->value() != 0.0) {
                m_viewport->rotateCadModels(rxSpin->value(), rySpin->value(), rzSpin->value(), true);
                rxSpin->setValue(0.0); rySpin->setValue(0.0); rzSpin->setValue(0.0);
            }
            if (dxSpin->value() != 0.0 || dySpin->value() != 0.0 || dzSpin->value() != 0.0) {
                m_viewport->translateCadModels(dxSpin->value(), dySpin->value(), dzSpin->value());
                dxSpin->setValue(0.0); dySpin->setValue(0.0); dzSpin->setValue(0.0);
            }
            });
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        layout->addWidget(buttons);

        dlg.exec();
    }

    void MainWindow::onCadFaceSelected(const CadFaceInfo& faceInfo) {
        if (!faceInfo.success) {
            m_hasArmedCadNominal = false;
            return;
        }

        m_hasArmedCadNominal = true;
        m_armedCadDiameterMm = faceInfo.nominalDiameter;
        m_armedCadLocationMm = Eigen::Vector3d(
            faceInfo.centerOrCentroid.X(),
            faceInfo.centerOrCentroid.Y(),
            faceInfo.centerOrCentroid.Z()
        );
        m_armedCadNormalMm = Eigen::Vector3d(
            faceInfo.normalOrAxis.X(),
            faceInfo.normalOrAxis.Y(),
            faceInfo.normalOrAxis.Z()
        ).normalized();
        m_armedCadIsInternal = faceInfo.isInternal;
        m_armedCadDescription = QString::fromStdString(faceInfo.surfaceType);

        if (m_compModeCombo) {
            if (faceInfo.nominalDiameter > 0.0) {
                m_compModeCombo->setCurrentIndex(faceInfo.isInternal ? 1 : 0);
            }
            else {
                m_compModeCombo->setCurrentIndex(0);
            }
        }

        if (m_workplaneCombo && (m_armedCadDescription.contains("Cylinder") || m_armedCadDescription.contains("Plane"))) {
            const double ax = std::abs(m_armedCadNormalMm.x());
            const double ay = std::abs(m_armedCadNormalMm.y());
            const double az = std::abs(m_armedCadNormalMm.z());
            if (az >= 0.999) {
                m_workplaneCombo->setCurrentIndex(0);
            }
            else if (ax >= 0.999) {
                m_workplaneCombo->setCurrentIndex(1);
            }
            else if (ay >= 0.999) {
                m_workplaneCombo->setCurrentIndex(2);
            }
            else {
                m_lastFittedPlaneFrame = WorkplaneFrame::fromPlane(m_armedCadLocationMm, m_armedCadNormalMm);
                m_hasCustomPlane = true;
                m_workplaneCombo->setCurrentIndex(3);
            }
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        if (faceInfo.nominalDiameter > 0.0) {
            statusBar()->showMessage(QString("Selected CAD %1 — Nominal Ø: %2%3 (%4) | Ctrl+Click to probe or click 'Auto-Probe CAD'")
                .arg(m_armedCadDescription)
                .arg(toDisplayUnit(faceInfo.nominalDiameter, m_unitSystem), 0, 'f', decimals)
                .arg(unitStr)
                .arg(faceInfo.isInternal ? "Internal Bore" : "External Boss"));
        }
        else {
            statusBar()->showMessage(QString("Selected CAD %1 at (%2, %3, %4)%5 | Ctrl+Click to probe or click 'Auto-Probe CAD'")
                .arg(m_armedCadDescription)
                .arg(toDisplayUnit(m_armedCadLocationMm.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(m_armedCadLocationMm.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(m_armedCadLocationMm.z(), m_unitSystem), 0, 'f', decimals)
                .arg(unitStr));
        }
    }

    void MainWindow::onCadSurfaceProbed(const CadSurfaceHit& hit) {
        if (!hit.success) return;

        const Eigen::Vector3d surfPt(hit.surfacePoint.X(), hit.surfacePoint.Y(), hit.surfacePoint.Z());
        const Eigen::Vector3d surfNorm(hit.surfaceNormal.X(), hit.surfaceNormal.Y(), hit.surfaceNormal.Z());

        const double rProbe = (currentCompMode() == ProbeCompMode::Uncompensated) ? 0.0 : m_effectiveProbeRadius;
        const Eigen::Vector3d rubyCenter = surfPt + surfNorm * rProbe;

        m_lastApproachDir = -surfNorm;
        m_recordedDmisHits.push_back({ surfPt, surfNorm });

        Position3D rubyPos{ rubyCenter.x(), rubyCenter.y(), rubyCenter.z() };
        onProbeHit(rubyPos);

        if (m_activePointsGroup && m_activePointsGroup->childCount() > 0) {
            QTreeWidgetItem* lastItem = m_activePointsGroup->child(m_activePointsGroup->childCount() - 1);
            if (lastItem) {
                lastItem->setText(1, QString("N(%1, %2, %3)")
                    .arg(surfNorm.x(), 0, 'f', 3)
                    .arg(surfNorm.y(), 0, 'f', 3)
                    .arg(surfNorm.z(), 0, 'f', 3));
                lastItem->setText(4, "CAD PTMEAS");
            }
        }
    }

    void MainWindow::onAutoProbeCadFaceClicked() {
        if (!m_viewport || !m_viewport->hasCadModel()) {
            QMessageBox::information(this, "No STEP Model Loaded",
                "Please load a 3D STEP model first ('Load STEP' in the CAD & Deviation tab).");
            return;
        }
        if (!m_viewport->hasSelectedCadFace()) {
            QMessageBox::information(this, "Select a CAD Face",
                "Please left-click a face (Plane, Bore, Boss, or Sphere) on the 3D CAD model first, then click 'Auto-Probe CAD'.\n\n"
                "Tip: You can also hold Ctrl + Left-Click directly on any CAD surface to probe individual points.");
            return;
        }

        const CadFaceInfo& faceInfo = m_viewport->lastSelectedFace();
        const QString surfType = QString::fromStdString(faceInfo.surfaceType);

        int defaultPts = 4;
        if (surfType.contains("Cylinder") || surfType.contains("Sphere")) {
            defaultPts = 5;
        }

        std::vector<CadSurfaceHit> sampledHits = m_viewport->sampleSelectedCadFace(defaultPts);
        if (sampledHits.empty()) {
            QMessageBox::warning(this, "Sampling Failed",
                "Could not find valid interior UV sample points on the selected CAD face.");
            return;
        }

        finishFeatureBuffer();
        m_suppressDmisStepAdvance = true;

        for (const auto& hit : sampledHits) {
            onCadSurfaceProbed(hit);
        }

        m_suppressDmisStepAdvance = false;

        if (surfType.contains("Plane") && m_recordedHits.size() >= 3) {
            onFitPlaneClicked();
        }
        else if (surfType.contains("Cylinder") && m_recordedHits.size() >= 5) {
            onFitCylinderClicked();
        }
        else if (surfType.contains("Cylinder") && m_recordedHits.size() >= 3) {
            onFitCircleClicked();
        }
        else if (surfType.contains("Sphere") && m_recordedHits.size() >= 4) {
            onFitSphereClicked();
        }
        else {
            statusBar()->showMessage(QString("Sampled %1 points on %2 — click 'Surface Dev' or a Fit tool to evaluate.")
                .arg(sampledHits.size())
                .arg(surfType));
        }
    }

    void MainWindow::onCreateOfflineFeatureClicked() {
        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";
        const double toDisp = (m_unitSystem == UnitSystem::Millimeters) ? 1.0 : (1.0 / 25.4);
        const double toMm = (m_unitSystem == UnitSystem::Millimeters) ? 1.0 : 25.4;

        QDialog dlg(this);
        dlg.setWindowTitle("Create Offline Inspection Feature (DMIS Path Generator)");
        dlg.setMinimumWidth(480);

        auto* mainLayout = new QVBoxLayout(&dlg);
        auto* form = new QFormLayout();
        form->setSpacing(8);

        const int nextNum = static_cast<int>(m_dmisProgram.features.size() + m_solvedFeatures.size()) + 1;
        auto* labelEdit = new QLineEdit(QString("FEAT_%1").arg(nextNum), &dlg);

        auto* typeCombo = new QComboBox(&dlg);
        typeCombo->addItem("3D Cylinder (FEAT/CYLNDR)", "CYLNDR");
        typeCombo->addItem("2D Circle (FEAT/CIRCLE)", "CIRCLE");
        typeCombo->addItem("3D Plane (FEAT/PLANE)", "PLANE");
        typeCombo->addItem("3D Sphere (FEAT/SPHERE)", "SPHERE");
        typeCombo->addItem("2D Line (FEAT/LINE)", "LINE");

        auto* boundCombo = new QComboBox(&dlg);
        boundCombo->addItem("Internal Bore / Cavity (INNER)", "INNER");
        boundCombo->addItem("External Boss / Stud (OUTER)", "OUTER");
        boundCombo->addItem("Unbounded / Surface (UNBND)", "UNBND");

        auto* datumCombo = new QComboBox(&dlg);
        datumCombo->addItem("None (Inspection Feature Only)", "");
        datumCombo->addItem("Primary Datum A (Level +Z)", "A");
        datumCombo->addItem("Secondary Datum B (Rotate +X)", "B");
        datumCombo->addItem("Tertiary Datum C (Origin)", "C");

        auto makeCoordSpin = [&](double valMm) {
            auto* s = new QDoubleSpinBox(&dlg);
            s->setRange(-10000.0, 10000.0);
            s->setDecimals(decimals);
            s->setSuffix(unitStr);
            s->setValue(valMm * toDisp);
            return s;
            };

        auto makeVecSpin = [&](double val) {
            auto* s = new QDoubleSpinBox(&dlg);
            s->setRange(-1.0, 1.0);
            s->setSingleStep(0.1);
            s->setDecimals(5);
            s->setValue(val);
            return s;
            };

        auto* nomXSpin = makeCoordSpin(0.0);
        auto* nomYSpin = makeCoordSpin(0.0);
        auto* nomZSpin = makeCoordSpin(0.0);

        auto* nomISpin = makeVecSpin(0.0);
        auto* nomJSpin = makeVecSpin(0.0);
        auto* nomKSpin = makeVecSpin(1.0);

        auto* diamSpin = makeCoordSpin(20.0);
        diamSpin->setRange(0.1 * toDisp, 5000.0 * toDisp);

        auto* depthSpanSpin = makeCoordSpin(12.0);
        depthSpanSpin->setRange(-2000.0 * toDisp, 2000.0 * toDisp);
        depthSpanSpin->setToolTip("For Cylinders: axial depth span (negative = down into bore). For Planes/Lines: sampling spread/length.");

        auto* ptsPerRingSpin = new QSpinBox(&dlg);
        ptsPerRingSpin->setRange(2, 36);
        ptsPerRingSpin->setValue(4);

        auto* numRingsSpin = new QSpinBox(&dlg);
        numRingsSpin->setRange(1, 10);
        numRingsSpin->setValue(2);

        auto* useCadUvCheck = new QCheckBox("Sample touch points directly from selected 3D CAD face UV", &dlg);
        const bool hasCadFace = (m_viewport && m_viewport->hasSelectedCadFace());
        useCadUvCheck->setEnabled(hasCadFace);
        useCadUvCheck->setChecked(hasCadFace);

        auto* modeCombo = new QComboBox(&dlg);
        modeCombo->addItem("Add to Planned DMIS Program Queue (Offline Path)", 0);
        modeCombo->addItem("Simulate & Solve Immediately", 1);

        auto populateFromCadFace = [&]() {
            if (!m_viewport || !m_viewport->hasSelectedCadFace()) return;
            const CadFaceInfo& fi = m_viewport->lastSelectedFace();
            const QString st = QString::fromStdString(fi.surfaceType);

            nomXSpin->setValue(fi.centerOrCentroid.X() * toDisp);
            nomYSpin->setValue(fi.centerOrCentroid.Y() * toDisp);
            nomZSpin->setValue(fi.centerOrCentroid.Z() * toDisp);
            nomISpin->setValue(fi.normalOrAxis.X());
            nomJSpin->setValue(fi.normalOrAxis.Y());
            nomKSpin->setValue(fi.normalOrAxis.Z());

            if (st.contains("Cylinder")) {
                typeCombo->setCurrentIndex(0);
                boundCombo->setCurrentIndex(fi.isInternal ? 0 : 1);
                diamSpin->setValue(fi.nominalDiameter * toDisp);
                numRingsSpin->setValue(3);
                ptsPerRingSpin->setValue(4);
                labelEdit->setText(QString("CYL_%1").arg(nextNum));
            }
            else if (st.contains("Plane")) {
                typeCombo->setCurrentIndex(2);
                boundCombo->setCurrentIndex(2);
                numRingsSpin->setValue(1);
                ptsPerRingSpin->setValue(4);
                labelEdit->setText(QString("PLN_%1").arg(nextNum));
            }
            else if (st.contains("Sphere")) {
                typeCombo->setCurrentIndex(3);
                boundCombo->setCurrentIndex(fi.isInternal ? 0 : 1);
                diamSpin->setValue(fi.nominalDiameter * toDisp);
                numRingsSpin->setValue(1);
                ptsPerRingSpin->setValue(6);
                labelEdit->setText(QString("SPH_%1").arg(nextNum));
            }
            };

        if (hasCadFace) {
            populateFromCadFace();
        }

        connect(typeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg, [&](int idx) {
            const QString fType = typeCombo->itemData(idx).toString();
            const bool isCyl = (fType == "CYLNDR");
            const bool hasDiam = (fType == "CYLNDR" || fType == "CIRCLE" || fType == "SPHERE");
            diamSpin->setEnabled(hasDiam);
            numRingsSpin->setEnabled(isCyl);
            if (!isCyl) numRingsSpin->setValue(1);
            else if (numRingsSpin->value() < 2) numRingsSpin->setValue(2);
            });

        form->addRow("Feature Label:", labelEdit);
        form->addRow("Feature Type:", typeCombo);
        form->addRow("Boundary Subtype:", boundCombo);
        form->addRow("Assign Datum:", datumCombo);
        form->addRow("Nominal Center X:", nomXSpin);
        form->addRow("Nominal Center Y:", nomYSpin);
        form->addRow("Nominal Center Z:", nomZSpin);
        form->addRow("Axis / Normal I:", nomISpin);
        form->addRow("Axis / Normal J:", nomJSpin);
        form->addRow("Axis / Normal K:", nomKSpin);
        form->addRow("Nominal Ø:", diamSpin);
        form->addRow("Axial Depth / Span:", depthSpanSpin);
        form->addRow("Points per Ring:", ptsPerRingSpin);
        form->addRow("Depth Rings (Cyl):", numRingsSpin);
        form->addRow("", useCadUvCheck);
        form->addRow("Action:", modeCombo);

        mainLayout->addLayout(form);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        mainLayout->addWidget(buttons);

        if (dlg.exec() != QDialog::Accepted) return;

        using DmisFeatureT = typename decltype(m_dmisProgram.features)::value_type;
        DmisFeatureT df;
        using DmisPointT = typename decltype(df.points)::value_type;

        df.label = labelEdit->text().trimmed().isEmpty()
            ? QString("FEAT_%1").arg(nextNum).toStdString()
            : labelEdit->text().trimmed().toStdString();
        df.refId = QString("10000000-0000-4000-8000-%1").arg(nextNum, 12, 10, QChar('0')).toStdString();
        df.featureType = typeCombo->currentData().toString().toStdString();
        df.boundarySub = boundCombo->currentData().toString().toStdString();
        df.datumLetter = datumCombo->currentData().toString().toStdString();
        df.nomX = nomXSpin->value() * toMm;
        df.nomY = nomYSpin->value() * toMm;
        df.nomZ = nomZSpin->value() * toMm;

        Eigen::Vector3d axisVec(nomISpin->value(), nomJSpin->value(), nomKSpin->value());
        if (axisVec.norm() < 1e-6) axisVec = Eigen::Vector3d::UnitZ();
        axisVec.normalize();
        df.nomI = axisVec.x();
        df.nomJ = axisVec.y();
        df.nomK = axisVec.z();
        df.nominalDiameter = diamSpin->value() * toMm;

        const double spanMm = depthSpanSpin->value() * toMm;
        const int ptsPerRing = ptsPerRingSpin->value();
        const int numRings = (df.featureType == "CYLNDR") ? std::max(2, numRingsSpin->value()) : 1;
        const bool isInternal = (df.boundarySub == "INNER");

        if (useCadUvCheck->isChecked() && m_viewport && m_viewport->hasSelectedCadFace()) {
            std::vector<CadSurfaceHit> uvHits = m_viewport->sampleSelectedCadFace(ptsPerRing);
            for (const auto& h : uvHits) {
                DmisPointT tp;
                tp.x = h.surfacePoint.X();
                tp.y = h.surfacePoint.Y();
                tp.z = h.surfacePoint.Z();
                tp.i = h.surfaceNormal.X();
                tp.j = h.surfaceNormal.Y();
                tp.k = h.surfaceNormal.Z();
                df.points.push_back(tp);
            }
        }

        if (df.points.empty()) {
            const Eigen::Vector3d center(df.nomX, df.nomY, df.nomZ);
            WorkplaneFrame frame = WorkplaneFrame::fromPlane(center, axisVec);
            const double radius = 0.5 * df.nominalDiameter;

            auto appendPt = [&](const Eigen::Vector3d& p, const Eigen::Vector3d& n) {
                DmisPointT tp;
                tp.x = p.x();
                tp.y = p.y();
                tp.z = p.z();
                tp.i = n.x();
                tp.j = n.y();
                tp.k = n.z();
                df.points.push_back(tp);
                };

            if (df.featureType == "CYLNDR" || df.featureType == "CIRCLE") {
                for (int r = 0; r < numRings; ++r) {
                    const double axialOffset = (numRings > 1)
                        ? (spanMm * (static_cast<double>(r) / (numRings - 1)))
                        : 0.0;
                    const Eigen::Vector3d ringCenter = center + axialOffset * axisVec;

                    for (int i = 0; i < ptsPerRing; ++i) {
                        const double theta = (2.0 * 3.14159265358979323846 * i) / ptsPerRing;
                        const Eigen::Vector3d radialOut = std::cos(theta) * frame.uAxis + std::sin(theta) * frame.vAxis;
                        const Eigen::Vector3d surfPt = ringCenter + radius * radialOut;
                        const Eigen::Vector3d surfNorm = isInternal ? -radialOut : radialOut;
                        appendPt(surfPt, surfNorm);
                    }
                }
            }
            else if (df.featureType == "PLANE") {
                const double spread = std::max(5.0, std::abs(spanMm));
                for (int i = 0; i < ptsPerRing; ++i) {
                    const double theta = (2.0 * 3.14159265358979323846 * i) / ptsPerRing + 0.25;
                    const Eigen::Vector3d surfPt = center + spread * (std::cos(theta) * frame.uAxis + std::sin(theta) * frame.vAxis);
                    appendPt(surfPt, axisVec);
                }
            }
            else if (df.featureType == "SPHERE") {
                const double goldenAngle = 2.399963229728653;
                for (int i = 0; i < ptsPerRing; ++i) {
                    const double theta = i * goldenAngle;
                    const double phi = std::acos(1.0 - 0.65 * (i + 0.5) / ptsPerRing);
                    const Eigen::Vector3d localDir =
                        std::sin(phi) * std::cos(theta) * frame.uAxis +
                        std::sin(phi) * std::sin(theta) * frame.vAxis +
                        std::cos(phi) * frame.wAxis;
                    const Eigen::Vector3d surfPt = center + radius * localDir;
                    const Eigen::Vector3d surfNorm = isInternal ? -localDir : localDir;
                    appendPt(surfPt, surfNorm);
                }
            }
            else if (df.featureType == "LINE") {
                const double len = std::max(5.0, std::abs(spanMm));
                for (int i = 0; i < ptsPerRing; ++i) {
                    const double t = (ptsPerRing > 1) ? (-0.5 + static_cast<double>(i) / (ptsPerRing - 1)) : 0.0;
                    const Eigen::Vector3d surfPt = center + (t * len) * frame.uAxis;
                    appendPt(surfPt, axisVec);
                }
            }
        }

        if (modeCombo->currentData().toInt() == 1) {
            finishFeatureBuffer();
            m_hasArmedCadNominal = true;
            m_armedCadDiameterMm = df.nominalDiameter;
            m_armedCadLocationMm = Eigen::Vector3d(df.nomX, df.nomY, df.nomZ);
            m_armedCadNormalMm = axisVec;
            m_armedCadIsInternal = isInternal;
            m_activeDmisFeatureName = QString("%1 (%2)").arg(QString::fromStdString(df.label), QString::fromStdString(df.featureType));
            m_activeDmisRefId = QString::fromStdString(df.refId);

            if (m_compModeCombo) {
                if (df.featureType == "CYLNDR" || df.featureType == "CIRCLE" || df.featureType == "SPHERE") {
                    m_compModeCombo->setCurrentIndex(isInternal ? 1 : 0);
                }
                else {
                    m_compModeCombo->setCurrentIndex(0);
                }
            }

            m_suppressDmisStepAdvance = true;
            for (const auto& pt : df.points) {
                CadSurfaceHit simHit;
                simHit.success = true;
                simHit.surfacePoint = gp_Pnt(pt.x, pt.y, pt.z);
                simHit.surfaceNormal = gp_Dir(pt.i, pt.j, pt.k);
                onCadSurfaceProbed(simHit);
            }
            m_suppressDmisStepAdvance = false;

            if (df.featureType == "CYLNDR") onFitCylinderClicked();
            else if (df.featureType == "CIRCLE") onFitCircleClicked();
            else if (df.featureType == "PLANE") onFitPlaneClicked();
            else if (df.featureType == "SPHERE") onFitSphereClicked();
            else if (df.featureType == "LINE") onFitLineClicked();

            if (!df.datumLetter.empty() && !m_solvedFeatures.empty()) {
                assignFeatureAsDatum(m_solvedFeatures.back().id, QString::fromStdString(df.datumLetter), true);
            }
            return;
        }

        m_dmisProgram.valid = true;
        m_dmisProgram.stylusDiameterMm = m_effectiveProbeRadius * 2.0;
        if (!df.datumLetter.empty()) {
            m_dmisProgram.datumAlignment.has321Alignment = true;
        }
        const int numPts = static_cast<int>(df.points.size());
        m_dmisProgram.features.push_back(df);

        const QString datSuffix = df.datumLetter.empty()
            ? QString()
            : QString(" [DAT %1]").arg(QString::fromStdString(df.datumLetter));
        const QString featTitle = QString("%1 (%2)%3")
            .arg(QString::fromStdString(df.label), QString::fromStdString(df.featureType), datSuffix);
        const double nomVal = (df.featureType == "CIRCLE" || df.featureType == "CYLNDR" || df.featureType == "SPHERE")
            ? df.nominalDiameter : 0.0;

        auto* item = new QTreeWidgetItem(m_featureTree, {
            featTitle,
            QString("%1%2").arg(toDisplayUnit(nomVal, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "--",
            QString("±%1").arg(toDisplayUnit(m_defaultTolMm, m_unitSystem), 0, 'f', decimals),
            QString("PLANNED (0/%1 pts)").arg(numPts)
            });
        item->setForeground(4, QBrush(QColor("#e5c07b")));
        m_plannedDmisTreeItems.push_back(item);

        const int newIdx = static_cast<int>(m_dmisProgram.features.size()) - 1;
        if (!m_isDmisProgramActive) {
            m_isDmisProgramActive = true;
            startDmisGuidedFeature(newIdx);
        }
        else {
            statusBar()->showMessage(QString("Queued offline feature '%1' (%2 touch points) — click 'Run DMIS' or 'Save DMIS' anytime.")
                .arg(featTitle)
                .arg(numPts));
        }
    }

    void MainWindow::onAssignCadDatumClicked(const QString& datumLetter) {
        const size_t prevCount = m_solvedFeatures.size();

        if (m_viewport && m_viewport->hasCadModel() && m_viewport->hasSelectedCadFace()) {
            const CadFaceInfo& fi = m_viewport->lastSelectedFace();
            // Only block truly vertical side walls (where normal Z is nearly 0)
            if (datumLetter == "A" && fi.surfaceType == "PLANE" && std::abs(fi.normalOrAxis.Z()) < 0.05) {
                QMessageBox::warning(
                    this,
                    "Invalid Datum A Face",
                    "Datum A requires a horizontal leveling plane.\n\n"
                    "The currently selected face is a vertical side wall (normal is perpendicular to Z), which cannot level the Z axis."
                );
                return;
            }
            onAutoProbeCadFaceClicked();
            if (m_solvedFeatures.size() > prevCount) {
                assignFeatureAsDatum(m_solvedFeatures.back().id, datumLetter, true);
                return;
            }
        }

        QTreeWidgetItem* cur = m_featureTree->currentItem();
        while (cur && cur->parent() != nullptr) {
            cur = cur->parent();
        }
        if (cur && cur != m_activePointsGroup && cur->data(0, ROLE_FEATURE_ID).isValid()) {
            assignFeatureAsDatum(cur->data(0, ROLE_FEATURE_ID).toInt(), datumLetter, true);
            return;
        }

        if (!m_solvedFeatures.empty()) {
            assignFeatureAsDatum(m_solvedFeatures.back().id, datumLetter, true);
            return;
        }

        QMessageBox::information(
            this,
            "Specify Datum " + datumLetter,
            QString("To specify Datum %1:\n\n"
                "1. Left-click a valid face on the 3D CAD model and click 'Set Datum %1', OR\n"
                "2. Right-click any solved feature in the Inspection Feature Tree and choose 'Set as Datum %1'.")
            .arg(datumLetter)
        );
    }

    void MainWindow::assignFeatureAsDatum(int featureId, const QString& datumLetter, bool autoSolve321) {
        const QString letter = datumLetter.trimmed().toUpper();
        if (letter != "A" && letter != "B" && letter != "C") return;

        for (auto it = m_featureDatumLetters.begin(); it != m_featureDatumLetters.end(); ) {
            if (it->second == letter && it->first != featureId) {
                int oldId = it->first;
                it = m_featureDatumLetters.erase(it);
                for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                    QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                    if (top && top->data(0, ROLE_FEATURE_ID).toInt() == oldId) {
                        QString t = top->text(0);
                        t.remove(QString(" [DAT %1]").arg(letter));
                        top->setText(0, t);
                    }
                }
            }
            else {
                ++it;
            }
        }

        m_featureDatumLetters[featureId] = letter;
        if (letter == "A") m_datumAFeatureId = featureId;
        else if (letter == "B") m_datumBFeatureId = featureId;
        else if (letter == "C") m_datumCFeatureId = featureId;

        const QString badge = QString(" [DAT %1]").arg(letter);
        for (auto& mf : m_solvedFeatures) {
            if (mf.id == featureId) {
                QString n = QString::fromStdString(mf.name);
                n.remove(" [DAT A]"); n.remove(" [DAT B]"); n.remove(" [DAT C]");
                n += badge;
                mf.name = n.toStdString();
            }
        }

        for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
            QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
            if (top && top->data(0, ROLE_FEATURE_ID).isValid() && top->data(0, ROLE_FEATURE_ID).toInt() == featureId) {
                QString title = top->text(0);
                title.remove(" [DAT A]"); title.remove(" [DAT B]"); title.remove(" [DAT C]");
                title += badge;
                top->setText(0, title);
                break;
            }
        }

        statusBar()->showMessage(QString("Assigned Feature #%1 as Datum %2.").arg(featureId).arg(letter));

        if (autoSolve321 && m_datumAFeatureId > 0 && m_datumBFeatureId > 0 && m_datumCFeatureId > 0) {
            tryAutoSolve321Alignment(false);
        }
    }

    bool MainWindow::tryAutoSolve321Alignment(bool showFailureDialog) {
        const MeasuredFeature* featA = nullptr;
        const MeasuredFeature* featB = nullptr;
        const MeasuredFeature* featC = nullptr;

        for (const auto& mf : m_solvedFeatures) {
            if (mf.id == m_datumAFeatureId) featA = &mf;
            if (mf.id == m_datumBFeatureId) featB = &mf;
            if (mf.id == m_datumCFeatureId) featC = &mf;
        }

        if (!featA || !featB || !featC) {
            return false;
        }

        Alignment321Result alignRes = MetrologyMath::compute321Alignment(*featA, *featB, *featC);
        if (!alignRes.success) {
            if (showFailureDialog) {
                playSolveChime(false);
                QMessageBox::warning(this, "3-2-1 Alignment Notice",
                    "Could not compute an orthogonal 3-2-1 coordinate frame from Datums A, B, and C.\n"
                    "Ensure Datum A is a 3D Plane and Datums B & C define a non-collinear axis/origin.");
            }
            return false;
        }

        m_pcsMatrix = alignRes.transformationMatrix;
        m_isPartAligned = true;

        if (m_alignStatusBtn) {
            m_alignStatusBtn->setChecked(true);
            m_alignStatusBtn->setText("🌐\nPCS (A-B-C)");
            m_alignStatusBtn->setStyleSheet(
                "QPushButton { background-color: #1e4620; color: #98c379; border: 1px solid #98c379; "
                "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
        }

        if (m_viewport) {
            m_viewport->setPcsTriadVisual(m_pcsMatrix, 45.0);
        }

        updateDroDisplay();
        playSolveChime(true);
        statusBar()->showMessage(QString("3-2-1 Part Coordinate System (PCS) Active — Aligned to Datums A (%1), B (%2), C (%3).")
            .arg(QString::fromStdString(featA->name))
            .arg(QString::fromStdString(featB->name))
            .arg(QString::fromStdString(featC->name)));
        return true;
    }

    void MainWindow::onLoadDmisProgramClicked() {
        stopDmisAutoRun();

        QString filePath = QFileDialog::getOpenFileName(
            this,
            "Load DMIS 5.3 Inspection Program",
            QString(),
            "DMIS 5.3 Programs (*.dmis *.dmi *.dms *.txt);;All Files (*.*)"
        );
        if (filePath.isEmpty()) return;

        DmisProgramData prog = DMIS53Parser::parseDmisProgramFile(filePath.toStdString());
        if (!prog.valid || prog.features.empty()) {
            QMessageBox::warning(
                this,
                "DMIS 5.3 Load Error",
                QString("Could not parse executable features from:\n%1\n\nDetails: %2")
                .arg(filePath, QString::fromStdString(prog.errorMessage))
            );
            return;
        }

        finishFeatureBuffer();
        m_solvedFeatures.clear();
        m_featureDmisHits.clear();
        m_featureRefIds.clear();
        m_featureDatumLetters.clear();
        m_datumAFeatureId = -1;
        m_datumBFeatureId = -1;
        m_datumCFeatureId = -1;
        m_isPartAligned = false;
        m_pcsMatrix = Eigen::Matrix4d::Identity();
        if (m_viewport) {
            m_viewport->clearPcsTriadVisual();
        }
        m_featureCounter = 1;
        m_plannedDmisTreeItems.clear();
        m_activeDmisTreeItem = nullptr;

        for (int i = m_featureTree->topLevelItemCount() - 1; i >= 0; --i) {
            QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
            if (top != m_activePointsGroup) {
                delete m_featureTree->takeTopLevelItem(i);
            }
        }
        if (m_viewport) {
            m_viewport->clearFittedFeatures();
            m_viewport->clearHitMarkers();
            m_viewport->clearProgramTargetMarkers();
        }

        if (prog.stylusDiameterMm > 0.1) {
            m_effectiveProbeRadius = 0.5 * prog.stylusDiameterMm;
            updateProbeStatusDisplay();
        }

        m_dmisProgram = prog;
        m_isDmisProgramActive = true;

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        for (size_t i = 0; i < m_dmisProgram.features.size(); ++i) {
            const auto& df = m_dmisProgram.features[i];
            const QString datSuffix = df.datumLetter.empty()
                ? QString()
                : QString(" [DAT %1]").arg(QString::fromStdString(df.datumLetter));
            const QString featTitle = QString("%1 (%2)%3")
                .arg(QString::fromStdString(df.label), QString::fromStdString(df.featureType), datSuffix);
            const double nomVal = (df.featureType == "CIRCLE" || df.featureType == "CYLNDR" || df.featureType == "CYL" || df.featureType == "SPHERE")
                ? df.nominalDiameter : 0.0;

            auto* item = new QTreeWidgetItem(m_featureTree, {
                featTitle,
                QString("%1%2").arg(toDisplayUnit(nomVal, m_unitSystem), 0, 'f', decimals).arg(unitStr),
                "--",
                QString("±%1").arg(toDisplayUnit(m_defaultTolMm, m_unitSystem), 0, 'f', decimals),
                QString("PLANNED (0/%1 pts)").arg(df.points.size())
                });
            item->setForeground(4, QBrush(QColor("#e5c07b")));
            m_plannedDmisTreeItems.push_back(item);
        }

        startDmisGuidedFeature(0);
    }

    void MainWindow::startDmisGuidedFeature(int featureIndex) {
        if (!m_isDmisProgramActive || featureIndex < 0 || featureIndex >= static_cast<int>(m_dmisProgram.features.size())) {
            m_isDmisProgramActive = false;
            m_activeDmisFeatureIdx = -1;
            m_activeDmisTreeItem = nullptr;
            m_activeDmisFeatureName.clear();
            m_activeDmisRefId.clear();
            stopDmisAutoRun();
            if (m_viewport) {
                m_viewport->clearProgramTargetMarkers();
            }
            updateManualPromptBanner();
            playSolveChime(true);
            statusBar()->showMessage(QString("Guided Inspection Complete — All %1 features measured and evaluated!")
                .arg(m_dmisProgram.features.size()));
            return;
        }

        m_activeDmisFeatureIdx = featureIndex;
        const auto& df = m_dmisProgram.features[featureIndex];
        m_activeDmisTreeItem = (featureIndex < static_cast<int>(m_plannedDmisTreeItems.size()))
            ? m_plannedDmisTreeItems[featureIndex] : nullptr;
        m_activeDmisFeatureName = QString("%1 (%2)")
            .arg(QString::fromStdString(df.label), QString::fromStdString(df.featureType));
        m_activeDmisRefId = QString::fromStdString(df.refId);

        if (m_activeDmisTreeItem) {
            m_featureTree->setCurrentItem(m_activeDmisTreeItem);
            m_activeDmisTreeItem->setText(4, QString("ACTIVE (0/%1 pts)").arg(df.points.size()));
            m_activeDmisTreeItem->setForeground(4, QBrush(QColor("#61afef")));
        }

        m_hasArmedCadNominal = true;
        m_armedCadDiameterMm = df.nominalDiameter;
        m_armedCadLocationMm = Eigen::Vector3d(df.nomX, df.nomY, df.nomZ);
        m_armedCadNormalMm = Eigen::Vector3d(df.nomI, df.nomJ, df.nomK).normalized();

        const bool isInternal = (df.boundarySub == "INNER" || df.boundarySub == "BORE");
        m_armedCadIsInternal = isInternal;
        if (m_compModeCombo) {
            if (df.featureType == "CIRCLE" || df.featureType == "CYLNDR" || df.featureType == "CYL" || df.featureType == "SPHERE") {
                m_compModeCombo->setCurrentIndex(isInternal ? 1 : 0);
            }
            else {
                m_compModeCombo->setCurrentIndex(0);
            }
        }

        if (m_workplaneCombo && (df.featureType == "CIRCLE" || df.featureType == "CYLNDR" || df.featureType == "CYL" || df.featureType == "PLANE" || df.featureType == "LINE")) {
            const double ax = std::abs(m_armedCadNormalMm.x());
            const double ay = std::abs(m_armedCadNormalMm.y());
            const double az = std::abs(m_armedCadNormalMm.z());
            if (az >= 0.999) m_workplaneCombo->setCurrentIndex(0);
            else if (ax >= 0.999) m_workplaneCombo->setCurrentIndex(1);
            else if (ay >= 0.999) m_workplaneCombo->setCurrentIndex(2);
            else {
                m_lastFittedPlaneFrame = WorkplaneFrame::fromPlane(m_armedCadLocationMm, m_armedCadNormalMm);
                m_hasCustomPlane = true;
                m_workplaneCombo->setCurrentIndex(3);
            }
        }

        updateDmisTargetVisuals();
        updateManualPromptBanner();
    }

    void MainWindow::updateDmisTargetVisuals() {
        if (!m_isDmisProgramActive || m_activeDmisFeatureIdx < 0 ||
            m_activeDmisFeatureIdx >= static_cast<int>(m_dmisProgram.features.size())) {
            return;
        }

        const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
        const int activePtIdx = static_cast<int>(m_recordedHits.size());

        std::vector<gp_Pnt> surfPts;
        std::vector<gp_Dir> surfNorms;
        for (const auto& pt : df.points) {
            surfPts.emplace_back(pt.x, pt.y, pt.z);
            const double len = std::hypot(pt.i, std::hypot(pt.j, pt.k));
            if (len > 1e-9) {
                surfNorms.emplace_back(pt.i / len, pt.j / len, pt.k / len);
            }
            else {
                surfNorms.emplace_back(0.0, 0.0, 1.0);
            }
        }

        if (m_viewport) {
            const double rProbe = (currentCompMode() == ProbeCompMode::Uncompensated) ? 0.0 : m_effectiveProbeRadius;
            m_viewport->setProgramTargetMarkers(surfPts, surfNorms, activePtIdx, rProbe);
        }

        if (activePtIdx < static_cast<int>(df.points.size())) {
            const auto& tgt = df.points[activePtIdx];
            const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 3 : 4;
            const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? "mm" : "in";
            statusBar()->showMessage(QString("[DMIS Feature %1/%2: %3] Touch Gold Target Pt #%4/%5 at (%6, %7, %8) %9 — or click 'Simulate Hit' / 'Run DMIS'")
                .arg(m_activeDmisFeatureIdx + 1)
                .arg(m_dmisProgram.features.size())
                .arg(QString::fromStdString(df.label))
                .arg(activePtIdx + 1)
                .arg(df.points.size())
                .arg(toDisplayUnit(tgt.x, m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(tgt.y, m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(tgt.z, m_unitSystem), 0, 'f', decimals)
                .arg(unitStr));
        }
        updateManualPromptBanner();
        focusViewportOnActiveTarget();
    }

    void MainWindow::stopDmisAutoRun() {
        if (m_dmisPlaybackTimer && m_dmisPlaybackTimer->isActive()) {
            m_dmisPlaybackTimer->stop();
        }
        m_playbackPhase = 0;
        m_playbackSubStep = 0;
        if (m_autoRunDmisBtnHome) m_autoRunDmisBtnHome->setText("▶\nRun DMIS");
        if (m_autoRunDmisBtnCad) m_autoRunDmisBtnCad->setText("▶\nRun DMIS");
    }

    void MainWindow::onToggleAutoRunDmisClicked() {
        if (m_dmisPlaybackTimer && m_dmisPlaybackTimer->isActive()) {
            stopDmisAutoRun();
            statusBar()->showMessage("Paused DMIS 5.3 offline trajectory playback.");
            return;
        }

        if (!m_isDmisProgramActive || m_dmisProgram.features.empty()) {
            onLoadDmisProgramClicked();
            if (!m_isDmisProgramActive || m_dmisProgram.features.empty()) {
                return;
            }
        }

        m_playbackPhase = 0;
        m_playbackSubStep = 0;
        m_playbackStartPos = Eigen::Vector3d(m_lastPosition.x, m_lastPosition.y, m_lastPosition.z);

        if (m_autoRunDmisBtnHome) m_autoRunDmisBtnHome->setText("⏹\nStop DMIS");
        if (m_autoRunDmisBtnCad) m_autoRunDmisBtnCad->setText("⏹\nStop DMIS");

        m_dmisPlaybackTimer->start();
    }

    void MainWindow::onDmisPlaybackTick() {
        if (!m_isDmisProgramActive || m_activeDmisFeatureIdx < 0 ||
            m_activeDmisFeatureIdx >= static_cast<int>(m_dmisProgram.features.size())) {
            stopDmisAutoRun();
            return;
        }

        const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
        const int ptIdx = static_cast<int>(m_recordedHits.size());
        if (ptIdx >= static_cast<int>(df.points.size())) {
            stopDmisAutoRun();
            return;
        }

        const auto& tgt = df.points[ptIdx];
        const Eigen::Vector3d surfPt(tgt.x, tgt.y, tgt.z);
        Eigen::Vector3d surfNorm(tgt.i, tgt.j, tgt.k);
        if (surfNorm.norm() > 1e-9) surfNorm.normalize();
        else surfNorm = Eigen::Vector3d::UnitZ();

        const double rProbe = (currentCompMode() == ProbeCompMode::Uncompensated) ? 0.0 : m_effectiveProbeRadius;
        const Eigen::Vector3d touchPos = surfPt + surfNorm * rProbe;
        const Eigen::Vector3d standoffPos = surfPt + surfNorm * (rProbe + 18.0);

        // Use program clearance Z or default to 25mm above highest point
        const double zClear = std::max(m_dmisProgram.clearanceZMm, std::max(standoffPos.z(), m_lastPosition.z) + 25.0);

        // Phase 0: Retract to clearance plane, traverse to feature standoff, descend
        if (m_playbackPhase == 0) {
            const int totalSteps = (ptIdx == 0) ? 14 : 8;
            if (m_playbackSubStep == 0) {
                m_playbackStartPos = Eigen::Vector3d(m_lastPosition.x, m_lastPosition.y, m_lastPosition.z);
            }
            m_playbackSubStep++;
            const double t = std::clamp(static_cast<double>(m_playbackSubStep) / totalSteps, 0.0, 1.0);

            // 3-Stage Arc/Bridge trajectory for inter-feature safety
            Eigen::Vector3d interp;
            if (ptIdx == 0) {
                // First point in feature: retract up to Z clear, move XY, descend to standoff
                if (t < 0.33) {
                    double subT = t / 0.33;
                    interp = m_playbackStartPos + subT * (Eigen::Vector3d(m_playbackStartPos.x(), m_playbackStartPos.y(), zClear) - m_playbackStartPos);
                }
                else if (t < 0.66) {
                    double subT = (t - 0.33) / 0.33;
                    Eigen::Vector3d startUp(m_playbackStartPos.x(), m_playbackStartPos.y(), zClear);
                    Eigen::Vector3d endUp(standoffPos.x(), standoffPos.y(), zClear);
                    interp = startUp + subT * (endUp - startUp);
                }
                else {
                    double subT = (t - 0.66) / 0.34;
                    Eigen::Vector3d endUp(standoffPos.x(), standoffPos.y(), zClear);
                    interp = endUp + subT * (standoffPos - endUp);
                }
            }
            else {
                interp = m_playbackStartPos + t * (standoffPos - m_playbackStartPos);
            }

            onPositionUpdated({ interp.x(), interp.y(), interp.z() });

            if (m_playbackSubStep >= totalSteps) {
                m_playbackPhase = 1;
                m_playbackSubStep = 0;
                m_playbackStartPos = standoffPos;
            }
        }
        // Phase 1: Controlled feed move from standoff to surface touch point
        else if (m_playbackPhase == 1) {
            const int totalSteps = 6;
            m_playbackSubStep++;
            const double t = std::clamp(static_cast<double>(m_playbackSubStep) / totalSteps, 0.0, 1.0);
            const Eigen::Vector3d interp = standoffPos + t * (touchPos - standoffPos);
            onPositionUpdated({ interp.x(), interp.y(), interp.z() });

            if (m_playbackSubStep >= totalSteps) {
                m_playbackPhase = 2;
                m_playbackSubStep = 0;
                m_playbackStartPos = touchPos;
                m_playbackEndPos = standoffPos;

                CadSurfaceHit simHit;
                simHit.success = true;
                simHit.surfacePoint = gp_Pnt(surfPt.x(), surfPt.y(), surfPt.z());
                simHit.surfaceNormal = gp_Dir(surfNorm.x(), surfNorm.y(), surfNorm.z());
                onCadSurfaceProbed(simHit);
            }
        }
        // Phase 2: Retract back to standoff along normal vector
        else if (m_playbackPhase == 2) {
            const int totalSteps = 5;
            m_playbackSubStep++;
            const double t = std::clamp(static_cast<double>(m_playbackSubStep) / totalSteps, 0.0, 1.0);
            const Eigen::Vector3d interp = m_playbackStartPos + t * (m_playbackEndPos - m_playbackStartPos);
            onPositionUpdated({ interp.x(), interp.y(), interp.z() });

            if (m_playbackSubStep >= totalSteps) {
                m_playbackPhase = 0;
                m_playbackSubStep = 0;
            }
        }
    }

    void MainWindow::onPositionUpdated(const Position3D& pos) {
        m_lastPosition = pos;
        updateDroDisplay();
        if (m_viewport) { m_viewport->updateProbePosition(pos.x, pos.y, pos.z); }

        evaluateProbeCollisionSafety(Eigen::Vector3d(pos.x, pos.y, pos.z));

        if (m_isDmisProgramActive && (!m_dmisPlaybackTimer || !m_dmisPlaybackTimer->isActive()) &&
            m_activeDmisFeatureIdx >= 0 && m_activeDmisFeatureIdx < static_cast<int>(m_dmisProgram.features.size())) {
            const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
            const int ptIdx = static_cast<int>(m_recordedHits.size());
            if (ptIdx < static_cast<int>(df.points.size())) {
                const auto& tgt = df.points[ptIdx];
                const double rProbe = (currentCompMode() == ProbeCompMode::Uncompensated) ? 0.0 : m_effectiveProbeRadius;
                const Eigen::Vector3d targetRuby(
                    tgt.x + tgt.i * rProbe,
                    tgt.y + tgt.j * rProbe,
                    tgt.z + tgt.k * rProbe
                );
                const Eigen::Vector3d delta = targetRuby - Eigen::Vector3d(pos.x, pos.y, pos.z);
                const double distMm = delta.norm();
                const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 3 : 4;
                const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? "mm" : "in";

                statusBar()->showMessage(QString("[DMIS %1 — Pt #%2/%3] Guide to Target: dX=%4  dY=%5  dZ=%6 %7  (3D Dist: %8 %7)")
                    .arg(QString::fromStdString(df.label))
                    .arg(ptIdx + 1)
                    .arg(df.points.size())
                    .arg(toDisplayUnit(delta.x(), m_unitSystem), 0, 'f', decimals)
                    .arg(toDisplayUnit(delta.y(), m_unitSystem), 0, 'f', decimals)
                    .arg(toDisplayUnit(delta.z(), m_unitSystem), 0, 'f', decimals)
                    .arg(unitStr)
                    .arg(toDisplayUnit(distMm, m_unitSystem), 0, 'f', decimals));

                if (m_promptDeltaLbl && m_manualPromptBanner && m_manualPromptBanner->isVisible()) {
                    QString proxColor = "#61afef"; // Blue (> 10 mm)
                    QString proxTag = "APPROACH TARGET";
                    if (distMm <= 1.5) {
                        proxColor = "#4ade80"; // Bright Green (<= 1.5 mm)
                        proxTag = "ON TARGET — TOUCH PART NOW";
                    }
                    else if (distMm <= 10.0) {
                        proxColor = "#fbbf24"; // Gold/Amber (<= 10 mm)
                        proxTag = "NEAR TARGET";
                    }

                    m_promptDeltaLbl->setStyleSheet(QString(
                        "background-color: #0d0f12; color: %1; border: 1px solid %1; border-radius: 4px; "
                        "padding: 4px 10px; font-family: 'Consolas', monospace; font-size: 14px; font-weight: bold;"
                    ).arg(proxColor));

                    m_promptDeltaLbl->setText(QString(
                        "[%1]   dX: %2   dY: %3   dZ: %4 %5   |   3D Dist: %6 %5"
                    )
                        .arg(proxTag)
                        .arg(toDisplayUnit(delta.x(), m_unitSystem), 7, 'f', decimals)
                        .arg(toDisplayUnit(delta.y(), m_unitSystem), 7, 'f', decimals)
                        .arg(toDisplayUnit(delta.z(), m_unitSystem), 7, 'f', decimals)
                        .arg(unitStr)
                        .arg(toDisplayUnit(distMm, m_unitSystem), 6, 'f', decimals));
                }
            }
        }
    }

    void MainWindow::updateDroDisplay() {
        Eigen::Vector3d pos(m_lastPosition.x, m_lastPosition.y, m_lastPosition.z);
        if (m_isPartAligned && m_alignStatusBtn && m_alignStatusBtn->isChecked()) {
            pos = MetrologyMath::transformPoint(pos, m_pcsMatrix);
        }
        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        m_droX->setText(QString::number(toDisplayUnit(pos.x(), m_unitSystem), 'f', decimals));
        m_droY->setText(QString::number(toDisplayUnit(pos.y(), m_unitSystem), 'f', decimals));
        m_droZ->setText(QString::number(toDisplayUnit(pos.z(), m_unitSystem), 'f', decimals));
    }

    void MainWindow::onProbeHit(const Position3D& hitPos) {
        playHitChime();
        m_lastPosition = hitPos;
        updateDroDisplay();
        const Eigen::Vector3d hitVec(hitPos.x, hitPos.y, hitPos.z);
        m_recordedHits.push_back(hitVec);

        if (m_recordedDmisHits.size() < m_recordedHits.size()) {
            Eigen::Vector3d hitNorm = -m_lastApproachDir.normalized();
            Eigen::Vector3d surfContact = hitVec;

            if (m_isDmisProgramActive && m_activeDmisFeatureIdx >= 0 &&
                m_activeDmisFeatureIdx < static_cast<int>(m_dmisProgram.features.size())) {
                const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
                const size_t idx = m_recordedHits.size() - 1;
                if (idx < df.points.size()) {
                    hitNorm = Eigen::Vector3d(df.points[idx].i, df.points[idx].j, df.points[idx].k).normalized();
                    const double rProbe = (currentCompMode() == ProbeCompMode::Uncompensated) ? 0.0 : m_effectiveProbeRadius;
                    surfContact = hitVec - hitNorm * rProbe;
                }
            }
            m_recordedDmisHits.push_back({ surfContact, hitNorm });
        }

        const int count = static_cast<int>(m_recordedHits.size());
        m_hitCountLabel->setText(QString("Active Feature Hits: %1").arg(count));
        m_activePointsGroup->setText(2, QString("%1 pts").arg(count));

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* hitItem = new QTreeWidgetItem(m_activePointsGroup, {
            QString("Hit #%1").arg(count),
            "--",
            QString("(%1, %2, %3)%4")
                .arg(toDisplayUnit(hitPos.x, m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(hitPos.y, m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(hitPos.z, m_unitSystem), 0, 'f', decimals)
                .arg(unitStr),
            "--",
            "RECORDED"
            });
        hitItem->setForeground(4, QBrush(QColor("#61afef")));
        m_activePointsGroup->setExpanded(true);

        if (m_viewport) {
            m_viewport->updateProbePosition(hitPos.x, hitPos.y, hitPos.z);
            m_viewport->addHitPointMarker(hitPos.x, hitPos.y, hitPos.z);
        }

        if (m_isDmisProgramActive && !m_suppressDmisStepAdvance &&
            m_activeDmisFeatureIdx >= 0 && m_activeDmisFeatureIdx < static_cast<int>(m_dmisProgram.features.size())) {
            const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
            const int targetCount = static_cast<int>(df.points.size());

            if (count < targetCount) {
                if (m_activeDmisTreeItem) {
                    m_activeDmisTreeItem->setText(4, QString("MEASURING (%1/%2 pts)").arg(count).arg(targetCount));
                }
                updateDmisTargetVisuals();
            }
            else if (count >= targetCount && targetCount > 0) {
                const std::string fType = df.featureType;
                if (fType == "PLANE" && count >= 3) {
                    onFitPlaneClicked();
                }
                else if ((fType == "CYLNDR" || fType == "CYL") && count >= 5) {
                    onFitCylinderClicked();
                }
                else if ((fType == "CIRCLE" || fType == "CYLNDR" || fType == "CYL") && count >= 3) {
                    onFitCircleClicked();
                }
                else if (fType == "SPHERE" && count >= 4) {
                    onFitSphereClicked();
                }
                else if (fType == "LINE" && count >= 2) {
                    onFitLineClicked();
                }

                if (!df.datumLetter.empty() && !m_solvedFeatures.empty()) {
                    assignFeatureAsDatum(
                        m_solvedFeatures.back().id,
                        QString::fromStdString(df.datumLetter),
                        m_dmisProgram.datumAlignment.has321Alignment
                    );
                }

                startDmisGuidedFeature(m_activeDmisFeatureIdx + 1);
            }
        }
    }

    void MainWindow::finishFeatureBuffer() {
        m_recordedHits.clear();
        m_recordedDmisHits.clear();
        m_hitCountLabel->setText("Active Feature Hits: 0");
        while (m_activePointsGroup->childCount() > 0) {
            delete m_activePointsGroup->takeChild(0);
        }
        m_activePointsGroup->setText(2, "0 pts");
        if (m_viewport) { m_viewport->commitActiveHitMarkers(); }
    }

    bool MainWindow::writeDmisProgramToFile(const QString& filePath, const QString& partName) {
        QFile dmisFile(filePath);
        if (!dmisFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "Export Error", "Could not write DMIS 5.3 file to:\n" + filePath);
            return false;
        }

        QTextStream dmis(&dmisFile);
        const QString timestamp = QDateTime::currentDateTime().toString(Qt::ISODate);

        double zClearMm = 100.0;
        double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        if (m_viewport && m_viewport->getCadBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)) {
            zClearMm = zmax + 25.0;
        }

        dmis << "DMISMN/'OpenCMM Offline Inspection Program', 05.3\n"
            << "$$ Generated by OpenCMM Studio (JLS Machine LLC) on " << timestamp << "\n"
            << "FILNAM/'" << partName << "', 05.3\n"
            << "UNITS/MM, ANGDEC\n"
            << "WKPLAN/XYPLAN\n"
            << "MODE/PROG, MAN\n"
            << "SNSTYP/INDEX, 1, " << QString::number(m_effectiveProbeRadius * 2.0, 'f', 4) << "\n"
            << "CLEARP/ZPLUS, " << QString::number(zClearMm, 'f', 4) << "\n\n";

        for (const auto& mf : m_solvedFeatures) {
            const QString rawLabel = QString("FEAT_%1").arg(mf.id);
            const QString qpidUuid = m_featureRefIds.count(mf.id) && !m_featureRefIds[mf.id].isEmpty()
                ? m_featureRefIds[mf.id]
                : QString("10000000-0000-4000-8000-%1").arg(mf.id, 12, 10, QChar('0'));

            dmis << "$$ =========================================================\n"
                << "$$ Feature: " << QString::fromStdString(mf.name) << "\n"
                << "REFID/F(" << rawLabel << "), QIF, 'QPId', '" << qpidUuid << "'\n";

            if (mf.kind == FeatureKind::Plane3D) {
                dmis << "F(" << rawLabel << ") = FEAT/PLANE, CART, "
                    << QString::number(mf.location.x(), 'f', 4) << ", "
                    << QString::number(mf.location.y(), 'f', 4) << ", "
                    << QString::number(mf.location.z(), 'f', 4) << ", "
                    << QString::number(mf.primaryVector.x(), 'f', 5) << ", "
                    << QString::number(mf.primaryVector.y(), 'f', 5) << ", "
                    << QString::number(mf.primaryVector.z(), 'f', 5) << "\n";
            }
            else if (mf.kind == FeatureKind::Circle2D) {
                dmis << "F(" << rawLabel << ") = FEAT/CIRCLE, INNER, CART, "
                    << QString::number(mf.location.x(), 'f', 4) << ", "
                    << QString::number(mf.location.y(), 'f', 4) << ", "
                    << QString::number(mf.location.z(), 'f', 4) << ", "
                    << QString::number(mf.primaryVector.x(), 'f', 5) << ", "
                    << QString::number(mf.primaryVector.y(), 'f', 5) << ", "
                    << QString::number(mf.primaryVector.z(), 'f', 5) << ", "
                    << QString::number(mf.diameter, 'f', 4) << "\n";
            }
            else if (mf.kind == FeatureKind::Sphere3D) {
                dmis << "F(" << rawLabel << ") = FEAT/SPHERE, OUTER, CART, "
                    << QString::number(mf.location.x(), 'f', 4) << ", "
                    << QString::number(mf.location.y(), 'f', 4) << ", "
                    << QString::number(mf.location.z(), 'f', 4) << ", "
                    << QString::number(mf.diameter, 'f', 4) << "\n";
            }
            else if (mf.kind == FeatureKind::Line2D) {
                dmis << "F(" << rawLabel << ") = FEAT/LINE, UNBND, CART, "
                    << QString::number(mf.location.x(), 'f', 4) << ", "
                    << QString::number(mf.location.y(), 'f', 4) << ", "
                    << QString::number(mf.location.z(), 'f', 4) << ", "
                    << QString::number(mf.primaryVector.x(), 'f', 5) << ", "
                    << QString::number(mf.primaryVector.y(), 'f', 5) << ", "
                    << QString::number(mf.primaryVector.z(), 'f', 5) << ", "
                    << QString::number(mf.secondaryVector.x(), 'f', 5) << ", "
                    << QString::number(mf.secondaryVector.y(), 'f', 5) << ", "
                    << QString::number(mf.secondaryVector.z(), 'f', 5) << "\n";
            }

            const auto& pts = m_featureDmisHits[mf.id];
            const QString dmisMeasType = (mf.kind == FeatureKind::Plane3D) ? "PLANE"
                : (mf.kind == FeatureKind::Circle2D ? "CIRCLE"
                    : (mf.kind == FeatureKind::Sphere3D ? "SPHERE" : "LINE"));

            dmis << "MEAS/" << dmisMeasType << ", F(" << rawLabel << "), " << pts.size() << "\n";
            for (const auto& pt : pts) {
                dmis << "  PTMEAS/CART, "
                    << QString::number(pt.surfacePt.x(), 'f', 4) << ", "
                    << QString::number(pt.surfacePt.y(), 'f', 4) << ", "
                    << QString::number(pt.surfacePt.z(), 'f', 4) << ", "
                    << QString::number(pt.normalVec.x(), 'f', 5) << ", "
                    << QString::number(pt.normalVec.y(), 'f', 5) << ", "
                    << QString::number(pt.normalVec.z(), 'f', 5) << "\n";
            }
            dmis << "ENDMES\n";

            if (m_featureDatumLetters.count(mf.id) && !m_featureDatumLetters[mf.id].isEmpty()) {
                const QString datLet = m_featureDatumLetters[mf.id];
                dmis << "DATDEF/F(" << rawLabel << "), DAT(" << datLet << ")\n";
            }
            dmis << "\n";
        }

        const size_t startPlannedIdx = m_solvedFeatures.size();
        for (size_t i = startPlannedIdx; i < m_dmisProgram.features.size(); ++i) {
            const auto& df = m_dmisProgram.features[i];
            const QString rawLabel = QString::fromStdString(df.label);
            const QString qpidUuid = df.refId.empty()
                ? QString("10000000-0000-4000-8000-%1").arg(i + 1, 12, 10, QChar('0'))
                : QString::fromStdString(df.refId);

            dmis << "$$ =========================================================\n"
                << "$$ Planned Offline Feature: " << rawLabel << "\n"
                << "REFID/F(" << rawLabel << "), QIF, 'QPId', '" << qpidUuid << "'\n";

            if (df.featureType == "PLANE") {
                dmis << "F(" << rawLabel << ") = FEAT/PLANE, CART, "
                    << QString::number(df.nomX, 'f', 4) << ", "
                    << QString::number(df.nomY, 'f', 4) << ", "
                    << QString::number(df.nomZ, 'f', 4) << ", "
                    << QString::number(df.nomI, 'f', 5) << ", "
                    << QString::number(df.nomJ, 'f', 5) << ", "
                    << QString::number(df.nomK, 'f', 5) << "\n";
            }
            else if (df.featureType == "CYLNDR" || df.featureType == "CYL") {
                dmis << "F(" << rawLabel << ") = FEAT/CYLNDR, " << QString::fromStdString(df.boundarySub) << ", CART, "
                    << QString::number(df.nomX, 'f', 4) << ", "
                    << QString::number(df.nomY, 'f', 4) << ", "
                    << QString::number(df.nomZ, 'f', 4) << ", "
                    << QString::number(df.nomI, 'f', 5) << ", "
                    << QString::number(df.nomJ, 'f', 5) << ", "
                    << QString::number(df.nomK, 'f', 5) << ", "
                    << QString::number(df.nominalDiameter, 'f', 4) << "\n";
            }
            else if (df.featureType == "CIRCLE") {
                dmis << "F(" << rawLabel << ") = FEAT/CIRCLE, " << QString::fromStdString(df.boundarySub) << ", CART, "
                    << QString::number(df.nomX, 'f', 4) << ", "
                    << QString::number(df.nomY, 'f', 4) << ", "
                    << QString::number(df.nomZ, 'f', 4) << ", "
                    << QString::number(df.nomI, 'f', 5) << ", "
                    << QString::number(df.nomJ, 'f', 5) << ", "
                    << QString::number(df.nomK, 'f', 5) << ", "
                    << QString::number(df.nominalDiameter, 'f', 4) << "\n";
            }
            else if (df.featureType == "SPHERE") {
                dmis << "F(" << rawLabel << ") = FEAT/SPHERE, " << QString::fromStdString(df.boundarySub) << ", CART, "
                    << QString::number(df.nomX, 'f', 4) << ", "
                    << QString::number(df.nomY, 'f', 4) << ", "
                    << QString::number(df.nomZ, 'f', 4) << ", "
                    << QString::number(df.nominalDiameter, 'f', 4) << "\n";
            }
            else if (df.featureType == "LINE") {
                dmis << "F(" << rawLabel << ") = FEAT/LINE, UNBND, CART, "
                    << QString::number(df.nomX, 'f', 4) << ", "
                    << QString::number(df.nomY, 'f', 4) << ", "
                    << QString::number(df.nomZ, 'f', 4) << ", "
                    << QString::number(df.nomI, 'f', 5) << ", "
                    << QString::number(df.nomJ, 'f', 5) << ", "
                    << QString::number(df.nomK, 'f', 5) << ", 0.00000, 0.00000, 1.00000\n";
            }

            dmis << "MEAS/" << QString::fromStdString(df.featureType) << ", F(" << rawLabel << "), " << df.points.size() << "\n";
            for (const auto& pt : df.points) {
                dmis << "  PTMEAS/CART, "
                    << QString::number(pt.x, 'f', 4) << ", "
                    << QString::number(pt.y, 'f', 4) << ", "
                    << QString::number(pt.z, 'f', 4) << ", "
                    << QString::number(pt.i, 'f', 5) << ", "
                    << QString::number(pt.j, 'f', 5) << ", "
                    << QString::number(pt.k, 'f', 5) << "\n";
            }
            dmis << "ENDMES\n";

            if (!df.datumLetter.empty()) {
                dmis << "DATDEF/F(" << rawLabel << "), DAT(" << QString::fromStdString(df.datumLetter) << ")\n";
            }
            dmis << "\n";
        }

        if ((m_datumAFeatureId > 0 && m_datumBFeatureId > 0 && m_datumCFeatureId > 0) ||
            m_dmisProgram.datumAlignment.has321Alignment) {
            dmis << "$$ =========================================================\n"
                << "$$ Automated 3-2-1 Datum Reference Frame Alignment (PCS)\n"
                << "D(PCS_ABC) = DATSET/DAT(A), ZDIR, ZORIG, DAT(B), XDIR, YORIG, DAT(C), XORIG\n"
                << "SAVE/D(PCS_ABC)\n\n";
        }

        dmis << "ENDFIL\n";
        dmisFile.close();
        statusBar()->showMessage("Saved DMIS 5.3 Inspection Program to: " + filePath);
        return true;
    }

    void MainWindow::onExportDmisProgramClicked() {
        if (m_solvedFeatures.empty() && m_dmisProgram.features.empty()) {
            QMessageBox::information(
                this,
                "No Features to Export",
                "Please create at least one offline feature ('➕ Create Feature') or solve a feature before saving a DMIS 5.3 (.dmis) program."
            );
            return;
        }

        bool ok = false;
        QString partName = QInputDialog::getText(
            this,
            "Save DMIS 5.3 Inspection Program",
            "Enter Part / Job Number / Program Name:",
            QLineEdit::Normal,
            "JLS-PART-001",
            &ok
        );
        if (!ok) return;
        if (partName.trimmed().isEmpty()) partName = "JLS-PART-001";

        QString filePath = QFileDialog::getSaveFileName(
            this,
            "Save DMIS 5.3 Inspection Program (.dmis)",
            partName + ".dmis",
            "DMIS 5.3 Inspection Program (*.dmis);;All Files (*.*)"
        );
        if (filePath.isEmpty()) return;

        if (!filePath.endsWith(".dmis", Qt::CaseInsensitive)) {
            filePath += ".dmis";
        }

        writeDmisProgramToFile(filePath, partName);
    }

    void MainWindow::onExportReportClicked() {
        if (m_featureTree->topLevelItemCount() <= 1) {
            QMessageBox::information(this, "No Features to Export", "Please measure and solve at least one feature before exporting an inspection report or DMIS program.");
            return;
        }

        QDialog expDlg(this);
        expDlg.setWindowTitle("Export Inspection Report / DMIS 5.3 Program");
        expDlg.setMinimumWidth(420);

        auto* dlgLayout = new QVBoxLayout(&expDlg);
        auto* formLayout = new QFormLayout();
        formLayout->setSpacing(10);

        auto* partEdit = new QLineEdit("JLS-PART-001", &expDlg);
        auto* formatCombo = new QComboBox(&expDlg);
        formatCombo->addItem("PDF Printable Inspection Certificate (.pdf)", "pdf");
        formatCombo->addItem("DMIS 5.3 Inspection Program (.dmis)", "dmis");
        formatCombo->addItem("HTML Printable Inspection Report (.html)", "html");
        formatCombo->addItem("QIF 2.1 Quality Information Framework (.qif)", "qif");
        formatCombo->addItem("CSV Spreadsheet (.csv)", "csv");

        formLayout->addRow("Part / Job Number:", partEdit);
        formLayout->addRow("Export Format:", formatCombo);
        dlgLayout->addLayout(formLayout);

        auto* btnBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &expDlg);
        connect(btnBox, &QDialogButtonBox::accepted, &expDlg, &QDialog::accept);
        connect(btnBox, &QDialogButtonBox::rejected, &expDlg, &QDialog::reject);
        dlgLayout->addWidget(btnBox);

        if (expDlg.exec() != QDialog::Accepted) return;

        QString partName = partEdit->text().trimmed();
        if (partName.isEmpty()) partName = "JLS-PART-001";

        const QString fmt = formatCombo->currentData().toString();
        QString defaultFileName;
        QString filterStr;

        if (fmt == "pdf") {
            defaultFileName = partName + "_Inspection_Certificate.pdf";
            filterStr = "PDF Inspection Certificate (*.pdf);;All Files (*.*)";
        }
        else if (fmt == "dmis") {
            defaultFileName = partName + ".dmis";
            filterStr = "DMIS 5.3 Inspection Program (*.dmis);;All Files (*.*)";
        }
        else if (fmt == "qif") {
            defaultFileName = partName + "_Inspection.qif";
            filterStr = "QIF 2.1 Quality Information Framework (*.qif);;All Files (*.*)";
        }
        else if (fmt == "csv") {
            defaultFileName = partName + "_Inspection_Report.csv";
            filterStr = "CSV Spreadsheet (*.csv);;All Files (*.*)";
        }
        else {
            defaultFileName = partName + "_Inspection_Report.html";
            filterStr = "HTML Printable Inspection Report (*.html);;All Files (*.*)";
        }

        QString filePath = QFileDialog::getSaveFileName(
            this,
            "Save Exported File",
            defaultFileName,
            filterStr
        );
        if (filePath.isEmpty()) return;

        if (fmt == "pdf" && !filePath.endsWith(".pdf", Qt::CaseInsensitive)) filePath += ".pdf";
        else if (fmt == "dmis" && !filePath.endsWith(".dmis", Qt::CaseInsensitive)) filePath += ".dmis";
        else if (fmt == "qif" && !filePath.endsWith(".qif", Qt::CaseInsensitive)) filePath += ".qif";
        else if (fmt == "csv" && !filePath.endsWith(".csv", Qt::CaseInsensitive)) filePath += ".csv";
        else if (fmt == "html" && !filePath.endsWith(".html", Qt::CaseInsensitive)) filePath += ".html";

        // --- 0. Native PDF Inspection Certificate Export (.pdf) ---
        if (fmt == "pdf" || filePath.endsWith(".pdf", Qt::CaseInsensitive)) {
            QPdfWriter pdfWriter(filePath);
            pdfWriter.setPageSize(QPageSize(QPageSize::A4));
            pdfWriter.setPageOrientation(QPageLayout::Portrait);
            pdfWriter.setTitle("JLS Machine LLC — Inspection Certificate");

            QPainter painter(&pdfWriter);
            int yPos = 1200;
            const int leftMargin = 1000;

            painter.setFont(QFont("Segoe UI", 16, QFont::Bold));
            painter.drawText(leftMargin, yPos, "JLS MACHINE LLC — INSPECTION CERTIFICATE");
            yPos += 400;

            painter.setFont(QFont("Segoe UI", 10, QFont::Normal));
            const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
            painter.drawText(leftMargin, yPos, "Part / Job Number: " + partName); yPos += 250;
            painter.drawText(leftMargin, yPos, "Inspection Date: " + timestamp); yPos += 250;
            painter.drawText(leftMargin, yPos, QString("Stylus Effective Radius (R_eff): %1 mm").arg(m_effectiveProbeRadius, 0, 'f', 4)); yPos += 400;

            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(30, 41, 59));
            painter.drawRect(leftMargin, yPos, 19000, 350);

            painter.setPen(Qt::white);
            painter.setFont(QFont("Segoe UI", 9, QFont::Bold));
            painter.drawText(leftMargin + 100, yPos + 240, "Feature / Parameter");
            painter.drawText(leftMargin + 7000, yPos + 240, "Nominal");
            painter.drawText(leftMargin + 10500, yPos + 240, "Actual");
            painter.drawText(leftMargin + 14000, yPos + 240, "±Tol");
            painter.drawText(leftMargin + 17000, yPos + 240, "Status");
            yPos += 450;

            painter.setFont(QFont("Segoe UI", 9, QFont::Normal));
            for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                if (top == m_activePointsGroup) continue;

                if (yPos > 26000) {
                    pdfWriter.newPage();
                    yPos = 1200;
                }

                const QString statusTxt = top->text(4);
                const QColor statusCol = statusTxt.contains("FAIL") ? QColor(185, 28, 28) : QColor(21, 128, 61);

                painter.setPen(QColor(15, 23, 42));
                painter.drawText(leftMargin + 100, yPos, top->text(0));
                painter.drawText(leftMargin + 7000, yPos, top->text(1));
                painter.drawText(leftMargin + 10500, yPos, top->text(2));
                painter.drawText(leftMargin + 14000, yPos, top->text(3));

                painter.setPen(statusCol);
                painter.drawText(leftMargin + 17000, yPos, statusTxt);

                yPos += 350;

                painter.setPen(QColor(100, 116, 139));
                painter.setFont(QFont("Consolas", 8, QFont::Normal));
                for (int c = 0; c < top->childCount(); ++c) {
                    QTreeWidgetItem* child = top->child(c);
                    if (yPos > 26000) { pdfWriter.newPage(); yPos = 1200; }
                    painter.drawText(leftMargin + 300, yPos, "  ↳ " + child->text(0) + " : " + child->text(2));
                    yPos += 280;
                }
                painter.setFont(QFont("Segoe UI", 9, QFont::Normal));
                yPos += 100;
            }

            painter.end();
            QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
            statusBar()->showMessage("Exported PDF Inspection Certificate to: " + filePath);
            return;
        }

        // --- 1. DMIS 5.3 Program Export (.dmis) ---
        if (fmt == "dmis" || filePath.endsWith(".dmis", Qt::CaseInsensitive)) {
            writeDmisProgramToFile(filePath, partName);
            return;
        }

        // --- 2. QIF 2.1 XML Export ---
        if (filePath.endsWith(".qif", Qt::CaseInsensitive)) {
            std::vector<InspectionResultItem> qifItems;

            for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                if (top == m_activePointsGroup) continue;

                const int featId = top->data(0, ROLE_FEATURE_ID).isValid() ? top->data(0, ROLE_FEATURE_ID).toInt() : i;
                InspectionResultItem item;
                item.featureName = top->text(0);
                item.refId = (m_featureRefIds.count(featId) && !m_featureRefIds[featId].isEmpty())
                    ? m_featureRefIds[featId]
                    : ("REFID_" + QString::number(featId));
                item.featureType = top->text(0).contains("Circle") || top->text(0).contains("Bore") || top->text(0).contains("Cylinder") ? "Bore"
                    : (top->text(0).contains("Plane") ? "Plane" : "Feature");
                item.nominalMm = top->data(0, ROLE_NOMINAL_MM).isValid() ? top->data(0, ROLE_NOMINAL_MM).toDouble() : 0.0;
                item.actualMm = top->data(0, ROLE_ACTUAL_MM).isValid() ? top->data(0, ROLE_ACTUAL_MM).toDouble() : 0.0;
                item.plusTolMm = top->data(0, ROLE_PLUS_TOL_MM).isValid() ? top->data(0, ROLE_PLUS_TOL_MM).toDouble() : m_defaultTolMm;
                item.minusTolMm = top->data(0, ROLE_MINUS_TOL_MM).isValid() ? top->data(0, ROLE_MINUS_TOL_MM).toDouble() : m_defaultTolMm;
                item.passed = !top->text(4).contains("FAIL");

                qifItems.push_back(item);
            }

            if (QifExporter::exportToQif(filePath, partName, qifItems)) {
                QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
                statusBar()->showMessage("Exported QIF 2.1 Inspection Data to: " + filePath);
            }
            else {
                QMessageBox::critical(this, "Export Error", "Could not write QIF file to: " + filePath);
            }
            return;
        }

        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "Export Error", "Could not open file for writing:\n" + filePath);
            return;
        }
        QTextStream out(&file);

        // --- 3. CSV Export ---
        if (filePath.endsWith(".csv", Qt::CaseInsensitive)) {
            out << "Feature / Parameter,Nominal,Actual,Tolerance,Deviation / Status\n";
            for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                if (top == m_activePointsGroup) continue;

                out << "\"" << top->text(0) << "\","
                    << "\"" << top->text(1) << "\","
                    << "\"" << top->text(2) << "\","
                    << "\"" << top->text(3) << "\","
                    << "\"" << top->text(4) << "\"\n";

                for (int c = 0; c < top->childCount(); ++c) {
                    QTreeWidgetItem* child = top->child(c);
                    out << "  \"" << child->text(0) << "\","
                        << "\"" << child->text(1) << "\","
                        << "\"" << child->text(2) << "\","
                        << "\"" << child->text(3) << "\","
                        << "\"" << child->text(4) << "\"\n";
                }
            }
            file.close();
            QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
            statusBar()->showMessage("Exported CSV Inspection Report to: " + filePath);
            return;
        }

        // --- 4. HTML Printable Report Export ---
        const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
        const QString unitLabel = (m_unitSystem == UnitSystem::Millimeters) ? "Millimeters (mm)" : "Inches (in)";

        out << "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Inspection Report - " << partName.toHtmlEscaped() << "</title>"
            << "<style>"
            << "body { font-family: 'Segoe UI', Tahoma, sans-serif; margin: 32px; color: #1e2227; }"
            << "h1 { margin-bottom: 4px; color: #0f172a; }"
            << ".meta { margin-bottom: 20px; font-size: 13px; color: #475569; }"
            << "table { width: 100%; border-collapse: collapse; margin-top: 12px; font-size: 13px; }"
            << "th, td { border: 1px solid #cbd5e1; padding: 8px 10px; text-align: left; }"
            << "th { background-color: #1e293b; color: #ffffff; }"
            << ".subrow { background-color: #f8fafc; color: #475569; font-family: Consolas, monospace; font-size: 12px; }"
            << ".pass { color: #15803d; font-weight: bold; }"
            << ".fail { color: #b91c1c; font-weight: bold; }"
            << "</style></head><body>"
            << "<h1>JLS Machine LLC — Dimensional Inspection Report</h1>"
            << "<div class='meta'>"
            << "<b>Part / Job:</b> " << partName.toHtmlEscaped() << " &nbsp;|&nbsp; "
            << "<b>Date:</b> " << timestamp << " &nbsp;|&nbsp; "
            << "<b>Units:</b> " << unitLabel << " &nbsp;|&nbsp; "
            << "<b>Stylus R_eff:</b> " << QString::number(m_effectiveProbeRadius, 'f', 4) << " mm"
            << "</div>"
            << "<table><thead><tr>"
            << "<th>Feature / Parameter</th><th>Nominal</th><th>Actual</th><th>±Tol</th><th>Dev / Status</th>"
            << "</tr></thead><tbody>";

        for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
            QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
            if (top == m_activePointsGroup) continue;

            const QString statusText = top->text(4);
            const QString statusClass = statusText.contains("FAIL") ? "fail" : (statusText.contains("PASS") ? "pass" : "");

            out << "<tr>"
                << "<td><b>" << top->text(0).toHtmlEscaped() << "</b></td>"
                << "<td>" << top->text(1).toHtmlEscaped() << "</td>"
                << "<td><b>" << top->text(2).toHtmlEscaped() << "</b></td>"
                << "<td>" << top->text(3).toHtmlEscaped() << "</td>"
                << "<td class='" << statusClass << "'>" << statusText.toHtmlEscaped() << "</td>"
                << "</tr>";

            for (int c = 0; c < top->childCount(); ++c) {
                QTreeWidgetItem* child = top->child(c);
                out << "<tr class='subrow'>"
                    << "<td style='padding-left:24px;'>&hookrightarrow; " << child->text(0).toHtmlEscaped() << "</td>"
                    << "<td>" << child->text(1).toHtmlEscaped() << "</td>"
                    << "<td>" << child->text(2).toHtmlEscaped() << "</td>"
                    << "<td>" << child->text(3).toHtmlEscaped() << "</td>"
                    << "<td>" << child->text(4).toHtmlEscaped() << "</td>"
                    << "</tr>";
            }
        }

        out << "</tbody></table></body></html>";
        file.close();
        QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
        statusBar()->showMessage("Exported HTML Inspection Report to: " + filePath);
    }

    void MainWindow::onSimulateHitClicked() {
        if (m_isDmisProgramActive && m_activeDmisFeatureIdx >= 0 &&
            m_activeDmisFeatureIdx < static_cast<int>(m_dmisProgram.features.size())) {
            const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
            const int ptIdx = static_cast<int>(m_recordedHits.size());
            if (ptIdx < static_cast<int>(df.points.size())) {
                const auto& tgt = df.points[ptIdx];
                CadSurfaceHit simHit;
                simHit.success = true;
                simHit.surfacePoint = gp_Pnt(tgt.x, tgt.y, tgt.z);
                const double len = std::hypot(tgt.i, std::hypot(tgt.j, tgt.k));
                simHit.surfaceNormal = (len > 1e-9)
                    ? gp_Dir(tgt.i / len, tgt.j / len, tgt.k / len)
                    : gp_Dir(0.0, 0.0, 1.0);
                onCadSurfaceProbed(simHit);
                return;
            }
        }

        const int idx = static_cast<int>(m_recordedHits.size());
        const int mode = m_simTargetCombo ? m_simTargetCombo->currentData().toInt() : 0;
        const double rProbe = m_effectiveProbeRadius;

        Position3D simHit{ 0.0, 0.0, 0.0 };

        if (mode == 0) {
            const double R = 12.7000 + rProbe;
            const double goldenAngle = 2.399963229728653;
            const double theta = idx * goldenAngle;
            const double phi = std::acos(1.0 - 0.65 * ((idx % 12) + 0.5) / 12.0);
            simHit = {
                60.0 + R * std::sin(phi) * std::cos(theta),
                40.0 + R * std::sin(phi) * std::sin(theta),
                25.0 + R * std::cos(phi)
            };
        }
        else if (mode == 1) {
            const double R = 10.0000 - rProbe;
            const double angle = idx * (2.0 * 3.141592653589793 / 5.0) + 0.2;
            const double zLevel = 10.0 + (idx / 5) * 5.0;
            simHit = { 50.0 + R * std::cos(angle), 30.0 + R * std::sin(angle), zLevel };
        }
        else if (mode == 2) {
            const double R = 6.0000 - rProbe;
            const double angle = idx * (2.0 * 3.141592653589793 / 5.0) + 0.4;
            const double zLevel = 10.0 + (idx / 5) * 5.0;
            simHit = { 100.0 + R * std::cos(angle), 30.0 + R * std::sin(angle), zLevel };
        }
        else if (mode == 3) {
            const double xGrid[6] = { 15.0, 85.0, 85.0, 15.0, 50.0, 50.0 };
            const double yGrid[6] = { 10.0, 10.0, 55.0, 55.0, 30.0, 45.0 };
            simHit = { xGrid[idx % 6], yGrid[idx % 6], 20.0 + rProbe };
        }
        else if (mode == 4) {
            simHit = { 15.0 + idx * 20.0, -rProbe, 15.0 };
        }
        else if (mode == 5) {
            simHit = { -rProbe, 10.0 + idx * 15.0, 15.0 };
        }

        onProbeHit(simHit);
    }

    void MainWindow::onCalibrateProbeClicked() {
        if (m_recordedHits.size() < 4) {
            QMessageBox::warning(this, "Insufficient Points", "Please record at least 4 hits on the Master Ball before launching the calibration wizard.");
            return;
        }

        CalibrationWidget wizard(m_effectiveProbeRadius, m_certifiedBallDiameterMm, this);

        for (const auto& pt : m_recordedHits) {
            wizard.addCalibrationHit(pt);
        }

        if (wizard.exec() != QDialog::Accepted) return;

        m_effectiveProbeRadius = wizard.getEffectiveProbeRadius();
        m_lastProbeLobingMm = wizard.getSphericityError();
        m_certifiedBallDiameterMm = wizard.getCertifiedBallDiameter();
        m_isProbeCalibrated = true;

        updateProbeStatusDisplay();
        playSolveChime(m_lastProbeLobingMm <= m_defaultTolMm);
        finishFeatureBuffer();

        statusBar()->showMessage(QString("Stylus calibrated: R_eff = %1 mm, Lobing = %2 mm")
            .arg(m_effectiveProbeRadius, 0, 'f', 4)
            .arg(m_lastProbeLobingMm, 0, 'f', 4));
    }

    void MainWindow::onFitPlaneClicked() {
        if (m_recordedHits.size() < 3) {
            QMessageBox::warning(this, "Insufficient Points", "At least 3 points are required to fit a 3D Plane.");
            return;
        }

        PlaneFitResult res;

        // If we have an armed CAD plane nominal, use it directly to guarantee zero singularities!
        if (m_hasArmedCadNominal && m_armedCadDiameterMm <= 0.0) {
            res.success = true;
            res.centroid = m_armedCadLocationMm;
            res.normal = m_armedCadNormalMm;
            res.flatness = 0.0001;
        }
        else {
            Eigen::Vector3d avgPos = Eigen::Vector3d::Zero();
            for (const auto& p : m_recordedHits) avgPos += p;
            avgPos /= static_cast<double>(m_recordedHits.size());

            Eigen::Vector3d refPt = avgPos + Eigen::Vector3d(0.0, 0.0, 25.0);
            if (!m_recordedDmisHits.empty()) {
                Eigen::Vector3d avgNorm = Eigen::Vector3d::Zero();
                for (const auto& dh : m_recordedDmisHits) avgNorm += dh.normalVec;
                if (avgNorm.norm() > 1e-6) {
                    refPt = avgPos + avgNorm.normalized() * 25.0;
                }
            }

            PlaneFitResult res = MetrologyMath::fitPlaneLeastSquares(
                m_recordedHits,
                m_effectiveProbeRadius,
                currentCompMode(),
                &refPt
            );

            // Fallback to armed CAD plane only if the sampled points were singular/collinear
            if (!res.success && m_hasArmedCadNominal && m_armedCadDiameterMm <= 0.0) {
                res.success = true;
                res.centroid = m_armedCadLocationMm;
                res.normal = m_armedCadNormalMm;
                res.flatness = 0.0001;
            }

            res = MetrologyMath::fitPlaneLeastSquares(
                m_recordedHits,
                m_effectiveProbeRadius,
                currentCompMode(),
                &refPt
            );
        }

        if (!res.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Fit Failed", "Singular point distribution — could not solve 3D Plane.");
            return;
        }

        m_lastFittedPlaneFrame = WorkplaneFrame::fromPlane(res.centroid, res.normal);
        m_hasCustomPlane = true;

        const int id = m_featureCounter++;
        const QString featName = (!m_activeDmisFeatureName.isEmpty() && m_activeDmisTreeItem)
            ? m_activeDmisFeatureName : QString("Plane #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Plane3D;
        mf.location = res.centroid;
        mf.primaryVector = res.normal;
        mf.diameter = 0.0;
        mf.formError = res.flatness;
        m_featureDmisHits[id] = m_recordedDmisHits;
        if (!m_activeDmisRefId.isEmpty()) m_featureRefIds[id] = m_activeDmisRefId;
        m_solvedFeatures.push_back(mf);

        const bool passed = (res.flatness <= m_defaultTolMm);
        playSolveChime(passed);

        QTreeWidgetItem* planeItem = m_activeDmisTreeItem;
        if (planeItem) {
            planeItem->setText(0, featName);
            while (planeItem->childCount() > 0) delete planeItem->takeChild(0);
            m_activeDmisTreeItem = nullptr;
        }
        else {
            planeItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
        }
        planeItem->setData(0, ROLE_FEATURE_ID, id);
        applyEvaluationToTreeItem(planeItem, res.flatness, true, 0.0, m_defaultTolMm, m_defaultTolMm);

        if (m_viewport) {
            m_viewport->addFittedPlaneVisual(id, res.centroid.x(), res.centroid.y(), res.centroid.z(),
                res.normal.x(), res.normal.y(), res.normal.z());
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        new QTreeWidgetItem(planeItem, {
            "Centroid (X, Y, Z)", "--",
            QString("(%1, %2, %3)%4")
                .arg(toDisplayUnit(res.centroid.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.z(), m_unitSystem), 0, 'f', decimals)
                .arg(unitStr),
            "--", "INFO"
            });
        new QTreeWidgetItem(planeItem, {
            "Normal (I, J, K)", "--",
            QString("(%1, %2, %3)")
                .arg(res.normal.x(), 0, 'f', 5)
                .arg(res.normal.y(), 0, 'f', 5)
                .arg(res.normal.z(), 0, 'f', 5),
            "--", "INFO"
            });
        planeItem->setExpanded(true);

        finishFeatureBuffer();
        statusBar()->showMessage(QString("Solved %1 — Flatness: %2%3")
            .arg(featName)
            .arg(toDisplayUnit(res.flatness, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr));
    }

    void MainWindow::onFitSphereClicked() {
        if (m_recordedHits.size() < 4) {
            QMessageBox::warning(this, "Insufficient Points", "At least 4 points are required to fit a 3D Sphere.");
            return;
        }

        SphereFitResult res = MetrologyMath::fitSphereLeastSquares(
            m_recordedHits,
            m_effectiveProbeRadius,
            currentCompMode()
        );

        if (!res.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Fit Failed", "Could not solve 3D Sphere from the recorded points.");
            return;
        }

        const double actualDiam = res.radius * 2.0;
        const double nominalDiam = m_hasArmedCadNominal ? m_armedCadDiameterMm : m_certifiedBallDiameterMm;

        const int id = m_featureCounter++;
        const QString featName = (!m_activeDmisFeatureName.isEmpty() && m_activeDmisTreeItem)
            ? m_activeDmisFeatureName : QString("Sphere #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Sphere3D;
        mf.location = res.center;
        mf.diameter = actualDiam;
        mf.formError = res.formError;
        m_featureDmisHits[id] = m_recordedDmisHits;
        if (!m_activeDmisRefId.isEmpty()) m_featureRefIds[id] = m_activeDmisRefId;
        m_solvedFeatures.push_back(mf);

        const bool passed = (std::abs(actualDiam - nominalDiam) <= m_defaultTolMm);
        playSolveChime(passed);

        QTreeWidgetItem* sphereItem = m_activeDmisTreeItem;
        if (sphereItem) {
            sphereItem->setText(0, featName);
            while (sphereItem->childCount() > 0) delete sphereItem->takeChild(0);
            m_activeDmisTreeItem = nullptr;
        }
        else {
            sphereItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
        }
        sphereItem->setData(0, ROLE_FEATURE_ID, id);
        applyEvaluationToTreeItem(sphereItem, actualDiam, true, nominalDiam, m_defaultTolMm, m_defaultTolMm);

        if (m_viewport) {
            m_viewport->addFittedSphereVisual(id, res.center.x(), res.center.y(), res.center.z(), res.radius);
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        new QTreeWidgetItem(sphereItem, {
            "Center (X, Y, Z)", "--",
            QString("(%1, %2, %3)%4")
                .arg(toDisplayUnit(res.center.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.z(), m_unitSystem), 0, 'f', decimals)
                .arg(unitStr),
            "--", "INFO"
            });
        new QTreeWidgetItem(sphereItem, {
            "Sphericity / Form", "--",
            QString("%1%2").arg(toDisplayUnit(res.formError, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "--", "FORM"
            });
        sphereItem->setExpanded(true);

        finishFeatureBuffer();
        statusBar()->showMessage(QString("Solved %1 — Ø%2%3")
            .arg(featName)
            .arg(toDisplayUnit(actualDiam, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr));
    }

    void MainWindow::onFitCircleClicked() {
        if (m_recordedHits.size() < 3) {
            QMessageBox::warning(this, "Insufficient Points", "At least 3 points are required to fit a 2D Circle.");
            return;
        }

        WorkplaneFrame wp = currentWorkplane();
        Circle2DFitResult res = MetrologyMath::fitCircle2DLeastSquares(
            m_recordedHits,
            wp,
            m_effectiveProbeRadius,
            currentCompMode()
        );

        if (!res.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Fit Failed", "Collinear points — could not solve 2D Circle.");
            return;
        }

        const double actualDiam = res.radius * 2.0;
        double nominalDiam = 20.0;
        if (m_hasArmedCadNominal && m_armedCadDiameterMm > 0.0) {
            nominalDiam = m_armedCadDiameterMm;
        }
        else if (m_simTargetCombo && m_simTargetCombo->currentData().toInt() == 2) {
            nominalDiam = 12.0;
        }

        const int id = m_featureCounter++;
        const QString featName = (!m_activeDmisFeatureName.isEmpty() && m_activeDmisTreeItem)
            ? m_activeDmisFeatureName : QString("Circle #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Circle2D;
        mf.location = res.center;
        mf.primaryVector = res.planeNormal;
        mf.diameter = actualDiam;
        mf.formError = res.circularity;
        m_featureDmisHits[id] = m_recordedDmisHits;
        if (!m_activeDmisRefId.isEmpty()) m_featureRefIds[id] = m_activeDmisRefId;
        m_solvedFeatures.push_back(mf);

        const bool passed = (std::abs(actualDiam - nominalDiam) <= m_defaultTolMm);
        playSolveChime(passed);

        QTreeWidgetItem* circleItem = m_activeDmisTreeItem;
        if (circleItem) {
            circleItem->setText(0, featName);
            while (circleItem->childCount() > 0) delete circleItem->takeChild(0);
            m_activeDmisTreeItem = nullptr;
        }
        else {
            circleItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
        }
        circleItem->setData(0, ROLE_FEATURE_ID, id);
        applyEvaluationToTreeItem(circleItem, actualDiam, true, nominalDiam, m_defaultTolMm, m_defaultTolMm);

        if (m_viewport) {
            m_viewport->addFittedCircleVisual(id, res.center.x(), res.center.y(), res.center.z(),
                res.planeNormal.x(), res.planeNormal.y(), res.planeNormal.z(),
                res.radius);
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        new QTreeWidgetItem(circleItem, {
            "Center (X, Y, Z)", "--",
            QString("(%1, %2, %3)%4")
                .arg(toDisplayUnit(res.center.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.z(), m_unitSystem), 0, 'f', decimals)
                .arg(unitStr),
            "--", "INFO"
            });
        new QTreeWidgetItem(circleItem, {
            "Circularity", "--",
            QString("%1%2").arg(toDisplayUnit(res.circularity, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "--", "FORM"
            });
        circleItem->setExpanded(true);

        finishFeatureBuffer();
        statusBar()->showMessage(QString("Solved %1 — Ø%2%3")
            .arg(featName)
            .arg(toDisplayUnit(actualDiam, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr));
    }

    void MainWindow::onFitLineClicked() {
        if (m_recordedHits.size() < 2) {
            QMessageBox::warning(this, "Insufficient Points", "At least 2 points are required to fit a 2D Line.");
            return;
        }

        WorkplaneFrame wp = currentWorkplane();
        Line2DFitResult res = MetrologyMath::fitLine2DLeastSquares(
            m_recordedHits,
            wp,
            m_effectiveProbeRadius,
            currentCompMode()
        );

        if (!res.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Fit Failed", "Could not solve 2D Line from the recorded points.");
            return;
        }

        const int id = m_featureCounter++;
        const QString featName = (!m_activeDmisFeatureName.isEmpty() && m_activeDmisTreeItem)
            ? m_activeDmisFeatureName : QString("Line #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Line2D;
        mf.location = res.centroid;
        mf.primaryVector = res.direction;
        mf.secondaryVector = res.edgeNormal;
        mf.diameter = 0.0;
        mf.formError = res.straightness;
        m_featureDmisHits[id] = m_recordedDmisHits;
        if (!m_activeDmisRefId.isEmpty()) m_featureRefIds[id] = m_activeDmisRefId;
        m_solvedFeatures.push_back(mf);

        const bool passed = (res.straightness <= m_defaultTolMm);
        playSolveChime(passed);

        QTreeWidgetItem* lineItem = m_activeDmisTreeItem;
        if (lineItem) {
            lineItem->setText(0, featName);
            while (lineItem->childCount() > 0) delete lineItem->takeChild(0);
            m_activeDmisTreeItem = nullptr;
        }
        else {
            lineItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
        }
        lineItem->setData(0, ROLE_FEATURE_ID, id);
        applyEvaluationToTreeItem(lineItem, res.straightness, true, 0.0, m_defaultTolMm, m_defaultTolMm);

        if (m_viewport && m_recordedHits.size() >= 2) {
            double tMin = 0.0, tMax = 0.0;
            for (size_t i = 0; i < m_recordedHits.size(); ++i) {
                const double t = (m_recordedHits[i] - res.centroid).dot(res.direction);
                if (i == 0 || t < tMin) tMin = t;
                if (i == 0 || t > tMax) tMax = t;
            }
            const Eigen::Vector3d p1 = res.centroid + tMin * res.direction;
            const Eigen::Vector3d p2 = res.centroid + tMax * res.direction;
            m_viewport->addFittedLineVisual(id, p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z());
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        new QTreeWidgetItem(lineItem, {
            "Centroid (X, Y, Z)", "--",
            QString("(%1, %2, %3)%4")
                .arg(toDisplayUnit(res.centroid.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.z(), m_unitSystem), 0, 'f', decimals)
                .arg(unitStr),
            "--", "INFO"
            });
        new QTreeWidgetItem(lineItem, {
            "Direction (I, J, K)", "--",
            QString("(%1, %2, %3)")
                .arg(res.direction.x(), 0, 'f', 5)
                .arg(res.direction.y(), 0, 'f', 5)
                .arg(res.direction.z(), 0, 'f', 5),
            "--", "INFO"
            });
        lineItem->setExpanded(true);

        finishFeatureBuffer();
        statusBar()->showMessage(QString("Solved %1 — Straightness: %2%3")
            .arg(featName)
            .arg(toDisplayUnit(res.straightness, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr));
    }

    void MainWindow::onFitCylinderClicked() {
        if (m_recordedHits.size() < 5) {
            QMessageBox::warning(this, "Insufficient Points", "At least 5 points across multiple depths are required to fit a 3D Cylinder.");
            return;
        }

        CylinderFitResult res = MetrologyMath::fitCylinderLeastSquares(
            m_recordedHits,
            m_effectiveProbeRadius,
            currentCompMode(),
            m_hasArmedCadNominal ? &m_armedCadNormalMm : nullptr
        );

        if (!res.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Fit Failed", "Could not solve 3D Cylinder from the recorded points.");
            return;
        }

        const double actualDiam = res.diameter;
        double nominalDiam = 20.0;
        if (m_hasArmedCadNominal && m_armedCadDiameterMm > 0.0) {
            nominalDiam = m_armedCadDiameterMm;
        }

        const int id = m_featureCounter++;
        const QString featName = (!m_activeDmisFeatureName.isEmpty() && m_activeDmisTreeItem)
            ? m_activeDmisFeatureName : QString("Cylinder #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Circle2D;
        mf.location = res.center;
        mf.primaryVector = res.axis;
        mf.diameter = actualDiam;
        mf.formError = res.cylindricity;
        m_featureDmisHits[id] = m_recordedDmisHits;
        if (!m_activeDmisRefId.isEmpty()) m_featureRefIds[id] = m_activeDmisRefId;
        m_solvedFeatures.push_back(mf);

        const bool passed = (std::abs(actualDiam - nominalDiam) <= m_defaultTolMm);
        playSolveChime(passed);

        QTreeWidgetItem* cylItem = m_activeDmisTreeItem;
        if (cylItem) {
            cylItem->setText(0, featName);
            while (cylItem->childCount() > 0) delete cylItem->takeChild(0);
            m_activeDmisTreeItem = nullptr;
        }
        else {
            cylItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
        }
        cylItem->setData(0, ROLE_FEATURE_ID, id);
        applyEvaluationToTreeItem(cylItem, actualDiam, true, nominalDiam, m_defaultTolMm, m_defaultTolMm);

        if (m_viewport) {
            m_viewport->addFittedCircleVisual(id, res.center.x(), res.center.y(), res.center.z(),
                res.axis.x(), res.axis.y(), res.axis.z(), res.radius);
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        new QTreeWidgetItem(cylItem, {
            "Axis Point (X, Y, Z)", "--",
            QString("(%1, %2, %3)%4")
                .arg(toDisplayUnit(res.center.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.z(), m_unitSystem), 0, 'f', decimals)
                .arg(unitStr),
            "--", "INFO"
            });
        new QTreeWidgetItem(cylItem, {
            "Axis Vector (I, J, K)", "--",
            QString("(%1, %2, %3)")
                .arg(res.axis.x(), 0, 'f', 5)
                .arg(res.axis.y(), 0, 'f', 5)
                .arg(res.axis.z(), 0, 'f', 5),
            "--", "INFO"
            });
        new QTreeWidgetItem(cylItem, {
            "Cylindricity / Form", "--",
            QString("%1%2").arg(toDisplayUnit(res.cylindricity, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "--", "FORM"
            });
        cylItem->setExpanded(true);

        finishFeatureBuffer();
        statusBar()->showMessage(QString("Solved %1 — Ø%2%3, Cylindricity: %4%3")
            .arg(featName)
            .arg(toDisplayUnit(actualDiam, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr)
            .arg(toDisplayUnit(res.cylindricity, m_unitSystem), 0, 'f', decimals));
    }

    void MainWindow::onMeasureRelationClicked() {
        if (m_solvedFeatures.size() < 2) {
            QMessageBox::information(this, "Need At Least 2 Features",
                "Please fit at least two geometric features before computing a distance or angle relation.");
            return;
        }

        QDialog dlg(this);
        dlg.setWindowTitle("Compute Feature-to-Feature Relation");
        auto* form = new QFormLayout(&dlg);

        auto* comboA = new QComboBox(&dlg);
        auto* comboB = new QComboBox(&dlg);
        for (size_t i = 0; i < m_solvedFeatures.size(); ++i) {
            const QString label = QString::fromStdString(m_solvedFeatures[i].name);
            comboA->addItem(label, static_cast<int>(i));
            comboB->addItem(label, static_cast<int>(i));
        }
        comboA->setCurrentIndex(static_cast<int>(m_solvedFeatures.size()) - 2);
        comboB->setCurrentIndex(static_cast<int>(m_solvedFeatures.size()) - 1);

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* hasNomCheck = new QCheckBox("Evaluate against nominal distance", &dlg);
        hasNomCheck->setChecked(true);

        auto* nomSpin = new QDoubleSpinBox(&dlg);
        nomSpin->setRange(0.0, 10000.0);
        nomSpin->setDecimals(decimals);
        nomSpin->setValue(toDisplayUnit(50.0, m_unitSystem));
        nomSpin->setSuffix(unitStr);

        auto* tolSpin = new QDoubleSpinBox(&dlg);
        tolSpin->setRange(0.0001, 100.0);
        tolSpin->setDecimals(decimals);
        tolSpin->setValue(toDisplayUnit(m_defaultTolMm, m_unitSystem));
        tolSpin->setSuffix(unitStr);

        form->addRow("Feature A:", comboA);
        form->addRow("Feature B:", comboB);
        form->addRow("", hasNomCheck);
        form->addRow("Nominal 2D Distance:", nomSpin);
        form->addRow("±Tolerance:", tolSpin);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        form->addRow(buttons);

        if (dlg.exec() != QDialog::Accepted) return;

        const int idxA = comboA->currentData().toInt();
        const int idxB = comboB->currentData().toInt();
        if (idxA == idxB) {
            QMessageBox::warning(this, "Invalid Selection", "Please select two distinct features.");
            return;
        }

        const MeasuredFeature& featA = m_solvedFeatures[idxA];
        const MeasuredFeature& featB = m_solvedFeatures[idxB];
        FeatureRelationResult rel = MetrologyMath::computeFeatureRelation(featA, featB, currentWorkplane());

        if (!rel.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Relation Failed", "Could not compute geometric relation.");
            return;
        }

        const bool useNom = hasNomCheck->isChecked();
        const double scaleToMm = (m_unitSystem == UnitSystem::Millimeters) ? 1.0 : 25.4;
        const double nomMm = nomSpin->value() * scaleToMm;
        const double tolMm = tolSpin->value() * scaleToMm;

        const QString relTitle = QString("Dist [%1 → %2]")
            .arg(QString::fromStdString(featA.name), QString::fromStdString(featB.name));

        auto* relItem = new QTreeWidgetItem(m_featureTree, { relTitle, "", "", "", "" });
        applyEvaluationToTreeItem(relItem, rel.distance2D, useNom, nomMm, tolMm, tolMm);

        new QTreeWidgetItem(relItem, {
            "3D Center Distance", "--",
            QString("%1%2").arg(toDisplayUnit(rel.distance3D, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "--", "INFO"
            });
        new QTreeWidgetItem(relItem, {
            "3D Angle", "--",
            QString("%1°").arg(rel.angleDegrees, 0, 'f', 3),
            "--", "ANGLE"
            });
        new QTreeWidgetItem(relItem, {
            "Delta (dX, dY, dZ)", "--",
            QString("(%1, %2, %3)%4")
                .arg(toDisplayUnit(rel.deltaXYZ.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(rel.deltaXYZ.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(rel.deltaXYZ.z(), m_unitSystem), 0, 'f', decimals)
                .arg(unitStr),
            "--", "INFO"
            });
        relItem->setExpanded(true);

        const bool passed = !useNom || (std::abs(rel.distance2D - nomMm) <= tolMm);
        playSolveChime(passed);
        statusBar()->showMessage(QString("Computed %1: 2D = %2%3, Angle = %4°")
            .arg(relTitle)
            .arg(toDisplayUnit(rel.distance2D, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr)
            .arg(rel.angleDegrees, 0, 'f', 3));
    }

    void MainWindow::onRun321AlignmentClicked() {
        if (m_solvedFeatures.size() < 3) {
            QMessageBox::information(this, "3-2-1 Datum Alignment",
                "Please solve at least 3 features first (e.g., Primary Plane, Secondary Line, Tertiary Point/Circle).");
            return;
        }

        QDialog dlg(this);
        dlg.setWindowTitle("3-2-1 Part Coordinate System (PCS) Alignment");
        auto* form = new QFormLayout(&dlg);

        auto* levelCombo = new QComboBox(&dlg);
        auto* rotateCombo = new QComboBox(&dlg);
        auto* originCombo = new QComboBox(&dlg);

        for (size_t i = 0; i < m_solvedFeatures.size(); ++i) {
            const QString label = QString::fromStdString(m_solvedFeatures[i].name);
            levelCombo->addItem(label, static_cast<int>(i));
            rotateCombo->addItem(label, static_cast<int>(i));
            originCombo->addItem(label, static_cast<int>(i));
        }

        levelCombo->setCurrentIndex(0);
        rotateCombo->setCurrentIndex(std::min<int>(1, static_cast<int>(m_solvedFeatures.size()) - 1));
        originCombo->setCurrentIndex(std::min<int>(2, static_cast<int>(m_solvedFeatures.size()) - 1));

        form->addRow("1. Primary Datum (Level +Z):", levelCombo);
        form->addRow("2. Secondary Datum (Rotate +X):", rotateCombo);
        form->addRow("3. Tertiary Datum (Origin 0,0,0):", originCombo);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        form->addRow(buttons);

        if (dlg.exec() != QDialog::Accepted) return;

        const MeasuredFeature& levelFeat = m_solvedFeatures[levelCombo->currentData().toInt()];
        const MeasuredFeature& rotateFeat = m_solvedFeatures[rotateCombo->currentData().toInt()];
        const MeasuredFeature& originFeat = m_solvedFeatures[originCombo->currentData().toInt()];

        assignFeatureAsDatum(levelFeat.id, "A", false);
        assignFeatureAsDatum(rotateFeat.id, "B", false);
        assignFeatureAsDatum(originFeat.id, "C", false);

        if (!tryAutoSolve321Alignment(true)) {
            return;
        }
    }

    void MainWindow::onEditToleranceClicked() {
        QTreeWidgetItem* item = m_featureTree->currentItem();
        if (!item || item == m_activePointsGroup) {
            QMessageBox::information(this, "Select a Feature",
                "Please highlight a solved feature row in the Inspection Feature Tree to edit its nominal and tolerance.");
            return;
        }
        while (item->parent() != nullptr) {
            item = item->parent();
        }
        if (!item->data(0, ROLE_ACTUAL_MM).isValid()) {
            QMessageBox::information(this, "Not Evaluatable", "The selected row does not contain a primary measurement value.");
            return;
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";
        const double scaleToMm = (m_unitSystem == UnitSystem::Millimeters) ? 1.0 : 25.4;

        const double actualMm = item->data(0, ROLE_ACTUAL_MM).toDouble();
        const double curNomMm = item->data(0, ROLE_NOMINAL_MM).toDouble();
        const double curPlusMm = item->data(0, ROLE_PLUS_TOL_MM).isValid() ? item->data(0, ROLE_PLUS_TOL_MM).toDouble() : m_defaultTolMm;
        const double curMinusMm = item->data(0, ROLE_MINUS_TOL_MM).isValid() ? item->data(0, ROLE_MINUS_TOL_MM).toDouble() : m_defaultTolMm;

        QDialog dlg(this);
        dlg.setWindowTitle("Edit Feature Nominal & Tolerance — " + item->text(0));
        auto* form = new QFormLayout(&dlg);

        auto* nomSpin = new QDoubleSpinBox(&dlg);
        nomSpin->setRange(-10000.0, 10000.0);
        nomSpin->setDecimals(decimals);
        nomSpin->setValue(toDisplayUnit(curNomMm, m_unitSystem));
        nomSpin->setSuffix(unitStr);

        auto* plusSpin = new QDoubleSpinBox(&dlg);
        plusSpin->setRange(0.0, 100.0);
        plusSpin->setDecimals(decimals);
        plusSpin->setValue(toDisplayUnit(curPlusMm, m_unitSystem));
        plusSpin->setSuffix(unitStr);

        auto* minusSpin = new QDoubleSpinBox(&dlg);
        minusSpin->setRange(0.0, 100.0);
        minusSpin->setDecimals(decimals);
        minusSpin->setValue(toDisplayUnit(curMinusMm, m_unitSystem));
        minusSpin->setSuffix(unitStr);

        form->addRow("Feature:", new QLabel(item->text(0), &dlg));
        form->addRow("Nominal Value:", nomSpin);
        form->addRow("+Tolerance:", plusSpin);
        form->addRow("-Tolerance:", minusSpin);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        form->addRow(buttons);

        if (dlg.exec() != QDialog::Accepted) return;

        applyEvaluationToTreeItem(
            item,
            actualMm,
            true,
            nomSpin->value() * scaleToMm,
            plusSpin->value() * scaleToMm,
            minusSpin->value() * scaleToMm
        );
        statusBar()->showMessage("Updated nominal & tolerance for " + item->text(0));
    }

    void MainWindow::onClearPointsClicked() {
        finishFeatureBuffer();
        statusBar()->showMessage("Cleared active point buffer.");
    }

    void MainWindow::onEvaluateCadDeviationClicked() {
        if (m_recordedHits.empty()) {
            QMessageBox::information(this, "No Points in Buffer",
                "Please probe or simulate at least one hit before evaluating 3D CAD surface deviation.");
            return;
        }
        if (!m_viewport || !m_viewport->hasCadModel()) {
            QMessageBox::warning(this, "No STEP Model Loaded",
                "Please load a 3D STEP model ('Load STEP' in the CAD & Deviation tab) before running surface deviation.");
            return;
        }

        const TopoDS_Shape shape = m_viewport->getCombinedCadShape();
        double maxAbsDev = 0.0;

        const int id = m_featureCounter++;
        const QString groupName = QString("CAD Surface Dev #%1 (%2 pts)").arg(id).arg(m_recordedHits.size());
        auto* devGroup = new QTreeWidgetItem(m_featureTree, { groupName, "", "", "", "" });

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        for (size_t i = 0; i < m_recordedHits.size(); ++i) {
            DeviationResult dev = MetrologyMath::computePointToCadDeviation(
                m_recordedHits[i],
                shape,
                m_effectiveProbeRadius,
                currentCompMode()
            );
            if (!dev.success) continue;

            maxAbsDev = std::max(maxAbsDev, std::abs(dev.signedDeviationMm));
            auto* ptRow = new QTreeWidgetItem(devGroup, { QString("Pt #%1").arg(i + 1), "", "", "", "" });
            applyEvaluationToTreeItem(ptRow, dev.signedDeviationMm, true, 0.0, m_defaultTolMm, m_defaultTolMm);

            m_viewport->addDeviationPointMarker(
                dev.measuredPoint.x(),
                dev.measuredPoint.y(),
                dev.measuredPoint.z(),
                dev.signedDeviationMm,
                m_defaultTolMm
            );
        }

        applyEvaluationToTreeItem(devGroup, maxAbsDev, true, 0.0, m_defaultTolMm, m_defaultTolMm);
        devGroup->setExpanded(true);

        playSolveChime(maxAbsDev <= m_defaultTolMm);
        finishFeatureBuffer();
        statusBar()->showMessage(QString("Evaluated CAD Surface Deviation — Max |Dev|: %1%2")
            .arg(toDisplayUnit(maxAbsDev, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr));
    }

    void MainWindow::onFeatureTreeContextMenuRequested(const QPoint& pos) {
        QTreeWidgetItem* item = m_featureTree->itemAt(pos);
        if (!item || item == m_activePointsGroup) {
            return;
        }

        QTreeWidgetItem* topItem = item;
        while (topItem->parent() != nullptr && topItem->parent() != m_activePointsGroup) {
            topItem = topItem->parent();
        }

        QMenu contextMenu(this);
        QAction* datAAction = nullptr;
        QAction* datBAction = nullptr;
        QAction* datCAction = nullptr;
        if (topItem && topItem != m_activePointsGroup && topItem->data(0, ROLE_FEATURE_ID).isValid()) {
            datAAction = contextMenu.addAction("🅰 Set as Primary Datum A (Level +Z)");
            datBAction = contextMenu.addAction("🅱 Set as Secondary Datum B (Rotate +X)");
            datCAction = contextMenu.addAction("🅲 Set as Tertiary Datum C (Origin 0,0,0)");
            contextMenu.addSeparator();
        }
        QAction* deleteAction = contextMenu.addAction("🗑 Delete Feature");

        QAction* selectedAction = contextMenu.exec(m_featureTree->mapToGlobal(pos));
        if (!selectedAction) return;

        if (selectedAction == deleteAction) {
            onDeleteSelectedFeature();
        }
        else if (topItem && topItem->data(0, ROLE_FEATURE_ID).isValid()) {
            const int fId = topItem->data(0, ROLE_FEATURE_ID).toInt();
            if (selectedAction == datAAction) assignFeatureAsDatum(fId, "A", true);
            else if (selectedAction == datBAction) assignFeatureAsDatum(fId, "B", true);
            else if (selectedAction == datCAction) assignFeatureAsDatum(fId, "C", true);
        }
    }

    void MainWindow::onDeleteSelectedFeature() {
        QTreeWidgetItem* item = m_featureTree->currentItem();
        if (!item || item == m_activePointsGroup) return;

        QTreeWidgetItem* topItem = item;
        while (topItem->parent() != nullptr && topItem->parent() != m_activePointsGroup) {
            topItem = topItem->parent();
        }

        if (topItem == m_activePointsGroup) return;

        if (topItem->data(0, ROLE_FEATURE_ID).isValid()) {
            const int featureId = topItem->data(0, ROLE_FEATURE_ID).toInt();

            auto it = std::remove_if(m_solvedFeatures.begin(), m_solvedFeatures.end(), [featureId](const MeasuredFeature& f) {
                return f.id == featureId;
                });
            if (it != m_solvedFeatures.end()) {
                m_solvedFeatures.erase(it, m_solvedFeatures.end());
            }

            if (m_viewport) {
                m_viewport->highlightFittedFeature(-1);
            }
        }

        delete topItem;
        statusBar()->showMessage("Deleted feature from inspection tree.");
    }

    void MainWindow::onClearAllFeaturesClicked() {
        if (m_solvedFeatures.empty() && m_featureTree->topLevelItemCount() <= 1) {
            QMessageBox::information(this, "No Features", "There are no solved features to clear.");
            return;
        }

        const auto resp = QMessageBox::question(
            this,
            "Clear All Features",
            "Are you sure you want to remove all solved features from the inspection tree and viewport?",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        if (resp != QMessageBox::Yes) return;

        stopDmisAutoRun();
        m_isDmisProgramActive = false;
        m_activeDmisFeatureIdx = -1;
        m_dmisProgram = DmisProgramData{};
        m_plannedDmisTreeItems.clear();
        m_activeDmisTreeItem = nullptr;
        m_activeDmisFeatureName.clear();
        m_activeDmisRefId.clear();
        m_featureDmisHits.clear();
        m_featureRefIds.clear();
        m_featureDatumLetters.clear();
        m_datumAFeatureId = -1;
        m_datumBFeatureId = -1;
        m_datumCFeatureId = -1;
        if (m_viewport) {
            m_viewport->clearProgramTargetMarkers();
            m_viewport->clearHitMarkers();
            m_viewport->clearPcsTriadVisual();
        }
        m_solvedFeatures.clear();
        m_featureCounter = 1;
        m_hasCustomPlane = false;
        m_isPartAligned = false;
        m_pcsMatrix = Eigen::Matrix4d::Identity();

        if (m_alignStatusBtn) {
            m_alignStatusBtn->setChecked(false);
            m_alignStatusBtn->setText("🌐\nCoord: MCS");
            m_alignStatusBtn->setStyleSheet(
                "QPushButton { background-color: #3a3f4b; color: #e5c07b; border: 1px solid #e5c07b; "
                "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
        }

        for (int i = m_featureTree->topLevelItemCount() - 1; i >= 0; --i) {
            QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
            if (top != m_activePointsGroup) {
                delete m_featureTree->takeTopLevelItem(i);
            }
        }

        if (m_viewport) {
            m_viewport->clearFittedFeatures();
            m_viewport->highlightFittedFeature(-1);
        }

        updateDroDisplay();
        statusBar()->showMessage("Cleared all solved features and reset alignment.");
    }

    void MainWindow::onThemeToggled(bool dark) {
        m_isDarkMode = dark;

        if (dark) {
            setStyleSheet(
                "QMainWindow { background-color: #1e2227; color: #e0e0e0; }"
                "QDockWidget { color: #ffffff; font-weight: bold; }"
                "QDockWidget::title { background: #282c34; padding: 6px; border: 1px solid #3a3f4b; }"
                "QGroupBox { border: 1px solid #3a3f4b; border-radius: 4px; margin-top: 10px; padding-top: 12px; color: #abb2bf; font-weight: bold; }"
                "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
            );

            if (m_droContainer) m_droContainer->setStyleSheet("background-color: #181a1f;");
            if (m_treeContainer) m_treeContainer->setStyleSheet("background-color: #181a1f;");
            if (m_collisionContainer) m_collisionContainer->setStyleSheet("background-color: #181a1f;");

            if (m_droX) m_droX->setStyleSheet("color: #ffffff; background-color: #0d0f12; border: 1px solid #2c313a; border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 190px;");
            if (m_droY) m_droY->setStyleSheet("color: #ffffff; background-color: #0d0f12; border: 1px solid #2c313a; border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 190px;");
            if (m_droZ) m_droZ->setStyleSheet("color: #ffffff; background-color: #0d0f12; border: 1px solid #2c313a; border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 190px;");

            if (auto* ribbon = findChild<QTabWidget*>("RibbonBar")) {
                ribbon->setStyleSheet(
                    "QTabWidget#RibbonBar::pane { background-color: #21252b; border-top: 1px solid #3a3f4b; border-bottom: 2px solid #181a1f; }"
                    "QTabWidget#RibbonBar QTabBar::tab { background-color: #181a1f; color: #abb2bf; padding: 6px 20px; font-size: 12px; font-weight: bold; border: 1px solid #2c313a; border-bottom: none; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 3px; margin-top: 2px; }"
                    "QTabWidget#RibbonBar QTabBar::tab:hover { background-color: #2c313a; color: #ffffff; }"
                    "QTabWidget#RibbonBar QTabBar::tab:selected { background-color: #21252b; color: #61afef; border-top: 2px solid #61afef; }"
                    "QFrame#RibbonGroup { border-right: 1px solid #3a3f4b; background: transparent; }"
                    "QLabel#RibbonGroupTitle { color: #7f848e; font-size: 9px; font-weight: bold; letter-spacing: 1px; }"
                    "QComboBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 4px 8px; font-weight: bold; min-width: 145px; }"
                    "QComboBox QAbstractItemView { background-color: #181a1f; color: #ffffff; selection-background-color: #2d5a88; }"
                );
            }

            if (m_featureTree) {
                m_featureTree->setStyleSheet(
                    "QTreeWidget { background-color: #181a1f; color: #e0e0e0; border: 1px solid #3a3f4b; }"
                    "QHeaderView::section { background-color: #282c34; color: #abb2bf; padding: 4px; border: 1px solid #3a3f4b; }"
                );
            }

            for (auto* btn : findChildren<QPushButton*>()) {
                if (btn && btn->parent() && btn->parent()->objectName() == "RibbonGroup") {
                    btn->setStyleSheet(
                        "QPushButton { background-color: #282c34; color: #abb2bf; border: 1px solid #3a3f4b; "
                        "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
                        "QPushButton:hover { background-color: #3e4451; border: 1px solid #61afef; color: #ffffff; }"
                        "QPushButton:pressed { background-color: #181a1f; }"
                    );
                }
            }

            if (m_viewport) {
                m_viewport->setBackgroundColor(true);
                m_viewport->setTheme(true);
            }

            statusBar()->setStyleSheet("background-color: #007acc; color: white; font-weight: bold;");
            statusBar()->showMessage("Switched to Dark Metrology Theme.");
        }
        else {
            setStyleSheet(
                "QMainWindow { background-color: #f1f5f9; color: #1e293b; }"
                "QDockWidget { color: #1e293b; font-weight: bold; }"
                "QDockWidget::title { background: #cbd5e1; padding: 6px; border: 1px solid #94a3b8; }"
                "QGroupBox { border: 1px solid #cbd5e1; border-radius: 4px; margin-top: 10px; padding-top: 12px; color: #334155; font-weight: bold; }"
                "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
            );

            if (m_droContainer) m_droContainer->setStyleSheet("background-color: #f8fafc;");
            if (m_treeContainer) m_treeContainer->setStyleSheet("background-color: #f8fafc;");
            if (m_collisionContainer) m_collisionContainer->setStyleSheet("background-color: #f8fafc;");

            if (m_droX) m_droX->setStyleSheet("color: #0f172a; background-color: #ffffff; border: 1px solid #94a3b8; border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 190px;");
            if (m_droY) m_droY->setStyleSheet("color: #0f172a; background-color: #ffffff; border: 1px solid #94a3b8; border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 190px;");
            if (m_droZ) m_droZ->setStyleSheet("color: #0f172a; background-color: #ffffff; border: 1px solid #94a3b8; border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 190px;");

            if (auto* ribbon = findChild<QTabWidget*>("RibbonBar")) {
                ribbon->setStyleSheet(
                    "QTabWidget#RibbonBar::pane { background-color: #ffffff; border-top: 1px solid #cbd5e1; border-bottom: 2px solid #94a3b8; }"
                    "QTabWidget#RibbonBar QTabBar::tab { background-color: #e2e8f0; color: #475569; padding: 6px 20px; font-size: 12px; font-weight: bold; border: 1px solid #cbd5e1; border-bottom: none; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 3px; margin-top: 2px; }"
                    "QTabWidget#RibbonBar QTabBar::tab:hover { background-color: #cbd5e1; color: #0f172a; }"
                    "QTabWidget#RibbonBar QTabBar::tab:selected { background-color: #ffffff; color: #0284c7; border-top: 2px solid #0284c7; }"
                    "QFrame#RibbonGroup { border-right: 1px solid #cbd5e1; background: transparent; }"
                    "QLabel#RibbonGroupTitle { color: #64748b; font-size: 9px; font-weight: bold; letter-spacing: 1px; }"
                    "QComboBox { background-color: #ffffff; color: #0f172a; border: 1px solid #94a3b8; border-radius: 3px; padding: 4px 8px; font-weight: bold; min-width: 145px; }"
                    "QComboBox QAbstractItemView { background-color: #ffffff; color: #0f172a; selection-background-color: #bae6fd; }"
                );
            }

            if (m_featureTree) {
                m_featureTree->setStyleSheet(
                    "QTreeWidget { background-color: #ffffff; color: #1e293b; border: 1px solid #cbd5e1; }"
                    "QHeaderView::section { background-color: #e2e8f0; color: #334155; padding: 4px; border: 1px solid #cbd5e1; }"
                );
            }

            for (auto* btn : findChildren<QPushButton*>()) {
                if (btn && btn->parent() && btn->parent()->objectName() == "RibbonGroup") {
                    btn->setStyleSheet(
                        "QPushButton { background-color: #ffffff; color: #334155; border: 1px solid #cbd5e1; "
                        "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
                        "QPushButton:hover { background-color: #e2e8f0; border: 1px solid #0284c7; color: #0f172a; }"
                        "QPushButton:pressed { background-color: #cbd5e1; }"
                    );
                }
            }

            if (m_viewport) {
                m_viewport->setTheme(false);
            }

            statusBar()->setStyleSheet("background-color: #0284c7; color: white; font-weight: bold;");
            statusBar()->showMessage("Switched to Light Inspection Theme.");
        }
    }

    void MainWindow::onRibbonContextMenuRequested(const QPoint& pos) {
        auto* ribbonTabs = findChild<QTabWidget*>("RibbonBar");
        if (!ribbonTabs) return;

        QMenu contextMenu(this);

        // --- Windows Submenu with Checkmarks ---
        QMenu* windowsMenu = contextMenu.addMenu("🪟 Windows");
        QDockWidget* docks[3] = { m_droDock, m_treeDock, m_collisionDock };

        for (QDockWidget* dock : docks) {
            if (!dock) continue;
            QAction* winAct = windowsMenu->addAction(dock->windowTitle());
            winAct->setCheckable(true);
            winAct->setChecked(dock->isVisible());
            connect(winAct, &QAction::triggered, this, [this, dock](bool checked) {
                if (checked) {
                    dock->show();
                    dock->raise();
                }
                else {
                    dock->hide();
                }
                syncAutoHideTabForDock(dock);
                });
        }

        windowsMenu->addSeparator();
        QAction* showAllWinAct = windowsMenu->addAction("📌 Show && Pin All Windows");
        connect(showAllWinAct, &QAction::triggered, this, [this]() {
            if (m_droDock) setDockPinned(m_droDock, true);
            if (m_treeDock) setDockPinned(m_treeDock, true);
            if (m_collisionDock) setDockPinned(m_collisionDock, true);
            });

        contextMenu.addSeparator();
        QAction* customizeAction = contextMenu.addAction("🛠 Customize Ribbon Tabs & Commands...");
        QAction* addTabAction = contextMenu.addAction("➕ Add New Custom Tab...");
        QAction* shortcutsAction = contextMenu.addAction("⌨ Configure Keyboard Shortcuts..."); // <--- Add this line
        contextMenu.addSeparator();
        QAction* resetAction = contextMenu.addAction("↺ Reset Ribbon to Factory Defaults");

        QAction* selectedAction = contextMenu.exec(ribbonTabs->mapToGlobal(pos));
        if (!selectedAction) return;

        if (selectedAction == customizeAction) {
            onCustomizeRibbonClicked();
        }
        else if (selectedAction == addTabAction) {
            bool ok = false;
            QString tabName = QInputDialog::getText(
                this,
                "New Custom Ribbon Tab",
                "Enter a name for the new Ribbon Tab:",
                QLineEdit::Normal,
                "Shop Floor",
                &ok
            ).trimmed();
            if (ok && !tabName.isEmpty()) {
                auto layout = RibbonCustomizeDialog::loadSavedLayout();
                RibbonTabConfig customTab;
                customTab.name = tabName;
                customTab.visible = true;
                customTab.isCustom = true;
                customTab.commandIds = { "dro_zero", "cad_auto_probe", "prog_create_feature", "dmis_run", "rep_export_report" };
                layout.push_back(customTab);
                RibbonCustomizeDialog::saveLayout(layout);
                loadRibbonCustomization();
                ribbonTabs->setCurrentIndex(ribbonTabs->count() - 1);
                statusBar()->showMessage(QString("Created custom Ribbon tab '%1' — right-click the ribbon to customize its commands.").arg(tabName));
            }
        }
    
        else if (selectedAction == shortcutsAction) {
            onConfigureShortcutsClicked(); // <--- Handle selection
        }
        else if (selectedAction == resetAction) {
            const auto resp = QMessageBox::question(
                this,
                "Reset Ribbon to Defaults",
                "Restore all default Ribbon tabs and command ordering?",
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No
            );
            if (resp == QMessageBox::Yes) {
                RibbonCustomizeDialog::saveLayout(RibbonCustomizeDialog::defaultRibbonLayout());
                loadRibbonCustomization();
                statusBar()->showMessage("Restored Ribbon tabs and commands to factory defaults.");
            }
        }
    }

    void MainWindow::onCustomizeRibbonClicked() {
        RibbonCustomizeDialog dlg(m_isDarkMode, this);
        if (dlg.exec() != QDialog::Accepted) return;

        loadRibbonCustomization();
        statusBar()->showMessage("Ribbon tabs and command order updated.");
    }

    void MainWindow::loadRibbonCustomization() {
        auto* ribbonTabs = findChild<QTabWidget*>("RibbonBar");
        if (!ribbonTabs) return;

        const int savedSimIdx = m_simTargetCombo ? m_simTargetCombo->currentIndex() : 0;
        const int savedCompIdx = m_compModeCombo ? m_compModeCombo->currentIndex() : 0;
        const int savedWpIdx = m_workplaneCombo ? m_workplaneCombo->currentIndex() : 0;
        const int savedStyleIdx = m_featureRenderCombo ? m_featureRenderCombo->currentIndex() : 0;
        const QString prevTabTitle = (ribbonTabs->currentIndex() >= 0)
            ? ribbonTabs->tabText(ribbonTabs->currentIndex())
            : QString();

        m_settingsBtn = nullptr;
        m_mcpStatusBtn = nullptr;
        m_simTargetCombo = nullptr;
        m_compModeCombo = nullptr;
        m_workplaneCombo = nullptr;
        m_featureRenderCombo = nullptr;
        m_alignStatusBtn = nullptr;
        m_autoRunDmisBtnHome = nullptr;
        m_autoRunDmisBtnCad = nullptr;

        while (ribbonTabs->count() > 0) {
            QWidget* page = ribbonTabs->widget(0);
            ribbonTabs->removeTab(0);
            delete page;
        }

        const std::vector<RibbonTabConfig> layout = RibbonCustomizeDialog::loadSavedLayout();
        int restoreTabIdx = -1;

        for (size_t tIdx = 0; tIdx < layout.size(); ++tIdx) {
            const auto& tabCfg = layout[tIdx];

            auto* page = new QWidget(this);
            auto* pageLayout = new QHBoxLayout(page);
            pageLayout->setContentsMargins(6, 2, 6, 2);
            pageLayout->setSpacing(2);

            QString currentGroupTitle;
            QList<QWidget*> currentGroupWidgets;

            auto flushGroup = [&]() {
                if (!currentGroupWidgets.isEmpty()) {
                    pageLayout->addWidget(createRibbonGroup(currentGroupTitle, currentGroupWidgets));
                    currentGroupWidgets.clear();
                }
                };

            for (const QString& cmdId : tabCfg.commandIds) {
                const RibbonCommandMeta* meta = RibbonCustomizeDialog::findCommandMeta(cmdId);
                if (!meta) continue;

                QWidget* w = createCommandWidgetForId(cmdId);
                if (!w) continue;

                if (!currentGroupWidgets.isEmpty() && currentGroupTitle != meta->groupTitle) {
                    flushGroup();
                }
                currentGroupTitle = meta->groupTitle;
                currentGroupWidgets.append(w);
            }
            flushGroup();

            pageLayout->addStretch(1);
            const int addedIdx = ribbonTabs->addTab(page, tabCfg.name);
            ribbonTabs->setTabVisible(addedIdx, tabCfg.visible);

            if (!prevTabTitle.isEmpty() && tabCfg.name == prevTabTitle && tabCfg.visible) {
                restoreTabIdx = addedIdx;
            }
        }

        if (m_simTargetCombo && savedSimIdx >= 0 && savedSimIdx < m_simTargetCombo->count()) {
            m_simTargetCombo->setCurrentIndex(savedSimIdx);
        }
        if (m_compModeCombo && savedCompIdx >= 0 && savedCompIdx < m_compModeCombo->count()) {
            m_compModeCombo->setCurrentIndex(savedCompIdx);
        }
        if (m_workplaneCombo && savedWpIdx >= 0 && savedWpIdx < m_workplaneCombo->count()) {
            m_workplaneCombo->setCurrentIndex(savedWpIdx);
        }
        if (m_featureRenderCombo && savedStyleIdx >= 0 && savedStyleIdx < m_featureRenderCombo->count()) {
            m_featureRenderCombo->setCurrentIndex(savedStyleIdx);
        }

        if (restoreTabIdx >= 0) {
            ribbonTabs->setCurrentIndex(restoreTabIdx);
        }
        else {
            for (int i = 0; i < ribbonTabs->count(); ++i) {
                if (ribbonTabs->isTabVisible(i)) {
                    ribbonTabs->setCurrentIndex(i);
                    break;
                }
            }
        }
    }
    void MainWindow::resetWindowLayout() {
        // 1. Clear all saved window geometry and dock pin states from QSettings
        QSettings settings("JLS Machine", "OpenCMM");
        settings.remove("geometry");
        settings.remove("windowState");
        settings.remove("maximized");
        settings.remove("pinned_LiveDroDock");
        settings.remove("pinned_FeatureTreeDock");
        settings.remove("pinned_CollisionLogDock");
        settings.sync();

        // 2. Reset window geometry to default startup dimensions
        showNormal();
        resize(1280, 800);

        // 3. Forcefully reset in-memory dock states and positions
        if (m_droDock) {
            m_droDock->setFloating(false);
            removeDockWidget(m_droDock);
            addDockWidget(Qt::LeftDockWidgetArea, m_droDock);
            setDockPinned(m_droDock, true);
            m_droDock->show();
        }

        if (m_treeDock) {
            m_treeDock->setFloating(false);
            removeDockWidget(m_treeDock);
            addDockWidget(Qt::LeftDockWidgetArea, m_treeDock);
            setDockPinned(m_treeDock, true);
            m_treeDock->show();
        }

        if (m_collisionDock) {
            m_collisionDock->setFloating(false);
            removeDockWidget(m_collisionDock);
            addDockWidget(Qt::BottomDockWidgetArea, m_collisionDock);
            setDockPinned(m_collisionDock, false); // Unpinned / auto-hidden bottom tab
            m_collisionDock->hide();
        }

        // 4. Update edge tabs, toolbars, and trigger an immediate layout pass
        syncAutoHideTabForDock(m_collisionDock);
        centralWidget()->updateGeometry();
        update();

        statusBar()->showMessage("Window layout and docks successfully restored to factory defaults.");
    }
    namespace {
        struct ShortcutDef {
            const char* id;
            const char* label;
            const char* defaultSeq;
        };

        constexpr ShortcutDef kShortcutCatalog[] = {
            { "sc_zero_axes",        "Zero Axes (DRO)",                "Ctrl+0" },
            { "sc_toggle_units",     "Toggle Units (mm / in)",         "Ctrl+U" },
            { "sc_sim_hit",          "Simulate Probe Hit",             "Space" },
            { "sc_auto_probe_cad",   "Auto-Probe Selected CAD Face",   "Ctrl+Shift+P" },
            { "sc_create_feature",   "Create Offline Feature",         "Ctrl+N" },
            { "sc_run_dmis",         "Run / Stop DMIS Playback",       "F5" },
            { "sc_fit_plane",        "Fit 3D Plane",                   "Ctrl+P" },
            { "sc_fit_sphere",       "Fit 3D Sphere",                  "Ctrl+S" },
            { "sc_fit_circle",       "Fit 2D Circle",                  "Ctrl+C" },
            { "sc_fit_line",         "Fit 2D Line",                    "Ctrl+L" },
            { "sc_fit_cylinder",     "Fit 3D Cylinder",                "Ctrl+Y" },
            { "sc_clear_points",     "Clear Active Point Buffer",      "Escape" },
            { "sc_toggle_collision", "Toggle Collision & Clearance Log","Ctrl+Shift+L" },
            { "sc_open_settings",    "Open Settings",                  "Ctrl+," }
        };
    }

    void MainWindow::applyKeyboardShortcuts() {
        for (QShortcut* sc : m_appShortcuts) {
            delete sc;
        }
        m_appShortcuts.clear();

        QSettings settings("JLS Machine", "OpenCMM");
        settings.beginGroup("Shortcuts");

        auto bindAction = [&](const char* id, const char* defaultSeq, auto slotFunc) {
            const QString seqStr = settings.value(id, QString::fromLatin1(defaultSeq)).toString().trimmed();
            if (seqStr.isEmpty()) return;

            auto* sc = new QShortcut(QKeySequence(seqStr), this);
            sc->setContext(Qt::WindowShortcut);
            connect(sc, &QShortcut::activated, this, slotFunc);
            m_appShortcuts.push_back(sc);
            };

        bindAction("sc_zero_axes", "Ctrl+0", &MainWindow::onZeroAxesClicked);
        bindAction("sc_toggle_units", "Ctrl+U", &MainWindow::onUnitToggled);
        bindAction("sc_sim_hit", "Space", &MainWindow::onSimulateHitClicked);
        bindAction("sc_auto_probe_cad", "Ctrl+Shift+P", &MainWindow::onAutoProbeCadFaceClicked);
        bindAction("sc_create_feature", "Ctrl+N", &MainWindow::onCreateOfflineFeatureClicked);
        bindAction("sc_run_dmis", "F5", &MainWindow::onToggleAutoRunDmisClicked);
        bindAction("sc_fit_plane", "Ctrl+P", &MainWindow::onFitPlaneClicked);
        bindAction("sc_fit_sphere", "Ctrl+S", &MainWindow::onFitSphereClicked);
        bindAction("sc_fit_circle", "Ctrl+C", &MainWindow::onFitCircleClicked);
        bindAction("sc_fit_line", "Ctrl+L", &MainWindow::onFitLineClicked);
        bindAction("sc_fit_cylinder", "Ctrl+Y", &MainWindow::onFitCylinderClicked);
        bindAction("sc_clear_points", "Escape", &MainWindow::onClearPointsClicked);
        bindAction("sc_open_settings", "Ctrl+,", &MainWindow::onOpenSettingsClicked);
        bindAction("sc_guided_probe", "F6", &MainWindow::onStartManualGuidedInspectionClicked);
        bindAction("sc_undo_hit", "Ctrl+Z", &MainWindow::onUndoLastProbeHitClicked);

        // Toggle Collision & Clearance Log dock
        const QString colSeq = settings.value("sc_toggle_collision", "Ctrl+Shift+L").toString().trimmed();
        if (!colSeq.isEmpty()) {
            auto* sc = new QShortcut(QKeySequence(colSeq), this);
            sc->setContext(Qt::WindowShortcut);
            connect(sc, &QShortcut::activated, this, [this]() {
                if (!m_collisionDock) return;
                const bool show = !m_collisionDock->isVisible();
                m_collisionDock->setVisible(show);
                if (show) m_collisionDock->raise();
                syncAutoHideTabForDock(m_collisionDock);
                });
            m_appShortcuts.push_back(sc);
        }

        settings.endGroup();
    }

    void MainWindow::onConfigureShortcutsClicked() {
        QDialog dlg(this);
        dlg.setWindowTitle("Configure Keyboard Shortcuts");
        dlg.resize(540, 460);

        auto* layout = new QVBoxLayout(&dlg);
        auto* tree = new QTreeWidget(&dlg);
        tree->setHeaderLabels({ "Command / Action", "Assigned Shortcut" });
        tree->setColumnCount(2);
        tree->setRootIsDecorated(false);
        tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
        tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);

        QSettings settings("JLS Machine", "OpenCMM");
        settings.beginGroup("Shortcuts");

        struct EditorEntry {
            QString id;
            QString defaultSeq;
            QKeySequenceEdit* editor;
        };
        std::vector<EditorEntry> editors;

        for (const auto& def : kShortcutCatalog) {
            const QString savedSeq = settings.value(def.id, QString::fromLatin1(def.defaultSeq)).toString();
            auto* item = new QTreeWidgetItem(tree, { QString::fromLatin1(def.label), "" });

            auto* seqEdit = new QKeySequenceEdit(QKeySequence(savedSeq), tree);
            seqEdit->setMaximumSequenceLength(1);
            tree->setItemWidget(item, 1, seqEdit);

            editors.push_back({ QString::fromLatin1(def.id), QString::fromLatin1(def.defaultSeq), seqEdit });
        }
        settings.endGroup();

        layout->addWidget(tree);

        auto* infoLbl = new QLabel("Click any shortcut box and press a key combination to rebind.", &dlg);
        infoLbl->setStyleSheet("color: #7f848e; font-size: 11px; font-style: italic;");
        layout->addWidget(infoLbl);

        auto* btnBox = new QDialogButtonBox(
            QDialogButtonBox::Save | QDialogButtonBox::RestoreDefaults | QDialogButtonBox::Cancel,
            &dlg
        );

        connect(btnBox->button(QDialogButtonBox::RestoreDefaults), &QPushButton::clicked, &dlg, [&]() {
            for (auto& e : editors) {
                e.editor->setKeySequence(QKeySequence(e.defaultSeq));
            }
            });

        connect(btnBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(btnBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        layout->addWidget(btnBox);

        if (dlg.exec() != QDialog::Accepted) return;

        settings.beginGroup("Shortcuts");
        for (const auto& e : editors) {
            settings.setValue(e.id, e.editor->keySequence().toString(QKeySequence::PortableText));
        }
        settings.endGroup();
        settings.sync();

        applyKeyboardShortcuts();
        statusBar()->showMessage("Keyboard shortcut bindings saved and activated.");
    }
    bool MainWindow::validateDmisFeaturePath(int featureIndex) {
        if (!m_isDmisProgramActive || featureIndex < 0 || featureIndex >= static_cast<int>(m_dmisProgram.features.size())) {
            return true;
        }
        const auto& df = m_dmisProgram.features[featureIndex];
        if (!m_viewport || !m_viewport->hasCadModel()) return true;

        TopoDS_Shape cadShape = m_viewport->getCombinedCadShape();
        bool pathValid = true;

        for (size_t i = 0; i < df.points.size(); ++i) {
            const auto& pt = df.points[i];
            Eigen::Vector3d surfPt(pt.x, pt.y, pt.z);
            Eigen::Vector3d surfNorm(pt.i, pt.j, pt.k);
            if (surfNorm.norm() > 1e-9) surfNorm.normalize();

            // Check standoff approach position for proximity or collision
            Eigen::Vector3d standoffPos = surfPt + surfNorm * (m_effectiveProbeRadius + 18.0);
            CollisionCheckResult check = MetrologyMath::checkProbeCollision(
                standoffPos,
                m_effectiveProbeRadius,
                cadShape,
                10.0,
                1.5
            );

            if (check.collision || check.warningProximity) {
                QString warningMsg = QString("Pre-flight Path Warning [Feature %1, Pt #%2]: %3")
                    .arg(QString::fromStdString(df.label))
                    .arg(i + 1)
                    .arg(QString::fromStdString(check.message));

                logCollisionEvent(warningMsg.toStdString(), check.collision, check.warningProximity);
                if (check.collision) pathValid = false;
            }
        }

        return pathValid;
    }
   
    void MainWindow::onRunDmisPreFlightCheckClicked() {
        if (!m_isDmisProgramActive || m_dmisProgram.features.empty()) {
            QMessageBox::information(this, "No DMIS Program Loaded", "Please load or generate a DMIS inspection program first.");
            return;
        }

        int totalErrors = 0;
        for (size_t i = 0; i < m_dmisProgram.features.size(); ++i) {
            if (!validateDmisFeaturePath(static_cast<int>(i))) {
                totalErrors++;
            }
        }

        if (totalErrors == 0) {
            playSolveChime(true);
            QMessageBox::information(this, "Pre-Flight Check Passed",
                QString("All %1 planned DMIS inspection features passed CAD collision and clearance checks successfully.").arg(m_dmisProgram.features.size()));
            statusBar()->setStyleSheet("background-color: #28a745; color: white; font-weight: bold;");
            statusBar()->showMessage("Pre-Flight Check Passed: All toolpaths safe.");
        }
        else {
            playSolveChime(false);
            QMessageBox::warning(this, "Pre-Flight Warnings Found",
                QString("Pre-flight check completed with %1 collision error(s). Check the Collision & Clearance Log for details.").arg(totalErrors));
            statusBar()->setStyleSheet("background-color: #b91c1c; color: white; font-weight: bold;");
            statusBar()->showMessage(QString("Pre-Flight Failed: %1 path collision(s) detected.").arg(totalErrors));
        }
    }

    void MainWindow::tryLoadMcpPlugin() {
        const QString appDir = QCoreApplication::applicationDirPath();
        const QStringList candidatePaths = {
            QDir(appDir).filePath("plugins/opencmm_mcp.dll"),
            QDir(appDir).filePath("opencmm_mcp.dll")
        };

        for (const QString& path : candidatePaths) {
            if (!QFile::exists(path)) continue;

            m_mcpPluginLoader = new QPluginLoader(path, this);
            QObject* pluginInstance = m_mcpPluginLoader->instance();
            if (pluginInstance) {
                m_mcpPlugin = qobject_cast<IMcpServerPlugin*>(pluginInstance);
                if (m_mcpPlugin && m_mcpPlugin->startServer(this, 8765)) {
                    logCollisionEvent(
                        QString("MCP Server Module active on 127.0.0.1:%1").arg(m_mcpPlugin->serverPort()).toStdString(),
                        false, false
                    );
                    return;
                }
            }
            delete m_mcpPluginLoader;
            m_mcpPluginLoader = nullptr;
        }

        // If opencmm_mcp.dll is not installed, OpenCMM runs in strict ITAR / offline mode
        logCollisionEvent("ITAR Compliance Mode: MCP Server plugin not installed (Zero external AI endpoints).", false, false);
    }

    QJsonObject MainWindow::getMachineStatusJson() {
        return QJsonObject{
            { "droMm", QJsonObject{
                { "x", m_lastPosition.x },
                { "y", m_lastPosition.y },
                { "z", m_lastPosition.z }
            } },
            { "units", (m_unitSystem == UnitSystem::Millimeters) ? "mm" : "in" },
            { "stylusEffectiveRadiusMm", m_effectiveProbeRadius },
            { "isProbeCalibrated", m_isProbeCalibrated },
            { "isPartAligned321", m_isPartAligned },
            { "isDmisProgramActive", m_isDmisProgramActive }
        };
    }

    QJsonArray MainWindow::getSolvedFeaturesJson() {
        QJsonArray arr;
        for (const auto& mf : m_solvedFeatures) {
            arr.append(QJsonObject{
                { "id", mf.id },
                { "name", QString::fromStdString(mf.name) },
                { "diameterMm", mf.diameter },
                { "formErrorMm", mf.formError },
                { "locationMm", QJsonArray{ mf.location.x(), mf.location.y(), mf.location.z() } }
                });
        }
        return arr;
    }

    QJsonArray MainWindow::getPlannedDmisFeaturesJson() {
        QJsonArray arr;
        for (size_t i = 0; i < m_dmisProgram.features.size(); ++i) {
            const auto& df = m_dmisProgram.features[i];
            arr.append(QJsonObject{
                { "index", static_cast<int>(i) },
                { "label", QString::fromStdString(df.label) },
                { "featureType", QString::fromStdString(df.featureType) },
                { "boundarySub", QString::fromStdString(df.boundarySub) },
                { "nominalDiameterMm", df.nominalDiameter },
                { "pointCount", static_cast<int>(df.points.size()) }
                });
        }
        return arr;
    }

    QJsonObject MainWindow::runPreFlightCheckJson() {
        int collisionErrors = 0;
        for (size_t i = 0; i < m_dmisProgram.features.size(); ++i) {
            if (!validateDmisFeaturePath(static_cast<int>(i))) {
                collisionErrors++;
            }
        }
        return QJsonObject{
            { "checkedFeatures", static_cast<int>(m_dmisProgram.features.size()) },
            { "collisionErrors", collisionErrors },
            { "passed", (collisionErrors == 0) }
        };
    }

    QJsonObject MainWindow::createOfflineDmisFeatureJson(const QJsonObject& params) {
        using DmisFeatureT = typename decltype(m_dmisProgram.features)::value_type;
        using DmisPointT = typename decltype(DmisFeatureT::points)::value_type;

        const int nextNum = static_cast<int>(m_dmisProgram.features.size() + m_solvedFeatures.size()) + 1;
        const int faceIndex = params.contains("faceIndex") ? params.value("faceIndex").toInt(-1) : -1;

        DmisFeatureT df;
        df.label = params.value("label").toString(QString("MCP_FEAT_%1").arg(nextNum)).toStdString();
        df.refId = QString("10000000-0000-4000-8000-%1").arg(nextNum, 12, 10, QChar('0')).toStdString();
        df.featureType = params.value("featureType").toString("CIRCLE").toUpper().toStdString();
        df.boundarySub = params.value("boundarySub").toString("INNER").toUpper().toStdString();
        df.datumLetter = params.value("datumLetter").toString("").trimmed().toUpper().toStdString();
        if (!df.datumLetter.empty()) {
            m_dmisProgram.datumAlignment.has321Alignment = true;
        }

        df.nomX = params.value("nomX").toDouble(0.0);
        df.nomY = params.value("nomY").toDouble(0.0);
        df.nomZ = params.value("nomZ").toDouble(0.0);
        df.nomI = params.value("nomI").toDouble(0.0);
        df.nomJ = params.value("nomJ").toDouble(0.0);
        df.nomK = params.value("nomK").toDouble(1.0);
        df.nominalDiameter = params.value("nominalDiameter").toDouble(10.0);

        bool sampledFromCadUv = false;
        TopoDS_Face targetFace;

        // 1. If faceIndex is supplied, locate the exact TopoDS_Face on the loaded STEP model
        if (faceIndex >= 0 && m_viewport && m_viewport->hasCadModel()) {
            const TopoDS_Shape partShape = m_viewport->getCombinedCadShape();
            int curIdx = 0;
            for (TopExp_Explorer exp(partShape, TopAbs_FACE); exp.More(); exp.Next(), ++curIdx) {
                if (curIdx == faceIndex) {
                    targetFace = TopoDS::Face(exp.Current());
                    break;
                }
            }

            if (targetFace.IsNull()) {
                return QJsonObject{
                    { "success", false },
                    { "error", QString("faceIndex %1 is out of range (model has %2 faces).").arg(faceIndex).arg(curIdx) }
                };
            }

            BRepAdaptor_Surface surf(targetFace, true);
            const GeomAbs_SurfaceType geomType = surf.GetType();
            const bool isReversed = (targetFace.Orientation() == TopAbs_REVERSED);
            const double orientSign = isReversed ? -1.0 : 1.0;

            GProp_GProps props;
            BRepGProp::SurfaceProperties(targetFace, props);
            const gp_Pnt centroid = props.CentreOfMass();

            // Auto-populate featureType, boundarySub, and nominals from B-Rep geometry unless explicitly overridden
            if (geomType == GeomAbs_Plane) {
                if (!params.contains("featureType")) df.featureType = "PLANE";
                if (!params.contains("boundarySub")) df.boundarySub = "UNBND";
                const gp_Dir n = surf.Plane().Axis().Direction();
                if (!params.contains("nomX")) df.nomX = centroid.X();
                if (!params.contains("nomY")) df.nomY = centroid.Y();
                if (!params.contains("nomZ")) df.nomZ = centroid.Z();
                if (!params.contains("nomI")) df.nomI = n.X() * orientSign;
                if (!params.contains("nomJ")) df.nomJ = n.Y() * orientSign;
                if (!params.contains("nomK")) df.nomK = n.Z() * orientSign;
            }
            else if (geomType == GeomAbs_Cylinder) {
                if (!params.contains("featureType")) df.featureType = "CYLNDR";
                if (!params.contains("boundarySub")) df.boundarySub = isReversed ? "INNER" : "OUTER";
                const gp_Cylinder cyl = surf.Cylinder();
                const gp_Dir axis = cyl.Axis().Direction();
                const gp_Pnt loc = cyl.Location();
                const gp_Vec toCentroid(loc, centroid);
                const gp_Pnt axisCenter = loc.Translated(gp_Vec(axis).Multiplied(toCentroid.Dot(gp_Vec(axis))));

                if (!params.contains("nomX")) df.nomX = axisCenter.X();
                if (!params.contains("nomY")) df.nomY = axisCenter.Y();
                if (!params.contains("nomZ")) df.nomZ = axisCenter.Z();
                if (!params.contains("nomI")) df.nomI = axis.X();
                if (!params.contains("nomJ")) df.nomJ = axis.Y();
                if (!params.contains("nomK")) df.nomK = axis.Z();
                if (!params.contains("nominalDiameter")) df.nominalDiameter = cyl.Radius() * 2.0;
            }
            else if (geomType == GeomAbs_Sphere) {
                if (!params.contains("featureType")) df.featureType = "SPHERE";
                if (!params.contains("boundarySub")) df.boundarySub = isReversed ? "INNER" : "OUTER";
                const gp_Sphere sph = surf.Sphere();
                const gp_Pnt center = sph.Location();
                if (!params.contains("nomX")) df.nomX = center.X();
                if (!params.contains("nomY")) df.nomY = center.Y();
                if (!params.contains("nomZ")) df.nomZ = center.Z();
                if (!params.contains("nominalDiameter")) df.nominalDiameter = sph.Radius() * 2.0;
            }
        }

        const int defaultHits = (df.featureType == "CYLNDR") ? 6 : 4;
        const int hitCount = std::max(3, params.value("hitCount").toInt(defaultHits));

        // 2. Sample valid interior UV coordinates on targetFace using BRepTopAdaptor_FClass2d
        if (!targetFace.IsNull()) {
            double uMin = 0.0, uMax = 0.0, vMin = 0.0, vMax = 0.0;
            BRepTools::UVBounds(targetFace, uMin, uMax, vMin, vMax);

            const double uSpan = uMax - uMin;
            const double vSpan = vMax - vMin;

            if (uSpan > 1e-6 && vSpan > 1e-6) {
                BRepAdaptor_Surface surf(targetFace, true);
                BRepTopAdaptor_FClass2d classifier(targetFace, Precision::PConfusion());
                const bool isReversed = (targetFace.Orientation() == TopAbs_REVERSED);
                const GeomAbs_SurfaceType geomType = surf.GetType();

                auto tryAddUvHit = [&](double u, double v) -> bool {
                    if (classifier.Perform(gp_Pnt2d(u, v)) != TopAbs_IN) {
                        return false;
                    }
                    BRepLProp_SLProps slProps(surf, u, v, 1, 1e-6);
                    if (!slProps.IsNormalDefined()) {
                        return false;
                    }
                    const gp_Pnt pt = slProps.Value();
                    gp_Dir norm = slProps.Normal();
                    if (isReversed) {
                        norm.Reverse();
                    }

                    // Keep at least 1.5 mm separation between sampled points
                    for (const auto& existing : df.points) {
                        const double dist = std::hypot(pt.X() - existing.x, std::hypot(pt.Y() - existing.y, pt.Z() - existing.z));
                        if (dist < 1.5) {
                            return false;
                        }
                    }

                    DmisPointT tp{};
                    tp.x = pt.X(); tp.y = pt.Y(); tp.z = pt.Z();
                    tp.i = norm.X(); tp.j = norm.Y(); tp.k = norm.Z();
                    df.points.push_back(tp);
                    return true;
                    };

                // Structured multi-depth rings for cylinders/circles
                if (geomType == GeomAbs_Cylinder) {
                    const int rings = (df.featureType == "CYLNDR") ? 2 : 1;
                    const int ptsPerRing = std::max(3, (hitCount + rings - 1) / rings);

                    for (int r = 0; r < rings && static_cast<int>(df.points.size()) < hitCount; ++r) {
                        const double vFrac = (rings == 1) ? 0.5 : (0.28 + 0.44 * r);
                        const double vBase = vMin + vFrac * vSpan;
                        for (int i = 0; i < ptsPerRing && static_cast<int>(df.points.size()) < hitCount; ++i) {
                            const double uFrac = (i + 0.5) / static_cast<double>(ptsPerRing);
                            const double uBase = uMin + uFrac * uSpan;
                            if (!tryAddUvHit(uBase, vBase)) {
                                // Nudge along U/V if a cross-hole or pocket sits at (uBase, vBase)
                                for (int nudge = 1; nudge <= 5; ++nudge) {
                                    const double du = 0.04 * nudge * uSpan;
                                    if (tryAddUvHit(uBase + du, vBase) || tryAddUvHit(uBase - du, vBase)) {
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                // Low-discrepancy Halton-style interior UV sweep for planes, spheres, or remaining hits
                const double uMargin = 0.12 * uSpan;
                const double vMargin = 0.12 * vSpan;
                const double uInner = uSpan - 2.0 * uMargin;
                const double vInner = vSpan - 2.0 * vMargin;

                for (int attempt = 0; attempt < 160 && static_cast<int>(df.points.size()) < hitCount; ++attempt) {
                    // Plastic constant R2 quasi-random sequence for even 2D surface coverage
                    const double fu = std::fmod(0.5 + attempt * 0.7548776662466927, 1.0);
                    const double fv = std::fmod(0.5 + attempt * 0.5698402909980532, 1.0);
                    const double u = uMin + uMargin + fu * uInner;
                    const double v = vMin + vMargin + fv * vInner;
                    tryAddUvHit(u, v);
                }

                sampledFromCadUv = !df.points.empty();
            }
        }

        // 3. Fallback to parametric ring generation if no faceIndex was given or UV sampling yielded 0 points
        Eigen::Vector3d n(df.nomI, df.nomJ, df.nomK);
        if (n.norm() < 1e-6) n = Eigen::Vector3d(0.0, 0.0, 1.0);
        n.normalize();
        df.nomI = n.x(); df.nomJ = n.y(); df.nomK = n.z();

        if (df.points.empty()) {
            const double radius = std::max(0.5, df.nominalDiameter * 0.5);
            Eigen::Vector3d ref = (std::abs(n.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
            Eigen::Vector3d u = n.cross(ref).normalized();
            Eigen::Vector3d v = n.cross(u).normalized();
            Eigen::Vector3d center(df.nomX, df.nomY, df.nomZ);

            if (df.featureType == "CYLNDR") {
                const int rings = 2;
                const int ptsPerRing = std::max(3, (hitCount + 1) / 2);
                for (int r = 0; r < rings; ++r) {
                    const double zOff = (r == 0) ? -2.0 : 2.0;
                    const Eigen::Vector3d ringCenter = center + zOff * n;
                    for (int i = 0; i < ptsPerRing; ++i) {
                        const double angle = (2.0 * 3.14159265358979323846 * i) / ptsPerRing;
                        const Eigen::Vector3d radial = std::cos(angle) * u + std::sin(angle) * v;
                        const Eigen::Vector3d pos = ringCenter + radius * radial;
                        const Eigen::Vector3d surfNorm = (df.boundarySub == "OUTER") ? radial : -radial;
                        DmisPointT pt{};
                        pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
                        pt.i = surfNorm.x(); pt.j = surfNorm.y(); pt.k = surfNorm.z();
                        df.points.push_back(pt);
                    }
                }
            }
            else {
                for (int i = 0; i < hitCount; ++i) {
                    const double angle = (2.0 * 3.14159265358979323846 * i) / hitCount;
                    const Eigen::Vector3d radial = std::cos(angle) * u + std::sin(angle) * v;

                    DmisPointT pt{};
                    if (df.featureType == "CIRCLE" || df.featureType == "SPHERE") {
                        const Eigen::Vector3d pos = center + radius * radial;
                        const Eigen::Vector3d surfNorm = (df.boundarySub == "OUTER") ? radial : -radial;
                        pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
                        pt.i = surfNorm.x(); pt.j = surfNorm.y(); pt.k = surfNorm.z();
                    }
                    else {
                        const Eigen::Vector3d pos = center + (radius * 0.5) * radial;
                        pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
                        pt.i = n.x(); pt.j = n.y(); pt.k = n.z();
                    }
                    df.points.push_back(pt);
                }
            }
        }

        m_dmisProgram.valid = true;
        m_dmisProgram.stylusDiameterMm = m_effectiveProbeRadius * 2.0;
        m_dmisProgram.features.push_back(df);

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";
        const QString datSuffix = df.datumLetter.empty()
            ? QString()
            : QString(" [DAT %1]").arg(QString::fromStdString(df.datumLetter));
        const QString featTitle = QString("%1 (%2)%3")
            .arg(QString::fromStdString(df.label), QString::fromStdString(df.featureType), datSuffix);
        const double nomVal = (df.featureType == "CIRCLE" || df.featureType == "CYLNDR" || df.featureType == "SPHERE")
            ? df.nominalDiameter : 0.0;

        const double plusTol = params.value("plusToleranceMm").toDouble(m_defaultTolMm);
        const double minusTol = params.value("minusToleranceMm").toDouble(m_defaultTolMm);

        auto* item = new QTreeWidgetItem(m_featureTree, {
            featTitle,
            QString("%1%2").arg(toDisplayUnit(nomVal, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "--",
            QString("±%1").arg(toDisplayUnit(plusTol, m_unitSystem), 0, 'f', decimals),
            QString("PLANNED (0/%1 pts)").arg(df.points.size())
            });
        item->setForeground(4, QBrush(QColor("#e5c07b")));
        item->setData(0, ROLE_PLUS_TOL_MM, plusTol);
        item->setData(0, ROLE_MINUS_TOL_MM, minusTol);
        m_plannedDmisTreeItems.push_back(item);

        const int newIdx = static_cast<int>(m_dmisProgram.features.size()) - 1;
        if (!m_isDmisProgramActive || m_activeDmisFeatureIdx < 0) {
            m_isDmisProgramActive = true;
            startDmisGuidedFeature(newIdx);
        }

        const bool pathSafe = validateDmisFeaturePath(newIdx);

        if (m_autoRunDmisBtnHome) m_autoRunDmisBtnHome->setEnabled(true);
        if (m_autoRunDmisBtnCad)  m_autoRunDmisBtnCad->setEnabled(true);
        statusBar()->showMessage(
            QString("MCP queued feature '%1' (%2 pts, %3) — Pre-Flight: %4")
            .arg(QString::fromStdString(df.label))
            .arg(df.points.size())
            .arg(sampledFromCadUv ? QString("CAD Face #%1 UV").arg(faceIndex) : "Parametric")
            .arg(pathSafe ? "SAFE" : "COLLISION WARNING")
        );

        return QJsonObject{
            { "success", true },
            { "featureIndex", newIdx },
            { "faceIndex", faceIndex },
            { "sampledFromCadUv", sampledFromCadUv },
            { "featureType", QString::fromStdString(df.featureType) },
            { "boundarySub", QString::fromStdString(df.boundarySub) },
            { "datumLetter", QString::fromStdString(df.datumLetter) },
            { "nominalDiameter", df.nominalDiameter },
            { "pointCount", static_cast<int>(df.points.size()) },
            { "preFlightPathSafe", pathSafe }
        };
    }

    void MainWindow::onToggleMcpServerClicked() {
        if (!m_mcpPlugin) return;

        if (m_mcpPlugin->isRunning()) {
            m_mcpPlugin->stopServer();
            logCollisionEvent("MCP Server stopped by user (Local AI endpoint closed).", false, false);
            statusBar()->showMessage("MCP Server stopped — local AI endpoint closed.");
        }
        else {
            if (m_mcpPlugin->startServer(this, 8765)) {
                logCollisionEvent(
                    QString("MCP Server started on 127.0.0.1:%1").arg(m_mcpPlugin->serverPort()).toStdString(),
                    false, false
                );
                statusBar()->showMessage(QString("MCP Server listening on 127.0.0.1:%1").arg(m_mcpPlugin->serverPort()));
            }
            else {
                QMessageBox::warning(this, "MCP Server Error",
                    "Could not bind MCP Server to 127.0.0.1:8765. Verify that the port is not in use.");
            }
        }
        updateMcpButtonStyle();
    }

    void MainWindow::updateMcpButtonStyle() {
        if (!m_mcpStatusBtn || !m_mcpPlugin) return;

        const bool running = m_mcpPlugin->isRunning();
        m_mcpStatusBtn->setChecked(running);
        m_mcpStatusBtn->setText(running
            ? QString("🤖\nMCP :%1").arg(m_mcpPlugin->serverPort())
            : "🤖\nMCP: OFF");
        m_mcpStatusBtn->setToolTip(running
            ? QString("MCP Server active on 127.0.0.1:%1 — Click to disable external AI connection").arg(m_mcpPlugin->serverPort())
            : "MCP Server stopped — Click to start listening on 127.0.0.1:8765");

        if (running) {
            m_mcpStatusBtn->setStyleSheet(
                "QPushButton { background-color: #1e4620; color: #98c379; border: 1px solid #98c379; "
                "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
                "QPushButton:hover { background-color: #276749; color: #ffffff; border: 1px solid #98c379; }"
            );
        }
        else {
            m_mcpStatusBtn->setStyleSheet(
                "QPushButton { background-color: #282c34; color: #e5c07b; border: 1px solid #e5c07b; "
                "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
                "QPushButton:hover { background-color: #3e4451; color: #ffffff; border: 1px solid #e5c07b; }"
            );
        }
    }

    QJsonObject MainWindow::getCadFacesJson(const QJsonObject& filter) {
        if (!m_viewport || !m_viewport->hasCadModel()) {
            return QJsonObject{
                { "success", false },
                { "error", "No STEP CAD part model is currently loaded." },
                { "faces", QJsonArray{} }
            };
        }

        const TopoDS_Shape partShape = m_viewport->getCombinedCadShape();
        if (partShape.IsNull()) {
            return QJsonObject{
                { "success", false },
                { "error", "Combined CAD shape is empty." },
                { "faces", QJsonArray{} }
            };
        }

        const QString typeFilter = filter.value("surfaceType").toString("ALL").trimmed().toUpper();
        const int maxFaces = std::max(1, filter.value("maxFaces").toInt(100));

        QJsonArray facesArray;
        int faceIndex = 0;
        int matchedCount = 0;

        for (TopExp_Explorer exp(partShape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIndex) {
            const TopoDS_Face face = TopoDS::Face(exp.Current());
            BRepAdaptor_Surface surf(face, true);
            const GeomAbs_SurfaceType geomType = surf.GetType();

            QString dmisType = "OTHER";
            if (geomType == GeomAbs_Plane)         dmisType = "PLANE";
            else if (geomType == GeomAbs_Cylinder) dmisType = "CYLNDR";
            else if (geomType == GeomAbs_Sphere)   dmisType = "SPHERE";
            else if (geomType == GeomAbs_Cone)     dmisType = "CONE";

            if (typeFilter != "ALL" && !typeFilter.isEmpty() && dmisType != typeFilter) {
                continue;
            }

            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            const gp_Pnt centroid = props.CentreOfMass();
            const double areaMm2 = props.Mass();

            const bool isReversed = (face.Orientation() == TopAbs_REVERSED);
            const double orientSign = isReversed ? -1.0 : 1.0;

            QJsonObject faceObj;
            faceObj["faceIndex"] = faceIndex;
            faceObj["surfaceType"] = dmisType;
            faceObj["areaMm2"] = std::round(areaMm2 * 100.0) / 100.0;
            faceObj["centroid"] = QJsonObject{
                { "x", std::round(centroid.X() * 10000.0) / 10000.0 },
                { "y", std::round(centroid.Y() * 10000.0) / 10000.0 },
                { "z", std::round(centroid.Z() * 10000.0) / 10000.0 }
            };

            if (geomType == GeomAbs_Plane) {
                const gp_Pln pln = surf.Plane();
                const gp_Dir n = pln.Axis().Direction();
                faceObj["boundarySub"] = "UNBND";
                faceObj["normal"] = QJsonObject{
                    { "i", std::round(n.X() * orientSign * 10000.0) / 10000.0 },
                    { "j", std::round(n.Y() * orientSign * 10000.0) / 10000.0 },
                    { "k", std::round(n.Z() * orientSign * 10000.0) / 10000.0 }
                };
            }
            else if (geomType == GeomAbs_Cylinder) {
                const gp_Cylinder cyl = surf.Cylinder();
                const gp_Dir axis = cyl.Axis().Direction();
                const gp_Pnt loc = cyl.Location();

                const gp_Vec toCentroid(loc, centroid);
                const double axialProj = toCentroid.Dot(gp_Vec(axis));
                const gp_Pnt axisCenter = loc.Translated(gp_Vec(axis).Multiplied(axialProj));

                faceObj["boundarySub"] = isReversed ? "INNER" : "OUTER";
                faceObj["nominalDiameter"] = std::round(cyl.Radius() * 2.0 * 10000.0) / 10000.0;
                faceObj["axisCenter"] = QJsonObject{
                    { "x", std::round(axisCenter.X() * 10000.0) / 10000.0 },
                    { "y", std::round(axisCenter.Y() * 10000.0) / 10000.0 },
                    { "z", std::round(axisCenter.Z() * 10000.0) / 10000.0 }
                };
                faceObj["axis"] = QJsonObject{
                    { "i", std::round(axis.X() * 10000.0) / 10000.0 },
                    { "j", std::round(axis.Y() * 10000.0) / 10000.0 },
                    { "k", std::round(axis.Z() * 10000.0) / 10000.0 }
                };
            }
            else if (geomType == GeomAbs_Sphere) {
                const gp_Sphere sph = surf.Sphere();
                const gp_Pnt center = sph.Location();
                faceObj["boundarySub"] = isReversed ? "INNER" : "OUTER";
                faceObj["nominalDiameter"] = std::round(sph.Radius() * 2.0 * 10000.0) / 10000.0;
                faceObj["sphereCenter"] = QJsonObject{
                    { "x", std::round(center.X() * 10000.0) / 10000.0 },
                    { "y", std::round(center.Y() * 10000.0) / 10000.0 },
                    { "z", std::round(center.Z() * 10000.0) / 10000.0 }
                };
            }
            else if (geomType == GeomAbs_Cone) {
                const gp_Cone cone = surf.Cone();
                const gp_Dir axis = cone.Axis().Direction();
                faceObj["boundarySub"] = isReversed ? "INNER" : "OUTER";
                faceObj["semiAngleDeg"] = std::round((cone.SemiAngle() * 180.0 / 3.14159265358979323846) * 1000.0) / 1000.0;
                faceObj["axis"] = QJsonObject{
                    { "i", std::round(axis.X() * 10000.0) / 10000.0 },
                    { "j", std::round(axis.Y() * 10000.0) / 10000.0 },
                    { "k", std::round(axis.Z() * 10000.0) / 10000.0 }
                };
            }

            facesArray.append(faceObj);
            if (++matchedCount >= maxFaces) {
                break;
            }
        }

        return QJsonObject{
            { "success", true },
            { "totalFacesScanned", faceIndex },
            { "returnedFacesCount", matchedCount },
            { "faces", facesArray }
        };
    }

    QJsonObject MainWindow::runDmisPlaybackJson(const QJsonObject& params) {
        const QString action = params.value("action").toString("start").trimmed().toLower();

        if (m_dmisProgram.features.empty()) {
            return QJsonObject{
                { "success", false },
                { "error", "No DMIS features are loaded or queued for playback." }
            };
        }

        if (action == "stop") {
            stopDmisAutoRun();
            statusBar()->showMessage("MCP stopped DMIS simulation playback.");
            return QJsonObject{
                { "success", true },
                { "state", "stopped" }
            };
        }
        else if (action == "reset") {
            stopDmisAutoRun();
            finishFeatureBuffer();
            m_isDmisProgramActive = true;
            startDmisGuidedFeature(0);
            statusBar()->showMessage("MCP reset DMIS simulation playback to Feature #1.");
            return QJsonObject{
                { "success", true },
                { "state", "reset" }
            };
        }

        const QJsonObject preFlight = runPreFlightCheckJson();
        if (!m_isDmisProgramActive || m_activeDmisFeatureIdx < 0 ||
            m_activeDmisFeatureIdx >= static_cast<int>(m_dmisProgram.features.size())) {
            m_isDmisProgramActive = true;
            startDmisGuidedFeature(0);
        }

        if (!m_dmisPlaybackTimer || !m_dmisPlaybackTimer->isActive()) {
            onToggleAutoRunDmisClicked();
        }

        return QJsonObject{
            { "success", true },
            { "state", "running" },
            { "featureCount", static_cast<int>(m_dmisProgram.features.size()) },
            { "preFlightSummary", preFlight }
        };
    }
    QJsonObject MainWindow::exportInspectionFileJson(const QJsonObject& params) {
        const QString fmt = params.value("format").toString("dmis").trimmed().toLower();
        QString partName = params.value("partName").toString("JLS-PART-001").trimmed();
        if (partName.isEmpty()) partName = "JLS-PART-001";

        if (m_solvedFeatures.empty() && m_dmisProgram.features.empty()) {
            return QJsonObject{
                { "success", false },
                { "error", "No solved or planned features exist to export." }
            };
        }

        QString ext = ".dmis";
        if (fmt == "pdf") ext = ".pdf";
        else if (fmt == "qif") ext = ".qif";
        else if (fmt == "csv") ext = ".csv";
        else if (fmt == "html") ext = ".html";
        else if (fmt != "dmis") {
            return QJsonObject{
                { "success", false },
                { "error", "Unsupported format '" + fmt + "'. Use dmis, qif, pdf, csv, or html." }
            };
        }

        QString filePath = params.value("filePath").toString().trimmed();
        if (filePath.isEmpty()) {
            QDir exportDir(QCoreApplication::applicationDirPath());
            if (!exportDir.exists("exports")) {
                exportDir.mkpath("exports");
            }
            exportDir.cd("exports");
            filePath = exportDir.filePath(partName + ext);
        }
        else if (!filePath.endsWith(ext, Qt::CaseInsensitive)) {
            filePath += ext;
        }

        const QFileInfo fi(filePath);
        if (fi.dir().exists() == false) {
            QDir().mkpath(fi.dir().absolutePath());
        }
        const QString absPath = fi.absoluteFilePath();

        // 1. DMIS 5.3 (.dmis)
        if (fmt == "dmis") {
            if (!writeDmisProgramToFile(absPath, partName)) {
                return QJsonObject{ { "success", false },{ "error", "Failed to write DMIS file to: " + absPath } };
            }
        }
        // 2. QIF 2.1 (.qif)
        else if (fmt == "qif") {
            std::vector<InspectionResultItem> qifItems;
            for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                if (top == m_activePointsGroup) continue;

                const int featId = top->data(0, ROLE_FEATURE_ID).isValid() ? top->data(0, ROLE_FEATURE_ID).toInt() : i;
                InspectionResultItem item;
                item.featureName = top->text(0);
                item.refId = (m_featureRefIds.count(featId) && !m_featureRefIds[featId].isEmpty())
                    ? m_featureRefIds[featId]
                    : ("REFID_" + QString::number(featId));
                item.featureType = (top->text(0).contains("Circle") || top->text(0).contains("Bore") || top->text(0).contains("Cylinder") || top->text(0).contains("CYLNDR")) ? "Bore"
                    : (top->text(0).contains("Plane") || top->text(0).contains("PLANE") ? "Plane" : "Feature");
                item.nominalMm = top->data(0, ROLE_NOMINAL_MM).isValid() ? top->data(0, ROLE_NOMINAL_MM).toDouble() : 0.0;
                item.actualMm = top->data(0, ROLE_ACTUAL_MM).isValid() ? top->data(0, ROLE_ACTUAL_MM).toDouble() : 0.0;
                item.plusTolMm = top->data(0, ROLE_PLUS_TOL_MM).isValid() ? top->data(0, ROLE_PLUS_TOL_MM).toDouble() : m_defaultTolMm;
                item.minusTolMm = top->data(0, ROLE_MINUS_TOL_MM).isValid() ? top->data(0, ROLE_MINUS_TOL_MM).toDouble() : m_defaultTolMm;
                item.passed = !top->text(4).contains("FAIL");
                qifItems.push_back(item);
            }

            if (!QifExporter::exportToQif(absPath, partName, qifItems)) {
                return QJsonObject{ { "success", false },{ "error", "Failed to write QIF file to: " + absPath } };
            }
        }
        // --- 3. PDF Inspection Certificate Export (.pdf) ---
        if (fmt == "pdf") {
            const QString templateStyle = params.value("templateStyle").toString("standard").trimmed().toLower();
            const QString customerName = params.value("customerName").toString("AeroCorp Industries").trimmed();
            const QString poNumber = params.value("poNumber").toString("PO-48920").trimmed();
            const QString inspectorName = params.value("inspectorName").toString("Jason Sample (Quality Manager)").trimmed();
            const QString batchNumber = params.value("batchNumber").toString("LOT-2026-05").trimmed();

            QPdfWriter pdfWriter(absPath);
            pdfWriter.setPageSize(QPageSize(QPageSize::A4));
            pdfWriter.setPageOrientation(QPageLayout::Portrait);
            pdfWriter.setTitle("JLS Machine LLC — Inspection Certificate");

            QPainter painter(&pdfWriter);
            if (!painter.isActive()) {
                return QJsonObject{ { "success", false },{ "error", "Could not initialize PDF writer at: " + absPath } };
            }

            int yPos = 1000;
            const int leftMargin = 1000;
            const int pageWidth = 19000;

            // --- Header Banner ---
            painter.setPen(Qt::NoPen);
            painter.setBrush(templateStyle == "aerospace" ? QColor(15, 23, 42) : QColor(30, 41, 59));
            painter.drawRect(leftMargin, yPos, pageWidth, 1100);

            painter.setPen(Qt::white);
            painter.setFont(QFont("Segoe UI", 15, QFont::Bold));
            painter.drawText(leftMargin + 300, yPos + 380, "JLS MACHINE LLC — PRECISION CNC & METROLOGY");

            painter.setFont(QFont("Segoe UI", 9, QFont::Normal));
            painter.drawText(leftMargin + 300, yPos + 750, "Certified ISO 9001:2015 & AS9100 Quality Inspection Laboratory | (978) 228-3101");
            yPos += 1400;

            // --- Metadata Box ---
            painter.setPen(QColor(203, 213, 225));
            painter.setBrush(QColor(248, 250, 252));
            painter.drawRect(leftMargin, yPos, pageWidth, 1500);

            painter.setPen(QColor(15, 23, 42));
            painter.setFont(QFont("Segoe UI", 9, QFont::Bold));
            int col1 = leftMargin + 400;
            int col2 = leftMargin + 10000;

            painter.drawText(col1, yPos + 320, "Part / Job Number: " + partName);
            painter.drawText(col1, yPos + 680, "Customer: " + customerName);
            painter.drawText(col1, yPos + 1040, "PO Number: " + poNumber);

            painter.drawText(col2, yPos + 320, "Inspection Date: " + QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm"));
            painter.drawText(col2, yPos + 680, "Batch / Lot ID: " + batchNumber);
            painter.drawText(col2, yPos + 1040, "Inspector: " + inspectorName);
            yPos += 1900;

            // --- Table Header ---
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(51, 65, 85));
            painter.drawRect(leftMargin, yPos, pageWidth, 400);

            painter.setPen(Qt::white);
            painter.setFont(QFont("Segoe UI", 9, QFont::Bold));
            painter.drawText(leftMargin + 150, yPos + 270, "Feature / Parameter");
            painter.drawText(leftMargin + 7000, yPos + 270, "Nominal");
            painter.drawText(leftMargin + 10500, yPos + 270, "Actual");
            painter.drawText(leftMargin + 14000, yPos + 270, "±Tol");
            painter.drawText(leftMargin + 17000, yPos + 270, "Status");
            yPos += 500;

            // --- Table Rows ---
            painter.setFont(QFont("Segoe UI", 9, QFont::Normal));
            bool altRow = false;

            for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                if (top == m_activePointsGroup) continue;

                if (yPos > 24500) {
                    pdfWriter.newPage();
                    yPos = 1200;
                }

                if (altRow) {
                    painter.setPen(Qt::NoPen);
                    painter.setBrush(QColor(241, 245, 249));
                    painter.drawRect(leftMargin, yPos - 50, pageWidth, 350);
                }
                altRow = !altRow;

                const QString statusTxt = top->text(4);
                const QColor statusCol = statusTxt.contains("FAIL") ? QColor(185, 28, 28) : QColor(21, 128, 61);

                painter.setPen(QColor(15, 23, 42));
                painter.drawText(leftMargin + 150, yPos + 200, top->text(0));
                painter.drawText(leftMargin + 7000, yPos + 200, top->text(1));
                painter.drawText(leftMargin + 10500, yPos + 200, top->text(2));
                painter.drawText(leftMargin + 14000, yPos + 200, top->text(3));

                painter.setPen(statusCol);
                painter.setFont(QFont("Segoe UI", 9, QFont::Bold));
                painter.drawText(leftMargin + 17000, yPos + 200, statusTxt);
                painter.setFont(QFont("Segoe UI", 9, QFont::Normal));

                yPos += 350;

                for (int c = 0; c < top->childCount(); ++c) { // Note: top->childCount()
                    // Sub-parameters
                }
                yPos += 80;
            }

            // --- Quality Stamp Footer ---
            if (yPos > 23000) pdfWriter.newPage();
            yPos = std::max(yPos + 500, 23500);

            painter.setPen(QColor(203, 213, 225));
            painter.setBrush(QColor(255, 255, 255));
            painter.drawRect(leftMargin, yPos, pageWidth, 1600);

            painter.setPen(QColor(15, 23, 42));
            painter.setFont(QFont("Segoe UI", 9, QFont::Bold));
            painter.drawText(leftMargin + 400, yPos + 350, "CERTIFICATE OF CONFORMANCE & COMPLIANCE");

            painter.setFont(QFont("Segoe UI", 8, QFont::Normal));
            painter.drawText(leftMargin + 400, yPos + 750, "We hereby certify that the items detailed above were manufactured and inspected in accordance with customer specifications");
            painter.drawText(leftMargin + 400, yPos + 1100, "and are fully compliant with drawings, CAD models, and ISO 9001 metrology calibration standards.");

            painter.drawText(leftMargin + 13000, yPos + 1100, "Authorized Signature: ______________________");
            painter.end();
        }
        // 4. CSV (.csv) or HTML (.html)
        else {
            QFile file(absPath);
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                return QJsonObject{ { "success", false },{ "error", "Could not open file for writing: " + absPath } };
            }
            QTextStream out(&file);

            if (fmt == "csv") {
                out << "Feature / Parameter,Nominal,Actual,Tolerance,Deviation / Status\n";
                for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                    QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                    if (top == m_activePointsGroup) continue;
                    out << "\"" << top->text(0) << "\",\"" << top->text(1) << "\",\"" << top->text(2) << "\",\"" << top->text(3) << "\",\"" << top->text(4) << "\"\n";
                    for (int c = 0; c < top->childCount(); ++c) {
                        QTreeWidgetItem* child = top->child(c);
                        out << "  \"" << child->text(0) << "\",\"" << child->text(1) << "\",\"" << child->text(2) << "\",\"" << child->text(3) << "\",\"" << child->text(4) << "\"\n";
                    }
                }
            }
            else {
                const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
                out << "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Inspection Report - " << partName.toHtmlEscaped() << "</title></head><body>"
                    << "<h1>JLS Machine LLC — Dimensional Inspection Report (" << partName.toHtmlEscaped() << ")</h1>"
                    << "<p>Date: " << timestamp << " | Stylus R_eff: " << QString::number(m_effectiveProbeRadius, 'f', 4) << " mm</p>"
                    << "<table border='1' cellpadding='6' cellspacing='0'><thead><tr>"
                    << "<th>Feature</th><th>Nominal</th><th>Actual</th><th>±Tol</th><th>Status</th></tr></thead><tbody>";
                for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                    QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                    if (top == m_activePointsGroup) continue;
                    out << "<tr><td><b>" << top->text(0).toHtmlEscaped() << "</b></td><td>" << top->text(1).toHtmlEscaped()
                        << "</td><td>" << top->text(2).toHtmlEscaped() << "</td><td>" << top->text(3).toHtmlEscaped()
                        << "</td><td>" << top->text(4).toHtmlEscaped() << "</td></tr>";
                }
                out << "</tbody></table></body></html>";
            }
            file.close();
        }

        statusBar()->showMessage("MCP exported " + fmt.toUpper() + " file to: " + absPath);

        return QJsonObject{
            { "success", true },
            { "format", fmt },
            { "partName", partName },
            { "filePath", absPath },
            { "solvedFeatureCount", static_cast<int>(m_solvedFeatures.size()) },
            { "plannedFeatureCount", static_cast<int>(m_dmisProgram.features.size()) }
        };
    }
    QJsonObject MainWindow::solve321AlignmentJson(const QJsonObject& params) {
        if (m_solvedFeatures.empty()) {
            return QJsonObject{
                { "success", false },
                { "error", "No solved features exist to establish a 3-2-1 coordinate alignment." }
            };
        }

        // If explicit feature IDs/indices are provided, assign them as Datums A, B, and C
        if (params.contains("datumAFeatureId") && params.contains("datumBFeatureId") && params.contains("datumCFeatureId")) {
            const int idA = params.value("datumAFeatureId").toInt();
            const int idB = params.value("datumBFeatureId").toInt();
            const int idC = params.value("datumCFeatureId").toInt();

            assignFeatureAsDatum(idA, "A", false);
            assignFeatureAsDatum(idB, "B", false);
            assignFeatureAsDatum(idC, "C", false);
        }

        const bool solved = tryAutoSolve321Alignment(false);
        if (!solved) {
            return QJsonObject{
                { "success", false },
                { "error", "Could not solve 3-2-1 alignment. Ensure Datums A, B, and C are assigned and geometrically valid." }
            };
        }

        return QJsonObject{
            { "success", true },
            { "isPartAligned321", m_isPartAligned },
            { "datumAFeatureId", m_datumAFeatureId },
            { "datumBFeatureId", m_datumBFeatureId },
            { "datumCFeatureId", m_datumCFeatureId }
        };
    }
    void MainWindow::setupManualPromptBanner(QWidget* parentContainer) {
        m_manualPromptBanner = new QFrame(parentContainer);
        m_manualPromptBanner->setObjectName("ManualPromptBanner");
        m_manualPromptBanner->setStyleSheet(
            "QFrame#ManualPromptBanner { background-color: #161b22; border-bottom: 2px solid #eab308; padding: 6px 10px; }"
            "QPushButton { background-color: #21262d; color: #e6edf3; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 5px 10px; font-size: 11px; font-weight: bold; }"
            "QPushButton:hover { background-color: #30363d; border: 1px solid #58a6ff; }"
            "QPushButton:disabled { color: #484f58; border-color: #21262d; }"
        );

        auto* mainLay = new QVBoxLayout(m_manualPromptBanner);
        mainLay->setContentsMargins(6, 4, 6, 4);
        mainLay->setSpacing(4);

        // Top Row: Step Badge + Primary Operator Instruction + Control Buttons
        auto* topRow = new QHBoxLayout();
        topRow->setSpacing(10);

        m_promptStepBadge = new QLabel("STEP 1 / 1", m_manualPromptBanner);
        m_promptStepBadge->setStyleSheet(
            "background-color: #eab308; color: #0f172a; border-radius: 4px; "
            "padding: 4px 8px; font-size: 12px; font-weight: bold;"
        );
        topRow->addWidget(m_promptStepBadge, 0);

        m_promptInstructionLbl = new QLabel("Ready for Guided Manual Probing", m_manualPromptBanner);
        m_promptInstructionLbl->setStyleSheet("color: #ffffff; font-size: 16px; font-weight: bold;");
        topRow->addWidget(m_promptInstructionLbl, 1);

        QSettings settings("JLS Machine", "OpenCMM");
        m_autoCameraFocusEnabled = settings.value("guided_auto_camera", true).toBool();

        m_promptAutoCameraBtn = new QPushButton(
            m_autoCameraFocusEnabled ? "🎥 Auto-Zoom: ON" : "🎥 Auto-Zoom: OFF",
            m_manualPromptBanner
        );
        m_promptAutoCameraBtn->setCheckable(true);
        m_promptAutoCameraBtn->setChecked(m_autoCameraFocusEnabled);
        m_promptAutoCameraBtn->setToolTip("Automatically orbit and zoom the 3D camera to face the active probe target");
        auto updateAutoCamBtnStyle = [this]() {
            if (!m_promptAutoCameraBtn) return;
            m_promptAutoCameraBtn->setText(m_autoCameraFocusEnabled ? "🎥 Auto-Zoom: ON" : "🎥 Auto-Zoom: OFF");
            m_promptAutoCameraBtn->setStyleSheet(m_autoCameraFocusEnabled
                ? "QPushButton { background-color: #1e3a8a; color: #93c5fd; border: 1px solid #3b82f6; }"
                "QPushButton:hover { background-color: #1d4ed8; color: #ffffff; }"
                : "QPushButton { background-color: #21262d; color: #8b949e; border: 1px solid #30363d; }"
                "QPushButton:hover { background-color: #30363d; color: #ffffff; }");
            };
        updateAutoCamBtnStyle();

        connect(m_promptAutoCameraBtn, &QPushButton::clicked, this, [this, updateAutoCamBtnStyle](bool checked) {
            m_autoCameraFocusEnabled = checked;
            QSettings s("JLS Machine", "OpenCMM");
            s.setValue("guided_auto_camera", m_autoCameraFocusEnabled);
            updateAutoCamBtnStyle();
            if (m_autoCameraFocusEnabled) {
                focusViewportOnActiveTarget();
            }
            });
        topRow->addWidget(m_promptAutoCameraBtn);

        auto* simHitBtn = new QPushButton("⚡ Sim Hit (Space)", m_manualPromptBanner);
        simHitBtn->setStyleSheet(
            "QPushButton { background-color: #78350f; color: #fde68a; border: 1px solid #f59e0b; }"
            "QPushButton:hover { background-color: #92400e; color: #ffffff; }"
        );
        connect(simHitBtn, &QPushButton::clicked, this, &MainWindow::onSimulateHitClicked);
        topRow->addWidget(simHitBtn);

        m_promptUndoBtn = new QPushButton("↩ Undo Hit (Ctrl+Z)", m_manualPromptBanner);
        m_promptUndoBtn->setToolTip("Remove the last recorded probe hit if the stylus bumped or false-triggered");
        connect(m_promptUndoBtn, &QPushButton::clicked, this, &MainWindow::onUndoLastProbeHitClicked);
        topRow->addWidget(m_promptUndoBtn);

        m_promptSolveNowBtn = new QPushButton("✓ Solve Early", m_manualPromptBanner);
        m_promptSolveNowBtn->setToolTip("Fit and solve the current feature immediately using the hits collected so far");
        m_promptSolveNowBtn->setStyleSheet(
            "QPushButton { background-color: #14532d; color: #86efac; border: 1px solid #22c55e; }"
            "QPushButton:hover { background-color: #166534; color: #ffffff; }"
            "QPushButton:disabled { background-color: #161b22; color: #484f58; border-color: #21262d; }"
        );
        connect(m_promptSolveNowBtn, &QPushButton::clicked, this, &MainWindow::onForceSolveActiveDmisFeatureClicked);
        topRow->addWidget(m_promptSolveNowBtn);

        auto* prevFeatBtn = new QPushButton("◀ Prev", m_manualPromptBanner);
        connect(prevFeatBtn, &QPushButton::clicked, this, [this]() { onSkipDmisFeatureClicked(-1); });
        topRow->addWidget(prevFeatBtn);

        auto* nextFeatBtn = new QPushButton("Skip ▶", m_manualPromptBanner);
        connect(nextFeatBtn, &QPushButton::clicked, this, [this]() { onSkipDmisFeatureClicked(1); });
        topRow->addWidget(nextFeatBtn);

        auto* closeGuideBtn = new QPushButton("✕ Exit Guide", m_manualPromptBanner);
        closeGuideBtn->setStyleSheet(
            "QPushButton { background-color: #450a0a; color: #fca5a5; border: 1px solid #ef4444; }"
            "QPushButton:hover { background-color: #7f1d1d; color: #ffffff; }"
        );
        connect(closeGuideBtn, &QPushButton::clicked, this, [this]() {
            stopDmisAutoRun();
            m_isDmisProgramActive = false;
            m_activeDmisFeatureIdx = -1;
            if (m_viewport) m_viewport->clearProgramTargetMarkers();
            if (m_manualPromptBanner) m_manualPromptBanner->hide();
            statusBar()->showMessage("Exited Guided Manual Probing mode.");
            });
        topRow->addWidget(closeGuideBtn);

        mainLay->addLayout(topRow);

        // Bottom Row: Nominal / Surface Instruction + Live Delta XYZ Bullseye Readout
        auto* botRow = new QHBoxLayout();
        botRow->setSpacing(12);

        m_promptSubDetailLbl = new QLabel("", m_manualPromptBanner);
        m_promptSubDetailLbl->setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: bold;");
        botRow->addWidget(m_promptSubDetailLbl, 1);

        m_promptDeltaLbl = new QLabel("Move CMM probe toward Gold Bullseye Target...", m_manualPromptBanner);
        m_promptDeltaLbl->setStyleSheet(
            "background-color: #0d0f12; color: #61afef; border: 1px solid #30363d; border-radius: 4px; "
            "padding: 4px 10px; font-family: 'Consolas', monospace; font-size: 13px; font-weight: bold;"
        );
        botRow->addWidget(m_promptDeltaLbl, 0);

        mainLay->addLayout(botRow);
        m_manualPromptBanner->hide();
    }

    void MainWindow::updateManualPromptBanner() {
        if (!m_manualPromptBanner) return;

        if (!m_isDmisProgramActive || m_activeDmisFeatureIdx < 0 ||
            m_activeDmisFeatureIdx >= static_cast<int>(m_dmisProgram.features.size())) {
            m_manualPromptBanner->hide();
            return;
        }

        m_manualPromptBanner->show();

        const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
        const int totalFeats = static_cast<int>(m_dmisProgram.features.size());
        const int curHitIdx = static_cast<int>(m_recordedHits.size());
        const int totalHits = static_cast<int>(df.points.size());

        const QString datTag = df.datumLetter.empty()
            ? QString()
            : QString("  [DATUM %1]").arg(QString::fromStdString(df.datumLetter));

        m_promptStepBadge->setText(QString("FEATURE %1 OF %2%3")
            .arg(m_activeDmisFeatureIdx + 1)
            .arg(totalFeats)
            .arg(datTag));

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 3 : 4;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? "mm" : "in";

        QString actionVerb = "Probe Surface";
        int minRequiredHits = 3;
        if (df.featureType == "PLANE") {
            actionVerb = "Probe Flat Plane";
            minRequiredHits = 3;
        }
        else if (df.featureType == "CYLNDR" || df.featureType == "CYL") {
            actionVerb = (df.boundarySub == "OUTER") ? "Probe External Cylinder Boss" : "Probe Internal Bore Cylinder";
            minRequiredHits = 5;
        }
        else if (df.featureType == "CIRCLE") {
            actionVerb = (df.boundarySub == "OUTER") ? "Probe External Boss Circle" : "Probe Internal Bore Circle";
            minRequiredHits = 3;
        }
        else if (df.featureType == "SPHERE") {
            actionVerb = (df.boundarySub == "INNER") ? "Probe Spherical Cavity" : "Probe External Sphere";
            minRequiredHits = 4;
        }
        else if (df.featureType == "LINE") {
            actionVerb = "Probe Straight Edge Line";
            minRequiredHits = 2;
        }

        const QString diamInfo = (df.nominalDiameter > 0.0 &&
            (df.featureType == "CYLNDR" || df.featureType == "CYL" || df.featureType == "CIRCLE" || df.featureType == "SPHERE"))
            ? QString(" (Nom Ø%1 %2)").arg(toDisplayUnit(df.nominalDiameter, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            : QString();

        m_promptInstructionLbl->setText(QString("%1: %2%3  —  Touch Point #%4 of %5")
            .arg(actionVerb)
            .arg(QString::fromStdString(df.label))
            .arg(diamInfo)
            .arg(std::min(curHitIdx + 1, std::max(1, totalHits)))
            .arg(totalHits));

        if (curHitIdx < totalHits) {
            const auto& tgt = df.points[curHitIdx];
            const QString compStr = (currentCompMode() == ProbeCompMode::Internal)
                ? "Internal (+R)"
                : (currentCompMode() == ProbeCompMode::External ? "External (-R)" : "Uncompensated");

            m_promptSubDetailLbl->setText(QString(
                "Target #%1 Nominals: (X: %2, Y: %3, Z: %4 %5)   •   Surface Normal: (%6, %7, %8)   •   Tip Comp: %9"
            )
                .arg(curHitIdx + 1)
                .arg(toDisplayUnit(tgt.x, m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(tgt.y, m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(tgt.z, m_unitSystem), 0, 'f', decimals)
                .arg(unitStr)
                .arg(tgt.i, 0, 'f', 3)
                .arg(tgt.j, 0, 'f', 3)
                .arg(tgt.k, 0, 'f', 3)
                .arg(compStr));
        }

        if (m_promptUndoBtn) {
            m_promptUndoBtn->setEnabled(curHitIdx > 0);
        }
        if (m_promptSolveNowBtn) {
            m_promptSolveNowBtn->setEnabled(curHitIdx >= minRequiredHits);
        }
    }

    void MainWindow::onStartManualGuidedInspectionClicked() {
        stopDmisAutoRun();

        using DmisFeatureT = typename decltype(m_dmisProgram.features)::value_type;
        using DmisPointT = typename decltype(DmisFeatureT::points)::value_type;

        // 1. If the user created features on the CAD model (Auto-Probe CAD / solved features)
        //    and hasn't built a separate DMIS queue yet, convert solved features into the guided program!
        if (m_dmisProgram.features.empty() && !m_solvedFeatures.empty()) {
            m_dmisProgram.valid = true;
            m_dmisProgram.stylusDiameterMm = m_effectiveProbeRadius * 2.0;

            for (const auto& mf : m_solvedFeatures) {
                DmisFeatureT df;
                QString cleanName = QString::fromStdString(mf.name);
                cleanName.remove(" [DAT A]");
                cleanName.remove(" [DAT B]");
                cleanName.remove(" [DAT C]");
                df.label = cleanName.trimmed().toStdString();
                df.refId = m_featureRefIds.count(mf.id)
                    ? m_featureRefIds[mf.id].toStdString()
                    : QString("10000000-0000-4000-8000-%1").arg(mf.id, 12, 10, QChar('0')).toStdString();

                if (m_featureDatumLetters.count(mf.id)) {
                    df.datumLetter = m_featureDatumLetters[mf.id].toStdString();
                    m_dmisProgram.datumAlignment.has321Alignment = true;
                }

                if (mf.kind == FeatureKind::Plane3D) {
                    df.featureType = "PLANE";
                    df.boundarySub = "UNBND";
                }
                else if (mf.kind == FeatureKind::Sphere3D) {
                    df.featureType = "SPHERE";
                    df.boundarySub = "OUTER";
                }
                else if (mf.kind == FeatureKind::Line2D) {
                    df.featureType = "LINE";
                    df.boundarySub = "UNBND";
                }
                else {
                    const auto& recordedPts = m_featureDmisHits[mf.id];
                    df.featureType = (recordedPts.size() >= 5) ? "CYLNDR" : "CIRCLE";
                    df.boundarySub = "INNER";
                }

                df.nomX = mf.location.x();
                df.nomY = mf.location.y();
                df.nomZ = mf.location.z();
                df.nomI = mf.primaryVector.x();
                df.nomJ = mf.primaryVector.y();
                df.nomK = mf.primaryVector.z();
                df.nominalDiameter = mf.diameter;

                const auto& hits = m_featureDmisHits[mf.id];
                for (const auto& h : hits) {
                    DmisPointT pt{};
                    pt.x = h.surfacePt.x();
                    pt.y = h.surfacePt.y();
                    pt.z = h.surfacePt.z();
                    pt.i = h.normalVec.x();
                    pt.j = h.normalVec.y();
                    pt.k = h.normalVec.z();
                    df.points.push_back(pt);
                }

                if (!df.points.empty()) {
                    m_dmisProgram.features.push_back(df);
                }
            }
        }

        if (m_dmisProgram.features.empty()) {
            QMessageBox::information(
                this,
                "No Programmed Features Found",
                "To start Guided Manual Probing:\n\n"
                "1. Click faces on your 3D STEP model and click 'Auto-Probe CAD' (or '➕ Create Feature') to define the features you want to measure, OR\n"
                "2. Click '📂 Load DMIS' to open an existing inspection program.\n\n"
                "Then click '🧭 Start Guided Part Probe (F6)' to measure the physical part step-by-step."
            );
            return;
        }

        // 2. Reset previous actuals & tree items so the operator measures a clean part from Feature #1
        finishFeatureBuffer();
        m_solvedFeatures.clear();
        m_featureDmisHits.clear();
        m_featureRefIds.clear();
        m_featureDatumLetters.clear();
        m_datumAFeatureId = -1;
        m_datumBFeatureId = -1;
        m_datumCFeatureId = -1;
        m_isPartAligned = false;
        m_pcsMatrix = Eigen::Matrix4d::Identity();
        m_featureCounter = 1;
        m_plannedDmisTreeItems.clear();
        m_activeDmisTreeItem = nullptr;

        for (int i = m_featureTree->topLevelItemCount() - 1; i >= 0; --i) {
            QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
            if (top != m_activePointsGroup) {
                delete m_featureTree->takeTopLevelItem(i);
            }
        }

        if (m_viewport) {
            m_viewport->clearFittedFeatures();
            m_viewport->clearHitMarkers();
            m_viewport->clearProgramTargetMarkers();
            m_viewport->clearPcsTriadVisual();
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        for (size_t i = 0; i < m_dmisProgram.features.size(); ++i) {
            const auto& df = m_dmisProgram.features[i];
            const QString datSuffix = df.datumLetter.empty()
                ? QString()
                : QString(" [DAT %1]").arg(QString::fromStdString(df.datumLetter));
            const QString featTitle = QString("%1 (%2)%3")
                .arg(QString::fromStdString(df.label), QString::fromStdString(df.featureType), datSuffix);
            const double nomVal = (df.featureType == "CIRCLE" || df.featureType == "CYLNDR" || df.featureType == "CYL" || df.featureType == "SPHERE")
                ? df.nominalDiameter : 0.0;

            auto* item = new QTreeWidgetItem(m_featureTree, {
                featTitle,
                QString("%1%2").arg(toDisplayUnit(nomVal, m_unitSystem), 0, 'f', decimals).arg(unitStr),
                "--",
                QString("±%1").arg(toDisplayUnit(m_defaultTolMm, m_unitSystem), 0, 'f', decimals),
                QString("PLANNED (0/%1 pts)").arg(df.points.size())
                });
            item->setForeground(4, QBrush(QColor("#e5c07b")));
            m_plannedDmisTreeItems.push_back(item);
        }

        m_isDmisProgramActive = true;
        startDmisGuidedFeature(0);
        onPositionUpdated(m_lastPosition);
    }

    void MainWindow::onUndoLastProbeHitClicked() {
        if (m_recordedHits.empty()) {
            statusBar()->showMessage("No recorded probe hits in the active buffer to undo.");
            return;
        }

        m_recordedHits.pop_back();
        if (!m_recordedDmisHits.empty()) {
            m_recordedDmisHits.pop_back();
        }

        if (m_activePointsGroup && m_activePointsGroup->childCount() > 0) {
            delete m_activePointsGroup->takeChild(m_activePointsGroup->childCount() - 1);
        }

        const int count = static_cast<int>(m_recordedHits.size());
        m_hitCountLabel->setText(QString("Active Feature Hits: %1").arg(count));
        if (m_activePointsGroup) {
            m_activePointsGroup->setText(2, QString("%1 pts").arg(count));
        }

        if (m_viewport) {
            m_viewport->clearHitMarkers();
            for (const auto& pt : m_recordedHits) {
                m_viewport->addHitPointMarker(pt.x(), pt.y(), pt.z());
            }
        }

        if (m_isDmisProgramActive && m_activeDmisFeatureIdx >= 0 &&
            m_activeDmisFeatureIdx < static_cast<int>(m_dmisProgram.features.size())) {
            const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
            if (m_activeDmisTreeItem) {
                m_activeDmisTreeItem->setText(4, QString("MEASURING (%1/%2 pts)").arg(count).arg(df.points.size()));
            }
            updateDmisTargetVisuals();
            updateManualPromptBanner();
            onPositionUpdated(m_lastPosition);
        }

        statusBar()->showMessage(QString("Undid last probe hit — %1 hit(s) remaining in buffer.").arg(count));
    }

    void MainWindow::onForceSolveActiveDmisFeatureClicked() {
        if (!m_isDmisProgramActive || m_activeDmisFeatureIdx < 0 ||
            m_activeDmisFeatureIdx >= static_cast<int>(m_dmisProgram.features.size())) {
            return;
        }

        const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
        const int count = static_cast<int>(m_recordedHits.size());
        const std::string fType = df.featureType;

        if (fType == "PLANE" && count >= 3) onFitPlaneClicked();
        else if ((fType == "CYLNDR" || fType == "CYL") && count >= 5) onFitCylinderClicked();
        else if ((fType == "CIRCLE" || fType == "CYLNDR" || fType == "CYL") && count >= 3) onFitCircleClicked();
        else if (fType == "SPHERE" && count >= 4) onFitSphereClicked();
        else if (fType == "LINE" && count >= 2) onFitLineClicked();
        else {
            QMessageBox::information(this, "More Hits Needed",
                "Please record the minimum required points for this geometry type before solving.");
            return;
        }

        if (!df.datumLetter.empty() && !m_solvedFeatures.empty()) {
            assignFeatureAsDatum(
                m_solvedFeatures.back().id,
                QString::fromStdString(df.datumLetter),
                m_dmisProgram.datumAlignment.has321Alignment
            );
        }

        startDmisGuidedFeature(m_activeDmisFeatureIdx + 1);
    }

    void MainWindow::onSkipDmisFeatureClicked(int delta) {
        if (!m_isDmisProgramActive || m_dmisProgram.features.empty()) return;

        finishFeatureBuffer();
        if (m_activeDmisTreeItem && !m_activeDmisTreeItem->data(0, ROLE_FEATURE_ID).isValid()) {
            const auto& curDf = m_dmisProgram.features[m_activeDmisFeatureIdx];
            m_activeDmisTreeItem->setText(4, QString("SKIPPED (0/%1 pts)").arg(curDf.points.size()));
            m_activeDmisTreeItem->setForeground(4, QBrush(QColor("#7f848e")));
        }

        const int nextIdx = std::clamp(
            m_activeDmisFeatureIdx + delta,
            0,
            static_cast<int>(m_dmisProgram.features.size()) - 1
        );
        startDmisGuidedFeature(nextIdx);
        onPositionUpdated(m_lastPosition);
    }
    void MainWindow::focusViewportOnActiveTarget() {
        if (!m_autoCameraFocusEnabled || !m_viewport) return;
        if (!m_isDmisProgramActive || m_activeDmisFeatureIdx < 0 ||
            m_activeDmisFeatureIdx >= static_cast<int>(m_dmisProgram.features.size())) {
            return;
        }

        // Do not fight the camera during high-speed automated 30 FPS playback
        if (m_dmisPlaybackTimer && m_dmisPlaybackTimer->isActive()) {
            return;
        }

        const auto& df = m_dmisProgram.features[m_activeDmisFeatureIdx];
        const int activePtIdx = static_cast<int>(m_recordedHits.size());

        double targetX = df.nomX;
        double targetY = df.nomY;
        double targetZ = df.nomZ;
        double normI = df.nomI;
        double normJ = df.nomJ;
        double normK = df.nomK;

        if (activePtIdx >= 0 && activePtIdx < static_cast<int>(df.points.size())) {
            const auto& pt = df.points[activePtIdx];
            // Center framing slightly between the feature center and the active gold bullseye point
            targetX = 0.35 * df.nomX + 0.65 * pt.x;
            targetY = 0.35 * df.nomY + 0.65 * pt.y;
            targetZ = 0.35 * df.nomZ + 0.65 * pt.z;
            normI = pt.i;
            normJ = pt.j;
            normK = pt.k;
        }

        // Scale the zoom window proportionally to the feature diameter (with sensible bounds)
        const double baseSpan = (df.nominalDiameter > 1.0)
            ? std::clamp(df.nominalDiameter * 3.2, 75.0, 280.0)
            : 120.0;

        m_viewport->focusCameraOnTarget(targetX, targetY, targetZ, normI, normJ, normK, baseSpan);
    }



} // namespace OpenCMM