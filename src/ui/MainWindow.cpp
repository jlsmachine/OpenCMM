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
#include <QShortcut>
#include <QSettings>
#include <QPdfWriter>
#include <QPageLayout>
#include <QPainter>
#include "dmis/QifExporter.h"

namespace {
    constexpr int ROLE_ACTUAL_MM = Qt::UserRole + 1;
    constexpr int ROLE_HAS_NOMINAL = Qt::UserRole + 2;
    constexpr int ROLE_NOMINAL_MM = Qt::UserRole + 3;
    constexpr int ROLE_PLUS_TOL_MM = Qt::UserRole + 4;
    constexpr int ROLE_MINUS_TOL_MM = Qt::UserRole + 5;
    constexpr int ROLE_FEATURE_ID = Qt::UserRole + 6;
}


namespace OpenCMM {

    namespace {
        constexpr int ROLE_ACTUAL_MM = Qt::UserRole + 1;
        constexpr int ROLE_HAS_NOMINAL = Qt::UserRole + 2;
        constexpr int ROLE_NOMINAL_MM = Qt::UserRole + 3;
        constexpr int ROLE_PLUS_TOL_MM = Qt::UserRole + 4;
        constexpr int ROLE_MINUS_TOL_MM = Qt::UserRole + 5;
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

        // --- 2. Now setup UI components with the correct theme active ---
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

        // Restore window geometry and state from previous session
        QSettings settings("JLS Machine", "OpenCMM");
        if (settings.contains("geometry")) {
            restoreGeometry(settings.value("geometry").toByteArray());
            restoreState(settings.value("windowState").toByteArray());
            if (settings.value("maximized", false).toBool()) {
                showMaximized();
            }
        }
        else {
            resize(1280, 800); // Default startup size if no saved state exists
        }
    }
    void MainWindow::closeEvent(QCloseEvent* event) {
        QSettings settings("JLS Machine", "OpenCMM");
        settings.setValue("geometry", saveGeometry());
        settings.setValue("windowState", saveState());
        settings.setValue("maximized", isMaximized());
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

        setupToolbar();
        setupDroDock();
        setupFeatureTreeDock();

        m_viewport = new OcctViewport(this);
        m_viewport->setFaceSelectedCallback([this](const CadFaceInfo& faceInfo) {
            onCadFaceSelected(faceInfo);
            });
        m_viewport->setSurfaceHitCallback([this](const CadSurfaceHit& hit) {
            onCadSurfaceProbed(hit);
            });
        setCentralWidget(m_viewport);

        // Apply initial theme immediately to the viewport and HUD
        m_viewport->setTheme(m_isDarkMode);

        statusBar()->setStyleSheet("background-color: #007acc; color: white; font-weight: bold;");
        statusBar()->showMessage("Ready - Click 'Settings...' to configure hardware/units, or probe features to begin inspection.");
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

        // Dynamic styling based on theme
        if (m_isDarkMode) {
            btn->setStyleSheet(QString(
                "QPushButton { background-color: %1; color: %3; border: 1px solid %2; "
                "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
                "QPushButton:hover { background-color: #3e4451; border: 1px solid #61afef; color: #ffffff; }"
                "QPushButton:pressed { background-color: #181a1f; }"
            ).arg(accentBg, borderCol, textCol));
        }
        else {
            // Clean light theme button styling
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

        auto makeRibbonPage = [this](const QList<QFrame*>& groups) -> QWidget* {
            auto* page = new QWidget(this);
            auto* layout = new QHBoxLayout(page);
            layout->setContentsMargins(6, 2, 6, 2);
            layout->setSpacing(2);
            for (QFrame* grp : groups) {
                layout->addWidget(grp);
            }
            layout->addStretch(1);
            return page;
            };

        auto makeStackedControl = [this](const QString& labelText, QWidget* control) -> QWidget* {
            auto* box = new QWidget(this);
            auto* v = new QVBoxLayout(box);
            v->setContentsMargins(2, 2, 2, 2);
            v->setSpacing(4);
            auto* lbl = new QLabel(labelText, box);
            lbl->setStyleSheet("color: #abb2bf; font-size: 10px; font-weight: bold;");
            v->addWidget(lbl);
            v->addWidget(control);
            v->addStretch(1);
            return box;
            };

        // --- Home Tab ---
        m_settingsBtn = createRibbonButton("⚙", "Settings...", "#282c34", "#61afef", "#61afef");
        connect(m_settingsBtn, &QPushButton::clicked, this, &MainWindow::onOpenSettingsClicked);

        auto* aboutBtn = createRibbonButton("ℹ", "About", "#282c34", "#abb2bf", "#abb2bf");
        connect(aboutBtn, &QPushButton::clicked, this, [this]() {
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

        auto* zeroBtn = createRibbonButton("🎯", "Zero XYZ");
        connect(zeroBtn, &QPushButton::clicked, this, &MainWindow::onZeroAxesClicked);

        auto* unitsBtn = createRibbonButton("📏", "Units (mm/in)");
        connect(unitsBtn, &QPushButton::clicked, this, &MainWindow::onUnitToggled);

        auto* clearBtn = createRibbonButton("🗑", "Clear Points", "#5c2b29", "#e06c75", "#ffffff");
        connect(clearBtn, &QPushButton::clicked, this, &MainWindow::onClearPointsClicked);

        m_simTargetCombo = new QComboBox(this);
        m_simTargetCombo->addItem("Master Ball Ø25.4mm", 0);
        m_simTargetCombo->addItem("Bore #1 Ø20.0mm (50,30)", 1);
        m_simTargetCombo->addItem("Bore #2 Ø12.0mm (100,30)", 2);
        m_simTargetCombo->addItem("Top Datum Plane (Z=20)", 3);
        m_simTargetCombo->addItem("Front X-Edge (Y=0)", 4);
        m_simTargetCombo->addItem("Left Y-Edge (X=0)", 5);

        auto* simHitBtn = createRibbonButton("⚡", "Simulate Hit", "#d19a66", "#e5c07b", "#1e2227");
        connect(simHitBtn, &QPushButton::clicked, this, &MainWindow::onSimulateHitClicked);

        auto* autoProbeCadBtn = createRibbonButton("🎯", "Auto-Probe CAD", "#2b6cb0", "#61afef", "#ffffff");
        autoProbeCadBtn->setToolTip("Automatically sample and solve the currently selected 3D CAD face (or Ctrl+Left-Click on CAD to probe individual points)");
        connect(autoProbeCadBtn, &QPushButton::clicked, this, &MainWindow::onAutoProbeCadFaceClicked);

        auto* saveDmisHomeBtn = createRibbonButton("💾", "Save DMIS", "#1e3a8a", "#60a5fa", "#ffffff");
        saveDmisHomeBtn->setToolTip("Export solved features, CAD touch points, and 3-2-1 Datums to a DMIS 5.3 (.dmis) program");
        connect(saveDmisHomeBtn, &QPushButton::clicked, this, &MainWindow::onExportDmisProgramClicked);

        auto* loadDmisHomeBtn = createRibbonButton("📂", "Load DMIS", "#1f7a8c", "#61afef", "#ffffff");
        loadDmisHomeBtn->setToolTip("Load a DMIS 5.3 (.dmis) inspection program and display 3D target guide markers");
        connect(loadDmisHomeBtn, &QPushButton::clicked, this, &MainWindow::onLoadDmisProgramClicked);

        m_autoRunDmisBtnHome = createRibbonButton("▶", "Run DMIS", "#2f855a", "#98c379", "#ffffff");
        m_autoRunDmisBtnHome->setToolTip("Animate the Digital Twin CMM through all loaded DMIS target points offline");
        connect(m_autoRunDmisBtnHome, &QPushButton::clicked, this, &MainWindow::onToggleAutoRunDmisClicked);

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("System & Hardware",{ m_settingsBtn, aboutBtn }),
            createRibbonGroup("Machine DRO",{ zeroBtn, unitsBtn }),
            createRibbonGroup("Point Buffer",{ clearBtn }),
            createRibbonGroup("Offline Simulator & CAD",{ makeStackedControl("Virtual Target:", m_simTargetCombo), simHitBtn, autoProbeCadBtn }),
            createRibbonGroup("DMIS 5.3 Program",{ saveDmisHomeBtn, loadDmisHomeBtn, m_autoRunDmisBtnHome })
            }), "Home");

        // --- Probe & Alignment Tab ---
        m_compModeCombo = new QComboBox(this);
        m_compModeCombo->addItem("External (-R)", static_cast<int>(ProbeCompMode::External));
        m_compModeCombo->addItem("Internal (+R)", static_cast<int>(ProbeCompMode::Internal));
        m_compModeCombo->addItem("Uncomp (0)", static_cast<int>(ProbeCompMode::Uncompensated));

        auto* calBtn = createRibbonButton("🔴", "Calibrate Tip", "#5a32a3", "#8957e5", "#ffffff");
        connect(calBtn, &QPushButton::clicked, this, &MainWindow::onCalibrateProbeClicked);

        auto* alignBtn = createRibbonButton("📐", "3-2-1 Align...", "#9b2c2c", "#e06c75", "#ffffff");
        connect(alignBtn, &QPushButton::clicked, this, &MainWindow::onRun321AlignmentClicked);

        m_alignStatusBtn = createRibbonButton("🌐", "Coord: MCS", "#3a3f4b", "#e5c07b", "#e5c07b");
        m_alignStatusBtn->setCheckable(true);
        m_alignStatusBtn->setChecked(false);
        connect(m_alignStatusBtn, &QPushButton::clicked, this, [this](bool checked) {
            if (!m_isPartAligned && checked) {
                QMessageBox::warning(this, "No Alignment Active", "Please run a 3-2-1 alignment first.");
                m_alignStatusBtn->setChecked(false);
                return;
            }
            if (checked) {
                m_alignStatusBtn->setText("🌐\nPCS (Aligned)");
                m_alignStatusBtn->setStyleSheet("QPushButton { background-color: #1e4620; color: #98c379; border: 1px solid #98c379; border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
            }
            else {
                m_alignStatusBtn->setText("🌐\nCoord: MCS");
                m_alignStatusBtn->setStyleSheet("QPushButton { background-color: #3a3f4b; color: #e5c07b; border: 1px solid #e5c07b; border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
            }
            updateDroDisplay();
            });

        auto* clearBtnTab2 = createRibbonButton("🗑", "Clear Hits", "#5c2b29", "#e06c75", "#ffffff");
        connect(clearBtnTab2, &QPushButton::clicked, this, &MainWindow::onClearPointsClicked);

        auto* setDatABtn = createRibbonButton("🅰", "Set Datum A", "#1e3a8a", "#60a5fa", "#ffffff");
        setDatABtn->setToolTip("Probe selected CAD face (or use last solved feature) as Primary Datum A (Level +Z)");
        connect(setDatABtn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("A"); });

        auto* setDatBBtn = createRibbonButton("🅱", "Set Datum B", "#14532d", "#4ade80", "#ffffff");
        setDatBBtn->setToolTip("Probe selected CAD face (or use last solved feature) as Secondary Datum B (Rotate +X)");
        connect(setDatBBtn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("B"); });

        auto* setDatCBtn = createRibbonButton("🅲", "Set Datum C", "#78350f", "#fbbf24", "#ffffff");
        setDatCBtn->setToolTip("Probe selected CAD face (or use last solved feature) as Tertiary Datum C (Origin 0,0,0)");
        connect(setDatCBtn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("C"); });

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("Stylus Compensation",{ makeStackedControl("Tip Compensation:", m_compModeCombo), calBtn }),
            createRibbonGroup("CAD / Feature Datums (A-B-C)",{ setDatABtn, setDatBBtn, setDatCBtn }),
            createRibbonGroup("Part Coordinate System (PCS)",{ alignBtn, m_alignStatusBtn }),
            createRibbonGroup("Quick Buffer",{ clearBtnTab2 })
            }), "Probe & Alignment");

        // --- Measure Tab ---
        m_workplaneCombo = new QComboBox(this);
        m_workplaneCombo->addItem("XY (+Z)", static_cast<int>(StandardWorkplane::XY_PlusZ));
        m_workplaneCombo->addItem("YZ (+X)", static_cast<int>(StandardWorkplane::YZ_PlusX));
        m_workplaneCombo->addItem("ZX (+Y)", static_cast<int>(StandardWorkplane::ZX_PlusY));
        m_workplaneCombo->addItem("Last Fitted Plane", static_cast<int>(StandardWorkplane::CustomPlane));

        auto* fitPlaneBtn = createRibbonButton("▱", "Fit Plane");
        connect(fitPlaneBtn, &QPushButton::clicked, this, &MainWindow::onFitPlaneClicked);
        auto* fitSphereBtn = createRibbonButton("🌐", "Fit Sphere");
        connect(fitSphereBtn, &QPushButton::clicked, this, &MainWindow::onFitSphereClicked);
        auto* fitCircleBtn = createRibbonButton("⭕", "Fit 2D Circle", "#2b6cb0", "#61afef", "#ffffff");
        connect(fitCircleBtn, &QPushButton::clicked, this, &MainWindow::onFitCircleClicked);
        auto* fitLineBtn = createRibbonButton("╱", "Fit 2D Line", "#2b6cb0", "#61afef", "#ffffff");
        connect(fitLineBtn, &QPushButton::clicked, this, &MainWindow::onFitLineClicked);
        auto* relBtn = createRibbonButton("↔", "Distance / ∠", "#2f855a", "#98c379", "#ffffff");
        connect(relBtn, &QPushButton::clicked, this, &MainWindow::onMeasureRelationClicked);

        m_featureRenderCombo = new QComboBox(this);
        m_featureRenderCombo->addItem("Fully Rendered (Shaded)", 0);
        m_featureRenderCombo->addItem("Wireframe", 1);
        connect(m_featureRenderCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
            if (m_viewport) {
                m_viewport->setFittedFeaturesWireframe(index == 1);
            }
            });

        // auto* clearFeatBtn = createRibbonButton("🧹", "Clear Features", "#5c2b29", "#e06c75", "#ffffff");
        // connect(clearFeatBtn, &QPushButton::clicked, this, [this]() {
        //     if (m_viewport) {
        ///         m_viewport->clearFittedFeatures();
        //     }
        //     statusBar()->showMessage("Cleared fitted 3D feature visuals from viewport.");
        //     });

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("Active Projection",{ makeStackedControl("2D Workplane:", m_workplaneCombo) }),
            createRibbonGroup("Feature Display",{ makeStackedControl("Feature Style:", m_featureRenderCombo) }),
            createRibbonGroup("3D Features",{ fitPlaneBtn, fitSphereBtn }),
            createRibbonGroup("2D Workplane Features",{ fitCircleBtn, fitLineBtn }),
            createRibbonGroup("Dimensions & Relations",{ relBtn })
            }), "Measure");

        // --- CAD & Deviation Tab ---
        auto* loadStepBtn = createRibbonButton("📦", "Load STEP", "#2d5a88", "#61afef", "#ffffff");
        connect(loadStepBtn, &QPushButton::clicked, this, &MainWindow::onLoadStepClicked);
        auto* transformCadBtn = createRibbonButton("🧭", "Orient CAD...", "#3b6998", "#61afef", "#ffffff");
        connect(transformCadBtn, &QPushButton::clicked, this, &MainWindow::onTransformCadClicked);
        auto* autoProbeTabBtn = createRibbonButton("🎯", "Auto-Probe Face", "#2b6cb0", "#61afef", "#ffffff");
        connect(autoProbeTabBtn, &QPushButton::clicked, this, &MainWindow::onAutoProbeCadFaceClicked);
        auto* saveDmisCadBtn = createRibbonButton("💾", "Save DMIS", "#1e3a8a", "#60a5fa", "#ffffff");
        saveDmisCadBtn->setToolTip("Export authored CAD features and Datums to a DMIS 5.3 (.dmis) inspection program");
        connect(saveDmisCadBtn, &QPushButton::clicked, this, &MainWindow::onExportDmisProgramClicked);
        auto* loadDmisCadBtn = createRibbonButton("📂", "Load DMIS", "#1f7a8c", "#61afef", "#ffffff");
        connect(loadDmisCadBtn, &QPushButton::clicked, this, &MainWindow::onLoadDmisProgramClicked);
        m_autoRunDmisBtnCad = createRibbonButton("▶", "Run DMIS", "#2f855a", "#98c379", "#ffffff");
        connect(m_autoRunDmisBtnCad, &QPushButton::clicked, this, &MainWindow::onToggleAutoRunDmisClicked);
        auto* cadDevBtn = createRibbonButton("🌡", "Surface Dev", "#0d9488", "#4fd1c5", "#ffffff");
        connect(cadDevBtn, &QPushButton::clicked, this, &MainWindow::onEvaluateCadDeviationClicked);

        auto* cadDatABtn = createRibbonButton("🅰", "CAD Datum A", "#1e3a8a", "#60a5fa", "#ffffff");
        cadDatABtn->setToolTip("Auto-probe the selected CAD face and assign it as Primary Datum A (Level +Z)");
        connect(cadDatABtn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("A"); });

        auto* cadDatBBtn = createRibbonButton("🅱", "CAD Datum B", "#14532d", "#4ade80", "#ffffff");
        cadDatBBtn->setToolTip("Auto-probe the selected CAD face and assign it as Secondary Datum B (Rotate +X)");
        connect(cadDatBBtn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("B"); });

        auto* cadDatCBtn = createRibbonButton("🅲", "CAD Datum C", "#78350f", "#fbbf24", "#ffffff");
        cadDatCBtn->setToolTip("Auto-probe the selected CAD face and assign it as Tertiary Datum C (Origin 0,0,0)");
        connect(cadDatCBtn, &QPushButton::clicked, this, [this]() { onAssignCadDatumClicked("C"); });

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("3D CAD Model",{ loadStepBtn, transformCadBtn }),
            createRibbonGroup("Offline CAD Authoring",{ autoProbeTabBtn, saveDmisCadBtn, loadDmisCadBtn, m_autoRunDmisBtnCad }),
            createRibbonGroup("Specify CAD Datums (3-2-1)",{ cadDatABtn, cadDatBBtn, cadDatCBtn }),
            createRibbonGroup("Point Cloud vs. CAD",{ cadDevBtn })
            }), "CAD & Deviation");

        // --- Reporting Tab ---
        auto* editTolBtn = createRibbonButton("±", "Edit Tol...", "#2d3748", "#e5c07b", "#e5c07b");
        connect(editTolBtn, &QPushButton::clicked, this, &MainWindow::onEditToleranceClicked);

        auto* clearAllFeaturesBtn = createRibbonButton("🧹", "Clear All", "#5c2b29", "#e06c75", "#ffffff");
        connect(clearAllFeaturesBtn, &QPushButton::clicked, this, &MainWindow::onClearAllFeaturesClicked);

        auto* saveDmisRepBtn = createRibbonButton("💾", "Export DMIS", "#1e3a8a", "#60a5fa", "#ffffff");
        saveDmisRepBtn->setToolTip("Export inspection program to DMIS 5.3 (.dmis)");
        connect(saveDmisRepBtn, &QPushButton::clicked, this, &MainWindow::onExportDmisProgramClicked);

        auto* exportBtn = createRibbonButton("📄", "Export Report", "#1f7a8c", "#61afef", "#ffffff");
        connect(exportBtn, &QPushButton::clicked, this, &MainWindow::onExportReportClicked);

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("Feature Evaluation",{ editTolBtn }),
            createRibbonGroup("Manage Features",{ clearAllFeaturesBtn }),
            createRibbonGroup("Quality & DMIS Export",{ saveDmisRepBtn, exportBtn })
            }), "Reporting");

        setMenuWidget(ribbonTabs);
    }

    void MainWindow::setupDroDock() {
        auto* droDock = new QDockWidget("LIVE CMM DRO & PROBE STATUS", this);
        droDock->setObjectName("LiveDroDock");
        m_droContainer = new QWidget(droDock);
        m_droContainer->setStyleSheet(m_isDarkMode ? "background-color: #181a1f;" : "background-color: #f8fafc;");
        auto* layout = new QVBoxLayout(m_droContainer);
        auto* grid = new QGridLayout();

        const char* axisNames[3] = { "X", "Y", "Z" };
        const char* axisColors[3] = { "#e06c75", "#98c379", "#61afef" };
        QLabel** droPtrs[3] = { &m_droX, &m_droY, &m_droZ };

        for (int i = 0; i < 3; ++i) {
            auto* axisLbl = new QLabel(axisNames[i], m_droContainer);
            axisLbl->setStyleSheet(QString("color: %1; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace;").arg(axisColors[i]));

            *droPtrs[i] = new QLabel("   0.0000", m_droContainer);
            (*droPtrs[i])->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            (*droPtrs[i])->setStyleSheet("color: #0f172a; background-color: #ffffff; border: 1px solid #94a3b8; border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 190px;");

            m_unitLabels[i] = new QLabel("mm", m_droContainer);
            m_unitLabels[i]->setStyleSheet("color: #64748b; font-size: 16px; font-weight: bold;");

            grid->addWidget(axisLbl, i, 0);
            grid->addWidget(*droPtrs[i], i, 1);
            grid->addWidget(m_unitLabels[i], i, 2);
        }

        layout->addLayout(grid);
        m_hitCountLabel = new QLabel("Active Feature Hits: 0", m_droContainer);
        layout->addWidget(m_hitCountLabel);

        auto* probeGroup = new QGroupBox("ACTIVE PROBE CALIBRATION", m_droContainer);
        auto* probeLayout = new QVBoxLayout(probeGroup);
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

        droDock->setWidget(m_droContainer);
        addDockWidget(Qt::LeftDockWidgetArea, droDock);
    }

    void MainWindow::setupFeatureTreeDock() {
        auto* treeDock = new QDockWidget("INSPECTION FEATURE TREE", this);
        treeDock->setObjectName("FeatureTreeDock");
        m_treeContainer = new QWidget(treeDock);
        m_treeContainer->setStyleSheet(m_isDarkMode ? "background-color: #181a1f;" : "background-color: #f8fafc;");
        auto* dockLayout = new QVBoxLayout(m_treeContainer);

        m_featureTree = new QTreeWidget(m_treeContainer);
        m_featureTree->setColumnCount(5);
        m_featureTree->setHeaderLabels({ "Feature / Parameter", "Nominal", "Actual", "±Tol", "Dev / Status" });

        // --- Enable Context Menu ---
        m_featureTree->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(m_featureTree, &QTreeWidget::customContextMenuRequested,
            this, &MainWindow::onFeatureTreeContextMenuRequested);
        // ---------------------------

        // --- Keyboard Shortcut Support (Delete & Backspace) ---
        auto* deleteKeyShortcut = new QShortcut(QKeySequence::Delete, m_featureTree);
        connect(deleteKeyShortcut, &QShortcut::activated, this, &MainWindow::onDeleteSelectedFeature);

        auto* backspaceKeyShortcut = new QShortcut(QKeySequence(Qt::Key_Backspace), m_featureTree);
        connect(backspaceKeyShortcut, &QShortcut::activated, this, &MainWindow::onDeleteSelectedFeature);
        // ------------------------------------------------------

        m_activePointsGroup = new QTreeWidgetItem(m_featureTree, { "Active Probe Buffer", "", "0 pts", "", "READY" });
        m_activePointsGroup->setExpanded(true);

        dockLayout->addWidget(m_featureTree);
        treeDock->setWidget(m_treeContainer);
        addDockWidget(Qt::LeftDockWidgetArea, treeDock);

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


    void MainWindow::onOpenSettingsClicked() {
        SettingsDialog dlg(this);

        dlg.setCurrentUnitSystem(m_unitSystem);
        dlg.setDefaultToleranceMm(m_defaultTolMm);
        dlg.setAudioChimesEnabled(m_audioEnabled);
        dlg.setDarkModeEnabled(m_isDarkMode);
        dlg.setSelectedMachinePath(m_machinePackagePath);

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

            // Reset probe calibration to nominal defaults
            m_isProbeCalibrated = false;
            m_certifiedBallDiameterMm = 25.4000;
            m_effectiveProbeRadius = 1.5000;
            m_lastProbeLobingMm = 0.0;

            // Reset units to Millimeters
            if (m_unitSystem != UnitSystem::Millimeters) {
                onUnitToggled();
            }

            // Reset controller scale & sign
            if (m_controller) {
                m_controller->setAxisCalibration(Eigen::Vector3d(0.001, 0.001, 0.001), Eigen::Vector3i(1, 1, 1));
            }

            // Re-apply Dark Metrology Theme and refresh displays
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

        // Apply theme selection from dialog
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

        m_defaultTolMm = dlg.getDefaultToleranceMm();
        m_audioEnabled = dlg.areAudioChimesEnabled();
        m_machinePackagePath = dlg.getSelectedMachinePath();

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
                applyEvaluationToTreeItem(item, item->data(0, ROLE_ACTUAL_MM).toDouble(), item->data(0, ROLE_HAS_NOMINAL).toBool(), item->data(0, ROLE_NOMINAL_MM).toDouble(), item->data(0, ROLE_PLUS_TOL_MM).toDouble(), item->data(0, ROLE_MINUS_TOL_MM).toDouble());
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
                m_compModeCombo->setCurrentIndex(0); // External (-R) for planar surfaces
            }
        }

        // Automatically align active 2D Workplane to the selected CAD Cylinder axis or Plane normal
        if (m_workplaneCombo && (m_armedCadDescription.contains("Cylinder") || m_armedCadDescription.contains("Plane"))) {
            const double ax = std::abs(m_armedCadNormalMm.x());
            const double ay = std::abs(m_armedCadNormalMm.y());
            const double az = std::abs(m_armedCadNormalMm.z());
            if (az >= 0.999) {
                m_workplaneCombo->setCurrentIndex(0); // XY (+Z)
            }
            else if (ax >= 0.999) {
                m_workplaneCombo->setCurrentIndex(1); // YZ (+X)
            }
            else if (ay >= 0.999) {
                m_workplaneCombo->setCurrentIndex(2); // ZX (+Y)
            }
            else {
                m_lastFittedPlaneFrame = WorkplaneFrame::fromPlane(m_armedCadLocationMm, m_armedCadNormalMm);
                m_hasCustomPlane = true;
                m_workplaneCombo->setCurrentIndex(3); // Custom Plane aligned to tilted feature
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

        // Offset contact point along outward material-free normal by effective ruby radius
        const double rProbe = (currentCompMode() == ProbeCompMode::Uncompensated) ? 0.0 : m_effectiveProbeRadius;
        const Eigen::Vector3d rubyCenter = surfPt + surfNorm * rProbe;

        m_lastApproachDir = -surfNorm;
        m_recordedDmisHits.push_back({ surfPt, surfNorm });

        Position3D rubyPos{ rubyCenter.x(), rubyCenter.y(), rubyCenter.z() };
        onProbeHit(rubyPos);

        // Annotate the latest tree item with the DMIS surface normal vector (I, J, K)
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

    void MainWindow::onAssignCadDatumClicked(const QString& datumLetter) {
        const size_t prevCount = m_solvedFeatures.size();

        // If a 3D CAD face is currently selected, auto-probe and solve it right on the model
        if (m_viewport && m_viewport->hasCadModel() && m_viewport->hasSelectedCadFace()) {
            onAutoProbeCadFaceClicked();
            if (m_solvedFeatures.size() > prevCount) {
                assignFeatureAsDatum(m_solvedFeatures.back().id, datumLetter, true);
                return;
            }
        }

        // Otherwise, if a solved feature is highlighted in the Inspection Feature Tree, assign it
        QTreeWidgetItem* cur = m_featureTree->currentItem();
        while (cur && cur->parent() != nullptr) {
            cur = cur->parent();
        }
        if (cur && cur != m_activePointsGroup && cur->data(0, ROLE_FEATURE_ID).isValid()) {
            assignFeatureAsDatum(cur->data(0, ROLE_FEATURE_ID).toInt(), datumLetter, true);
            return;
        }

        // Or fallback to the most recently solved feature if one exists
        if (!m_solvedFeatures.empty()) {
            assignFeatureAsDatum(m_solvedFeatures.back().id, datumLetter, true);
            return;
        }

        QMessageBox::information(
            this,
            "Specify Datum " + datumLetter,
            QString("To specify Datum %1:\n\n"
                    "1. Left-click any face (Plane, Bore, Boss, or Edge) directly on the 3D CAD model and click 'CAD Datum %1', OR\n"
                    "2. Right-click any solved feature in the Inspection Feature Tree and choose 'Set as Datum %1'.")
                .arg(datumLetter)
        );
    }

    void MainWindow::assignFeatureAsDatum(int featureId, const QString& datumLetter, bool autoSolve321) {
        const QString letter = datumLetter.trimmed().toUpper();
        if (letter != "A" && letter != "B" && letter != "C") return;

        // Clear any previous feature holding the same datum letter
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

        // Tag the feature in m_solvedFeatures and QTreeWidget with [DAT A/B/C]
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

        // Clear previous inspection tree items and visuals
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

        // Populate Inspection Feature Tree with PLANNED DMIS features
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
            statusBar()->showMessage(QString("DMIS 5.3 Program Execution Complete — All %1 features measured and evaluated!")
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

        // Arm nominal diameter and compensation mode for the active DMIS feature
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

        // Align 2D Workplane to the DMIS feature's normal/axis
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

        if (m_playbackPhase == 0) {
            // Phase 0: Move to standoff approach point (via clearance Z if first point of feature)
            const int totalSteps = (ptIdx == 0) ? 10 : 6;
            if (m_playbackSubStep == 0) {
                m_playbackStartPos = Eigen::Vector3d(m_lastPosition.x, m_lastPosition.y, m_lastPosition.z);
            }
            m_playbackSubStep++;
            const double t = std::clamp(static_cast<double>(m_playbackSubStep) / totalSteps, 0.0, 1.0);
            Eigen::Vector3d interp = m_playbackStartPos + t * (standoffPos - m_playbackStartPos);
            if (ptIdx == 0) {
                // Arc through clearance plane Z on feature transitions
                const double zClear = std::max(m_dmisProgram.clearanceZMm, std::max(m_playbackStartPos.z(), standoffPos.z()) + 15.0);
                interp.z() += 4.0 * t * (1.0 - t) * (zClear - 0.5 * (m_playbackStartPos.z() + standoffPos.z()));
            }
            onPositionUpdated({ interp.x(), interp.y(), interp.z() });

            if (m_playbackSubStep >= totalSteps) {
                m_playbackPhase = 1;
                m_playbackSubStep = 0;
                m_playbackStartPos = standoffPos;
            }
        }
        else if (m_playbackPhase == 1) {
            // Phase 1: Probing approach along normal vector into surface contact
            const int totalSteps = 5;
            m_playbackSubStep++;
            const double t = std::clamp(static_cast<double>(m_playbackSubStep) / totalSteps, 0.0, 1.0);
            const Eigen::Vector3d interp = standoffPos + t * (touchPos - standoffPos);
            onPositionUpdated({ interp.x(), interp.y(), interp.z() });

            if (m_playbackSubStep >= totalSteps) {
                m_playbackPhase = 2;
                m_playbackSubStep = 0;
                m_playbackStartPos = touchPos;
                m_playbackEndPos = standoffPos;

                // Trigger the virtual probe hit at the exact surface point & normal
                CadSurfaceHit simHit;
                simHit.success = true;
                simHit.surfacePoint = gp_Pnt(surfPt.x(), surfPt.y(), surfPt.z());
                simHit.surfaceNormal = gp_Dir(surfNorm.x(), surfNorm.y(), surfNorm.z());
                onCadSurfaceProbed(simHit);
            }
        }
        else if (m_playbackPhase == 2) {
            // Phase 2: Retract along normal back to standoff position
            const int totalSteps = 4;
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

        // Live shop-floor proximity guidance when a DMIS 5.3 program is active (and not in auto-playback)
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

            // If guided DMIS mode is active and this was a physical CMM hit, use the planned target's nominal normal
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

        // --- Add individual hit as a child item in the feature tree ---
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
        // -------------------------------------------------------------

        if (m_viewport) {
            m_viewport->updateProbePosition(hitPos.x, hitPos.y, hitPos.z);
            m_viewport->addHitPointMarker(hitPos.x, hitPos.y, hitPos.z);
        }

        // --- Guided DMIS 5.3 Execution Progression ---
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
                else if ((fType == "CIRCLE" || fType == "CYLNDR" || fType == "CYL") && count >= 3) {
                    onFitCircleClicked();
                }
                else if (fType == "SPHERE" && count >= 4) {
                    onFitSphereClicked();
                }
                else if (fType == "LINE" && count >= 2) {
                    onFitLineClicked();
                }

                // If the just-solved DMIS feature is a declared Datum (DATDEF A, B, or C), assign it and auto-align when A-B-C are ready
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

        if (m_datumAFeatureId > 0 && m_datumBFeatureId > 0 && m_datumCFeatureId > 0) {
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
        // If the user has points in the active buffer that haven't been fitted yet, offer to fit or inform them
        if (m_solvedFeatures.empty()) {
            QMessageBox::information(
                this,
                "No Solved Features to Export",
                "Please solve at least one feature before saving a DMIS 5.3 (.dmis) program:\n\n"
                "• Click any CAD face and press '🎯 Auto-Probe CAD' (or '🅰/🅱/🅲 CAD Datum'), OR\n"
                "• Probe points (Ctrl + Left-Click on CAD) and click Fit Plane / Fit 2D Circle / Fit Sphere."
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

        // Custom Export Dialog with explicit Format Selector
        QDialog expDlg(this);
        expDlg.setWindowTitle("Export Inspection Report / DMIS 5.3 Program");
        expDlg.setMinimumWidth(420);

        auto* dlgLayout = new QVBoxLayout(&expDlg);
        auto* formLayout = new QFormLayout();
        formLayout->setSpacing(10);

        auto* partEdit = new QLineEdit("JLS-PART-001", &expDlg);
        auto* formatCombo = new QComboBox(&expDlg);
        formatCombo->addItem("PDF Printable Inspection Certificate (.pdf)", "pdf"); // <--- Add this option
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

        // Guarantee correct extension
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
            int yPos = 1200; // coordinate units in 1/10th of a mm
            const int leftMargin = 1000;

            // Header Title
            painter.setFont(QFont("Segoe UI", 16, QFont::Bold));
            painter.drawText(leftMargin, yPos, "JLS MACHINE LLC — INSPECTION CERTIFICATE");
            yPos += 400;

            // Metadata Block
            painter.setFont(QFont("Segoe UI", 10, QFont::Normal));
            const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
            painter.drawText(leftMargin, yPos, "Part / Job Number: " + partName); yPos += 250;
            painter.drawText(leftMargin, yPos, "Inspection Date: " + timestamp); yPos += 250;
            painter.drawText(leftMargin, yPos, QString("Stylus Effective Radius (R_eff): %1 mm").arg(m_effectiveProbeRadius, 0, 'f', 4)); yPos += 400;

            // Table Header Background
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

            // Populate Table Rows from Feature Tree
            painter.setFont(QFont("Segoe UI", 9, QFont::Normal));
            for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                if (top == m_activePointsGroup) continue;

                if (yPos > 26000) { // New page if page full
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

                // Sub-rows (hits / info)
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


        // --- 0. DMIS 5.3 Program Export (.dmis) ---
        if (fmt == "dmis" || filePath.endsWith(".dmis", Qt::CaseInsensitive)) {
            writeDmisProgramToFile(filePath, partName);
            return;
        }

        // --- 1. QIF 2.1 XML Export ---
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
                item.featureType = top->text(0).contains("Circle") || top->text(0).contains("Bore") ? "Bore"
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

        // --- 2. CSV Export ---
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

        // --- 3. HTML Printable Report Export ---
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
        // ... existing CSV and HTML export handling continues below ...

    void MainWindow::onSimulateHitClicked() {
        // If a DMIS 5.3 program is loaded and active, Simulate Hit fires the next planned PTMEAS point!
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
            simHit = { 50.0 + R * std::cos(angle), 30.0 + R * std::sin(angle), 15.0 };
        }
        else if (mode == 2) {
            const double R = 6.0000 - rProbe;
            const double angle = idx * (2.0 * 3.141592653589793 / 5.0) + 0.4;
            simHit = { 100.0 + R * std::cos(angle), 30.0 + R * std::sin(angle), 15.0 };
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
            QMessageBox::warning(this, "Insufficient Points", "Please record at least 4 hits on the Master Ball to calibrate the stylus tip.");
            return;
        }

        bool ok = false;
        double certDiam = QInputDialog::getDouble(
            this,
            "Master Ball Calibration",
            "Certified Master Sphere Diameter (mm):",
            m_certifiedBallDiameterMm,
            1.0, 200.0, 4, &ok
        );
        if (!ok) return;

        ProbeCalibrationResult cal = MetrologyMath::calibrateProbeOnMasterBall(m_recordedHits, certDiam);
        if (!cal.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Calibration Failed", "Could not solve sphere fit for probe calibration.");
            return;
        }

        m_certifiedBallDiameterMm = certDiam;
        m_effectiveProbeRadius = cal.effectiveRadius;
        m_lastProbeLobingMm = cal.formError;
        m_isProbeCalibrated = true;

        updateProbeStatusDisplay();
        playSolveChime(cal.formError <= m_defaultTolMm);
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
            return; // Don't show context menu for empty space or the active point buffer header
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

        // Ensure we target the top-level parent feature node
        QTreeWidgetItem* topItem = item;
        while (topItem->parent() != nullptr && topItem->parent() != m_activePointsGroup) {
            topItem = topItem->parent();
        }

        if (topItem == m_activePointsGroup) return;

        // If the item has a registered feature ID, remove its corresponding 3D visual and solved data
        if (topItem->data(0, ROLE_FEATURE_ID).isValid()) {
            const int featureId = topItem->data(0, ROLE_FEATURE_ID).toInt();

            // Remove from m_solvedFeatures vector
            auto it = std::remove_if(m_solvedFeatures.begin(), m_solvedFeatures.end(), [featureId](const MeasuredFeature& f) {
                return f.id == featureId;
                });
            if (it != m_solvedFeatures.end()) {
                m_solvedFeatures.erase(it, m_solvedFeatures.end());
            }

            // Clear its highlight / visual representation in the viewport if supported
            if (m_viewport) {
                m_viewport->highlightFittedFeature(-1);
            }
        }

        // Delete from the QTreeWidget
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

        // 1. Clear solved feature data model & DMIS guided state
        stopDmisAutoRun();
        m_isDmisProgramActive = false;
        m_activeDmisFeatureIdx = -1;
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

        // 2. Remove all top-level items except the active point buffer
        for (int i = m_featureTree->topLevelItemCount() - 1; i >= 0; --i) {
            QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
            if (top != m_activePointsGroup) {
                delete m_featureTree->takeTopLevelItem(i);
            }
        }

        // 3. Clear 3D viewport fitted feature visuals
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
            // --- Dark Theme Studio Palette ---
            setStyleSheet(
                "QMainWindow { background-color: #1e2227; color: #e0e0e0; }"
                "QDockWidget { color: #ffffff; font-weight: bold; }"
                "QDockWidget::title { background: #282c34; padding: 6px; border: 1px solid #3a3f4b; }"
                "QGroupBox { border: 1px solid #3a3f4b; border-radius: 4px; margin-top: 10px; padding-top: 12px; color: #abb2bf; font-weight: bold; }"
                "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
            );

            if (m_droContainer) m_droContainer->setStyleSheet("background-color: #181a1f;");
            if (m_treeContainer) m_treeContainer->setStyleSheet("background-color: #181a1f;");

            // DRO Numerical Display Styles (Dark)
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

            // Reset ribbon buttons to dark style
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

			// Update viewport background for dark theme
            if (m_viewport) {
                m_viewport->setBackgroundColor(true);
            }
            if (m_viewport) {
                m_viewport->setTheme(dark);
            }

            statusBar()->setStyleSheet("background-color: #007acc; color: white; font-weight: bold;");
            statusBar()->showMessage("Switched to Dark Metrology Theme.");
        }
        else {
            // --- Clean Light Inspection Palette ---
            setStyleSheet(
                "QMainWindow { background-color: #f1f5f9; color: #1e293b; }"
                "QDockWidget { color: #1e293b; font-weight: bold; }"
                "QDockWidget::title { background: #cbd5e1; padding: 6px; border: 1px solid #94a3b8; }"
                "QGroupBox { border: 1px solid #cbd5e1; border-radius: 4px; margin-top: 10px; padding-top: 12px; color: #334155; font-weight: bold; }"
                "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
            );

            if (m_droContainer) m_droContainer->setStyleSheet("background-color: #f8fafc;");
            if (m_treeContainer) m_treeContainer->setStyleSheet("background-color: #f8fafc;");

            // DRO Numerical Display Styles (Light)
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

            // Apply clean light style to all ribbon buttons
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
            // Reset 3D viewport background color and HUD to light theme
            if (m_viewport) {
                m_viewport->setTheme(false);
            }

            statusBar()->setStyleSheet("background-color: #0284c7; color: white; font-weight: bold;");
            statusBar()->showMessage("Switched to Light Inspection Theme.");
        }
    }

} // namespace OpenCMM