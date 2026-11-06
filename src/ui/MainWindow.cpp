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

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("System & Hardware",{ m_settingsBtn, aboutBtn }),
            createRibbonGroup("Machine DRO",{ zeroBtn, unitsBtn }),
            createRibbonGroup("Point Buffer",{ clearBtn }),
            createRibbonGroup("Offline Simulator",{ makeStackedControl("Virtual Target:", m_simTargetCombo), simHitBtn })
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

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("Stylus Compensation",{ makeStackedControl("Tip Compensation:", m_compModeCombo), calBtn }),
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
        auto* cadDevBtn = createRibbonButton("🌡", "Surface Dev", "#0d9488", "#4fd1c5", "#ffffff");
        connect(cadDevBtn, &QPushButton::clicked, this, &MainWindow::onEvaluateCadDeviationClicked);

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("3D CAD Model",{ loadStepBtn, transformCadBtn }),
            createRibbonGroup("Point Cloud vs. CAD",{ cadDevBtn })
            }), "CAD & Deviation");

        // --- Reporting Tab ---
        auto* editTolBtn = createRibbonButton("±", "Edit Tol...", "#2d3748", "#e5c07b", "#e5c07b");
        connect(editTolBtn, &QPushButton::clicked, this, &MainWindow::onEditToleranceClicked);

        auto* clearAllFeaturesBtn = createRibbonButton("🧹", "Clear All", "#5c2b29", "#e06c75", "#ffffff");
        connect(clearAllFeaturesBtn, &QPushButton::clicked, this, &MainWindow::onClearAllFeaturesClicked);

        auto* exportBtn = createRibbonButton("📄", "Export Report", "#1f7a8c", "#61afef", "#ffffff");
        connect(exportBtn, &QPushButton::clicked, this, &MainWindow::onExportReportClicked);

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("Feature Evaluation",{ editTolBtn }),
            createRibbonGroup("Manage Features",{ clearAllFeaturesBtn }),
            createRibbonGroup("Quality Documentation",{ exportBtn })
            }), "Reporting");

        setMenuWidget(ribbonTabs);
    }

    void MainWindow::setupDroDock() {
        auto* droDock = new QDockWidget("LIVE CMM DRO & PROBE STATUS", this);
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
        m_armedCadDescription = QString::fromStdString(faceInfo.surfaceType);

        if (m_compModeCombo && faceInfo.nominalDiameter > 0.0) {
            m_compModeCombo->setCurrentIndex(faceInfo.isInternal ? 1 : 0);
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        if (faceInfo.nominalDiameter > 0.0) {
            statusBar()->showMessage(QString("Selected CAD %1 — Armed Nominal Ø: %2%3 (%4)")
                .arg(m_armedCadDescription)
                .arg(toDisplayUnit(faceInfo.nominalDiameter, m_unitSystem), 0, 'f', decimals)
                .arg(unitStr)
                .arg(faceInfo.isInternal ? "Internal Bore" : "External Boss"));
        }
        else {
            statusBar()->showMessage(QString("Selected CAD %1 at (%2, %3, %4)%5")
                .arg(m_armedCadDescription)
                .arg(toDisplayUnit(m_armedCadLocationMm.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(m_armedCadLocationMm.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(m_armedCadLocationMm.z(), m_unitSystem), 0, 'f', decimals)
                .arg(unitStr));
        }
    }

    void MainWindow::onPositionUpdated(const Position3D& pos) {
        m_lastPosition = pos;
        updateDroDisplay();
        if (m_viewport) { m_viewport->updateProbePosition(pos.x, pos.y, pos.z); }
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
        m_recordedHits.push_back(Eigen::Vector3d(hitPos.x, hitPos.y, hitPos.z));
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
    }

    void MainWindow::finishFeatureBuffer() {
        m_recordedHits.clear();
        m_hitCountLabel->setText("Active Feature Hits: 0");
        while (m_activePointsGroup->childCount() > 0) {
            delete m_activePointsGroup->takeChild(0);
        }
        m_activePointsGroup->setText(2, "0 pts");
        if (m_viewport) { m_viewport->commitActiveHitMarkers(); }
    }
    void MainWindow::onExportReportClicked() {
        if (m_featureTree->topLevelItemCount() <= 1) {
            QMessageBox::information(this, "No Features to Export", "Please measure and solve at least one feature before exporting an inspection report.");
            return;
        }

        bool ok = false;
        QString partName = QInputDialog::getText(
            this,
            "Inspection Report Header",
            "Enter Part / Job Number / Description:",
            QLineEdit::Normal,
            "JLS-PART-001",
            &ok
        );
        if (!ok) return;
        if (partName.trimmed().isEmpty()) partName = "JLS-PART-001";

        QString selectedFilter;
        QString filePath = QFileDialog::getSaveFileName(
            this,
            "Export Inspection Report",
            partName + "_Inspection_Report.html",
            "HTML Printable Inspection Report (*.html);;CSV Spreadsheet (*.csv);;QIF 2.1 Quality Information Framework (*.qif)",
            &selectedFilter
        );
        if (filePath.isEmpty()) return;

        // --- 1. QIF 2.1 XML Export ---
        if (filePath.endsWith(".qif", Qt::CaseInsensitive)) {
            std::vector<InspectionResultItem> qifItems;

            for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                if (top == m_activePointsGroup) continue;

                InspectionResultItem item;
                item.featureName = top->text(0);
                item.refId = "REFID_" + QString::number(i);
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

        PlaneFitResult res = MetrologyMath::fitPlaneLeastSquares(
            m_recordedHits,
            m_effectiveProbeRadius,
            currentCompMode()
        );

        if (!res.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Fit Failed", "Singular point distribution — could not solve 3D Plane.");
            return;
        }

        m_lastFittedPlaneFrame = WorkplaneFrame::fromPlane(res.centroid, res.normal);
        m_hasCustomPlane = true;

        const int id = m_featureCounter++;
        const QString featName = QString("Plane #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Plane3D;
        mf.location = res.centroid;
        mf.primaryVector = res.normal;
        mf.diameter = 0.0;
        mf.formError = res.flatness;
        m_solvedFeatures.push_back(mf);

        const bool passed = (res.flatness <= m_defaultTolMm);
        playSolveChime(passed);

        auto* planeItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
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
        const QString featName = QString("Sphere #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Sphere3D;
        mf.location = res.center;
        mf.diameter = actualDiam;
        mf.formError = res.formError;
        m_solvedFeatures.push_back(mf);

        const bool passed = (std::abs(actualDiam - nominalDiam) <= m_defaultTolMm);
        playSolveChime(passed);

        auto* sphereItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
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
        const QString featName = QString("Circle #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Circle2D;
        mf.location = res.center;
        mf.primaryVector = res.planeNormal;
        mf.diameter = actualDiam;
        mf.formError = res.circularity;
        m_solvedFeatures.push_back(mf);

        const bool passed = (std::abs(actualDiam - nominalDiam) <= m_defaultTolMm);
        playSolveChime(passed);

        auto* circleItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
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
        const QString featName = QString("Line #%1").arg(id);

        MeasuredFeature mf;
        mf.id = id;
        mf.name = featName.toStdString();
        mf.kind = FeatureKind::Line2D;
        mf.location = res.centroid;
        mf.primaryVector = res.direction;
        mf.secondaryVector = res.edgeNormal;
        mf.diameter = 0.0;
        mf.formError = res.straightness;
        m_solvedFeatures.push_back(mf);

        const bool passed = (res.straightness <= m_defaultTolMm);
        playSolveChime(passed);

        auto* lineItem = new QTreeWidgetItem(m_featureTree, { featName, "", "", "", "" });
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

        Alignment321Result alignRes = MetrologyMath::compute321Alignment(levelFeat, rotateFeat, originFeat);
        if (!alignRes.success) {
            playSolveChime(false);
            QMessageBox::critical(this, "Alignment Failed",
                "Could not build an orthogonal 3-2-1 coordinate frame from the selected features.");
            return;
        }

        m_pcsMatrix = alignRes.transformationMatrix;
        m_isPartAligned = true;

        if (m_alignStatusBtn) {
            m_alignStatusBtn->setChecked(true);
            m_alignStatusBtn->setText("🌐\nPCS (Aligned)");
            m_alignStatusBtn->setStyleSheet(
                "QPushButton { background-color: #1e4620; color: #98c379; border: 1px solid #98c379; "
                "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }");
        }

        updateDroDisplay();
        playSolveChime(true);
        statusBar()->showMessage("3-2-1 Part Coordinate System (PCS) active.");
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

        QMenu contextMenu(this);
        QAction* deleteAction = contextMenu.addAction("🗑 Delete Feature");

        QAction* selectedAction = contextMenu.exec(m_featureTree->mapToGlobal(pos));
        if (selectedAction == deleteAction) {
            onDeleteSelectedFeature();
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

        // 1. Clear solved feature data model
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