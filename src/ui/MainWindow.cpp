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

        setupUi();

        // --- Load QSettings Persistence on Startup ---
        QString savedMachinePath, savedPort;
        int savedBaud = 115200;
        Eigen::Vector3d savedScale(0.001, 0.001, 0.001);
        Eigen::Vector3i savedSign(1, 1, 1);
        AppSettings::loadSettings(
            savedMachinePath, savedPort, savedBaud,
            savedScale, savedSign,
            m_unitSystem, m_defaultTolMm, m_audioEnabled
        );

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
        // ---------------------------------------------

        connect(m_controller, &PicoSerialController::positionUpdated,
            this, &MainWindow::onPositionUpdated);
        connect(m_controller, &PicoSerialController::probeTriggered,
            this, &MainWindow::onProbeHit);
        connect(m_controller, &PicoSerialController::connectionStatusChanged,
            this, &MainWindow::onConnectionChanged);

        updateProbeStatusDisplay();
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
        btn->setStyleSheet(QString(
            "QPushButton { background-color: %1; color: %3; border: 1px solid %2; "
            "border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold; }"
            "QPushButton:hover { background-color: #3e4451; border: 1px solid #61afef; color: #ffffff; }"
            "QPushButton:pressed { background-color: #181a1f; }"
        ).arg(accentBg, borderCol, textCol));
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
            }), "Probe && Alignment");

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

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("Active Projection",{ makeStackedControl("2D Workplane:", m_workplaneCombo) }),
            createRibbonGroup("3D Features",{ fitPlaneBtn, fitSphereBtn }),
            createRibbonGroup("2D Workplane Features",{ fitCircleBtn, fitLineBtn }),
            createRibbonGroup("Dimensions & Relations",{ relBtn })
            }), "Measure");

        auto* loadStepBtn = createRibbonButton("📦", "Load STEP", "#2d5a88", "#61afef", "#ffffff");
        connect(loadStepBtn, &QPushButton::clicked, this, &MainWindow::onLoadStepClicked);
        auto* transformCadBtn = createRibbonButton("🧭", "Orient CAD...", "#3b6998", "#61afef", "#ffffff");
        connect(transformCadBtn, &QPushButton::clicked, this, &MainWindow::onTransformCadClicked);
        auto* cadDevBtn = createRibbonButton("🌡", "Surface Dev", "#0d9488", "#4fd1c5", "#ffffff");
        connect(cadDevBtn, &QPushButton::clicked, this, &MainWindow::onEvaluateCadDeviationClicked);

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("3D CAD Model",{ loadStepBtn, transformCadBtn }),
            createRibbonGroup("Point Cloud vs. CAD",{ cadDevBtn })
            }), "CAD && Deviation");

        auto* editTolBtn = createRibbonButton("±", "Edit Tol...", "#2d3748", "#e5c07b", "#e5c07b");
        connect(editTolBtn, &QPushButton::clicked, this, &MainWindow::onEditToleranceClicked);
        auto* exportBtn = createRibbonButton("📄", "Export Report", "#1f7a8c", "#61afef", "#ffffff");
        connect(exportBtn, &QPushButton::clicked, this, &MainWindow::onExportReportClicked);

        ribbonTabs->addTab(makeRibbonPage({
            createRibbonGroup("Feature Evaluation",{ editTolBtn }),
            createRibbonGroup("Quality Documentation",{ exportBtn })
            }), "Reporting");

        setMenuWidget(ribbonTabs);
    }

    void MainWindow::setupDroDock() {
        auto* droDock = new QDockWidget("LIVE CMM DRO & PROBE STATUS", this);
        auto* container = new QWidget(droDock);
        container->setStyleSheet("background-color: #181a1f;");
        auto* layout = new QVBoxLayout(container);
        auto* grid = new QGridLayout();

        const char* axisNames[3] = { "X", "Y", "Z" };
        const char* axisColors[3] = { "#e06c75", "#98c379", "#61afef" };
        QLabel** droPtrs[3] = { &m_droX, &m_droY, &m_droZ };

        for (int i = 0; i < 3; ++i) {
            auto* axisLbl = new QLabel(axisNames[i], container);
            axisLbl->setStyleSheet(QString("color: %1; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace;").arg(axisColors[i]));

            *droPtrs[i] = new QLabel("   0.0000", container);
            (*droPtrs[i])->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            (*droPtrs[i])->setStyleSheet("color: #ffffff; background-color: #0d0f12; border: 1px solid #2c313a; border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace; min-width: 190px;");

            m_unitLabels[i] = new QLabel("mm", container);
            m_unitLabels[i]->setStyleSheet("color: #5c6370; font-size: 16px; font-weight: bold;");

            grid->addWidget(axisLbl, i, 0);
            grid->addWidget(*droPtrs[i], i, 1);
            grid->addWidget(m_unitLabels[i], i, 2);
        }

        layout->addLayout(grid);
        m_hitCountLabel = new QLabel("Active Feature Hits: 0", container);
        layout->addWidget(m_hitCountLabel);

        auto* probeGroup = new QGroupBox("ACTIVE PROBE CALIBRATION", container);
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

        droDock->setWidget(container);
        addDockWidget(Qt::LeftDockWidgetArea, droDock);
    }

    void MainWindow::setupFeatureTreeDock() {
        auto* treeDock = new QDockWidget("INSPECTION FEATURE TREE", this);
        auto* container = new QWidget(treeDock);
        container->setStyleSheet("background-color: #181a1f;");
        auto* dockLayout = new QVBoxLayout(container);

        m_featureTree = new QTreeWidget(container);
        m_featureTree->setColumnCount(5);
        m_featureTree->setHeaderLabels({ "Feature / Parameter", "Nominal", "Actual", "±Tol", "Dev / Status" });
        m_activePointsGroup = new QTreeWidgetItem(m_featureTree, { "Active Probe Buffer", "", "0 pts", "", "READY" });
        m_activePointsGroup->setExpanded(true);

        dockLayout->addWidget(m_featureTree);
        treeDock->setWidget(container);
        addDockWidget(Qt::LeftDockWidgetArea, treeDock);
    }

    void MainWindow::onOpenSettingsClicked() {
        SettingsDialog dlg(this);

        dlg.setCurrentUnitSystem(m_unitSystem);
        dlg.setDefaultToleranceMm(m_defaultTolMm);
        dlg.setAudioChimesEnabled(m_audioEnabled);
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
            m_audioEnabled
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

    void MainWindow::onTransformCadClicked() {}
    void MainWindow::onCadFaceSelected(const CadFaceInfo&) {}

    void MainWindow::onPositionUpdated(const Position3D& pos) {
        m_lastPosition = pos;
        updateDroDisplay();
        if (m_viewport) { m_viewport->updateProbePosition(pos.x, pos.y, pos.z); }
    }

    void MainWindow::updateDroDisplay() {
        Eigen::Vector3d pos(m_lastPosition.x, m_lastPosition.y, m_lastPosition.z);
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

    void MainWindow::onSimulateHitClicked() {}
    void MainWindow::onCalibrateProbeClicked() {}
    void MainWindow::onFitSphereClicked() {}
    void MainWindow::onFitPlaneClicked() {}
    void MainWindow::onFitCircleClicked() {}
    void MainWindow::onFitLineClicked() {}
    void MainWindow::onMeasureRelationClicked() {}
    void MainWindow::onRun321AlignmentClicked() {}
    void MainWindow::onEditToleranceClicked() {}
    void MainWindow::onExportReportClicked() {}
    void MainWindow::onClearPointsClicked() { finishFeatureBuffer(); }
    void MainWindow::onEvaluateCadDeviationClicked() {}

} // namespace OpenCMM