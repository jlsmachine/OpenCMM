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

#include <QToolBar>
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
        // Custom data roles stored on top-level feature QTreeWidgetItems (column 0)
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

        connect(m_controller, &PicoSerialController::positionUpdated,
            this, &MainWindow::onPositionUpdated);
        connect(m_controller, &PicoSerialController::probeTriggered,
            this, &MainWindow::onProbeHit);
        connect(m_controller, &PicoSerialController::connectionStatusChanged,
            this, &MainWindow::onConnectionChanged);

        updateProbeStatusDisplay();
    }

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

    void MainWindow::setupToolbar() {
        const QString barStyle =
            "QToolBar { background-color: #21252b; border-bottom: 1px solid #3a3f4b; spacing: 8px; padding: 5px; }"
            "QPushButton { background-color: #3a3f4b; color: white; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #4b5263; }"
            "QComboBox { background-color: #1e2227; color: white; border: 1px solid #4b5263; padding: 4px 8px; font-weight: bold; }"
            "QComboBox QAbstractItemView { background-color: #1e2227; color: white; selection-background-color: #2d5a88; }";

        // Row 1: Settings, Zero XYZ, CAD Controls, Report Export & Simulator
        auto* hwToolbar = addToolBar("Studio & CAD Controls");
        hwToolbar->setMovable(false);
        hwToolbar->setStyleSheet(barStyle);

        m_settingsBtn = new QPushButton("⚙ Settings...", this);
        m_settingsBtn->setStyleSheet(
            "background-color: #282c34; color: #61afef; border: 1px solid #61afef; "
            "border-radius: 3px; padding: 5px 12px; font-weight: bold;"
        );
        m_settingsBtn->setToolTip("Configure Machine Post (.cmm), Pico 2 Serial Port, Units (MM/Inch), Default Tolerances, and Audio");
        connect(m_settingsBtn, &QPushButton::clicked, this, &MainWindow::onOpenSettingsClicked);
        hwToolbar->addWidget(m_settingsBtn);

        hwToolbar->addSeparator();

        auto* zeroBtn = new QPushButton("Zero XYZ", this);
        connect(zeroBtn, &QPushButton::clicked, this, &MainWindow::onZeroAxesClicked);
        hwToolbar->addWidget(zeroBtn);

        hwToolbar->addSeparator();

        auto* loadStepBtn = new QPushButton("Load STEP CAD", this);
        loadStepBtn->setStyleSheet("background-color: #2d5a88; color: white; padding: 5px 12px; font-weight: bold;");
        connect(loadStepBtn, &QPushButton::clicked, this, &MainWindow::onLoadStepClicked);
        hwToolbar->addWidget(loadStepBtn);

        auto* transformCadBtn = new QPushButton("Orient CAD...", this);
        transformCadBtn->setStyleSheet("background-color: #3b6998; color: white; padding: 5px 12px; font-weight: bold;");
        connect(transformCadBtn, &QPushButton::clicked, this, &MainWindow::onTransformCadClicked);
        hwToolbar->addWidget(transformCadBtn);

        auto* simLabel = new QLabel(" Simulator: ", this);
        simLabel->setStyleSheet("color: #abb2bf; font-weight: bold;");
        hwToolbar->addWidget(simLabel);

        m_simTargetCombo = new QComboBox(this);
        m_simTargetCombo->addItem("Master Ball Ø25.4mm", 0);
        m_simTargetCombo->addItem("Bore #1 Ø20.0mm (50,30)", 1);
        m_simTargetCombo->addItem("Bore #2 Ø12.0mm (100,30)", 2);
        m_simTargetCombo->addItem("Top Datum Plane (Z=20)", 3);
        m_simTargetCombo->addItem("Front X-Edge (Y=0)", 4);
        m_simTargetCombo->addItem("Left Y-Edge (X=0)", 5);
        hwToolbar->addWidget(m_simTargetCombo);

        auto* simHitBtn = new QPushButton("Simulate Probe Hit", this);
        simHitBtn->setStyleSheet("background-color: #d19a66; color: #1e2227; padding: 5px 12px; font-weight: bold;");
        connect(simHitBtn, &QPushButton::clicked, this, &MainWindow::onSimulateHitClicked);
        hwToolbar->addWidget(simHitBtn);

        // Row 2: Metrology Workplane, Tip Compensation, Solvers & Alignment
        addToolBarBreak(Qt::TopToolBarArea);
        auto* metroToolbar = addToolBar("Metrology Feature Solvers");
        metroToolbar->setMovable(false);
        metroToolbar->setStyleSheet(barStyle);

        auto* wpLabel = new QLabel(" Workplane: ", this);
        wpLabel->setStyleSheet("color: #abb2bf; font-weight: bold;");
        metroToolbar->addWidget(wpLabel);

        m_workplaneCombo = new QComboBox(this);
        m_workplaneCombo->addItem("XY (+Z)", static_cast<int>(StandardWorkplane::XY_PlusZ));
        m_workplaneCombo->addItem("YZ (+X)", static_cast<int>(StandardWorkplane::YZ_PlusX));
        m_workplaneCombo->addItem("ZX (+Y)", static_cast<int>(StandardWorkplane::ZX_PlusY));
        m_workplaneCombo->addItem("Last Fitted Plane", static_cast<int>(StandardWorkplane::CustomPlane));
        metroToolbar->addWidget(m_workplaneCombo);

        auto* compLabel = new QLabel(" Tip Comp: ", this);
        compLabel->setStyleSheet("color: #abb2bf; font-weight: bold;");
        metroToolbar->addWidget(compLabel);

        m_compModeCombo = new QComboBox(this);
        m_compModeCombo->addItem("External (-R)", static_cast<int>(ProbeCompMode::External));
        m_compModeCombo->addItem("Internal (+R)", static_cast<int>(ProbeCompMode::Internal));
        m_compModeCombo->addItem("Uncomp (0)", static_cast<int>(ProbeCompMode::Uncompensated));
        metroToolbar->addWidget(m_compModeCombo);

        auto* calBtn = new QPushButton("Calibrate Probe", this);
        calBtn->setStyleSheet("background-color: #8957e5; color: white; padding: 5px 10px; font-weight: bold;");
        connect(calBtn, &QPushButton::clicked, this, &MainWindow::onCalibrateProbeClicked);
        metroToolbar->addWidget(calBtn);

        metroToolbar->addSeparator();

        auto* fitSphereBtn = new QPushButton("Fit Sphere", this);
        connect(fitSphereBtn, &QPushButton::clicked, this, &MainWindow::onFitSphereClicked);
        metroToolbar->addWidget(fitSphereBtn);

        auto* fitPlaneBtn = new QPushButton("Fit Plane", this);
        connect(fitPlaneBtn, &QPushButton::clicked, this, &MainWindow::onFitPlaneClicked);
        metroToolbar->addWidget(fitPlaneBtn);

        auto* fitCircleBtn = new QPushButton("Fit 2D Circle", this);
        fitCircleBtn->setStyleSheet("background-color: #2b6cb0; color: white; padding: 5px 10px; font-weight: bold;");
        connect(fitCircleBtn, &QPushButton::clicked, this, &MainWindow::onFitCircleClicked);
        metroToolbar->addWidget(fitCircleBtn);

        auto* fitLineBtn = new QPushButton("Fit 2D Line", this);
        fitLineBtn->setStyleSheet("background-color: #2b6cb0; color: white; padding: 5px 10px; font-weight: bold;");
        connect(fitLineBtn, &QPushButton::clicked, this, &MainWindow::onFitLineClicked);
        metroToolbar->addWidget(fitLineBtn);

        auto* relBtn = new QPushButton("Measure Distance", this);
        relBtn->setStyleSheet("background-color: #2f855a; color: white; padding: 5px 10px; font-weight: bold;");
        connect(relBtn, &QPushButton::clicked, this, &MainWindow::onMeasureRelationClicked);
        metroToolbar->addWidget(relBtn);

        metroToolbar->addSeparator();

        auto* alignBtn = new QPushButton("3-2-1 Alignment...", this);
        alignBtn->setStyleSheet("background-color: #d15757; color: white; padding: 5px 10px; font-weight: bold;");
        connect(alignBtn, &QPushButton::clicked, this, &MainWindow::onRun321AlignmentClicked);
        metroToolbar->addWidget(alignBtn);

        m_alignStatusBtn = new QPushButton("MCS", this);
        m_alignStatusBtn->setCheckable(true);
        m_alignStatusBtn->setChecked(false);
        m_alignStatusBtn->setToolTip("Toggle between Machine Coordinate System (MCS) and 3-2-1 Part Coordinate System (PCS)");
        m_alignStatusBtn->setStyleSheet("background-color: #3a3f4b; color: #e5c07b; padding: 5px 10px; font-weight: bold;");
        connect(m_alignStatusBtn, &QPushButton::clicked, this, [this](bool checked) {
            if (!m_isPartAligned && checked) {
                QMessageBox::warning(this, "No Alignment Active", "Please run a 3-2-1 alignment first.");
                m_alignStatusBtn->setChecked(false);
                return;
            }
            updateDroDisplay();
            });
        metroToolbar->addWidget(m_alignStatusBtn);

        metroToolbar->addSeparator();

        auto* clearBtn = new QPushButton("Clear Points", this);
        connect(clearBtn, &QPushButton::clicked, this, &MainWindow::onClearPointsClicked);
        metroToolbar->addWidget(clearBtn);
    }

    void MainWindow::setupDroDock() {
        auto* droDock = new QDockWidget("LIVE CMM DRO & PROBE STATUS", this);
        droDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

        auto* container = new QWidget(droDock);
        container->setStyleSheet("background-color: #181a1f;");
        auto* layout = new QVBoxLayout(container);

        auto* grid = new QGridLayout();
        grid->setVerticalSpacing(10);

        const char* axisNames[3] = { "X", "Y", "Z" };
        const char* axisColors[3] = { "#e06c75", "#98c379", "#61afef" };
        QLabel** droPtrs[3] = { &m_droX, &m_droY, &m_droZ };

        for (int i = 0; i < 3; ++i) {
            auto* axisLbl = new QLabel(axisNames[i], container);
            axisLbl->setStyleSheet(QString("color: %1; font-size: 28px; font-weight: bold; font-family: 'Consolas', monospace;").arg(axisColors[i]));

            *droPtrs[i] = new QLabel("   0.0000", container);
            (*droPtrs[i])->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            (*droPtrs[i])->setStyleSheet(
                "color: #ffffff; background-color: #0d0f12; border: 1px solid #2c313a; "
                "border-radius: 4px; padding: 6px 12px; font-size: 28px; font-weight: bold; "
                "font-family: 'Consolas', monospace; min-width: 190px;"
            );

            m_unitLabels[i] = new QLabel("mm", container);
            m_unitLabels[i]->setStyleSheet("color: #5c6370; font-size: 16px; font-weight: bold;");

            grid->addWidget(axisLbl, i, 0);
            grid->addWidget(*droPtrs[i], i, 1);
            grid->addWidget(m_unitLabels[i], i, 2);
        }

        layout->addLayout(grid);

        m_hitCountLabel = new QLabel("Active Feature Hits: 0", container);
        m_hitCountLabel->setStyleSheet("color: #61afef; font-size: 14px; font-weight: bold; margin-top: 6px;");
        layout->addWidget(m_hitCountLabel);

        auto* probeGroup = new QGroupBox("ACTIVE PROBE CALIBRATION", container);
        auto* probeLayout = new QVBoxLayout(probeGroup);
        probeLayout->setSpacing(6);

        m_probeStatusBadge = new QLabel("UNINITIALIZED (Nominal Default)", probeGroup);
        m_probeStatusBadge->setStyleSheet("color: #e5c07b; font-size: 12px; font-weight: bold;");

        m_probeDiamLabel = new QLabel("Effective Tip Ø: 3.0000 mm", probeGroup);
        m_probeDiamLabel->setStyleSheet("color: #dcdfe4; font-family: 'Consolas', monospace; font-size: 13px;");

        m_probeRadLabel = new QLabel("Effective Radius (R_eff): 1.5000 mm", probeGroup);
        m_probeRadLabel->setStyleSheet("color: #dcdfe4; font-family: 'Consolas', monospace; font-size: 13px;");

        m_probeLobingLabel = new QLabel("Sphericity / Lobing: --", probeGroup);
        m_probeLobingLabel->setStyleSheet("color: #abb2bf; font-family: 'Consolas', monospace; font-size: 13px;");

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
        treeDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);

        auto* container = new QWidget(treeDock);
        container->setStyleSheet("background-color: #181a1f;");
        auto* dockLayout = new QVBoxLayout(container);
        dockLayout->setContentsMargins(4, 4, 4, 4);
        dockLayout->setSpacing(4);

        // Action bar right above the tree table
        auto* actionBar = new QHBoxLayout();
        actionBar->setSpacing(6);

        auto* editTolBtn = new QPushButton("Edit Nominal / Tol... (Ctrl+T)", container);
        editTolBtn->setStyleSheet("background-color: #2d3748; color: #e5c07b; border: 1px solid #4b5263; border-radius: 3px; padding: 4px 10px; font-weight: bold; font-size: 11px;");
        editTolBtn->setShortcut(QKeySequence("Ctrl+T"));
        connect(editTolBtn, &QPushButton::clicked, this, &MainWindow::onEditToleranceClicked);
        actionBar->addWidget(editTolBtn);

        auto* exportDockBtn = new QPushButton("Export Report... (Ctrl+E)", container);
        exportDockBtn->setStyleSheet("background-color: #1f7a8c; color: white; border: 1px solid #4b5263; border-radius: 3px; padding: 4px 10px; font-weight: bold; font-size: 11px;");
        connect(exportDockBtn, &QPushButton::clicked, this, &MainWindow::onExportReportClicked);
        actionBar->addWidget(exportDockBtn);

        dockLayout->addLayout(actionBar);

        m_featureTree = new QTreeWidget(container);
        m_featureTree->setColumnCount(5);
        m_featureTree->setHeaderLabels({ "Feature / Parameter", "Nominal", "Actual", "±Tol", "Dev / Status" });
        m_featureTree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        m_featureTree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
        m_featureTree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        m_featureTree->header()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        m_featureTree->header()->setSectionResizeMode(4, QHeaderView::Stretch);
        m_featureTree->setMinimumWidth(460);
        m_featureTree->setStyleSheet(
            "QTreeWidget { background-color: #181a1f; color: #dcdfe4; border: 1px solid #282c34; font-family: 'Segoe UI', sans-serif; font-size: 12px; }"
            "QHeaderView::section { background-color: #21252b; color: #abb2bf; padding: 5px; border: 1px solid #3a3f4b; font-weight: bold; }"
        );

        m_activePointsGroup = new QTreeWidgetItem(m_featureTree, { "Active Probe Buffer", "", "0 pts", "", "READY" });
        m_activePointsGroup->setExpanded(true);

        connect(m_featureTree, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* /*item*/, int /*col*/) {
            onEditToleranceClicked();
            });

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
                setWindowTitle(QString("OpenCMM - %1 (%2)").arg(cfg.machineName, cfg.manufacturer));
                statusBar()->showMessage(QString("Loaded Machine Post: %1 (%2)").arg(cfg.machineName, path));
            }
            else {
                QMessageBox::warning(&dlg, "Machine Package Load Failed",
                    "Could not parse the selected .cmm / .cmm.json machine package. Verify the archive contains a valid JSON manifest.");
            }
            });

        connect(&dlg, &SettingsDialog::machinePackageReset, this, [this]() {
            m_machinePackagePath.clear();
            if (m_viewport) {
                m_viewport->resetDefaultMachine();
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

        if (dlg.exec() != QDialog::Accepted) return;

        UnitSystem newUnits = dlg.getSelectedUnitSystem();
        if (newUnits != m_unitSystem) {
            onUnitToggled();
        }

        m_defaultTolMm = dlg.getDefaultToleranceMm();
        m_audioEnabled = dlg.areAudioChimesEnabled();

        statusBar()->showMessage("Settings updated successfully.");
    }

    void MainWindow::playHitChime() const {
        if (!m_audioEnabled) return;
#ifdef _WIN32
        std::thread([]() {
            Beep(1250, 35);
            }).detach();
#endif
    }

    void MainWindow::playSolveChime(bool passed) const {
        if (!m_audioEnabled) return;
#ifdef _WIN32
        std::thread([passed]() {
            if (passed) {
                Beep(980, 40);
                Beep(1480, 65);
            }
            else {
                Beep(420, 90);
                Beep(340, 120);
            }
            }).detach();
#endif
    }

    void MainWindow::applyEvaluationToTreeItem(QTreeWidgetItem* item,
        double actualMm,
        bool hasNominal,
        double nominalMm,
        double plusTolMm,
        double minusTolMm) {
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

        if (std::abs(plusTolMm - minusTolMm) < 1e-6) {
            item->setText(3, QString("±%1").arg(toDisplayUnit(plusTolMm, m_unitSystem), 0, 'f', decimals));
        }
        else {
            item->setText(3, QString("+%1 / -%2")
                .arg(toDisplayUnit(plusTolMm, m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(minusTolMm, m_unitSystem), 0, 'f', decimals));
        }

        const double dispDev = toDisplayUnit(devMm, m_unitSystem);
        const QString signPrefix = (dispDev >= 0.0) ? "+" : "";
        const QString statusTag = passed ? "PASS" : "FAIL";

        item->setText(4, QString("%1%2 (%3)")
            .arg(signPrefix)
            .arg(dispDev, 0, 'f', decimals)
            .arg(statusTag));

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
            m_probeLobingLabel->setText(
                QString("Sphericity / Lobing: %1%2")
                .arg(toDisplayUnit(m_lastProbeLobingMm, m_unitSystem), 0, 'f', decimals)
                .arg(unitStr)
            );
        }
        else {
            m_probeStatusBadge->setText("DEFAULT NOMINAL (Uncalibrated)");
            m_probeStatusBadge->setStyleSheet("color: #e5c07b; font-size: 12px; font-weight: bold;");
            m_probeLobingLabel->setText("Sphericity / Lobing: --");
        }

        m_probeDiamLabel->setText(
            QString("Effective Tip Ø: %1%2")
            .arg(toDisplayUnit(m_effectiveProbeRadius * 2.0, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr)
        );
        m_probeRadLabel->setText(
            QString("Effective Radius (R_eff): %1%2")
            .arg(toDisplayUnit(m_effectiveProbeRadius, m_unitSystem), 0, 'f', decimals)
            .arg(unitStr)
        );
    }

    void MainWindow::onConnectionChanged(bool connected, const QString& message) {
        if (connected) {
            if (m_settingsBtn) {
                m_settingsBtn->setText("⚙ Settings (Connected)");
                m_settingsBtn->setStyleSheet(
                    "background-color: #1e3a29; color: #98c379; border: 1px solid #98c379; "
                    "border-radius: 3px; padding: 5px 12px; font-weight: bold;"
                );
            }
            statusBar()->setStyleSheet("background-color: #28a745; color: white; font-weight: bold;");
        }
        else {
            if (m_settingsBtn) {
                m_settingsBtn->setText("⚙ Settings...");
                m_settingsBtn->setStyleSheet(
                    "background-color: #282c34; color: #61afef; border: 1px solid #61afef; "
                    "border-radius: 3px; padding: 5px 12px; font-weight: bold;"
                );
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
            for (auto* lbl : m_unitLabels) {
                if (lbl) lbl->setText("in");
            }
        }
        else {
            m_unitSystem = UnitSystem::Millimeters;
            for (auto* lbl : m_unitLabels) {
                if (lbl) lbl->setText("mm");
            }
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
        QString filePath = QFileDialog::getOpenFileName(
            this,
            "Load STEP CAD Model",
            QString(),
            "STEP Files (*.stp *.step *.STP *.STEP)"
        );
        if (!filePath.isEmpty() && m_viewport) {
            m_viewport->loadStepFile(filePath);
            statusBar()->showMessage("Loaded STEP model: " + filePath + " (Click any CAD face to arm nominal geometry)");
        }
    }

    void MainWindow::onTransformCadClicked() {
        if (!m_viewport || !m_viewport->hasCadModel()) {
            QMessageBox::information(
                this,
                "No STEP Model Loaded",
                "Please load a STEP CAD model first before opening the Transform CAD tool."
            );
            return;
        }

        auto* dlg = new QDialog(this, Qt::Tool);
        dlg->setAttribute(Qt::WA_DeleteOnClose);
        dlg->setWindowTitle("Transform & Orient CAD Model");
        dlg->setMinimumWidth(440);
        dlg->setStyleSheet(
            "QDialog { background-color: #21252b; color: #e0e0e0; }"
            "QGroupBox { border: 1px solid #3a3f4b; border-radius: 4px; margin-top: 8px; padding-top: 10px; color: #61afef; font-weight: bold; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }"
            "QLabel { color: #dcdfe4; font-size: 12px; font-weight: bold; }"
            "QPushButton { background-color: #3a3f4b; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #4b5263; }"
            "QDoubleSpinBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 4px; font-weight: bold; }"
            "QCheckBox { color: #dcdfe4; font-weight: bold; }"
        );

        auto* mainLayout = new QVBoxLayout(dlg);
        mainLayout->setSpacing(10);

        auto* bboxGroup = new QGroupBox("CURRENT CAD BOUNDING BOX", dlg);
        auto* bboxLayout = new QVBoxLayout(bboxGroup);
        auto* bboxLabel = new QLabel(bboxGroup);
        bboxLabel->setStyleSheet("font-family: 'Consolas', monospace; font-size: 12px; color: #98c379;");
        bboxLayout->addWidget(bboxLabel);
        mainLayout->addWidget(bboxGroup);

        auto updateBboxText = [this, bboxLabel]() {
            double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
            if (!m_viewport->getCadBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)) {
                bboxLabel->setText("No bounding box available.");
                return;
            }
            const int dec = (m_unitSystem == UnitSystem::Millimeters) ? 3 : 4;
            const QString u = (m_unitSystem == UnitSystem::Millimeters) ? "mm" : "in";
            bboxLabel->setText(
                QString("X: [%1 .. %2] %7\nY: [%3 .. %4] %7\nZ: [%5 .. %6] %7")
                .arg(toDisplayUnit(xmin, m_unitSystem), 0, 'f', dec)
                .arg(toDisplayUnit(xmax, m_unitSystem), 0, 'f', dec)
                .arg(toDisplayUnit(ymin, m_unitSystem), 0, 'f', dec)
                .arg(toDisplayUnit(ymax, m_unitSystem), 0, 'f', dec)
                .arg(toDisplayUnit(zmin, m_unitSystem), 0, 'f', dec)
                .arg(toDisplayUnit(zmax, m_unitSystem), 0, 'f', dec)
                .arg(u)
            );
            };
        updateBboxText();

        auto* rotGroup = new QGroupBox("ROTATE MODEL (ORIENTATION)", dlg);
        auto* rotLayout = new QVBoxLayout(rotGroup);

        auto* pivotCheck = new QCheckBox("Rotate around Part Bounding-Box Center", rotGroup);
        pivotCheck->setChecked(true);
        rotLayout->addWidget(pivotCheck);

        auto* grid90 = new QGridLayout();
        auto addRotBtn = [&](const QString& text, double rx, double ry, double rz, int row, int col) {
            auto* btn = new QPushButton(text, rotGroup);
            connect(btn, &QPushButton::clicked, dlg, [this, rx, ry, rz, pivotCheck, updateBboxText]() {
                m_viewport->rotateCadModels(rx, ry, rz, pivotCheck->isChecked());
                updateBboxText();
                });
            grid90->addWidget(btn, row, col);
            };

        addRotBtn("Rot X -90°", -90.0, 0.0, 0.0, 0, 0);
        addRotBtn("Rot X +90°", +90.0, 0.0, 0.0, 0, 1);
        addRotBtn("Rot Y -90°", 0.0, -90.0, 0.0, 1, 0);
        addRotBtn("Rot Y +90°", 0.0, +90.0, 0.0, 1, 1);
        addRotBtn("Rot Z -90°", 0.0, 0.0, -90.0, 2, 0);
        addRotBtn("Rot Z +90°", 0.0, 0.0, +90.0, 2, 1);
        rotLayout->addLayout(grid90);

        auto* customRotRow = new QHBoxLayout();
        auto* spinRx = new QDoubleSpinBox(rotGroup);
        auto* spinRy = new QDoubleSpinBox(rotGroup);
        auto* spinRz = new QDoubleSpinBox(rotGroup);
        for (auto* sp : { spinRx, spinRy, spinRz }) {
            sp->setRange(-360.0, 360.0);
            sp->setDecimals(2);
            sp->setSuffix("°");
        }
        customRotRow->addWidget(new QLabel("RX:", rotGroup));
        customRotRow->addWidget(spinRx);
        customRotRow->addWidget(new QLabel("RY:", rotGroup));
        customRotRow->addWidget(spinRy);
        customRotRow->addWidget(new QLabel("RZ:", rotGroup));
        customRotRow->addWidget(spinRz);

        auto* applyRotBtn = new QPushButton("Apply ∠", rotGroup);
        applyRotBtn->setStyleSheet("background-color: #2b6cb0; color: white;");
        connect(applyRotBtn, &QPushButton::clicked, dlg, [this, spinRx, spinRy, spinRz, pivotCheck, updateBboxText]() {
            m_viewport->rotateCadModels(spinRx->value(), spinRy->value(), spinRz->value(), pivotCheck->isChecked());
            updateBboxText();
            });
        customRotRow->addWidget(applyRotBtn);
        rotLayout->addLayout(customRotRow);

        auto* faceAlignRow = new QHBoxLayout();
        auto* levelTopBtn = new QPushButton("Level Selected Face -> +Z (Top)", rotGroup);
        levelTopBtn->setStyleSheet("background-color: #8957e5; color: white;");
        connect(levelTopBtn, &QPushButton::clicked, dlg, [this, dlg, updateBboxText]() {
            if (!m_viewport->alignSelectedFaceToZ(true)) {
                QMessageBox::information(dlg, "Select a CAD Face First", "Click any planar face on the STEP model in the 3D viewport, then click this button to level it to +Z.");
                return;
            }
            updateBboxText();
            statusBar()->showMessage("Leveled selected CAD face normal to +Z.");
            });

        auto* levelBotBtn = new QPushButton("Level Selected Face -> -Z (Bottom)", rotGroup);
        connect(levelBotBtn, &QPushButton::clicked, dlg, [this, dlg, updateBboxText]() {
            if (!m_viewport->alignSelectedFaceToZ(false)) {
                QMessageBox::information(dlg, "Select a CAD Face First", "Click any planar face on the STEP model in the 3D viewport, then click this button to orient it to -Z.");
                return;
            }
            updateBboxText();
            statusBar()->showMessage("Leveled selected CAD face normal to -Z.");
            });
        faceAlignRow->addWidget(levelTopBtn);
        faceAlignRow->addWidget(levelBotBtn);
        rotLayout->addLayout(faceAlignRow);

        mainLayout->addWidget(rotGroup);

        const bool isInch = (m_unitSystem == UnitSystem::Inches);
        const QString unitSuffix = isInch ? " in" : " mm";
        auto* transGroup = new QGroupBox(QString("TRANSLATE / SNAP MODEL (%1)").arg(isInch ? "INCHES" : "MM"), dlg);
        auto* transLayout = new QVBoxLayout(transGroup);

        auto* jogRow = new QHBoxLayout();
        auto* spinDx = new QDoubleSpinBox(transGroup);
        auto* spinDy = new QDoubleSpinBox(transGroup);
        auto* spinDz = new QDoubleSpinBox(transGroup);
        for (auto* sp : { spinDx, spinDy, spinDz }) {
            sp->setRange(-10000.0, 10000.0);
            sp->setDecimals(isInch ? 4 : 3);
            sp->setSingleStep(isInch ? 0.1000 : 5.000);
            sp->setSuffix(unitSuffix);
        }
        jogRow->addWidget(new QLabel("ΔX:", transGroup));
        jogRow->addWidget(spinDx);
        jogRow->addWidget(new QLabel("ΔY:", transGroup));
        jogRow->addWidget(spinDy);
        jogRow->addWidget(new QLabel("ΔZ:", transGroup));
        jogRow->addWidget(spinDz);

        auto* applyTransBtn = new QPushButton("Move XYZ", transGroup);
        applyTransBtn->setStyleSheet("background-color: #2b6cb0; color: white;");
        connect(applyTransBtn, &QPushButton::clicked, dlg, [this, spinDx, spinDy, spinDz, isInch, updateBboxText]() {
            const double scale = isInch ? MM_PER_INCH : 1.0;
            m_viewport->translateCadModels(spinDx->value() * scale, spinDy->value() * scale, spinDz->value() * scale);
            updateBboxText();
            });
        jogRow->addWidget(applyTransBtn);
        transLayout->addLayout(jogRow);

        auto* snapGrid = new QGridLayout();
        auto* dropGraniteBtn = new QPushButton("Drop Bottom to Granite (Z_min = 0)", transGroup);
        dropGraniteBtn->setStyleSheet("background-color: #2f855a; color: white;");
        connect(dropGraniteBtn, &QPushButton::clicked, dlg, [this, updateBboxText]() {
            m_viewport->dropCadModelsToGranite(0.0);
            updateBboxText();
            statusBar()->showMessage("Dropped CAD model bottom flush with Granite Table (Z = 0).");
            });

        auto* centerXyBtn = new QPushButton("Center XY on Origin (0, 0)", transGroup);
        connect(centerXyBtn, &QPushButton::clicked, dlg, [this, updateBboxText]() {
            m_viewport->centerCadModelsXY(0.0, 0.0);
            updateBboxText();
            statusBar()->showMessage("Centered CAD model bounding box at X = 0, Y = 0.");
            });

        auto* cornerOriginBtn = new QPushButton("Corner to Origin (X/Y/Z_min = 0)", transGroup);
        connect(cornerOriginBtn, &QPushButton::clicked, dlg, [this, updateBboxText]() {
            m_viewport->snapCadMinCornerToOrigin();
            updateBboxText();
            statusBar()->showMessage("Snapped CAD model front-left-bottom corner to (0, 0, 0).");
            });

        auto* resetPoseBtn = new QPushButton("Reset Original Pose", transGroup);
        resetPoseBtn->setStyleSheet("background-color: #d15757; color: white;");
        connect(resetPoseBtn, &QPushButton::clicked, dlg, [this, updateBboxText]() {
            m_viewport->resetCadModelsTransform();
            updateBboxText();
            statusBar()->showMessage("Reset CAD model to original imported STEP coordinates.");
            });

        snapGrid->addWidget(dropGraniteBtn, 0, 0);
        snapGrid->addWidget(centerXyBtn, 0, 1);
        snapGrid->addWidget(cornerOriginBtn, 1, 0);
        snapGrid->addWidget(resetPoseBtn, 1, 1);
        transLayout->addLayout(snapGrid);

        mainLayout->addWidget(transGroup);

        auto* closeBtn = new QPushButton("Close", dlg);
        connect(closeBtn, &QPushButton::clicked, dlg, &QDialog::close);
        mainLayout->addWidget(closeBtn);

        dlg->show();
    }

    void MainWindow::onCadFaceSelected(const CadFaceInfo& faceInfo) {
        if (!faceInfo.success) return;

        if (m_compModeCombo) {
            m_compModeCombo->setCurrentIndex(faceInfo.isInternal ? 1 : 0);
        }

        m_hasArmedCadNominal = true;
        m_armedCadDiameterMm = faceInfo.nominalDiameter;
        m_armedCadLocationMm = Eigen::Vector3d(
            faceInfo.centerOrCentroid.X(),
            faceInfo.centerOrCentroid.Y(),
            faceInfo.centerOrCentroid.Z()
        );
        m_armedCadDescription = QString::fromStdString(faceInfo.surfaceType);

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        if (faceInfo.nominalDiameter > 0.0) {
            statusBar()->showMessage(
                QString("CAD Nominal Armed [%1]: Ø %2%3 — Probe & fit feature to auto-evaluate Pass/Fail.")
                .arg(m_armedCadDescription)
                .arg(toDisplayUnit(faceInfo.nominalDiameter, m_unitSystem), 0, 'f', decimals)
                .arg(unitStr)
            );
        }
        else {
            statusBar()->showMessage(
                QString("CAD Face Selected [%1]: Normal (%2, %3, %4)")
                .arg(m_armedCadDescription)
                .arg(faceInfo.normalOrAxis.X(), 0, 'f', 4)
                .arg(faceInfo.normalOrAxis.Y(), 0, 'f', 4)
                .arg(faceInfo.normalOrAxis.Z(), 0, 'f', 4)
            );
        }
    }

    void MainWindow::onPositionUpdated(const Position3D& pos) {
        m_lastPosition = pos;
        updateDroDisplay();
        if (m_viewport) {
            m_viewport->updateProbePosition(pos.x, pos.y, pos.z);
        }
    }

    void MainWindow::updateDroDisplay() {
        Eigen::Vector3d pos(m_lastPosition.x, m_lastPosition.y, m_lastPosition.z);
        if (m_isPartAligned && m_alignStatusBtn && m_alignStatusBtn->isChecked()) {
            pos = MetrologyMath::transformPoint(pos, m_pcsMatrix);
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const double xVal = toDisplayUnit(pos.x(), m_unitSystem);
        const double yVal = toDisplayUnit(pos.y(), m_unitSystem);
        const double zVal = toDisplayUnit(pos.z(), m_unitSystem);

        m_droX->setText(QString::number(xVal, 'f', decimals));
        m_droY->setText(QString::number(yVal, 'f', decimals));
        m_droZ->setText(QString::number(zVal, 'f', decimals));
    }

    void MainWindow::onProbeHit(const Position3D& hitPos) {
        playHitChime();

        m_lastPosition = hitPos;
        updateDroDisplay();

        Eigen::Vector3d recordedPt(hitPos.x, hitPos.y, hitPos.z);
        if (m_isPartAligned && m_alignStatusBtn && m_alignStatusBtn->isChecked()) {
            recordedPt = MetrologyMath::transformPoint(recordedPt, m_pcsMatrix);
        }

        m_recordedHits.push_back(recordedPt);
        const int count = static_cast<int>(m_recordedHits.size());

        m_hitCountLabel->setText(QString("Active Feature Hits: %1").arg(count));
        m_activePointsGroup->setText(2, QString("%1 pts").arg(count));

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        QString coordsStr = QString("X: %1, Y: %2, Z: %3")
            .arg(toDisplayUnit(recordedPt.x(), m_unitSystem), 0, 'f', decimals)
            .arg(toDisplayUnit(recordedPt.y(), m_unitSystem), 0, 'f', decimals)
            .arg(toDisplayUnit(recordedPt.z(), m_unitSystem), 0, 'f', decimals);

        new QTreeWidgetItem(m_activePointsGroup, { QString("Hit #%1").arg(count), "", coordsStr, "", "" });

        if (m_viewport) {
            m_viewport->updateProbePosition(hitPos.x, hitPos.y, hitPos.z);
            m_viewport->addHitPointMarker(hitPos.x, hitPos.y, hitPos.z);
        }

        statusBar()->showMessage(QString("Probe hit #%1 captured at (%2)").arg(count).arg(coordsStr));
    }

    void MainWindow::finishFeatureBuffer() {
        m_recordedHits.clear();
        m_hitCountLabel->setText("Active Feature Hits: 0");
        while (m_activePointsGroup->childCount() > 0) {
            delete m_activePointsGroup->takeChild(0);
        }
        m_activePointsGroup->setText(2, "0 pts");

        if (m_viewport) {
            m_viewport->commitActiveHitMarkers();
        }
    }

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
        if (m_recordedHits.size() < 5) {
            QMessageBox::warning(
                this,
                "Probe Calibration Requires 5+ Hits",
                "Please capture at least 5 probe hits distributed across the Master Ball "
                "(4 around the equator + 1 on the top pole) before calibrating."
            );
            return;
        }

        const bool isInch = (m_unitSystem == UnitSystem::Inches);
        const int decimals = isInch ? 5 : 4;
        const QString unitStr = isInch ? " in" : " mm";
        const double defaultDispDiam = toDisplayUnit(m_certifiedBallDiameterMm, m_unitSystem);

        bool ok = false;
        double enteredDiam = QInputDialog::getDouble(
            this,
            "Master Ball Probe Calibration",
            QString("Enter Certified Master Ball Diameter (%1):").arg(isInch ? "Inches" : "Millimeters"),
            defaultDispDiam,
            isInch ? 0.10000 : 2.0000,
            isInch ? 10.00000 : 254.0000,
            decimals,
            &ok
        );

        if (!ok) return;

        m_certifiedBallDiameterMm = isInch ? (enteredDiam * MM_PER_INCH) : enteredDiam;

        ProbeCalibrationResult cal = MetrologyMath::calibrateProbeOnMasterBall(
            m_recordedHits,
            m_certifiedBallDiameterMm
        );

        if (!cal.success) {
            playSolveChime(false);
            QMessageBox::critical(
                this,
                "Calibration Failed",
                "Could not compute a valid effective probe radius. Verify that points are non-coplanar "
                "and that the certified Master Ball diameter is smaller than the probed center-path sphere."
            );
            return;
        }

        m_effectiveProbeRadius = cal.effectiveRadius;
        m_lastProbeLobingMm = cal.formError;
        m_isProbeCalibrated = true;
        updateProbeStatusDisplay();

        auto* calItem = new QTreeWidgetItem(m_featureTree, {
            QString("Probe_Cal (%1 pts)").arg(m_recordedHits.size()),
            QString("Ø %1%2").arg(toDisplayUnit(cal.certifiedDiameter, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            QString("R_eff: %1%2").arg(toDisplayUnit(cal.effectiveRadius, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "--",
            "CALIBRATED"
            });
        calItem->setForeground(4, QBrush(QColor("#98c379")));

        new QTreeWidgetItem(calItem, {
            "Effective Tip Diameter", "",
            QString("Ø %1%2").arg(toDisplayUnit(cal.effectiveRadius * 2.0, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "", ""
            });
        new QTreeWidgetItem(calItem, {
            "Master Ball Center (X, Y, Z)", "",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(cal.masterBallCenter.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(cal.masterBallCenter.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(cal.masterBallCenter.z(), m_unitSystem), 0, 'f', decimals),
            "", ""
            });
        new QTreeWidgetItem(calItem, {
            "3D Probe Lobing (Form Error)", "",
            QString("%1%2").arg(toDisplayUnit(cal.formError, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "", ""
            });

        calItem->setExpanded(true);
        finishFeatureBuffer();
        playSolveChime(true);
        statusBar()->showMessage("Probe calibrated on Master Ball. Active hit buffer reset for next feature.");
    }

    void MainWindow::onFitSphereClicked() {
        if (m_recordedHits.size() < 4) {
            statusBar()->showMessage("Need at least 4 probe hits to fit a 3D Sphere.");
            return;
        }

        const ProbeCompMode compMode = currentCompMode();
        SphereFitResult res = MetrologyMath::fitSphereLeastSquares(
            m_recordedHits,
            m_effectiveProbeRadius,
            compMode
        );

        if (!res.success) {
            playSolveChime(false);
            statusBar()->showMessage("Sphere least-squares fit failed (coplanar or singular points).");
            return;
        }

        const int id = m_featureCounter++;
        const QString featName = QString("Sphere_%1").arg(id);
        const double actualDiamMm = res.radius * 2.0;

        m_solvedFeatures.push_back({
            id,
            featName.toStdString(),
            FeatureKind::Sphere3D,
            res.center,
            Eigen::Vector3d::UnitZ(),
            Eigen::Vector3d::UnitY(),
            actualDiamMm,
            res.formError
            });

        bool hasNom = m_hasArmedCadNominal && (m_armedCadDiameterMm > 0.0);
        double nomMm = hasNom ? m_armedCadDiameterMm : m_certifiedBallDiameterMm;
        if (m_simTargetCombo && m_simTargetCombo->currentData().toInt() == 0 && !hasNom) {
            hasNom = true;
            nomMm = 25.4000;
        }
        m_hasArmedCadNominal = false;

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* sphereItem = new QTreeWidgetItem(m_featureTree);
        sphereItem->setText(0, QString("%1 Ø (%2 pts)").arg(featName).arg(m_recordedHits.size()));
        applyEvaluationToTreeItem(sphereItem, actualDiamMm, hasNom, nomMm, m_defaultTolMm, m_defaultTolMm);

        new QTreeWidgetItem(sphereItem, {
            "Center (X, Y, Z)", "",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.center.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.z(), m_unitSystem), 0, 'f', decimals),
            "", ""
            });
        new QTreeWidgetItem(sphereItem, {
            "Sphericity (Form Error)", "",
            QString("%1%2").arg(toDisplayUnit(res.formError, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "", ""
            });

        sphereItem->setExpanded(true);
        m_featureTree->setCurrentItem(sphereItem);
        finishFeatureBuffer();
        playSolveChime(true);
        statusBar()->showMessage(featName + " solved. Double-click row in tree to customize Nominal / Tolerance.");
    }

    void MainWindow::onFitPlaneClicked() {
        if (m_recordedHits.size() < 3) {
            statusBar()->showMessage("Need at least 3 probe hits to fit a Plane.");
            return;
        }

        const ProbeCompMode compMode = currentCompMode();
        Eigen::Vector3d currentMachinePos(m_lastPosition.x, m_lastPosition.y, m_lastPosition.z);

        PlaneFitResult res = MetrologyMath::fitPlaneLeastSquares(
            m_recordedHits,
            m_effectiveProbeRadius,
            compMode,
            &currentMachinePos
        );

        if (!res.success) {
            playSolveChime(false);
            statusBar()->showMessage("Plane SVD fit failed (collinear points).");
            return;
        }

        m_lastFittedPlaneFrame = WorkplaneFrame::fromPlane(res.centroid, res.normal);
        m_hasCustomPlane = true;

        const int id = m_featureCounter++;
        const QString featName = QString("Plane_%1").arg(id);

        m_solvedFeatures.push_back({
            id,
            featName.toStdString(),
            FeatureKind::Plane3D,
            res.centroid,
            res.normal,
            Eigen::Vector3d::UnitY(),
            0.0,
            res.flatness
            });

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;

        auto* planeItem = new QTreeWidgetItem(m_featureTree);
        planeItem->setText(0, QString("%1 Flatness (%2 pts)").arg(featName).arg(m_recordedHits.size()));
        applyEvaluationToTreeItem(planeItem, res.flatness, true, 0.0, m_defaultTolMm, 0.0);

        new QTreeWidgetItem(planeItem, {
            "Outward Normal (I, J, K)", "",
            QString("(%1, %2, %3)")
                .arg(res.normal.x(), 0, 'f', 5)
                .arg(res.normal.y(), 0, 'f', 5)
                .arg(res.normal.z(), 0, 'f', 5),
            "", ""
            });
        new QTreeWidgetItem(planeItem, {
            "Surface Centroid (X, Y, Z)", "",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.centroid.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.z(), m_unitSystem), 0, 'f', decimals),
            "", ""
            });

        planeItem->setExpanded(true);
        m_featureTree->setCurrentItem(planeItem);
        finishFeatureBuffer();
        playSolveChime(true);
        statusBar()->showMessage(featName + " solved and stored as custom workplane.");
    }

    void MainWindow::onFitCircleClicked() {
        if (m_recordedHits.size() < 3) {
            statusBar()->showMessage("Need at least 3 probe hits to fit a 2D Circle (Bore/Boss).");
            return;
        }

        ProbeCompMode compMode = currentCompMode();
        const int simMode = m_simTargetCombo ? m_simTargetCombo->currentData().toInt() : -1;
        if (simMode == 1 || simMode == 2) {
            compMode = ProbeCompMode::Internal;
            m_compModeCombo->setCurrentIndex(1);
        }

        const WorkplaneFrame wp = currentWorkplane();
        Circle2DFitResult res = MetrologyMath::fitCircle2DLeastSquares(
            m_recordedHits,
            wp,
            m_effectiveProbeRadius,
            compMode
        );

        if (!res.success) {
            playSolveChime(false);
            statusBar()->showMessage("2D Circle fit failed (collinear points in active workplane).");
            return;
        }

        const int id = m_featureCounter++;
        const QString prefix = (compMode == ProbeCompMode::Internal) ? "Bore" : "Boss";
        const QString featName = QString("%1_%2").arg(prefix).arg(id);
        const double actualDiamMm = res.radius * 2.0;

        m_solvedFeatures.push_back({
            id,
            featName.toStdString(),
            FeatureKind::Circle2D,
            res.center,
            res.planeNormal,
            Eigen::Vector3d::UnitY(),
            actualDiamMm,
            res.circularity
            });

        bool hasNom = m_hasArmedCadNominal && (m_armedCadDiameterMm > 0.0);
        double nomMm = hasNom ? m_armedCadDiameterMm : 0.0;
        if (!hasNom && simMode == 1) {
            hasNom = true;
            nomMm = 20.0000;
        }
        else if (!hasNom && simMode == 2) {
            hasNom = true;
            nomMm = 12.0000;
        }
        m_hasArmedCadNominal = false;

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* circleItem = new QTreeWidgetItem(m_featureTree);
        circleItem->setText(0, QString("%1 Ø (%2 pts)").arg(featName).arg(m_recordedHits.size()));
        applyEvaluationToTreeItem(circleItem, actualDiamMm, hasNom, nomMm, m_defaultTolMm, m_defaultTolMm);

        new QTreeWidgetItem(circleItem, {
            "Center (X, Y, Z)", "",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.center.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.z(), m_unitSystem), 0, 'f', decimals),
            "", ""
            });
        new QTreeWidgetItem(circleItem, {
            "Circularity (Roundness)", "",
            QString("%1%2").arg(toDisplayUnit(res.circularity, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "", ""
            });

        circleItem->setExpanded(true);
        m_featureTree->setCurrentItem(circleItem);
        finishFeatureBuffer();
        playSolveChime(true);
        statusBar()->showMessage(featName + " solved. Double-click row in tree to customize Nominal / Tolerance.");
    }

    void MainWindow::onFitLineClicked() {
        if (m_recordedHits.size() < 2) {
            statusBar()->showMessage("Need at least 2 probe hits to fit a 2D Line (Edge).");
            return;
        }

        const ProbeCompMode compMode = currentCompMode();
        const WorkplaneFrame wp = currentWorkplane();
        Eigen::Vector3d currentMachinePos(m_lastPosition.x, m_lastPosition.y, m_lastPosition.z);

        Line2DFitResult res = MetrologyMath::fitLine2DLeastSquares(
            m_recordedHits,
            wp,
            m_effectiveProbeRadius,
            compMode,
            &currentMachinePos
        );

        if (!res.success) {
            playSolveChime(false);
            statusBar()->showMessage("2D Line fit failed (coincident points).");
            return;
        }

        const int id = m_featureCounter++;
        const QString featName = QString("Line_%1").arg(id);

        m_solvedFeatures.push_back({
            id,
            featName.toStdString(),
            FeatureKind::Line2D,
            res.centroid,
            res.direction,
            res.edgeNormal,
            0.0,
            res.straightness
            });

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;

        auto* lineItem = new QTreeWidgetItem(m_featureTree);
        lineItem->setText(0, QString("%1 Straightness (%2 pts)").arg(featName).arg(m_recordedHits.size()));
        applyEvaluationToTreeItem(lineItem, res.straightness, true, 0.0, m_defaultTolMm, 0.0);

        new QTreeWidgetItem(lineItem, {
            "Edge Point (X, Y, Z)", "",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.centroid.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.z(), m_unitSystem), 0, 'f', decimals),
            "", ""
            });
        new QTreeWidgetItem(lineItem, {
            "Unit Direction (I, J, K)", "",
            QString("(%1, %2, %3)")
                .arg(res.direction.x(), 0, 'f', 5)
                .arg(res.direction.y(), 0, 'f', 5)
                .arg(res.direction.z(), 0, 'f', 5),
            "", ""
            });

        lineItem->setExpanded(true);
        m_featureTree->setCurrentItem(lineItem);
        finishFeatureBuffer();
        playSolveChime(true);
        statusBar()->showMessage(featName + " solved in active workplane.");
    }

    void MainWindow::onMeasureRelationClicked() {
        if (m_solvedFeatures.size() < 2) {
            QMessageBox::information(
                this,
                "Need 2+ Solved Features",
                "Please solve at least two features (e.g., two Bores, two Planes, or two Edges) "
                "before measuring a feature-to-feature relation."
            );
            return;
        }

        QDialog dlg(this);
        dlg.setWindowTitle("Measure Feature-to-Feature Relation");
        dlg.setMinimumWidth(380);
        dlg.setStyleSheet(
            "QDialog { background-color: #21252b; color: #e0e0e0; }"
            "QLabel { color: #ffffff; font-size: 13px; font-weight: bold; }"
            "QComboBox, QDoubleSpinBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 10px; font-size: 13px; font-weight: bold; min-width: 170px; }"
            "QComboBox QAbstractItemView { background-color: #181a1f; color: #ffffff; selection-background-color: #2d5a88; selection-color: #ffffff; }"
            "QPushButton { background-color: #3a3f4b; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 16px; font-weight: bold; }"
            "QPushButton:hover { background-color: #4b5263; }"
        );
        auto* form = new QFormLayout(&dlg);
        form->setContentsMargins(16, 16, 16, 16);
        form->setSpacing(12);

        auto* comboA = new QComboBox(&dlg);
        auto* comboB = new QComboBox(&dlg);

        for (size_t i = 0; i < m_solvedFeatures.size(); ++i) {
            QString label = QString::fromStdString(m_solvedFeatures[i].name);
            comboA->addItem(label, static_cast<int>(i));
            comboB->addItem(label, static_cast<int>(i));
        }
        comboA->setCurrentIndex(static_cast<int>(m_solvedFeatures.size()) - 2);
        comboB->setCurrentIndex(static_cast<int>(m_solvedFeatures.size()) - 1);

        const bool isInch = (m_unitSystem == UnitSystem::Inches);
        const int decimals = isInch ? 5 : 4;
        const QString unitStr = isInch ? " in" : " mm";

        auto* nomSpin = new QDoubleSpinBox(&dlg);
        nomSpin->setRange(0.0, 10000.0);
        nomSpin->setDecimals(decimals);
        nomSpin->setSuffix(unitStr);

        const auto& initA = m_solvedFeatures[comboA->currentData().toInt()];
        const auto& initB = m_solvedFeatures[comboB->currentData().toInt()];
        FeatureRelationResult previewRel = MetrologyMath::computeFeatureRelation(initA, initB, currentWorkplane());
        nomSpin->setValue(toDisplayUnit(previewRel.distance2D, m_unitSystem));

        form->addRow("Reference Feature A:", comboA);
        form->addRow("Target Feature B:", comboB);
        form->addRow("Nominal 2D Distance:", nomSpin);

        auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        form->addRow(btns);

        if (dlg.exec() != QDialog::Accepted) return;

        const auto& featA = m_solvedFeatures[comboA->currentData().toInt()];
        const auto& featB = m_solvedFeatures[comboB->currentData().toInt()];

        FeatureRelationResult rel = MetrologyMath::computeFeatureRelation(featA, featB, currentWorkplane());
        if (!rel.success) return;

        const double nominalMm = isInch ? (nomSpin->value() * MM_PER_INCH) : nomSpin->value();

        const QString relTitle = QString("Dim_%1 [%2 -> %3]")
            .arg(m_featureCounter++)
            .arg(QString::fromStdString(featA.name))
            .arg(QString::fromStdString(featB.name));

        auto* relItem = new QTreeWidgetItem(m_featureTree);
        relItem->setText(0, relTitle);
        applyEvaluationToTreeItem(relItem, rel.distance2D, true, nominalMm, m_defaultTolMm, m_defaultTolMm);

        new QTreeWidgetItem(relItem, {
            "3D Direct Distance", "",
            QString("%1%2").arg(toDisplayUnit(rel.distance3D, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "", ""
            });
        new QTreeWidgetItem(relItem, {
            "Delta (dX, dY, dZ)", "",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(rel.deltaXYZ.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(rel.deltaXYZ.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(rel.deltaXYZ.z(), m_unitSystem), 0, 'f', decimals),
            "", ""
            });
        new QTreeWidgetItem(relItem, {
            "Perpendicular / Normal Offset", "",
            QString("%1%2").arg(toDisplayUnit(rel.normalDistance, m_unitSystem), 0, 'f', decimals).arg(unitStr),
            "", ""
            });
        new QTreeWidgetItem(relItem, {
            "Angle Between Vectors", "",
            QString("%1°").arg(rel.angleDegrees, 0, 'f', 4),
            "", ""
            });

        if (rel.hasIntersection) {
            new QTreeWidgetItem(relItem, {
                "2D Corner Intersection (X, Y, Z)", "",
                QString("(%1, %2, %3)")
                    .arg(toDisplayUnit(rel.intersectionPoint.x(), m_unitSystem), 0, 'f', decimals)
                    .arg(toDisplayUnit(rel.intersectionPoint.y(), m_unitSystem), 0, 'f', decimals)
                    .arg(toDisplayUnit(rel.intersectionPoint.z(), m_unitSystem), 0, 'f', decimals),
                "", ""
                });
        }

        relItem->setExpanded(true);
        m_featureTree->setCurrentItem(relItem);
        playSolveChime(true);
        statusBar()->showMessage("Computed relation: " + relTitle);
    }

    void MainWindow::onRun321AlignmentClicked() {
        if (m_solvedFeatures.size() < 2) {
            QMessageBox::information(
                this,
                "Need Solved Features",
                "Please measure and solve at least a Plane (Level) and a Point/Bore/Line (Origin/Rotate) "
                "before running the 3-2-1 alignment wizard."
            );
            return;
        }

        QDialog dlg(this);
        dlg.setWindowTitle("Interactive 3-2-1 Part Alignment Wizard");
        dlg.setMinimumWidth(420);
        dlg.setStyleSheet(
            "QDialog { background-color: #21252b; color: #e0e0e0; }"
            "QLabel { color: #ffffff; font-size: 13px; font-weight: bold; }"
            "QComboBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 5px; font-size: 13px; font-weight: bold; min-width: 200px; }"
            "QComboBox QAbstractItemView { background-color: #181a1f; color: #ffffff; selection-background-color: #2d5a88; }"
            "QPushButton { background-color: #3a3f4b; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 6px 16px; font-weight: bold; }"
            "QPushButton:hover { background-color: #4b5263; }"
        );

        auto* form = new QFormLayout(&dlg);
        form->setContentsMargins(16, 16, 16, 16);
        form->setSpacing(14);

        auto* levelCombo = new QComboBox(&dlg);
        auto* rotateCombo = new QComboBox(&dlg);
        auto* originCombo = new QComboBox(&dlg);

        for (const auto& feat : m_solvedFeatures) {
            QString name = QString::fromStdString(feat.name);
            levelCombo->addItem(name, feat.id);
            rotateCombo->addItem(name, feat.id);
            originCombo->addItem(name, feat.id);
        }

        form->addRow("1. Level Z (Primary Plane):", levelCombo);
        form->addRow("2. Rotate X (Secondary Line/Feat):", rotateCombo);
        form->addRow("3. Origin XYZ (Tertiary Datum):", originCombo);

        auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        form->addRow(btns);

        if (dlg.exec() != QDialog::Accepted) return;

        int idL = levelCombo->currentData().toInt();
        int idR = rotateCombo->currentData().toInt();
        int idO = originCombo->currentData().toInt();

        auto itL = std::find_if(m_solvedFeatures.begin(), m_solvedFeatures.end(), [idL](const MeasuredFeature& f) { return f.id == idL; });
        auto itR = std::find_if(m_solvedFeatures.begin(), m_solvedFeatures.end(), [idR](const MeasuredFeature& f) { return f.id == idR; });
        auto itO = std::find_if(m_solvedFeatures.begin(), m_solvedFeatures.end(), [idO](const MeasuredFeature& f) { return f.id == idO; });

        if (itL == m_solvedFeatures.end() || itR == m_solvedFeatures.end() || itO == m_solvedFeatures.end()) {
            return;
        }

        Alignment321Result res = MetrologyMath::compute321Alignment(*itL, *itR, *itO);
        if (!res.success) {
            playSolveChime(false);
            QMessageBox::critical(
                this,
                "Alignment Failed",
                "Could not compute 3-2-1 transformation matrix. Ensure '1. Level Z' is a solved Plane feature."
            );
            return;
        }

        m_pcsMatrix = res.transformationMatrix;
        m_isPartAligned = true;
        m_alignStatusBtn->setChecked(true);
        m_alignStatusBtn->setStyleSheet("background-color: #28a745; color: white; padding: 5px 10px; font-weight: bold;");
        m_alignStatusBtn->setText("PCS (Aligned)");

        updateDroDisplay();
        playSolveChime(true);
        statusBar()->showMessage("3-2-1 Part Alignment active! DRO and new probe hits are now in PCS coordinates.");
    }

    void MainWindow::onEditToleranceClicked() {
        QTreeWidgetItem* item = m_featureTree->currentItem();
        if (!item) {
            QMessageBox::information(this, "Select a Feature Row", "Click or double-click any solved feature row in the Inspection Feature Tree to edit its Nominal and ±Tolerance.");
            return;
        }
        if (item->parent() != nullptr) {
            item = item->parent();
        }
        if (item == m_activePointsGroup || !item->data(0, ROLE_ACTUAL_MM).isValid()) {
            QMessageBox::information(this, "Select a Solved Feature", "Please select a solved feature or distance row (such as Bore_1, Sphere_1, or Dim_1) to edit its Nominal and Tolerance.");
            return;
        }

        const bool isInch = (m_unitSystem == UnitSystem::Inches);
        const double scale = isInch ? MM_PER_INCH : 1.0;
        const int decimals = isInch ? 5 : 4;
        const QString unitStr = isInch ? " in" : " mm";

        const double actualMm = item->data(0, ROLE_ACTUAL_MM).toDouble();
        const bool hadNom = item->data(0, ROLE_HAS_NOMINAL).toBool();
        const double currentNomMm = hadNom ? item->data(0, ROLE_NOMINAL_MM).toDouble() : actualMm;
        const double currentPlusMm = item->data(0, ROLE_PLUS_TOL_MM).toDouble();
        const double currentMinusMm = item->data(0, ROLE_MINUS_TOL_MM).toDouble();

        QDialog dlg(this);
        dlg.setWindowTitle("Edit Nominal & Tolerance: " + item->text(0));
        dlg.setMinimumWidth(360);
        dlg.setStyleSheet(
            "QDialog { background-color: #21252b; color: #e0e0e0; }"
            "QLabel { color: #ffffff; font-size: 13px; font-weight: bold; }"
            "QDoubleSpinBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 5px; font-size: 13px; font-weight: bold; }"
            "QPushButton { background-color: #3a3f4b; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 6px 14px; font-weight: bold; }"
            "QPushButton:hover { background-color: #4b5263; }"
        );

        auto* form = new QFormLayout(&dlg);
        form->setContentsMargins(16, 16, 16, 16);
        form->setSpacing(12);

        auto* actualLbl = new QLabel(QString("%1%2").arg(toDisplayUnit(actualMm, m_unitSystem), 0, 'f', decimals).arg(unitStr), &dlg);
        actualLbl->setStyleSheet("color: #61afef; font-family: 'Consolas', monospace; font-size: 14px;");

        auto* nomSpin = new QDoubleSpinBox(&dlg);
        nomSpin->setRange(-10000.0, 10000.0);
        nomSpin->setDecimals(decimals);
        nomSpin->setSuffix(unitStr);
        nomSpin->setValue(toDisplayUnit(currentNomMm, m_unitSystem));

        auto* plusSpin = new QDoubleSpinBox(&dlg);
        plusSpin->setRange(0.0, 1000.0);
        plusSpin->setDecimals(decimals);
        plusSpin->setSingleStep(isInch ? 0.0005 : 0.0100);
        plusSpin->setSuffix(unitStr);
        plusSpin->setValue(toDisplayUnit(currentPlusMm, m_unitSystem));

        auto* minusSpin = new QDoubleSpinBox(&dlg);
        minusSpin->setRange(0.0, 1000.0);
        minusSpin->setDecimals(decimals);
        minusSpin->setSingleStep(isInch ? 0.0005 : 0.0100);
        minusSpin->setSuffix(unitStr);
        minusSpin->setValue(toDisplayUnit(currentMinusMm, m_unitSystem));

        form->addRow("Measured (Actual):", actualLbl);
        form->addRow("Target Nominal:", nomSpin);
        form->addRow("+Tolerance Upper:", plusSpin);
        form->addRow("-Tolerance Lower:", minusSpin);

        auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        form->addRow(btns);

        if (dlg.exec() != QDialog::Accepted) return;

        const double newNomMm = nomSpin->value() * scale;
        const double newPlusMm = plusSpin->value() * scale;
        const double newMinusMm = minusSpin->value() * scale;
        m_defaultTolMm = newPlusMm;

        applyEvaluationToTreeItem(item, actualMm, true, newNomMm, newPlusMm, newMinusMm);
        statusBar()->showMessage("Updated Nominal and Tolerance evaluation for " + item->text(0));
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
            "HTML Printable Inspection Report (*.html);;CSV Spreadsheet (*.csv)",
            &selectedFilter
        );
        if (filePath.isEmpty()) return;

        const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
        const QString unitName = (m_unitSystem == UnitSystem::Millimeters) ? "Millimeters (mm)" : "Inches (in)";
        const QString coordMode = (m_isPartAligned && m_alignStatusBtn && m_alignStatusBtn->isChecked())
            ? "PCS (3-2-1 Datum Aligned)"
            : "MCS (Machine Coordinates)";
        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitShort = (m_unitSystem == UnitSystem::Millimeters) ? "mm" : "in";

        if (filePath.endsWith(".csv", Qt::CaseInsensitive)) {
            QFile file(filePath);
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QMessageBox::critical(this, "Export Error", "Could not write to file: " + filePath);
                return;
            }
            QTextStream out(&file);
            out << "OpenCMM Precision Inspection Report\n";
            out << "Part / Job Number," << partName << "\n";
            out << "Timestamp," << timestamp << "\n";
            out << "Units," << unitName << "\n";
            out << "Coordinate System," << coordMode << "\n";
            out << "Effective Probe Tip Dia," << QString::number(toDisplayUnit(m_effectiveProbeRadius * 2.0, m_unitSystem), 'f', decimals) << " " << unitShort << "\n\n";
            out << "Feature / Parameter,Sub-Parameter,Nominal,Actual,Tolerance,Deviation / Status\n";

            for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
                QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
                if (top == m_activePointsGroup) continue;

                out << "\"" << top->text(0) << "\",\"Primary\","
                    << "\"" << top->text(1) << "\","
                    << "\"" << top->text(2) << "\","
                    << "\"" << top->text(3) << "\","
                    << "\"" << top->text(4) << "\"\n";

                for (int c = 0; c < top->childCount(); ++c) {
                    QTreeWidgetItem* child = top->child(c);
                    out << "\"" << top->text(0) << "\","
                        << "\"" << child->text(0) << "\","
                        << "\"" << child->text(1) << "\","
                        << "\"" << child->text(2) << "\","
                        << "\"" << child->text(3) << "\","
                        << "\"" << child->text(4) << "\"\n";
                }
            }
            file.close();
            statusBar()->showMessage("Exported CSV Inspection Report to: " + filePath);
            return;
        }

        // Export Printable HTML Inspection Report
        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "Export Error", "Could not write to file: " + filePath);
            return;
        }

        int totalFeatures = 0, passedCount = 0, failedCount = 0;
        for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
            QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
            if (top == m_activePointsGroup) continue;
            ++totalFeatures;
            if (top->text(4).contains("FAIL")) ++failedCount;
            else if (top->text(4).contains("PASS")) ++passedCount;
        }

        QTextStream out(&file);
        out << "<!DOCTYPE html><html><head><meta charset='utf-8'>\n"
            << "<title>Inspection Report - " << partName.toHtmlEscaped() << "</title>\n"
            << "<style>\n"
            << "body { font-family: 'Segoe UI', Arial, sans-serif; margin: 28px; color: #1e2227; background: #ffffff; }\n"
            << "h1 { margin: 0 0 6px 0; font-size: 22px; color: #1a365d; border-bottom: 2px solid #1a365d; padding-bottom: 8px; }\n"
            << ".meta { display: table; width: 100%; margin: 14px 0 20px 0; border-collapse: collapse; }\n"
            << ".meta td { padding: 6px 12px; border: 1px solid #cbd5e0; font-size: 13px; }\n"
            << ".meta td.lbl { font-weight: bold; background: #edf2f7; width: 18%; }\n"
            << "table.report { width: 100%; border-collapse: collapse; font-size: 13px; }\n"
            << "table.report th { background: #2d3748; color: white; text-align: left; padding: 8px 10px; border: 1px solid #2d3748; }\n"
            << "table.report td { padding: 7px 10px; border: 1px solid #cbd5e0; font-family: 'Consolas', monospace; }\n"
            << "tr.feat-row { background: #f7fafc; font-weight: bold; }\n"
            << "tr.sub-row td { color: #4a5568; padding-left: 26px; font-weight: normal; }\n"
            << ".pass { color: #22543d; background: #c6f6d5; font-weight: bold; padding: 2px 8px; border-radius: 4px; }\n"
            << ".fail { color: #742a2a; background: #fed7d7; font-weight: bold; padding: 2px 8px; border-radius: 4px; }\n"
            << "@media print { button.print-btn { display: none; } }\n"
            << "</style></head><body>\n"
            << "<button class='print-btn' onclick='window.print()' style='float:right;padding:6px 14px;font-weight:bold;cursor:pointer;'>Print / Save PDF</button>\n"
            << "<h1>QUALITY INSPECTION REPORT</h1>\n"
            << "<table class='meta'>\n"
            << "<tr><td class='lbl'>Part / Job ID:</td><td>" << partName.toHtmlEscaped() << "</td>"
            << "<td class='lbl'>Date / Time:</td><td>" << timestamp << "</td></tr>\n"
            << "<tr><td class='lbl'>Units:</td><td>" << unitName << "</td>"
            << "<td class='lbl'>Alignment Datum:</td><td>" << coordMode << "</td></tr>\n"
            << "<tr><td class='lbl'>Probe Tip Ø:</td><td>" << QString::number(toDisplayUnit(m_effectiveProbeRadius * 2.0, m_unitSystem), 'f', decimals) << " " << unitShort
            << (m_isProbeCalibrated ? " (Calibrated)" : " (Nominal)") << "</td>"
            << "<td class='lbl'>Summary:</td><td>Total: " << totalFeatures << " | Passed: " << passedCount << " | Failed: " << failedCount << "</td></tr>\n"
            << "</table>\n"
            << "<table class='report'>\n"
            << "<thead><tr><th>Feature / Parameter</th><th>Nominal</th><th>Actual</th><th>±Tol</th><th>Deviation / Status</th></tr></thead>\n"
            << "<tbody>\n";

        for (int i = 0; i < m_featureTree->topLevelItemCount(); ++i) {
            QTreeWidgetItem* top = m_featureTree->topLevelItem(i);
            if (top == m_activePointsGroup) continue;

            QString statusHtml = top->text(4).toHtmlEscaped();
            if (statusHtml.contains("FAIL")) {
                statusHtml = "<span class='fail'>" + statusHtml + "</span>";
            }
            else if (statusHtml.contains("PASS")) {
                statusHtml = "<span class='pass'>" + statusHtml + "</span>";
            }

            out << "<tr class='feat-row'>"
                << "<td>" << top->text(0).toHtmlEscaped() << "</td>"
                << "<td>" << top->text(1).toHtmlEscaped() << "</td>"
                << "<td>" << top->text(2).toHtmlEscaped() << "</td>"
                << "<td>" << top->text(3).toHtmlEscaped() << "</td>"
                << "<td>" << statusHtml << "</td></tr>\n";

            for (int c = 0; c < top->childCount(); ++c) {
                QTreeWidgetItem* child = top->child(c);
                out << "<tr class='sub-row'>"
                    << "<td>↳ " << child->text(0).toHtmlEscaped() << "</td>"
                    << "<td>" << child->text(1).toHtmlEscaped() << "</td>"
                    << "<td>" << child->text(2).toHtmlEscaped() << "</td>"
                    << "<td>" << child->text(3).toHtmlEscaped() << "</td>"
                    << "<td>" << child->text(4).toHtmlEscaped() << "</td></tr>\n";
            }
        }

        out << "</tbody></table></body></html>\n";
        file.close();

        QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
        statusBar()->showMessage("Exported & opened HTML Inspection Report: " + filePath);
    }

    void MainWindow::onClearPointsClicked() {
        m_recordedHits.clear();
        m_hitCountLabel->setText("Active Feature Hits: 0");

        while (m_activePointsGroup->childCount() > 0) {
            delete m_activePointsGroup->takeChild(0);
        }
        m_activePointsGroup->setText(2, "0 pts");

        if (m_viewport) {
            m_viewport->clearHitMarkers();
        }

        statusBar()->showMessage("Active probe point buffer and all 3D hit markers cleared.");
    }

} // namespace OpenCMM