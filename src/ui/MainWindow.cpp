#include "ui/MainWindow.h"
#include "ui/OcctViewport.h"

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
#include <QSerialPortInfo>
#include <cmath>

namespace OpenCMM {

    MainWindow::MainWindow(QWidget* parent)
        : QMainWindow(parent),
        m_controller(new PicoSerialController(this)) {

        setWindowTitle("OpenCMM - Precision Coordinate Metrology Studio");
        resize(1600, 940);

        setupUi();

        connect(m_controller, &PicoSerialController::positionUpdated,
            this, &MainWindow::onPositionUpdated);
        connect(m_controller, &PicoSerialController::probeTriggered,
            this, &MainWindow::onProbeHit);
        connect(m_controller, &PicoSerialController::connectionStatusChanged,
            this, &MainWindow::onConnectionChanged);

        onRefreshPortsClicked();
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
        setCentralWidget(m_viewport);

        statusBar()->setStyleSheet("background-color: #007acc; color: white; font-weight: bold;");
        statusBar()->showMessage("Ready - Select workplane, capture probe hits, and solve 2D/3D features.");
    }

    void MainWindow::setupToolbar() {
        const QString barStyle =
            "QToolBar { background-color: #21252b; border-bottom: 1px solid #3a3f4b; spacing: 6px; padding: 5px; }"
            "QPushButton { background-color: #3a3f4b; color: white; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #4b5263; }"
            "QComboBox { background-color: #1e2227; color: white; border: 1px solid #4b5263; padding: 4px 8px; font-weight: bold; }"
            "QComboBox QAbstractItemView { background-color: #1e2227; color: white; selection-background-color: #2d5a88; }";

        // Row 1: Hardware, CAD Loader & Simulator
        auto* hwToolbar = addToolBar("Hardware & CAD Controls");
        hwToolbar->setMovable(false);
        hwToolbar->setStyleSheet(barStyle);

        auto* portLabel = new QLabel(" Pico 2: ", this);
        portLabel->setStyleSheet("color: #abb2bf; font-weight: bold;");
        hwToolbar->addWidget(portLabel);

        m_portCombo = new QComboBox(this);
        m_portCombo->setMinimumWidth(120);
        hwToolbar->addWidget(m_portCombo);

        auto* refreshBtn = new QPushButton("Refresh", this);
        connect(refreshBtn, &QPushButton::clicked, this, &MainWindow::onRefreshPortsClicked);
        hwToolbar->addWidget(refreshBtn);

        m_connectBtn = new QPushButton("Connect", this);
        m_connectBtn->setStyleSheet("background-color: #28a745; color: white; padding: 5px 12px; font-weight: bold;");
        connect(m_connectBtn, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
        hwToolbar->addWidget(m_connectBtn);

        hwToolbar->addSeparator();

        auto* zeroBtn = new QPushButton("Zero XYZ", this);
        connect(zeroBtn, &QPushButton::clicked, this, &MainWindow::onZeroAxesClicked);
        hwToolbar->addWidget(zeroBtn);

        m_unitBtn = new QPushButton("Units: MM", this);
        connect(m_unitBtn, &QPushButton::clicked, this, &MainWindow::onUnitToggled);
        hwToolbar->addWidget(m_unitBtn);

        hwToolbar->addSeparator();

        auto* loadStepBtn = new QPushButton("Load STEP CAD", this);
        loadStepBtn->setStyleSheet("background-color: #2d5a88; color: white; padding: 5px 10px; font-weight: bold;");
        connect(loadStepBtn, &QPushButton::clicked, this, &MainWindow::onLoadStepClicked);
        hwToolbar->addWidget(loadStepBtn);

        hwToolbar->addSeparator();

        auto* simLabel = new QLabel(" Sim: ", this);
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
        simHitBtn->setStyleSheet("background-color: #d19a66; color: #1e2227; padding: 5px 10px; font-weight: bold;");
        connect(simHitBtn, &QPushButton::clicked, this, &MainWindow::onSimulateHitClicked);
        hwToolbar->addWidget(simHitBtn);

        // Row 2: Metrology Workplane, Tip Compensation & Feature Solvers
        addToolBarBreak(Qt::TopToolBarArea);
        auto* metroToolbar = addToolBar("Metrology Feature Solvers");
        metroToolbar->setMovable(false);
        metroToolbar->setStyleSheet(barStyle);

        auto* wpLabel = new QLabel(" Workplane: ", this);
        wpLabel->setStyleSheet("color: #abb2bf; font-weight: bold;");
        metroToolbar->addWidget(wpLabel);

        m_workplaneCombo = new QComboBox(this);
        m_workplaneCombo->addItem("XY Plane (+Z)", static_cast<int>(StandardWorkplane::XY_PlusZ));
        m_workplaneCombo->addItem("YZ Plane (+X)", static_cast<int>(StandardWorkplane::YZ_PlusX));
        m_workplaneCombo->addItem("ZX Plane (+Y)", static_cast<int>(StandardWorkplane::ZX_PlusY));
        m_workplaneCombo->addItem("Last Fitted Plane", static_cast<int>(StandardWorkplane::CustomPlane));
        metroToolbar->addWidget(m_workplaneCombo);

        auto* compLabel = new QLabel(" Tip Comp: ", this);
        compLabel->setStyleSheet("color: #abb2bf; font-weight: bold;");
        metroToolbar->addWidget(compLabel);

        m_compModeCombo = new QComboBox(this);
        m_compModeCombo->addItem("External / Boss (-R)", static_cast<int>(ProbeCompMode::External));
        m_compModeCombo->addItem("Internal / Bore (+R)", static_cast<int>(ProbeCompMode::Internal));
        m_compModeCombo->addItem("Uncompensated (Ruby)", static_cast<int>(ProbeCompMode::Uncompensated));
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

        auto* fitCircleBtn = new QPushButton("Fit 2D Circle (Bore/Boss)", this);
        fitCircleBtn->setStyleSheet("background-color: #2b6cb0; color: white; padding: 5px 10px; font-weight: bold;");
        connect(fitCircleBtn, &QPushButton::clicked, this, &MainWindow::onFitCircleClicked);
        metroToolbar->addWidget(fitCircleBtn);

        auto* fitLineBtn = new QPushButton("Fit 2D Line (Edge)", this);
        fitLineBtn->setStyleSheet("background-color: #2b6cb0; color: white; padding: 5px 10px; font-weight: bold;");
        connect(fitLineBtn, &QPushButton::clicked, this, &MainWindow::onFitLineClicked);
        metroToolbar->addWidget(fitLineBtn);

        metroToolbar->addSeparator();

        auto* relBtn = new QPushButton("Measure Relation / Distance", this);
        relBtn->setStyleSheet("background-color: #2f855a; color: white; padding: 5px 10px; font-weight: bold;");
        connect(relBtn, &QPushButton::clicked, this, &MainWindow::onMeasureRelationClicked);
        metroToolbar->addWidget(relBtn);

        auto* clearBtn = new QPushButton("Clear All Points", this);
        connect(clearBtn, &QPushButton::clicked, this, &MainWindow::onClearPointsClicked);
        metroToolbar->addWidget(clearBtn);

        // In setupToolbar(), add an alignment button next to metrology solvers:
        auto* alignBtn = new QPushButton("3-2-1 Alignment...", this);
        alignBtn->setStyleSheet("background-color: #d15757; color: white; padding: 5px 12px; font-weight: bold;");
        connect(alignBtn, &QPushButton::clicked, this, &MainWindow::onRun321AlignmentClicked);
        metroToolbar->addWidget(alignBtn);

        m_alignStatusBtn = new QPushButton("Alignment: MCS (Machine)", this);
        m_alignStatusBtn->setCheckable(true);
        m_alignStatusBtn->setChecked(false);
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

        // Find selected features
        int idL = levelCombo->currentData().toInt();
        int idR = rotateCombo->currentData().toInt();
        int idO = originCombo->currentData().toInt();

        auto itL = std::find_if(m_solvedFeatures.begin(), m_solvedFeatures.end(), [idL](const MeasuredFeature& f) { return f.id == idL; });
        auto itR = std::find_if(m_solvedFeatures.begin(), m_solvedFeatures.end(), [idR](const MeasuredFeature& f) { return f.id == idR; });
        auto itO = std::find_if(m_solvedFeatures.begin(), m_solvedFeatures.end(), [idO](const MeasuredFeature& f) { return f.id == idO; });

        Alignment321Result res = MetrologyMath::compute321Alignment(*itL, *itR, *itO);
        if (!res.success) {
            QMessageBox::critical(this, "Alignment Failed", "Could not compute 3-2-1 transformation matrix.");
            return;
        }

        m_pcsMatrix = res.transformationMatrix;
        m_isPartAligned = true;
        m_alignStatusBtn->setChecked(true);
        m_alignStatusBtn->setStyleSheet("background-color: #28a745; color: white; padding: 5px 10px; font-weight: bold;");
        m_alignStatusBtn->setText("Alignment: PCS (Active Part Datum)");

        updateDroDisplay();
        statusBar()->showMessage("3-2-1 Part Alignment successfully applied! DRO is now displaying PCS coordinates.");
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
        treeDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

        m_featureTree = new QTreeWidget(treeDock);
        m_featureTree->setColumnCount(2);
        m_featureTree->setHeaderLabels({ "Feature / Parameter", "Measured Value" });
        m_featureTree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        m_featureTree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
        m_featureTree->setStyleSheet(
            "QTreeWidget { background-color: #181a1f; color: #dcdfe4; border: none; font-family: 'Segoe UI', sans-serif; font-size: 13px; }"
            "QHeaderView::section { background-color: #21252b; color: #abb2bf; padding: 5px; border: 1px solid #3a3f4b; font-weight: bold; }"
        );

        m_activePointsGroup = new QTreeWidgetItem(m_featureTree, { "Active Probe Buffer", "0 pts" });
        m_activePointsGroup->setExpanded(true);

        treeDock->setWidget(m_featureTree);
        addDockWidget(Qt::LeftDockWidgetArea, treeDock);
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

    void MainWindow::onRefreshPortsClicked() {
        m_portCombo->clear();
        const auto ports = QSerialPortInfo::availablePorts();
        for (const QSerialPortInfo& info : ports) {
            QString label = info.portName();
            if (!info.description().isEmpty()) {
                label += " (" + info.description() + ")";
            }
            m_portCombo->addItem(label, info.portName());
        }
        if (m_portCombo->count() == 0) {
            m_portCombo->addItem("No COM Ports Found", "");
        }
    }

    void MainWindow::onConnectClicked() {
        if (m_controller->isConnected()) {
            m_controller->disconnectFromPort();
            return;
        }

        QString portName = m_portCombo->currentData().toString();
        if (portName.isEmpty()) {
            statusBar()->showMessage("No valid COM port selected.");
            return;
        }

        m_controller->connectToPort(portName, 115200);
    }

    void MainWindow::onConnectionChanged(bool connected, const QString& message) {
        if (connected) {
            m_connectBtn->setText("Disconnect");
            m_connectBtn->setStyleSheet("background-color: #e06c75; color: white; padding: 5px 12px; font-weight: bold;");
            statusBar()->setStyleSheet("background-color: #28a745; color: white; font-weight: bold;");
        }
        else {
            m_connectBtn->setText("Connect");
            m_connectBtn->setStyleSheet("background-color: #28a745; color: white; padding: 5px 12px; font-weight: bold;");
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
            m_unitBtn->setText("Units: INCH");
            for (auto* lbl : m_unitLabels) lbl->setText("in");
        }
        else {
            m_unitSystem = UnitSystem::Millimeters;
            m_unitBtn->setText("Units: MM");
            for (auto* lbl : m_unitLabels) lbl->setText("mm");
        }
        updateDroDisplay();
        updateProbeStatusDisplay();
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
            statusBar()->showMessage("Loaded STEP model: " + filePath);
        }
    }

    void MainWindow::onPositionUpdated(const Position3D& pos) {
        m_lastPosition = pos;
        updateDroDisplay();
        if (m_viewport) {
            m_viewport->updateProbePosition(pos.x, pos.y, pos.z);
        }
    }

    

    void MainWindow::onProbeHit(const Position3D& hitPos) {
        m_lastPosition = hitPos;
        updateDroDisplay();

        m_recordedHits.emplace_back(hitPos.x, hitPos.y, hitPos.z);
        const int count = static_cast<int>(m_recordedHits.size());

        m_hitCountLabel->setText(QString("Active Feature Hits: %1").arg(count));
        m_activePointsGroup->setText(1, QString("%1 pts").arg(count));

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        QString coordsStr = QString("X: %1, Y: %2, Z: %3")
            .arg(toDisplayUnit(hitPos.x, m_unitSystem), 0, 'f', decimals)
            .arg(toDisplayUnit(hitPos.y, m_unitSystem), 0, 'f', decimals)
            .arg(toDisplayUnit(hitPos.z, m_unitSystem), 0, 'f', decimals);

        new QTreeWidgetItem(m_activePointsGroup, { QString("Hit #%1").arg(count), coordsStr });

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
        m_activePointsGroup->setText(1, "0 pts");

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
            QString("R_eff: %1%2").arg(toDisplayUnit(cal.effectiveRadius, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        new QTreeWidgetItem(calItem, {
            "Effective Tip Diameter",
            QString("Ø %1%2").arg(toDisplayUnit(cal.effectiveRadius * 2.0, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });
        new QTreeWidgetItem(calItem, {
            "Master Ball Center (X, Y, Z)",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(cal.masterBallCenter.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(cal.masterBallCenter.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(cal.masterBallCenter.z(), m_unitSystem), 0, 'f', decimals)
            });
        new QTreeWidgetItem(calItem, {
            "3D Probe Lobing (Form Error)",
            QString("%1%2").arg(toDisplayUnit(cal.formError, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        calItem->setExpanded(true);
        finishFeatureBuffer();
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
            statusBar()->showMessage("Sphere least-squares fit failed (coplanar or singular points).");
            return;
        }

        const int id = m_featureCounter++;
        const QString featName = QString("Sphere_%1").arg(id);

        m_solvedFeatures.push_back({
            id,
            featName.toStdString(),
            FeatureKind::Sphere3D,
            res.center,
            Eigen::Vector3d::UnitZ(),
            Eigen::Vector3d::UnitY(),
            res.radius * 2.0,
            res.formError
            });

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* sphereItem = new QTreeWidgetItem(m_featureTree, {
            QString("%1 (%2 pts)").arg(featName).arg(m_recordedHits.size()),
            QString("Ø %1%2").arg(toDisplayUnit(res.radius * 2.0, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        new QTreeWidgetItem(sphereItem, {
            "Center (X, Y, Z)",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.center.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.z(), m_unitSystem), 0, 'f', decimals)
            });
        new QTreeWidgetItem(sphereItem, {
            "Sphericity (Form Error)",
            QString("%1%2").arg(toDisplayUnit(res.formError, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        sphereItem->setExpanded(true);
        finishFeatureBuffer();
        statusBar()->showMessage(featName + " solved. Active point buffer reset for next feature.");
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
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* planeItem = new QTreeWidgetItem(m_featureTree, {
            QString("%1 (%2 pts)").arg(featName).arg(m_recordedHits.size()),
            QString("Flatness: %1%2").arg(toDisplayUnit(res.flatness, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        new QTreeWidgetItem(planeItem, {
            "Outward Normal (I, J, K)",
            QString("(%1, %2, %3)")
                .arg(res.normal.x(), 0, 'f', 5)
                .arg(res.normal.y(), 0, 'f', 5)
                .arg(res.normal.z(), 0, 'f', 5)
            });
        new QTreeWidgetItem(planeItem, {
            "Surface Centroid (X, Y, Z)",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.centroid.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.z(), m_unitSystem), 0, 'f', decimals)
            });

        planeItem->setExpanded(true);
        finishFeatureBuffer();
        statusBar()->showMessage(featName + " solved. Active point buffer reset for next feature.");
    }

    void MainWindow::onFitCircleClicked() {
        if (m_recordedHits.size() < 3) {
            statusBar()->showMessage("Need at least 3 probe hits to fit a 2D Circle (Bore/Boss).");
            return;
        }

        ProbeCompMode compMode = currentCompMode();
        if (m_simTargetCombo && (m_simTargetCombo->currentData().toInt() == 1 || m_simTargetCombo->currentData().toInt() == 2)) {
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
            statusBar()->showMessage("2D Circle fit failed (collinear points in active workplane).");
            return;
        }

        const int id = m_featureCounter++;
        const QString prefix = (compMode == ProbeCompMode::Internal) ? "Bore" : "Boss";
        const QString featName = QString("%1_%2").arg(prefix).arg(id);

        m_solvedFeatures.push_back({
            id,
            featName.toStdString(),
            FeatureKind::Circle2D,
            res.center,
            res.planeNormal,
            Eigen::Vector3d::UnitY(),
            res.radius * 2.0,
            res.circularity
            });

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* circleItem = new QTreeWidgetItem(m_featureTree, {
            QString("%1 (%2 pts)").arg(featName).arg(m_recordedHits.size()),
            QString("Ø %1%2").arg(toDisplayUnit(res.radius * 2.0, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        new QTreeWidgetItem(circleItem, {
            "Center (X, Y, Z)",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.center.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.center.z(), m_unitSystem), 0, 'f', decimals)
            });
        new QTreeWidgetItem(circleItem, {
            "Compensated Radius",
            QString("%1%2 (Comp: %3%2)")
                .arg(toDisplayUnit(res.radius, m_unitSystem), 0, 'f', decimals)
                .arg(unitStr)
                .arg(toDisplayUnit(res.appliedComp, m_unitSystem), 0, 'f', decimals)
            });
        new QTreeWidgetItem(circleItem, {
            "Circularity (Roundness)",
            QString("%1%2").arg(toDisplayUnit(res.circularity, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        circleItem->setExpanded(true);
        finishFeatureBuffer();
        statusBar()->showMessage(featName + " solved. Active point buffer reset for next feature.");
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
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* lineItem = new QTreeWidgetItem(m_featureTree, {
            QString("%1 (%2 pts)").arg(featName).arg(m_recordedHits.size()),
            QString("Straightness: %1%2").arg(toDisplayUnit(res.straightness, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        new QTreeWidgetItem(lineItem, {
            "Edge Point (X, Y, Z)",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.centroid.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.z(), m_unitSystem), 0, 'f', decimals)
            });
        new QTreeWidgetItem(lineItem, {
            "Unit Direction (I, J, K)",
            QString("(%1, %2, %3)")
                .arg(res.direction.x(), 0, 'f', 5)
                .arg(res.direction.y(), 0, 'f', 5)
                .arg(res.direction.z(), 0, 'f', 5)
            });
        new QTreeWidgetItem(lineItem, {
            "Outward Edge Normal",
            QString("(%1, %2, %3)")
                .arg(res.edgeNormal.x(), 0, 'f', 5)
                .arg(res.edgeNormal.y(), 0, 'f', 5)
                .arg(res.edgeNormal.z(), 0, 'f', 5)
            });

        lineItem->setExpanded(true);
        finishFeatureBuffer();
        statusBar()->showMessage(featName + " solved. Active point buffer reset for next feature.");
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
        dlg.setMinimumWidth(360);
        dlg.setStyleSheet(
            "QDialog { background-color: #21252b; color: #e0e0e0; }"
            "QLabel { color: #ffffff; font-size: 13px; font-weight: bold; }"
            "QComboBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 10px; font-size: 13px; font-weight: bold; min-width: 170px; }"
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

        form->addRow("Reference Feature A:", comboA);
        form->addRow("Target Feature B:", comboB);

        auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        form->addRow(btns);

        if (dlg.exec() != QDialog::Accepted) return;

        const auto& featA = m_solvedFeatures[comboA->currentData().toInt()];
        const auto& featB = m_solvedFeatures[comboB->currentData().toInt()];

        FeatureRelationResult rel = MetrologyMath::computeFeatureRelation(featA, featB, currentWorkplane());
        if (!rel.success) return;

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        const QString relTitle = QString("Dim_%1 [%2 -> %3]")
            .arg(m_featureCounter++)
            .arg(QString::fromStdString(featA.name))
            .arg(QString::fromStdString(featB.name));

        auto* relItem = new QTreeWidgetItem(m_featureTree, {
            relTitle,
            QString("2D Dist: %1%2").arg(toDisplayUnit(rel.distance2D, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        new QTreeWidgetItem(relItem, {
            "3D Direct Distance",
            QString("%1%2").arg(toDisplayUnit(rel.distance3D, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });
        new QTreeWidgetItem(relItem, {
            "Delta (dX, dY, dZ)",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(rel.deltaXYZ.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(rel.deltaXYZ.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(rel.deltaXYZ.z(), m_unitSystem), 0, 'f', decimals)
            });
        new QTreeWidgetItem(relItem, {
            "Perpendicular / Normal Offset",
            QString("%1%2").arg(toDisplayUnit(rel.normalDistance, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });
        new QTreeWidgetItem(relItem, {
            "Angle Between Vectors",
            QString("%1°").arg(rel.angleDegrees, 0, 'f', 4)
            });

        if (rel.hasIntersection) {
            new QTreeWidgetItem(relItem, {
                "2D Corner Intersection (X, Y, Z)",
                QString("(%1, %2, %3)")
                    .arg(toDisplayUnit(rel.intersectionPoint.x(), m_unitSystem), 0, 'f', decimals)
                    .arg(toDisplayUnit(rel.intersectionPoint.y(), m_unitSystem), 0, 'f', decimals)
                    .arg(toDisplayUnit(rel.intersectionPoint.z(), m_unitSystem), 0, 'f', decimals)
                });
        }

        relItem->setExpanded(true);
        statusBar()->showMessage("Computed relation: " + relTitle);
    }

    void MainWindow::onClearPointsClicked() {
        m_recordedHits.clear();
        m_hitCountLabel->setText("Active Feature Hits: 0");

        while (m_activePointsGroup->childCount() > 0) {
            delete m_activePointsGroup->takeChild(0);
        }
        m_activePointsGroup->setText(1, "0 pts");

        if (m_viewport) {
            m_viewport->clearHitMarkers();
        }

        statusBar()->showMessage("Active probe point buffer and all 3D hit markers cleared.");
    }

} // namespace OpenCMM