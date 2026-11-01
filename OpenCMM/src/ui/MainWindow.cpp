#include "ui/MainWindow.h"
#include "ui/OcctViewport.h"
#include "math/MetrologyMath.h"

#include <QToolBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QStatusBar>
#include <QFileDialog>
#include <QSerialPortInfo>
#include <cmath>

namespace OpenCMM {

    MainWindow::MainWindow(QWidget* parent)
        : QMainWindow(parent),
        m_controller(new PicoSerialController(this)) {

        setWindowTitle("OpenCMM - Precision Coordinate Metrology Studio");
        resize(1440, 900);

        setupUi();

        connect(m_controller, &PicoSerialController::positionUpdated,
            this, &MainWindow::onPositionUpdated);
        connect(m_controller, &PicoSerialController::probeTriggered,
            this, &MainWindow::onProbeHit);
        connect(m_controller, &PicoSerialController::connectionStatusChanged,
            this, &MainWindow::onConnectionChanged);

        onRefreshPortsClicked();
    }

    void MainWindow::setupUi() {
        setStyleSheet(
            "QMainWindow { background-color: #1e2227; color: #e0e0e0; }"
            "QDockWidget { color: #ffffff; font-weight: bold; }"
            "QDockWidget::title { background: #282c34; padding: 6px; border: 1px solid #3a3f4b; }"
            "QGroupBox { border: 1px solid #3a3f4b; border-radius: 4px; margin-top: 8px; padding-top: 10px; color: #abb2bf; font-weight: bold; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
        );

        setupToolbar();
        setupDroDock();
        setupFeatureTreeDock();

        // Central Viewport: Live Open CASCADE 3D View
        m_viewport = new OcctViewport(this);
        setCentralWidget(m_viewport);

        statusBar()->setStyleSheet("background-color: #007acc; color: white; font-weight: bold;");
        statusBar()->showMessage("Disconnected - Select Pico 2 COM port or use Simulate Probe Hit.");
    }

    void MainWindow::setupToolbar() {
        auto* toolbar = addToolBar("Hardware & Inspection Controls");
        toolbar->setMovable(false);
        toolbar->setStyleSheet(
            "QToolBar { background-color: #21252b; border-bottom: 1px solid #3a3f4b; spacing: 8px; padding: 6px; }"
            "QPushButton { background-color: #3a3f4b; color: white; border: 1px solid #4b5263; border-radius: 3px; padding: 5px 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #4b5263; }"
            "QComboBox { background-color: #1e2227; color: white; border: 1px solid #4b5263; padding: 4px 8px; min-width: 140px; }"
        );

        auto* portLabel = new QLabel(" Pico 2 Port: ", this);
        portLabel->setStyleSheet("color: #abb2bf; font-weight: bold;");
        toolbar->addWidget(portLabel);

        m_portCombo = new QComboBox(this);
        toolbar->addWidget(m_portCombo);

        auto* refreshBtn = new QPushButton("Refresh", this);
        connect(refreshBtn, &QPushButton::clicked, this, &MainWindow::onRefreshPortsClicked);
        toolbar->addWidget(refreshBtn);

        m_connectBtn = new QPushButton("Connect", this);
        m_connectBtn->setStyleSheet("background-color: #28a745; color: white; padding: 5px 14px; font-weight: bold;");
        connect(m_connectBtn, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
        toolbar->addWidget(m_connectBtn);

        toolbar->addSeparator();

        auto* zeroBtn = new QPushButton("Zero XYZ", this);
        connect(zeroBtn, &QPushButton::clicked, this, &MainWindow::onZeroAxesClicked);
        toolbar->addWidget(zeroBtn);

        m_unitBtn = new QPushButton("Units: MM", this);
        connect(m_unitBtn, &QPushButton::clicked, this, &MainWindow::onUnitToggled);
        toolbar->addWidget(m_unitBtn);

        toolbar->addSeparator();

        auto* loadStepBtn = new QPushButton("Load STEP CAD", this);
        loadStepBtn->setStyleSheet("background-color: #2d5a88; color: white; padding: 5px 12px; font-weight: bold;");
        connect(loadStepBtn, &QPushButton::clicked, this, &MainWindow::onLoadStepClicked);
        toolbar->addWidget(loadStepBtn);

        toolbar->addSeparator();

        auto* simHitBtn = new QPushButton("Simulate Probe Hit", this);
        simHitBtn->setStyleSheet("background-color: #d19a66; color: #1e2227; padding: 5px 12px; font-weight: bold;");
        connect(simHitBtn, &QPushButton::clicked, this, &MainWindow::onSimulateHitClicked);
        toolbar->addWidget(simHitBtn);

        auto* fitSphereBtn = new QPushButton("Fit Sphere / Master Ball", this);
        connect(fitSphereBtn, &QPushButton::clicked, this, &MainWindow::onFitSphereClicked);
        toolbar->addWidget(fitSphereBtn);

        auto* fitPlaneBtn = new QPushButton("Fit Plane", this);
        connect(fitPlaneBtn, &QPushButton::clicked, this, &MainWindow::onFitPlaneClicked);
        toolbar->addWidget(fitPlaneBtn);

        auto* clearBtn = new QPushButton("Clear Points", this);
        connect(clearBtn, &QPushButton::clicked, this, &MainWindow::onClearPointsClicked);
        toolbar->addWidget(clearBtn);
    }

    void MainWindow::setupDroDock() {
        auto* droDock = new QDockWidget("LIVE CMM DRO", this);
        droDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

        auto* container = new QWidget(droDock);
        container->setStyleSheet("background-color: #181a1f;");
        auto* layout = new QVBoxLayout(container);

        auto* grid = new QGridLayout();
        grid->setVerticalSpacing(12);

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

        m_hitCountLabel = new QLabel("Recorded Probe Hits: 0", container);
        m_hitCountLabel->setStyleSheet("color: #61afef; font-size: 14px; font-weight: bold; margin-top: 10px;");
        layout->addWidget(m_hitCountLabel);

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
            m_connectBtn->setStyleSheet("background-color: #e06c75; color: white; padding: 5px 14px; font-weight: bold;");
            statusBar()->setStyleSheet("background-color: #28a745; color: white; font-weight: bold;");
        }
        else {
            m_connectBtn->setText("Connect");
            m_connectBtn->setStyleSheet("background-color: #28a745; color: white; padding: 5px 14px; font-weight: bold;");
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

    void MainWindow::updateDroDisplay() {
        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const double xVal = toDisplayUnit(m_lastPosition.x, m_unitSystem);
        const double yVal = toDisplayUnit(m_lastPosition.y, m_unitSystem);
        const double zVal = toDisplayUnit(m_lastPosition.z, m_unitSystem);

        m_droX->setText(QString::number(xVal, 'f', decimals));
        m_droY->setText(QString::number(yVal, 'f', decimals));
        m_droZ->setText(QString::number(zVal, 'f', decimals));
    }

    void MainWindow::onProbeHit(const Position3D& hitPos) {
        m_lastPosition = hitPos;
        updateDroDisplay();

        m_recordedHits.emplace_back(hitPos.x, hitPos.y, hitPos.z);
        const int count = static_cast<int>(m_recordedHits.size());

        m_hitCountLabel->setText(QString("Recorded Probe Hits: %1").arg(count));
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

    void MainWindow::onSimulateHitClicked() {
        const int idx = static_cast<int>(m_recordedHits.size());
        const double R = 14.2000;
        const double cx = 60.0, cy = 40.0, cz = 25.0;

        const double goldenAngle = 2.399963229728653;
        const double theta = idx * goldenAngle;
        const double phi = std::acos(1.0 - 0.65 * ((idx % 12) + 0.5) / 12.0);

        Position3D simHit{
            cx + R * std::sin(phi) * std::cos(theta),
            cy + R * std::sin(phi) * std::sin(theta),
            cz + R * std::cos(phi)
        };

        onProbeHit(simHit);
    }

    void MainWindow::onFitSphereClicked() {
        if (m_recordedHits.size() < 4) {
            statusBar()->showMessage("Need at least 4 probe hits to fit a Sphere.");
            return;
        }

        SphereFitResult res = MetrologyMath::fitSphereLeastSquares(m_recordedHits);
        if (!res.success) {
            statusBar()->showMessage("Sphere least-squares fit failed (coplanar or singular points).");
            return;
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* sphereItem = new QTreeWidgetItem(m_featureTree, {
            QString("Sphere_%1 (%2 pts)").arg(m_featureCounter++).arg(m_recordedHits.size()),
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
            "Center-to-Center Radius",
            QString("%1%2").arg(toDisplayUnit(res.radius, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });
        new QTreeWidgetItem(sphereItem, {
            "Form Error (Sphericity)",
            QString("%1%2").arg(toDisplayUnit(res.formError, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });
        new QTreeWidgetItem(sphereItem, {
            "RMS Residual",
            QString("%1%2").arg(toDisplayUnit(res.rmsError, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        sphereItem->setExpanded(true);
        statusBar()->showMessage("Sphere feature solved and added to Inspection Tree.");
    }

    void MainWindow::onFitPlaneClicked() {
        if (m_recordedHits.size() < 3) {
            statusBar()->showMessage("Need at least 3 probe hits to fit a Plane.");
            return;
        }

        PlaneFitResult res = MetrologyMath::fitPlaneLeastSquares(m_recordedHits);
        if (!res.success) {
            statusBar()->showMessage("Plane SVD fit failed (collinear points).");
            return;
        }

        const int decimals = (m_unitSystem == UnitSystem::Millimeters) ? 4 : 5;
        const QString unitStr = (m_unitSystem == UnitSystem::Millimeters) ? " mm" : " in";

        auto* planeItem = new QTreeWidgetItem(m_featureTree, {
            QString("Plane_%1 (%2 pts)").arg(m_featureCounter++).arg(m_recordedHits.size()),
            QString("Flatness: %1%2").arg(toDisplayUnit(res.flatness, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        new QTreeWidgetItem(planeItem, {
            "Normal Vector (I, J, K)",
            QString("(%1, %2, %3)")
                .arg(res.normal.x(), 0, 'f', 5)
                .arg(res.normal.y(), 0, 'f', 5)
                .arg(res.normal.z(), 0, 'f', 5)
            });
        new QTreeWidgetItem(planeItem, {
            "Centroid (X, Y, Z)",
            QString("(%1, %2, %3)")
                .arg(toDisplayUnit(res.centroid.x(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.y(), m_unitSystem), 0, 'f', decimals)
                .arg(toDisplayUnit(res.centroid.z(), m_unitSystem), 0, 'f', decimals)
            });
        new QTreeWidgetItem(planeItem, {
            "RMS Residual",
            QString("%1%2").arg(toDisplayUnit(res.rmsError, m_unitSystem), 0, 'f', decimals).arg(unitStr)
            });

        planeItem->setExpanded(true);
        statusBar()->showMessage("Plane feature solved and added to Inspection Tree.");
    }

    void MainWindow::onClearPointsClicked() {
        m_recordedHits.clear();
        m_hitCountLabel->setText("Recorded Probe Hits: 0");

        while (m_activePointsGroup->childCount() > 0) {
            delete m_activePointsGroup->takeChild(0);
        }
        m_activePointsGroup->setText(1, "0 pts");

        if (m_viewport) {
            m_viewport->clearHitMarkers();
        }

        statusBar()->showMessage("Active probe point buffer and 3D hit markers cleared.");
    }

} // namespace OpenCMM