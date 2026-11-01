#pragma once
#include <QMainWindow>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QTreeWidget>
#include <vector>
#include <Eigen/Dense>

#include "core/Units.h"
#include "hal/PicoSerialController.h"
#include "math/MetrologyMath.h"

namespace OpenCMM {

    class OcctViewport;

    class MainWindow : public QMainWindow {
        Q_OBJECT
    public:
        explicit MainWindow(QWidget* parent = nullptr);
        ~MainWindow() override = default;

    private slots:
        void onConnectClicked();
        void onRefreshPortsClicked();
        void onZeroAxesClicked();
        void onUnitToggled();
        void onLoadStepClicked();

        void onSimulateHitClicked();
        void onCalibrateProbeClicked();
        void onFitSphereClicked();
        void onFitPlaneClicked();
        void onFitCircleClicked();
        void onFitLineClicked();
        void onMeasureRelationClicked();
        void onClearPointsClicked();

        void onPositionUpdated(const Position3D& pos);
        void onProbeHit(const Position3D& hitPos);
        void onConnectionChanged(bool connected, const QString& message);

    private:
        void setupUi();
        void setupToolbar();
        void setupDroDock();
        void setupFeatureTreeDock();
        void updateDroDisplay();
        void updateProbeStatusDisplay();
        void finishFeatureBuffer();
        ProbeCompMode currentCompMode() const;
        WorkplaneFrame currentWorkplane() const;

        PicoSerialController* m_controller = nullptr;
        OcctViewport* m_viewport = nullptr;

        UnitSystem m_unitSystem = UnitSystem::Millimeters;
        Position3D m_lastPosition{ 0.0, 0.0, 0.0 };

        double m_certifiedBallDiameterMm = 25.4000;
        double m_effectiveProbeRadius = 1.5000;
        double m_lastProbeLobingMm = 0.0;
        bool m_isProbeCalibrated = false;

        WorkplaneFrame m_lastFittedPlaneFrame = WorkplaneFrame::fromStandard(StandardWorkplane::XY_PlusZ);
        bool m_hasCustomPlane = false;

        std::vector<Eigen::Vector3d> m_recordedHits;
        std::vector<MeasuredFeature> m_solvedFeatures;
        int m_featureCounter = 1;

        // Toolbar controls
        QComboBox* m_portCombo = nullptr;
        QPushButton* m_connectBtn = nullptr;
        QPushButton* m_unitBtn = nullptr;

        QComboBox* m_workplaneCombo = nullptr;
        QComboBox* m_compModeCombo = nullptr;
        QComboBox* m_simTargetCombo = nullptr;

        // DRO Labels
        QLabel* m_droX = nullptr;
        QLabel* m_droY = nullptr;
        QLabel* m_droZ = nullptr;
        QLabel* m_unitLabels[3] = { nullptr, nullptr, nullptr };
        QLabel* m_hitCountLabel = nullptr;

        // Active Probe Status Labels
        QLabel* m_probeStatusBadge = nullptr;
        QLabel* m_probeDiamLabel = nullptr;
        QLabel* m_probeRadLabel = nullptr;
        QLabel* m_probeLobingLabel = nullptr;

        // Feature Tree
        QTreeWidget* m_featureTree = nullptr;
        QTreeWidgetItem* m_activePointsGroup = nullptr;
    };

} // namespace OpenCMM