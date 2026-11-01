#pragma once
#include <QMainWindow>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QCheckBox>
#include <QTreeWidget>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "core/Units.h"
#include "hal/PicoSerialController.h"
#include "math/MetrologyMath.h"

namespace OpenCMM {

    class OcctViewport;
    struct CadFaceInfo;

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
        void onTransformCadClicked();

        void onSimulateHitClicked();
        void onCalibrateProbeClicked();
        void onFitSphereClicked();
        void onFitPlaneClicked();
        void onFitCircleClicked();
        void onFitLineClicked();
        void onMeasureRelationClicked();
        void onRun321AlignmentClicked();
        void onEditToleranceClicked();
        void onExportReportClicked();
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
        void onCadFaceSelected(const CadFaceInfo& faceInfo);

        void applyEvaluationToTreeItem(QTreeWidgetItem* item,
            double actualMm,
            bool hasNominal,
            double nominalMm,
            double plusTolMm,
            double minusTolMm);
        void playHitChime() const;
        void playSolveChime(bool passed = true) const;

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

        // Default bilateral tolerance (0.0500 mm / ~0.0020 in)
        double m_defaultTolMm = 0.0500;

        // Armed CAD Nominal from Phase 4 Face Picking
        bool m_hasArmedCadNominal = false;
        double m_armedCadDiameterMm = 0.0;
        Eigen::Vector3d m_armedCadLocationMm = Eigen::Vector3d::Zero();
        QString m_armedCadDescription;

        WorkplaneFrame m_lastFittedPlaneFrame = WorkplaneFrame::fromStandard(StandardWorkplane::XY_PlusZ);
        bool m_hasCustomPlane = false;

        // 3-2-1 Alignment State
        Eigen::Matrix4d m_pcsMatrix = Eigen::Matrix4d::Identity();
        bool m_isPartAligned = false;
        QPushButton* m_alignStatusBtn = nullptr;

        std::vector<Eigen::Vector3d> m_recordedHits;
        std::vector<MeasuredFeature> m_solvedFeatures;
        int m_featureCounter = 1;

        // Toolbar controls
        QComboBox* m_portCombo = nullptr;
        QPushButton* m_connectBtn = nullptr;
        QPushButton* m_unitBtn = nullptr;
        QCheckBox* m_audioCheck = nullptr;

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