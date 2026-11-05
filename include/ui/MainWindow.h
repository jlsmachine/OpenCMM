#pragma once
#include <QMainWindow>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QTreeWidget>
#include <QFrame>
#include <QList>
#include <QString>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <deque>

#include "core/Units.h"
#include "hal/PicoSerialController.h"
#include "math/MetrologyMath.h"
#include "ui/SettingsDialog.h"

namespace OpenCMM {

    class OcctViewport;
    struct CadFaceInfo;

    class MainWindow : public QMainWindow {
        Q_OBJECT
    public:
        explicit MainWindow(QWidget* parent = nullptr);
        ~MainWindow() override;

    private slots:
        void onZeroAxesClicked();
        void onUnitToggled();
        void onLoadStepClicked();
        void onTransformCadClicked();
        void onOpenSettingsClicked();

        void onSimulateHitClicked();
        void onCalibrateProbeClicked();
        void onFitSphereClicked();
        void onFitPlaneClicked();
        void onFitCircleClicked();
        void onFitLineClicked();
        void onMeasureRelationClicked();
        void onEvaluateCadDeviationClicked();
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
        QFrame* createRibbonGroup(const QString& title, const QList<QWidget*>& widgets);
        QPushButton* createRibbonButton(const QString& iconSymbol,
            const QString& labelText,
            const QString& accentBg = "#2c313c",
            const QString& borderCol = "#3e4451",
            const QString& textCol = "#dcdfe4");
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
        QString m_currentPortName;
        QString m_machinePackagePath;
        bool m_audioEnabled = true;

        double m_certifiedBallDiameterMm = 25.4000;
        double m_effectiveProbeRadius = 1.5000;
        double m_lastProbeLobingMm = 0.0;
        bool m_isProbeCalibrated = false;

        double m_defaultTolMm = 0.0500;

        bool m_hasArmedCadNominal = false;
        double m_armedCadDiameterMm = 0.0;
        Eigen::Vector3d m_armedCadLocationMm = Eigen::Vector3d::Zero();
        QString m_armedCadDescription;

        WorkplaneFrame m_lastFittedPlaneFrame = WorkplaneFrame::fromStandard(StandardWorkplane::XY_PlusZ);
        bool m_hasCustomPlane = false;

        Eigen::Matrix4d m_pcsMatrix = Eigen::Matrix4d::Identity();
        bool m_isPartAligned = false;
        QPushButton* m_alignStatusBtn = nullptr;

        ::std::vector<Eigen::Vector3d> m_recordedHits;
        ::std::vector<MeasuredFeature> m_solvedFeatures;
        int m_featureCounter = 1;

        QPushButton* m_settingsBtn = nullptr;
        QComboBox* m_workplaneCombo = nullptr;
        QComboBox* m_compModeCombo = nullptr;
        QComboBox* m_simTargetCombo = nullptr;

        QLabel* m_droX = nullptr;
        QLabel* m_droY = nullptr;
        QLabel* m_droZ = nullptr;
        QLabel* m_unitLabels[3] = { nullptr, nullptr, nullptr };
        QLabel* m_hitCountLabel = nullptr;

        QLabel* m_probeStatusBadge = nullptr;
        QLabel* m_probeDiamLabel = nullptr;
        QLabel* m_probeRadLabel = nullptr;
        QLabel* m_probeLobingLabel = nullptr;

        QTreeWidget* m_featureTree = nullptr;
        QTreeWidgetItem* m_activePointsGroup = nullptr;

        ::std::deque<Eigen::Vector3d> m_droHistory;
        Eigen::Vector3d m_lastApproachDir{ 0.0, 0.0, -1.0 };
    };

} // namespace OpenCMM