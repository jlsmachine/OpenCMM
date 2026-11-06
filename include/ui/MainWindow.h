// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
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
#include <map>
#include <QTimer>
#include "dmis/DMIS53Parser.h"

#include "core/Units.h"
#include "hal/PicoSerialController.h"
#include "math/MetrologyMath.h"
#include "ui/SettingsDialog.h"

namespace OpenCMM {

    class OcctViewport;
    struct CadFaceInfo;
    struct CadSurfaceHit;

    struct DmisPointRecord {
        Eigen::Vector3d surfacePt = Eigen::Vector3d::Zero();
        Eigen::Vector3d normalVec = Eigen::Vector3d::UnitZ();
    };

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
        void onAutoProbeCadFaceClicked();
        void onLoadDmisProgramClicked();
        void onToggleAutoRunDmisClicked();
        void onDmisPlaybackTick();
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
        void onThemeToggled(bool dark);
    protected:
        void closeEvent(QCloseEvent* event) override;
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
        void onCadSurfaceProbed(const CadSurfaceHit& hit);
        void startDmisGuidedFeature(int featureIndex);
        void updateDmisTargetVisuals();
        void stopDmisAutoRun();

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
        Eigen::Vector3d m_armedCadNormalMm = Eigen::Vector3d::UnitZ();
        bool m_armedCadIsInternal = false;
        QString m_armedCadDescription;

        WorkplaneFrame m_lastFittedPlaneFrame = WorkplaneFrame::fromStandard(StandardWorkplane::XY_PlusZ);
        bool m_hasCustomPlane = false;

        Eigen::Matrix4d m_pcsMatrix = Eigen::Matrix4d::Identity();
        bool m_isPartAligned = false;
        QPushButton* m_alignStatusBtn = nullptr;

        ::std::vector<Eigen::Vector3d> m_recordedHits;
        ::std::vector<DmisPointRecord> m_recordedDmisHits;
        ::std::map<int, ::std::vector<DmisPointRecord>> m_featureDmisHits;
        ::std::map<int, QString> m_featureRefIds;
        ::std::vector<MeasuredFeature> m_solvedFeatures;

        // DMIS 5.3 Guided Execution & Offline Playback State
        DmisProgramData m_dmisProgram;
        bool m_isDmisProgramActive = false;
        int m_activeDmisFeatureIdx = -1;
        ::std::vector<QTreeWidgetItem*> m_plannedDmisTreeItems;
        QTreeWidgetItem* m_activeDmisTreeItem = nullptr;
        QString m_activeDmisFeatureName;
        QString m_activeDmisRefId;
        bool m_suppressDmisStepAdvance = false;

        QTimer* m_dmisPlaybackTimer = nullptr;
        QPushButton* m_autoRunDmisBtnHome = nullptr;
        QPushButton* m_autoRunDmisBtnCad = nullptr;
        int m_playbackPhase = 0; // 0 = Clearance/Approach, 1 = Touch, 2 = Retract
        int m_playbackSubStep = 0;
        Eigen::Vector3d m_playbackStartPos = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_playbackEndPos = Eigen::Vector3d::Zero();
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

        QComboBox* m_featureRenderCombo = nullptr;

        void onFeatureTreeContextMenuRequested(const QPoint& pos);
        void onDeleteSelectedFeature();
        void onClearAllFeaturesClicked();
        bool m_isDarkMode = true;
        QWidget* m_droContainer = nullptr;
        QWidget* m_treeContainer = nullptr;
    };

} // namespace OpenCMM