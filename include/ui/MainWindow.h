// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QTreeWidget>
#include <QTimer>
#include <QFrame>
#include <QCloseEvent>
#include <QTextEdit>
#include <QToolBar>
#include <QDockWidget>
#include <QAction>
#include <QEvent>

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <QPluginLoader>

#include "mcp/IMcpPlugin.h"
#include "hal/PicoSerialController.h"
#include "math/MetrologyMath.h"
#include "dmis/DMIS53Parser.h"
#include "core/Units.h"
#include "dmis/DmisParser.h" // Or the exact header file where DmisFeatureData is declared


namespace OpenCMM {

    class OcctViewport;
    struct CadFaceInfo;
    struct CadSurfaceHit;

    struct DmisRecordedHit {
        Eigen::Vector3d surfacePt = Eigen::Vector3d::Zero();
        Eigen::Vector3d normalVec = Eigen::Vector3d::UnitZ();
    };

    class MainWindow : public QMainWindow, public IMcpHostBridge {
        Q_OBJECT
    public:
        explicit MainWindow(QWidget* parent = nullptr);
        ~MainWindow() override;
        // IMcpHostBridge implementation
        QJsonObject getMachineStatusJson() override;
        QJsonArray getSolvedFeaturesJson() override;
        QJsonArray getPlannedDmisFeaturesJson() override;
        QJsonObject runPreFlightCheckJson() override;
        QJsonObject createOfflineDmisFeatureJson(const QJsonObject& params) override;
        QJsonObject getCadFacesJson(const QJsonObject& filter = QJsonObject()) override;
        QJsonObject runDmisPlaybackJson(const QJsonObject& params = QJsonObject()) override;
        QJsonObject exportInspectionFileJson(const QJsonObject& params) override;
        QJsonObject solve321AlignmentJson(const QJsonObject& params) override;

    protected:
        void closeEvent(QCloseEvent* event) override;
        bool eventFilter(QObject* watched, QEvent* event) override;

    private slots:
        void onOpenSettingsClicked();
        void onConnectionChanged(bool connected, const QString& message);
        void onZeroAxesClicked();
        void onUnitToggled();
        void onLoadStepClicked();
        void onTransformCadClicked();
        void onCadFaceSelected(const CadFaceInfo& faceInfo);
        void onCadSurfaceProbed(const CadSurfaceHit& hit);
        void onAutoProbeCadFaceClicked();
        void onCreateOfflineFeatureClicked();
        void onAssignCadDatumClicked(const QString& datumLetter);

        void onLoadDmisProgramClicked();
        void onExportDmisProgramClicked();
        void onToggleAutoRunDmisClicked();
        void onDmisPlaybackTick();

        void onPositionUpdated(const Position3D& pos);
        void onProbeHit(const Position3D& hitPos);

        void onExportReportClicked();
        void onSimulateHitClicked();
        void onCalibrateProbeClicked();

        void onFitPlaneClicked();
        void onFitSphereClicked();
        void onFitCircleClicked();
        void onFitLineClicked();
        void onFitCylinderClicked();
        void onMeasureRelationClicked();
        void onRun321AlignmentClicked();
        void onEditToleranceClicked();
        void onClearPointsClicked();
        void onEvaluateCadDeviationClicked();

        void onFeatureTreeContextMenuRequested(const QPoint& pos);
        void onDeleteSelectedFeature();
        void onClearAllFeaturesClicked();
        void onThemeToggled(bool dark);

        void onRibbonContextMenuRequested(const QPoint& pos);
        void onCustomizeRibbonClicked();
        void resetWindowLayout();
        void onConfigureShortcutsClicked(); // <--- Add this line
        void onRunDmisPreFlightCheckClicked();
        void onToggleMcpServerClicked();
        void applyKeyboardShortcuts();
    

    private:
        void setupUi();
        void setupToolbar();
        void setupAutoHideBars();
        void setupDroDock();
        void setupFeatureTreeDock();
        void setupCollisionLogDock();

        void attachPinTitleBar(QDockWidget* dock, Qt::DockWidgetArea area, bool startPinned);
        void setDockPinned(QDockWidget* dock, bool pinned);
        void syncAutoHideTabForDock(QDockWidget* dock);

        void evaluateProbeCollisionSafety(const Eigen::Vector3d& pos);
        void logCollisionEvent(const std::string& message, bool isCollision, bool isWarning);

        void loadRibbonCustomization();
        QFrame* createRibbonGroup(const QString& title, const QList<QWidget*>& widgets);
        QPushButton* createRibbonButton(
            const QString& iconSymbol,
            const QString& labelText,
            const QString& accentBg = "#282c34",
            const QString& borderCol = "#3a3f4b",
            const QString& textCol = "#abb2bf"
        );
        QWidget* createCommandWidgetForId(const QString& cmdId);

        void updateDroDisplay();
        void updateProbeStatusDisplay();
        void finishFeatureBuffer();

        void playHitChime() const;
        void playSolveChime(bool passed) const;

        void applyEvaluationToTreeItem(
            QTreeWidgetItem* item,
            double actualMm,
            bool hasNominal,
            double nominalMm,
            double plusTolMm,
            double minusTolMm
        );

        ProbeCompMode currentCompMode() const;
        WorkplaneFrame currentWorkplane() const;

        void assignFeatureAsDatum(int featureId, const QString& datumLetter, bool autoSolve321);
        bool tryAutoSolve321Alignment(bool showFailureDialog);

        void startDmisGuidedFeature(int featureIndex);
        void updateDmisTargetVisuals();
        void stopDmisAutoRun();
        bool writeDmisProgramToFile(const QString& filePath, const QString& partName);

        PicoSerialController* m_controller = nullptr;
        OcctViewport* m_viewport = nullptr;

        QDockWidget* m_droDock = nullptr;
        QDockWidget* m_treeDock = nullptr;
        QDockWidget* m_collisionDock = nullptr;
        QTextEdit* m_collisionLogText = nullptr;

        QToolBar* m_leftAutoHideBar = nullptr;
        QToolBar* m_bottomAutoHideBar = nullptr;
        std::map<QDockWidget*, bool> m_dockPinned;
        std::map<QDockWidget*, QPushButton*> m_dockPinButtons;
        std::map<QDockWidget*, QAction*> m_dockTabActions;

        QWidget* m_droContainer = nullptr;
        QWidget* m_treeContainer = nullptr;
        QWidget* m_collisionContainer = nullptr;

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

        QPushButton* m_settingsBtn = nullptr;
        QComboBox* m_simTargetCombo = nullptr;
        QComboBox* m_compModeCombo = nullptr;
        QComboBox* m_workplaneCombo = nullptr;
        QComboBox* m_featureRenderCombo = nullptr;
        QPushButton* m_alignStatusBtn = nullptr;
        QPushButton* m_autoRunDmisBtnHome = nullptr;
        QPushButton* m_autoRunDmisBtnCad = nullptr;

        UnitSystem m_unitSystem = UnitSystem::Millimeters;
        double m_defaultTolMm = 0.025;
        bool m_audioEnabled = true;
        bool m_isDarkMode = true;

        QString m_machinePackagePath;
        QString m_currentPortName;

        Position3D m_lastPosition{ 0.0, 0.0, 0.0 };
        Eigen::Vector3d m_lastApproachDir{ 0.0, 0.0, -1.0 };

        bool m_isProbeCalibrated = false;
        double m_certifiedBallDiameterMm = 25.4000;
        double m_effectiveProbeRadius = 1.5000;
        double m_lastProbeLobingMm = 0.0;

        std::vector<Eigen::Vector3d> m_recordedHits;
        std::vector<DmisRecordedHit> m_recordedDmisHits;
        std::vector<MeasuredFeature> m_solvedFeatures;
        std::map<int, std::vector<DmisRecordedHit>> m_featureDmisHits;
        std::map<int, QString> m_featureRefIds;
        std::map<int, QString> m_featureDatumLetters;
        int m_featureCounter = 1;

        int m_datumAFeatureId = -1;
        int m_datumBFeatureId = -1;
        int m_datumCFeatureId = -1;
        bool m_isPartAligned = false;
        Eigen::Matrix4d m_pcsMatrix = Eigen::Matrix4d::Identity();

        bool m_hasCustomPlane = false;
        WorkplaneFrame m_lastFittedPlaneFrame;

        bool m_hasArmedCadNominal = false;
        double m_armedCadDiameterMm = 0.0;
        Eigen::Vector3d m_armedCadLocationMm = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_armedCadNormalMm = Eigen::Vector3d::UnitZ();
        bool m_armedCadIsInternal = false;
        QString m_armedCadDescription;

        DmisProgramData m_dmisProgram;
        bool m_isDmisProgramActive = false;
        int m_activeDmisFeatureIdx = -1;
        std::vector<QTreeWidgetItem*> m_plannedDmisTreeItems;
        QTreeWidgetItem* m_activeDmisTreeItem = nullptr;
        QString m_activeDmisFeatureName;
        QString m_activeDmisRefId;
        bool m_suppressDmisStepAdvance = false;

        QTimer* m_dmisPlaybackTimer = nullptr;
        int m_playbackPhase = 0;
        int m_playbackSubStep = 0;
        Eigen::Vector3d m_playbackStartPos = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_playbackEndPos = Eigen::Vector3d::Zero();
        bool validateDmisFeaturePath(int featureIndex);
        void tryLoadMcpPlugin();

        QPluginLoader* m_mcpPluginLoader = nullptr;
        IMcpServerPlugin* m_mcpPlugin = nullptr;
        void updateMcpButtonStyle();
        QPushButton* m_mcpStatusBtn = nullptr;
        
        std::vector<QShortcut*> m_appShortcuts;
    };

} // namespace OpenCMM