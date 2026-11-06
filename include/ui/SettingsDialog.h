// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <QDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QStringList>
#include <QPlainTextEdit>
#include <Eigen/Dense>  // <--- Required for Eigen::Vector3d and Eigen::Vector3i
#include <QLabel>

#include "core/Units.h"

namespace OpenCMM {

    class SettingsDialog : public QDialog {
        Q_OBJECT
    public:
        explicit SettingsDialog(QWidget* parent = nullptr);
        ~SettingsDialog() override = default;

        UnitSystem getSelectedUnitSystem() const;
        double getDefaultToleranceMm() const;
        bool areAudioChimesEnabled() const;
        QString getSelectedMachinePath() const;

        Eigen::Vector3d getAxisScale() const;
        Eigen::Vector3i getAxisSign() const;

        void setCurrentUnitSystem(UnitSystem units);
        void setDefaultToleranceMm(double tolMm);
        void setAudioChimesEnabled(bool enabled);
        void setSelectedMachinePath(const QString& path);
        void setAxisCalibration(const Eigen::Vector3d& scale, const Eigen::Vector3i& sign);

        void setAvailableSerialPorts(const QStringList& ports, const QString& currentPort);

        // Inside public: section of SettingsDialog
        bool isDarkModeEnabled() const;
        void setDarkModeEnabled(bool dark);

    public slots:
        void appendSerialLog(const QString& line);
        void updateRawDiagnostics(int32_t rawX, int32_t rawY, int32_t rawZ, bool probeTriggered);

    signals:
        void machinePackageLoaded(const QString& packagePath);
        void machinePackageReset();
        void serialPortConnectRequested(const QString& portName, int baudRate);
        void sendSerialCommandRequested(const QString& cmd);
        void factoryResetRequested();

    private:
        void setupUi();

        QComboBox* m_unitCombo = nullptr;
        QDoubleSpinBox* m_defaultTolSpin = nullptr;
        QCheckBox* m_audioCheck = nullptr;
        QLineEdit* m_machinePathEdit = nullptr;

        QDoubleSpinBox* m_scaleXSpin = nullptr;
        QDoubleSpinBox* m_scaleYSpin = nullptr;
        QDoubleSpinBox* m_scaleZSpin = nullptr;
        QCheckBox* m_invertXCheck = nullptr;
        QCheckBox* m_invertYCheck = nullptr;
        QCheckBox* m_invertZCheck = nullptr;

        QComboBox* m_portCombo = nullptr;
        QComboBox* m_baudCombo = nullptr;
        QPlainTextEdit* m_serialLogView = nullptr;
        QLineEdit* m_cmdInputEdit = nullptr;
        QCheckBox* m_pauseStreamCheck = nullptr;
        QCheckBox* m_filterDroCheck = nullptr;
        QLabel* m_diagRawXLbl = nullptr;
        QLabel* m_diagRawYLbl = nullptr;
        QLabel* m_diagRawZLbl = nullptr;
        QLabel* m_diagProbeLbl = nullptr;
        // Inside private: section of SettingsDialog
        QComboBox* m_themeCombo = nullptr;
    };

} // namespace OpenCMM