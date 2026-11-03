#pragma once
#include <QDialog>
#include <QComboBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QStringList>
#include <QLabel>
#include <QPlainTextEdit>
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

        void setCurrentUnitSystem(UnitSystem units);
        void setDefaultToleranceMm(double tolMm);
        void setAudioChimesEnabled(bool enabled);
        void setSelectedMachinePath(const QString& path);
        void setAvailableSerialPorts(const QStringList& ports, const QString& currentPort);

        void appendSerialLog(const QString& line);
        void updateRawDiagnostics(int32_t rawX, int32_t rawY, int32_t rawZ, bool probeTriggered);

    signals:
        void machinePackageLoaded(const QString& packagePath);
        void machinePackageReset();
        void serialPortConnectRequested(const QString& portName, int baudRate);
        void settingsApplied();
        void sendSerialCommandRequested(const QString& cmd);

    private:
        void setupUi();

        QLineEdit* m_machinePathEdit = nullptr;
        QComboBox* m_portCombo = nullptr;
        QComboBox* m_baudCombo = nullptr;

        QComboBox* m_unitCombo = nullptr;
        QDoubleSpinBox* m_defaultTolSpin = nullptr;
        QCheckBox* m_audioCheck = nullptr;

        QLabel* m_diagRawXLbl = nullptr;
        QLabel* m_diagRawYLbl = nullptr;
        QLabel* m_diagRawZLbl = nullptr;
        QLabel* m_diagProbeLbl = nullptr;
        QPlainTextEdit* m_serialLogView = nullptr;
        QCheckBox* m_pauseStreamCheck = nullptr;
        QCheckBox* m_filterDroCheck = nullptr;
        QLineEdit* m_cmdInputEdit = nullptr;
    };

} // namespace OpenCMM