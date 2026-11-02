#pragma once
#include <QDialog>
#include <QComboBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QStringList>
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

    signals:
        void machinePackageLoaded(const QString& packagePath);
        void machinePackageReset();
        void serialPortConnectRequested(const QString& portName, int baudRate);
        void settingsApplied();

    private:
        void setupUi();

        QLineEdit* m_machinePathEdit = nullptr;
        QComboBox* m_portCombo = nullptr;
        QComboBox* m_baudCombo = nullptr;

        QComboBox* m_unitCombo = nullptr;
        QDoubleSpinBox* m_defaultTolSpin = nullptr;
        QCheckBox* m_audioCheck = nullptr;
    };

} // namespace OpenCMM