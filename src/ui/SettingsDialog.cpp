#include "ui/SettingsDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QDialogButtonBox>
#include <QFormLayout>

namespace OpenCMM {

    SettingsDialog::SettingsDialog(QWidget* parent) : QDialog(parent) {
        setWindowTitle("OpenCMM - Studio Settings");
        setMinimumWidth(500);
        setMinimumHeight(380);
        setStyleSheet(
            "QDialog { background-color: #21252b; color: #e0e0e0; }"
            "QTabWidget::pane { border: 1px solid #3a3f4b; background: #1e2227; border-radius: 4px; }"
            "QTabBar::tab { background: #282c34; color: #abb2bf; padding: 8px 16px; font-weight: bold; border-top-left-radius: 4px; border-top-right-radius: 4px; border: 1px solid #3a3f4b; }"
            "QTabBar::tab:selected { background: #1e2227; color: #ffffff; border-bottom: 2px solid #61afef; }"
            "QGroupBox { border: 1px solid #3a3f4b; border-radius: 4px; margin-top: 10px; padding-top: 14px; color: #61afef; font-weight: bold; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
            "QLabel { color: #dcdfe4; font-size: 12px; font-weight: bold; }"
            "QLineEdit, QComboBox, QDoubleSpinBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 5px; font-weight: bold; }"
            "QComboBox QAbstractItemView { background-color: #181a1f; color: #ffffff; selection-background-color: #2d5a88; }"
            "QPushButton { background-color: #3a3f4b; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 6px 14px; font-weight: bold; }"
            "QPushButton:hover { background-color: #4b5263; }"
            "QCheckBox { color: #dcdfe4; font-weight: bold; spacing: 6px; }"
        );

        setupUi();
    }

    void SettingsDialog::setupUi() {
        auto* mainLayout = new QVBoxLayout(this);
        mainLayout->setContentsMargins(12, 12, 12, 12);

        auto* tabWidget = new QTabWidget(this);

        // ==========================================
        // TAB 1: Machine Info & Hardware Connection
        // ==========================================
        auto* tabMachine = new QWidget(this);
        auto* machineLayout = new QVBoxLayout(tabMachine);
        machineLayout->setSpacing(14);

        auto* machineGroup = new QGroupBox("Digital Twin Machine Post (.cmm Archive)", tabMachine);
        auto* machineGrpLayout = new QVBoxLayout(machineGroup);

        auto* fileRow = new QHBoxLayout();
        m_machinePathEdit = new QLineEdit(machineGroup);
        m_machinePathEdit->setReadOnly(true);
        m_machinePathEdit->setPlaceholderText("No custom .cmm package loaded (using default bridge)");

        auto* browseBtn = new QPushButton("Browse .cmm...", machineGroup);
        browseBtn->setStyleSheet("background-color: #2d5a88; color: white;");
        connect(browseBtn, &QPushButton::clicked, this, [this]() {
            QString filePath = QFileDialog::getOpenFileName(
                this, "Load Machine Post Package", QString(), "OpenCMM Machine Packages (*.cmm);;All Files (*.*)"
            );
            if (!filePath.isEmpty()) {
                m_machinePathEdit->setText(filePath);
                emit machinePackageLoaded(filePath);
            }
            });

        fileRow->addWidget(m_machinePathEdit, 1);
        fileRow->addWidget(browseBtn);
        machineGrpLayout->addLayout(fileRow);
        machineLayout->addWidget(machineGroup);

        auto* serialGroup = new QGroupBox("Pico 2 Serial Hardware Connection", tabMachine);
        auto* serialLayout = new QVBoxLayout(serialGroup);

        auto* portRow = new QHBoxLayout();
        m_portCombo = new QComboBox(serialGroup);
        m_baudCombo = new QComboBox(serialGroup);
        m_baudCombo->addItems({ "9600", "19200", "38400", "57600", "115200", "230400" });
        m_baudCombo->setCurrentText("115200");

        portRow->addWidget(new QLabel("Port:", serialGroup));
        portRow->addWidget(m_portCombo, 1);
        portRow->addWidget(new QLabel("Baud:", serialGroup));
        portRow->addWidget(m_baudCombo);

        auto* connectTestBtn = new QPushButton("Connect Port", serialGroup);
        connectTestBtn->setStyleSheet("background-color: #28a745; color: white;");
        connect(connectTestBtn, &QPushButton::clicked, this, [this]() {
            emit serialPortConnectRequested(m_portCombo->currentData().toString(), m_baudCombo->currentText().toInt());
            });
        portRow->addWidget(connectTestBtn);

        serialLayout->addLayout(portRow);
        machineLayout->addWidget(serialGroup);
        machineLayout->addStretch();
        tabWidget->addTab(tabMachine, "Machine & Hardware");

        // ==========================================
        // TAB 2: Preferences, Units & Tolerances
        // ==========================================
        auto* tabPrefs = new QWidget(this);
        auto* prefsLayout = new QVBoxLayout(tabPrefs);
        prefsLayout->setSpacing(14);

        auto* prefsGroup = new QGroupBox("Metrology & Display Defaults", tabPrefs);
        auto* prefsForm = new QFormLayout(prefsGroup);
        prefsForm->setSpacing(12);

        m_unitCombo = new QComboBox(prefsGroup);
        m_unitCombo->addItem("Millimeters (mm)", static_cast<int>(UnitSystem::Millimeters));
        m_unitCombo->addItem("Inches (in)", static_cast<int>(UnitSystem::Inches));

        m_defaultTolSpin = new QDoubleSpinBox(prefsGroup);
        m_defaultTolSpin->setRange(0.0001, 10.0);
        m_defaultTolSpin->setDecimals(4);
        m_defaultTolSpin->setSingleStep(0.0100);

        m_audioCheck = new QCheckBox("Enable Eyes-Free Audio Chimes (Probe hits & feature completion)", prefsGroup);

        prefsForm->addRow("Default Unit System:", m_unitCombo);
        prefsForm->addRow("Default Bilateral Tolerance:", m_defaultTolSpin);
        prefsForm->addRow(m_audioCheck);

        prefsLayout->addWidget(prefsGroup);
        prefsLayout->addStretch();
        tabWidget->addTab(tabPrefs, "Preferences & Tolerances");

        mainLayout->addWidget(tabWidget);

        // Dialog Buttons
        auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        connect(buttonBox, &QDialogButtonBox::accepted, this, [this]() {
            emit settingsApplied();
            accept();
            });
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
        mainLayout->addWidget(buttonBox);
    }

    UnitSystem SettingsDialog::getSelectedUnitSystem() const {
        return static_cast<UnitSystem>(m_unitCombo->currentData().toInt());
    }

    double SettingsDialog::getDefaultToleranceMm() const {
        return m_defaultTolSpin->value();
    }

    bool SettingsDialog::areAudioChimesEnabled() const {
        return m_audioCheck->isChecked();
    }

    QString SettingsDialog::getSelectedMachinePath() const {
        return m_machinePathEdit->text();
    }

    void SettingsDialog::setCurrentUnitSystem(UnitSystem units) {
        m_unitCombo->setCurrentIndex(units == UnitSystem::Millimeters ? 0 : 1);
    }

    void SettingsDialog::setDefaultToleranceMm(double tolMm) {
        m_defaultTolSpin->setValue(tolMm);
    }

    void SettingsDialog::setAudioChimesEnabled(bool enabled) {
        m_audioCheck->setChecked(enabled);
    }

    void SettingsDialog::setAvailableSerialPorts(const QStringList& ports, const QString& currentPort) {
        m_portCombo->clear();
        for (const QString& p : ports) {
            m_portCombo->addItem(p, p);
        }
        if (!currentPort.isEmpty()) {
            m_portCombo->setCurrentText(currentPort);
        }
    }

} // namespace OpenCMM