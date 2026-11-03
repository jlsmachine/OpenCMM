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
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QCheckBox>
#include <QGridLayout>

namespace OpenCMM {

    SettingsDialog::SettingsDialog(QWidget* parent) : QDialog(parent) {
        setWindowTitle("OpenCMM - Studio Settings");
        setMinimumWidth(580);
        setMinimumHeight(540);
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
        // TAB 1: Machine & Hardware (Digital Twin)
        // ==========================================
        auto* tabMachine = new QWidget(this);
        auto* machineLayout = new QVBoxLayout(tabMachine);
        machineLayout->setSpacing(14);

        auto* machineGroup = new QGroupBox("Digital Twin Machine Post (.cmm Archive or .cmm.json)", tabMachine);
        auto* machineGrpLayout = new QVBoxLayout(machineGroup);

        auto* fileRow = new QHBoxLayout();
        m_machinePathEdit = new QLineEdit(machineGroup);
        m_machinePathEdit->setReadOnly(true);
        m_machinePathEdit->setPlaceholderText("Default Procedural Bridge CMM Active");

        auto* browseBtn = new QPushButton("Browse .cmm...", machineGroup);
        browseBtn->setStyleSheet("background-color: #2d5a88; color: white;");
        connect(browseBtn, &QPushButton::clicked, this, [this]() {
            QString filePath = QFileDialog::getOpenFileName(
                this,
                "Load Machine Post Package",
                QString(),
                "CMM Machine Packages (*.cmm *.cmm.json *.json *.zip);;All Files (*.*)"
            );
            if (!filePath.isEmpty()) {
                m_machinePathEdit->setText(filePath);
                emit machinePackageLoaded(filePath);
            }
            });

        auto* resetMachineBtn = new QPushButton("Reset Default", machineGroup);
        connect(resetMachineBtn, &QPushButton::clicked, this, [this]() {
            m_machinePathEdit->clear();
            emit machinePackageReset();
            });

        fileRow->addWidget(m_machinePathEdit, 1);
        fileRow->addWidget(browseBtn);
        fileRow->addWidget(resetMachineBtn);
        machineGrpLayout->addLayout(fileRow);
        machineLayout->addWidget(machineGroup);
        machineLayout->addStretch();
        tabWidget->addTab(tabMachine, "Machine & Hardware");

        // ==========================================
        // TAB 2: Serial Diagnostics & Console
        // ==========================================
        auto* tabDiag = new QWidget(this);
        auto* diagLayout = new QVBoxLayout(tabDiag);
        diagLayout->setSpacing(10);

        // 1. Connection Group Box (Moved from Tab 1)
        auto* serialGroup = new QGroupBox("Pico 2 Serial Hardware Connection", tabDiag);
        auto* serialLayout = new QHBoxLayout(serialGroup);

        m_portCombo = new QComboBox(serialGroup);
        m_baudCombo = new QComboBox(serialGroup);
        m_baudCombo->addItems({ "9600", "19200", "38400", "57600", "115200", "230400" });
        m_baudCombo->setCurrentText("115200");

        auto* connectTestBtn = new QPushButton("Connect Port", serialGroup);
        connectTestBtn->setStyleSheet("background-color: #28a745; color: white;");
        connect(connectTestBtn, &QPushButton::clicked, this, [this]() {
            emit serialPortConnectRequested(m_portCombo->currentData().toString(), m_baudCombo->currentText().toInt());
            });

        serialLayout->addWidget(new QLabel("Port:", serialGroup));
        serialLayout->addWidget(m_portCombo, 1);
        serialLayout->addWidget(new QLabel("Baud:", serialGroup));
        serialLayout->addWidget(m_baudCombo);
        serialLayout->addWidget(connectTestBtn);
        diagLayout->addWidget(serialGroup);

        // 2. Raw Encoder Counts & Probe Status
        auto* diagGroup = new QGroupBox("Pico 2 Raw Encoder & Serial Diagnostics", tabDiag);
        auto* diagGrpLayout = new QVBoxLayout(diagGroup);

        auto* countsGrid = new QGridLayout();
        m_diagRawXLbl = new QLabel("X Raw: 0 cts", diagGroup);
        m_diagRawYLbl = new QLabel("Y Raw: 0 cts", diagGroup);
        m_diagRawZLbl = new QLabel("Z Raw: 0 cts", diagGroup);
        m_diagProbeLbl = new QLabel("PROBE: SEATED (NC)", diagGroup);

        const QString countStyle = "background-color: #1e2227; color: #61afef; font-family: Consolas, monospace; font-size: 12px; font-weight: bold; padding: 6px; border: 1px solid #3a3f4b; border-radius: 4px;";
        m_diagRawXLbl->setStyleSheet(countStyle);
        m_diagRawYLbl->setStyleSheet(countStyle);
        m_diagRawZLbl->setStyleSheet(countStyle);
        m_diagProbeLbl->setStyleSheet("background-color: #1e3a2f; color: #98c379; font-family: Consolas, monospace; font-size: 12px; font-weight: bold; padding: 6px; border: 1px solid #2f855a; border-radius: 4px;");

        countsGrid->addWidget(m_diagRawXLbl, 0, 0);
        countsGrid->addWidget(m_diagRawYLbl, 0, 1);
        countsGrid->addWidget(m_diagRawZLbl, 0, 2);
        countsGrid->addWidget(m_diagProbeLbl, 0, 3);
        diagGrpLayout->addLayout(countsGrid);

        auto* filterRow = new QHBoxLayout();
        m_pauseStreamCheck = new QCheckBox("Pause Log Scroll", diagGroup);
        m_filterDroCheck = new QCheckBox("Hide 60Hz 'dro' Packets", diagGroup);
        auto* pingBtn = new QPushButton("Send PING", diagGroup);
        auto* zeroPicoBtn = new QPushButton("Send ZERO", diagGroup);
        auto* clearLogBtn = new QPushButton("Clear Log", diagGroup);

        filterRow->addWidget(m_pauseStreamCheck);
        filterRow->addWidget(m_filterDroCheck);
        filterRow->addStretch();
        filterRow->addWidget(pingBtn);
        filterRow->addWidget(zeroPicoBtn);
        filterRow->addWidget(clearLogBtn);
        diagGrpLayout->addLayout(filterRow);

        m_serialLogView = new QPlainTextEdit(diagGroup);
        m_serialLogView->setReadOnly(true);
        m_serialLogView->setMaximumBlockCount(300);
        m_serialLogView->setMinimumHeight(160);
        m_serialLogView->setStyleSheet("background-color: #15171c; color: #abb2bf; font-family: Consolas, monospace; font-size: 11px; border: 1px solid #3a3f4b;");
        diagGrpLayout->addWidget(m_serialLogView);

        auto* cmdRow = new QHBoxLayout();
        m_cmdInputEdit = new QLineEdit(diagGroup);
        m_cmdInputEdit->setPlaceholderText("Type Pico 2 command (e.g. PING, ZERO, STREAM_OFF, STREAM_ON)...");
        auto* sendCmdBtn = new QPushButton("Send", diagGroup);
        cmdRow->addWidget(m_cmdInputEdit, 1);
        cmdRow->addWidget(sendCmdBtn);
        diagGrpLayout->addLayout(cmdRow);

        diagLayout->addWidget(diagGroup);
        tabWidget->addTab(tabDiag, "Serial Diagnostics");

        connect(pingBtn, &QPushButton::clicked, this, [this]() {
            emit sendSerialCommandRequested("PING");
            });
        connect(zeroPicoBtn, &QPushButton::clicked, this, [this]() {
            emit sendSerialCommandRequested("ZERO");
            });
        connect(clearLogBtn, &QPushButton::clicked, m_serialLogView, &QPlainTextEdit::clear);
        connect(sendCmdBtn, &QPushButton::clicked, this, [this]() {
            if (!m_cmdInputEdit->text().trimmed().isEmpty()) {
                emit sendSerialCommandRequested(m_cmdInputEdit->text().trimmed());
                m_cmdInputEdit->clear();
            }
            });
        connect(m_cmdInputEdit, &QLineEdit::returnPressed, sendCmdBtn, &QPushButton::click);

        // ==========================================
        // TAB 3: Preferences, Units & Tolerances
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
        prefsForm->addRow("Default Bilateral Tolerance (mm):", m_defaultTolSpin);
        prefsForm->addRow(m_audioCheck);

        prefsLayout->addWidget(prefsGroup);
        prefsLayout->addStretch();
        tabWidget->addTab(tabPrefs, "Preferences & Tolerances");

        mainLayout->addWidget(tabWidget);

        auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        connect(buttonBox, &QDialogButtonBox::accepted, this, [this]() {
            emit settingsApplied();
            accept();
            });
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
        mainLayout->addWidget(buttonBox);
    }

    // ... getters, setters, and diagnostic update slots remain identical ...
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

    void SettingsDialog::setSelectedMachinePath(const QString& path) {
        m_machinePathEdit->setText(path);
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

    void SettingsDialog::appendSerialLog(const QString& line) {
        if (!m_serialLogView || (m_pauseStreamCheck && m_pauseStreamCheck->isChecked())) {
            return;
        }
        if (m_filterDroCheck && m_filterDroCheck->isChecked() && line.contains("\"type\":\"dro\"")) {
            return;
        }
        m_serialLogView->appendPlainText(line);
    }

    void SettingsDialog::updateRawDiagnostics(int32_t rawX, int32_t rawY, int32_t rawZ, bool probeTriggered) {
        if (m_diagRawXLbl) m_diagRawXLbl->setText(QString("X Raw: %1 cts").arg(rawX));
        if (m_diagRawYLbl) m_diagRawYLbl->setText(QString("Y Raw: %1 cts").arg(rawY));
        if (m_diagRawZLbl) m_diagRawZLbl->setText(QString("Z Raw: %1 cts").arg(rawZ));

        if (m_diagProbeLbl) {
            if (probeTriggered) {
                m_diagProbeLbl->setText("PROBE: TRIGGERED (HIT)");
                m_diagProbeLbl->setStyleSheet("background-color: #4b1d1d; color: #ff6b6b; font-family: Consolas, monospace; font-size: 12px; font-weight: bold; padding: 6px; border: 1px solid #e06c75; border-radius: 4px;");
            }
            else {
                m_diagProbeLbl->setText("PROBE: SEATED (NC)");
                m_diagProbeLbl->setStyleSheet("background-color: #1e3a2f; color: #98c379; font-family: Consolas, monospace; font-size: 12px; font-weight: bold; padding: 6px; border: 1px solid #2f855a; border-radius: 4px;");
            }
        }
    }

} // namespace OpenCMM