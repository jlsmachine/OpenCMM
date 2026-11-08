// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/CalibrationWidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QPainter>
#include <algorithm>

namespace OpenCMM {

    CalibrationWidget::CalibrationWidget(double currentEffectiveRadiusMm, double currentBallDiamMm, QWidget* parent)
        : QDialog(parent),
        m_certifiedBallDiamMm(currentBallDiamMm),
        m_solvedEffectiveRadiusMm(currentEffectiveRadiusMm) {

        setWindowTitle("OpenCMM — Master Ball Probe Calibration Wizard");
        setMinimumWidth(500);
        setMinimumHeight(440);
        setupUi();
        updateStatusDisplay();
    }

    void CalibrationWidget::setupUi() {
        auto* mainLayout = new QVBoxLayout(this);
        mainLayout->setSpacing(14);

        // Instructions Group
        auto* instGroup = new QGroupBox("Instructions & Protocol", this);
        auto* instLayout = new QVBoxLayout(instGroup);
        auto* instLbl = new QLabel(
            "1. Secure the certified Master Calibration Sphere on the CMM table.\n"
            "2. Take at least <b>5 non-coplanar hits</b> distributed around the equator and pole of the ball.\n"
            "3. Click <b>Calculate Calibration</b> to solve effective stylus radius ($R_{\text{eff}}$) and lobing form error.",
            instGroup
        );
        instLbl->setWordWrap(true);
        instLayout->addWidget(instLbl);
        mainLayout->addWidget(instGroup);

        // Parameters & Progress Group
        auto* progGroup = new QGroupBox("Calibration Buffer & Sphere Settings", this);
        auto* formLayout = new QFormLayout(progGroup);

        m_ballDiamSpin = new QDoubleSpinBox(progGroup);
        m_ballDiamSpin->setRange(1.0, 200.0);
        m_ballDiamSpin->setDecimals(4);
        m_ballDiamSpin->setValue(m_certifiedBallDiamMm);
        m_ballDiamSpin->setSuffix(" mm");

        m_hitCountLbl = new QLabel("0 hits recorded (Minimum 5 required)", progGroup);
        m_hitCountLbl->setStyleSheet("font-weight: bold; color: #61afef;");

        m_progressHits = new QProgressBar(progGroup);
        m_progressHits->setRange(0, 5);
        m_progressHits->setValue(0);
        m_progressHits->setTextVisible(true);

        formLayout->addRow("Master Ball Diameter:", m_ballDiamSpin);
        formLayout->addRow("Recorded Hits:", m_hitCountLbl);
        formLayout->addRow("Progress:", m_progressHits);
        mainLayout->addWidget(progGroup);

        // Results Group
        auto* resGroup = new QGroupBox("Calibration Solved Results", this);
        auto* resLayout = new QVBoxLayout(resGroup);

        m_statusBadgeLbl = new QLabel("STATUS: PENDING HITS", resGroup);
        m_statusBadgeLbl->setStyleSheet("font-size: 12px; font-weight: bold; color: #e5c07b;");

        m_effRadResultLbl = new QLabel(QString("Effective Stylus Radius (R_eff): %1 mm").arg(m_solvedEffectiveRadiusMm, 0, 'f', 4), resGroup);
        m_sphericityResultLbl = new QLabel("Sphericity / Lobing Error: --", resGroup);
        m_rmsResultLbl = new QLabel("RMS Residual Fit Error: --", resGroup);

        for (auto* lbl : { m_effRadResultLbl, m_sphericityResultLbl, m_rmsResultLbl }) {
            lbl->setStyleSheet("font-family: Consolas, monospace; font-size: 12px; font-weight: bold; color: #abb2bf;");
        }

        resLayout->addWidget(m_statusBadgeLbl);
        resLayout->addWidget(m_effRadResultLbl);
        resLayout->addWidget(m_sphericityResultLbl);
        resLayout->addWidget(m_rmsResultLbl);
        mainLayout->addWidget(resGroup);

        // Action Buttons Row
        auto* btnRow = new QHBoxLayout();
        m_computeBtn = new QPushButton("Calculate Calibration", this);
        m_computeBtn->setStyleSheet("background-color: #2d5a88; color: white; font-weight: bold; padding: 6px 14px;");
        m_computeBtn->setEnabled(false);

        m_clearBtn = new QPushButton("Clear Hits", this);
        m_clearBtn->setStyleSheet("background-color: #5c2b29; color: white; font-weight: bold; padding: 6px 14px;");

        connect(m_computeBtn, &QPushButton::clicked, this, &CalibrationWidget::onComputeCalibrationClicked);
        connect(m_clearBtn, &QPushButton::clicked, this, &CalibrationWidget::resetCalibrationBuffer);

        btnRow->addWidget(m_clearBtn);
        btnRow->addStretch();
        btnRow->addWidget(m_computeBtn);
        mainLayout->addLayout(btnRow);

        // Dialog Buttons
        auto* dialogButtons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        dialogButtons->button(QDialogButtonBox::Ok)->setEnabled(false);
        connect(dialogButtons, &QDialogButtonBox::accepted, this, &CalibrationWidget::accept);
        connect(dialogButtons, &QDialogButtonBox::rejected, this, &CalibrationWidget::reject);
        mainLayout->addWidget(dialogButtons);

        connect(this, &CalibrationWidget::calibrationCompleted, this, [dialogButtons](double, double) {
            dialogButtons->button(QDialogButtonBox::Ok)->setEnabled(true);
            });
    }

    void CalibrationWidget::addCalibrationHit(const Eigen::Vector3d& hitPos) {
        m_calibHits.push_back(hitPos);
        updateStatusDisplay();
    }

    void CalibrationWidget::resetCalibrationBuffer() {
        m_calibHits.clear();
        m_success = false;
        updateStatusDisplay();
        m_statusBadgeLbl->setText("STATUS: BUFFER CLEARED");
        m_statusBadgeLbl->setStyleSheet("font-size: 12px; font-weight: bold; color: #e5c07b;");
        m_effRadResultLbl->setText("Effective Stylus Radius (R_eff): --");
        m_sphericityResultLbl->setText("Sphericity / Lobing Error: --");
        m_rmsResultLbl->setText("RMS Residual Fit Error: --");

        auto* btnBox = findChild<QDialogButtonBox*>();
        if (btnBox) {
            btnBox->button(QDialogButtonBox::Ok)->setEnabled(false);
        }
    }

    void CalibrationWidget::updateStatusDisplay() {
        const int count = static_cast<int>(m_calibHits.size());
        m_hitCountLbl->setText(QString("%1 hits recorded (Minimum 5 required)").arg(count));
        m_progressHits->setValue(std::min(count, 5));

        if (count >= 5) {
            m_computeBtn->setEnabled(true);
            m_hitCountLbl->setStyleSheet("font-weight: bold; color: #98c379;");
        }
        else {
            m_computeBtn->setEnabled(false);
            m_hitCountLbl->setStyleSheet("font-weight: bold; color: #e5c07b;");
        }
    }

    void CalibrationWidget::onComputeCalibrationClicked() {
        if (m_calibHits.size() < 5) return;

        m_certifiedBallDiamMm = m_ballDiamSpin->value();
        ProbeCalibrationResult res = MetrologyMath::calibrateProbeOnMasterBall(m_calibHits, m_certifiedBallDiamMm);

        if (!res.success) {
            QMessageBox::critical(this, "Calibration Failed", "Sphere fitting failed. Ensure hits are well-distributed across the master sphere.");
            m_statusBadgeLbl->setText("STATUS: SOLVE FAILED (Singular Distribution)");
            m_statusBadgeLbl->setStyleSheet("font-size: 12px; font-weight: bold; color: #e06c75;");
            return;
        }

        m_solvedEffectiveRadiusMm = res.effectiveRadius;
        m_solvedSphericityMm = res.formError;
        m_success = true;

        const int decimals = 4;
        m_effRadResultLbl->setText(QString("Effective Stylus Radius (R_eff): %1 mm").arg(m_solvedEffectiveRadiusMm, 0, 'f', decimals));
        m_sphericityResultLbl->setText(QString("Sphericity / Lobing Error: %1 mm").arg(m_solvedSphericityMm, 0, 'f', decimals));
        m_statusBadgeLbl->setText("STATUS: PASSED & QUALIFIED");
        m_statusBadgeLbl->setStyleSheet("font-size: 12px; font-weight: bold; color: #98c379;");

        emit calibrationCompleted(m_solvedEffectiveRadiusMm, m_solvedSphericityMm);
        QMessageBox::information(this, "Calibration Successful",
            QString("Probe stylus successfully qualified!\n\nEffective Radius (R_eff): %1 mm\nLobing Error: %2 mm")
            .arg(m_solvedEffectiveRadiusMm, 0, 'f', 4)
            .arg(m_solvedSphericityMm, 0, 'f', 4));
    }

} // namespace OpenCMM