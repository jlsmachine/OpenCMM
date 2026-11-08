// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QProgressBar>
#include <vector>
#include <Eigen/Dense>
#include "math/MetrologyMath.h"

namespace OpenCMM {

    class CalibrationWidget : public QDialog {
        Q_OBJECT
    public:
        explicit CalibrationWidget(double currentEffectiveRadiusMm, double currentBallDiamMm, QWidget* parent = nullptr);
        ~CalibrationWidget() override = default;

        double getEffectiveProbeRadius() const { return m_solvedEffectiveRadiusMm; }
        double getCertifiedBallDiameter() const { return m_certifiedBallDiamMm; }
        double getSphericityError() const { return m_solvedSphericityMm; }
        bool isCalibrationSuccessful() const { return m_success; }

    public slots:
        void addCalibrationHit(const Eigen::Vector3d& hitPos);
        void resetCalibrationBuffer();

    signals:
        void calibrationCompleted(double effectiveRadiusMm, double sphericityMm);

    private slots:
        void onComputeCalibrationClicked();

    private:
        void setupUi();
        void updateStatusDisplay();

        double m_certifiedBallDiamMm = 25.4000;
        double m_solvedEffectiveRadiusMm = 1.5000;
        double m_solvedSphericityMm = 0.0;
        bool m_success = false;

        std::vector<Eigen::Vector3d> m_calibHits;

        QDoubleSpinBox* m_ballDiamSpin = nullptr;
        QLabel* m_hitCountLbl = nullptr;
        QLabel* m_statusBadgeLbl = nullptr;
        QLabel* m_effRadResultLbl = nullptr;
        QLabel* m_sphericityResultLbl = nullptr;
        QLabel* m_rmsResultLbl = nullptr;
        QProgressBar* m_progressHits = nullptr;
        QPushButton* m_computeBtn = nullptr;
        QPushButton* m_clearBtn = nullptr;
    };

} // namespace OpenCMM