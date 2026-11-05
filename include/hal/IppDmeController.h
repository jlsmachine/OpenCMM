// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "hal/ICmmController.h"
#include <QTcpSocket>

namespace OpenCMM {

class IppDmeController : public ICmmController {
    Q_OBJECT
public:
    explicit IppDmeController(const QString& host = "127.0.0.1", quint16 port = 1294, QObject* parent = nullptr);

    bool connectDevice() override;
    void disconnectDevice() override;
    bool isConnected() const override;
    bool isDriven() const override { return true; }

    void moveTo(const Eigen::Vector3d& targetMm, double speedMmPerSec) override;
    void measurePoint(const Eigen::Vector3d& targetMm, const Eigen::Vector3d& approachNormal) override;
    void homeMachine() override;

private slots:
    void onSocketReadyRead();

private:
    QString sendCommand(const QString& commandBody);

    QTcpSocket m_socket;
    QString m_host;
    quint16 m_port;
    int m_tagCounter = 1;
};

} // namespace OpenCMM