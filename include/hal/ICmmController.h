// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QObject>
#include <QString>
#include <cstdint>

#include "core/Units.h" // Provides Position3D

namespace OpenCMM {

    class ICmmController : public QObject {
        Q_OBJECT
    public:
        explicit ICmmController(QObject* parent = nullptr) : QObject(parent) {}
        ~ICmmController() override = default;

        virtual bool connectToPort(const QString& portName, int baudRate = 115200) = 0;
        virtual void disconnectFromPort() = 0;
        virtual bool isConnected() const = 0;
        virtual void zeroAxes() = 0;
        virtual void sendRawCommand(const QString& cmd) {}

    signals:
        void positionUpdated(const OpenCMM::Position3D& pos);
        void probeTriggered(const OpenCMM::Position3D& hitPos);
        void connectionStatusChanged(bool connected, const QString& message);
        void rawLineReceived(const QString& line);
        void rawCountsUpdated(int32_t rawX, int32_t rawY, int32_t rawZ, bool probeState);
    };

} // namespace OpenCMM