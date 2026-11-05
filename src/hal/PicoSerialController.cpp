// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#include "hal/PicoSerialController.h"
#include <nlohmann/json.hpp>

namespace OpenCMM {

    PicoSerialController::PicoSerialController(QObject* parent)
        : ICmmController(parent),
        m_serial(new QSerialPort(this)) {
        connect(m_serial, &QSerialPort::readyRead, this, &PicoSerialController::onReadyRead);
        connect(m_serial, &QSerialPort::errorOccurred, this, &PicoSerialController::onSerialError);
    }

    PicoSerialController::~PicoSerialController() {
        disconnectFromPort();
    }

    void PicoSerialController::setAxisCalibration(const Eigen::Vector3d& scaleMm, const Eigen::Vector3i& sign) {
        m_scaleMm = scaleMm;
        m_sign = sign;
    }

    bool PicoSerialController::connectToPort(const QString& portName, int baudRate) {
        if (m_serial->isOpen()) {
            m_serial->close();
        }

        m_serial->setPortName(portName);
        m_serial->setBaudRate(baudRate);
        m_serial->setDataBits(QSerialPort::Data8);
        m_serial->setParity(QSerialPort::NoParity);
        m_serial->setStopBits(QSerialPort::OneStop);
        m_serial->setFlowControl(QSerialPort::NoFlowControl);

        if (m_serial->open(QIODevice::ReadWrite)) {
            m_serial->setDataTerminalReady(true);
            m_serial->setRequestToSend(true);
            m_buffer.clear();
            m_lastHitId = -1;

            m_serial->write("STREAM_ON\n");

            emit connectionStatusChanged(true, "Connected to Pico 2 on " + portName);
            return true;
        }

        emit connectionStatusChanged(false, "Failed to open " + portName + ": " + m_serial->errorString());
        return false;
    }

    void PicoSerialController::disconnectFromPort() {
        if (m_serial->isOpen()) {
            m_serial->close();
            emit connectionStatusChanged(false, "Disconnected from serial port.");
        }
    }

    bool PicoSerialController::isConnected() const {
        return m_serial && m_serial->isOpen();
    }

    void PicoSerialController::zeroAxes() {
        m_offsetX = m_rawX;
        m_offsetY = m_rawY;
        m_offsetZ = m_rawZ;

        if (isConnected()) {
            m_serial->write("ZERO\n");
        }

        Position3D zeroPos{ 0.0, 0.0, 0.0 };
        emit positionUpdated(zeroPos);
    }

    void PicoSerialController::sendRawCommand(const QString& cmd) {
        if (!isConnected()) return;
        QByteArray payload = cmd.trimmed().toUtf8() + "\n";
        m_serial->write(payload);
        emit rawLineReceived("TX >> " + cmd.trimmed());
    }

    void PicoSerialController::onReadyRead() {
        if (!m_serial || !m_serial->isOpen()) {
            return;
        }
        m_buffer.append(m_serial->readAll());
        int newlineIdx = -1;
        while ((newlineIdx = m_buffer.indexOf('\n')) != -1) {
            QByteArray line = m_buffer.left(newlineIdx).trimmed();
            m_buffer.remove(0, newlineIdx + 1);
            if (!line.isEmpty()) {
                emit rawLineReceived(QString::fromUtf8(line));
                parseJsonLine(line);
            }
        }
    }

    void PicoSerialController::parseJsonLine(const QByteArray& line) {
        try {
            auto j = nlohmann::json::parse(line.constBegin(), line.constEnd());

            const std::string msgType = j.value("type", "");
            if (msgType == "pong") {
                return;
            }

            if (j.contains("x")) m_rawX = j["x"].get<int32_t>();
            if (j.contains("y")) m_rawY = j["y"].get<int32_t>();
            if (j.contains("z")) m_rawZ = j["z"].get<int32_t>();

            Position3D pos{
                (m_rawX - m_offsetX) * m_scaleMm.x() * m_sign.x(),
                (m_rawY - m_offsetY) * m_scaleMm.y() * m_sign.y(),
                (m_rawZ - m_offsetZ) * m_scaleMm.z() * m_sign.z()
            };

            emit positionUpdated(pos);

            bool probeTriggeredNow = false;
            const std::string eventType = j.value("event", "");
            if (msgType == "hit" || eventType == "hit") {
                probeTriggeredNow = true;
                const int hitId = j.value("id", -1);
                if (hitId == -1 || hitId != m_lastHitId) {
                    m_lastHitId = hitId;
                    emit probeTriggered(pos);
                }
                m_lastProbeState = true;
                emit rawCountsUpdated(m_rawX, m_rawY, m_rawZ, probeTriggeredNow);
                return;
            }

            if (j.contains("probe")) {
                if (j["probe"].is_boolean()) {
                    probeTriggeredNow = j["probe"].get<bool>();
                }
                else if (j["probe"].is_number_integer()) {
                    probeTriggeredNow = (j["probe"].get<int>() != 0);
                }

                if (probeTriggeredNow && !m_lastProbeState) {
                    emit probeTriggered(pos);
                }
                m_lastProbeState = probeTriggeredNow;
            }

            emit rawCountsUpdated(m_rawX, m_rawY, m_rawZ, probeTriggeredNow);
        }
        catch (...) {
            // Ignore malformed serial fragments
        }
    }

    void PicoSerialController::onSerialError(QSerialPort::SerialPortError error) {
        if (error == QSerialPort::ResourceError && m_serial->isOpen()) {
            m_serial->close();
            emit connectionStatusChanged(false, "Serial port disconnected unexpectedly.");
        }
    }

} // namespace OpenCMM