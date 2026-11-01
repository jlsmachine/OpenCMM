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
            m_buffer.clear();
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
    }

    void PicoSerialController::onReadyRead() {
        m_buffer.append(m_serial->readAll());
        int newlineIdx = -1;
        while ((newlineIdx = m_buffer.indexOf('\n')) != -1) {
            QByteArray line = m_buffer.left(newlineIdx).trimmed();
            m_buffer.remove(0, newlineIdx + 1);
            if (!line.isEmpty()) {
                parseJsonLine(line);
            }
        }
    }

    void PicoSerialController::parseJsonLine(const QByteArray& line) {
        try {
            auto j = nlohmann::json::parse(line.constBegin(), line.constEnd());
            m_rawX = j.value("x", m_rawX);
            m_rawY = j.value("y", m_rawY);
            m_rawZ = j.value("z", m_rawZ);

            Position3D pos{
                (m_rawX - m_offsetX) * ENCODER_SCALE_MM,
                (m_rawY - m_offsetY) * ENCODER_SCALE_MM,
                (m_rawZ - m_offsetZ) * ENCODER_SCALE_MM
            };

            emit positionUpdated(pos);

            bool probeTriggeredNow = false;
            if (j.contains("probe")) {
                if (j["probe"].is_boolean()) {
                    probeTriggeredNow = j["probe"].get<bool>();
                }
                else if (j["probe"].is_number_integer()) {
                    probeTriggeredNow = (j["probe"].get<int>() != 0);
                }
            }
            else if (j.value("event", "") == "hit") {
                probeTriggeredNow = true;
            }

            if (probeTriggeredNow && !m_lastProbeState) {
                emit probeTriggered(pos);
            }
            m_lastProbeState = probeTriggeredNow;
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