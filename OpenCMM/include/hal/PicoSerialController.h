#pragma once
#include "hal/ICmmController.h"
#include <QSerialPort>
#include <QByteArray>
#include <cstdint>

namespace OpenCMM {

    class PicoSerialController : public ICmmController {
        Q_OBJECT
    public:
        explicit PicoSerialController(QObject* parent = nullptr);
        ~PicoSerialController() override;

        bool connectToPort(const QString& portName, int baudRate = 115200) override;
        void disconnectFromPort() override;
        bool isConnected() const override;
        void zeroAxes() override;

    private slots:
        void onReadyRead();
        void onSerialError(QSerialPort::SerialPortError error);

    private:
        void parseJsonLine(const QByteArray& line);

        QSerialPort* m_serial = nullptr;
        QByteArray m_buffer;

        int32_t m_rawX = 0;
        int32_t m_rawY = 0;
        int32_t m_rawZ = 0;

        int32_t m_offsetX = 0;
        int32_t m_offsetY = 0;
        int32_t m_offsetZ = 0;

        bool m_lastProbeState = false;
    };

} // namespace OpenCMM