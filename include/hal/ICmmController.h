#pragma once
#include <QObject>
#include <QString>
#include "core/Units.h"

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

    signals:
        void positionUpdated(const OpenCMM::Position3D& pos);
        void probeTriggered(const OpenCMM::Position3D& hitPos);
        void connectionStatusChanged(bool connected, const QString& message);
    };

} // namespace OpenCMM