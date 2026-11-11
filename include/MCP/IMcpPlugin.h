#pragma once

#include <QtPlugin>
#include <QString>
#include <QJsonObject>
#include <QJsonArray>

namespace OpenCMM {

    class IMcpHostBridge {
    public:
        virtual ~IMcpHostBridge() = default;

        virtual QJsonObject getMachineStatusJson() = 0;
        virtual QJsonArray getSolvedFeaturesJson() = 0;
        virtual QJsonArray getPlannedDmisFeaturesJson() = 0;
        virtual QJsonObject runPreFlightCheckJson() = 0;
        virtual QJsonObject createOfflineDmisFeatureJson(const QJsonObject& params) = 0;
        virtual QJsonObject getCadFacesJson(const QJsonObject& filter = QJsonObject()) = 0;
        virtual QJsonObject runDmisPlaybackJson(const QJsonObject& params = QJsonObject()) = 0;
        virtual QJsonObject exportInspectionFileJson(const QJsonObject& params) = 0;
        virtual QJsonObject solve321AlignmentJson(const QJsonObject& params) = 0;
        virtual QJsonObject evaluateGdtCalloutJson(const QJsonObject& params) = 0;
    };

    class IMcpServerPlugin {
    public:
        virtual ~IMcpServerPlugin() = default;

        virtual bool startServer(IMcpHostBridge* bridge, quint16 port = 8765) = 0;
        virtual void stopServer() = 0;
        virtual bool isRunning() const = 0;
        virtual quint16 serverPort() const = 0;
    };

} // namespace OpenCMM

#define OpenCMM_IMcpServerPlugin_iid "com.jlsmachine.opencmm.IMcpServerPlugin/1.0"
Q_DECLARE_INTERFACE(OpenCMM::IMcpServerPlugin, OpenCMM_IMcpServerPlugin_iid)