// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "mcp/IMcpPlugin.h"
#include <QObject>
#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QScopedPointer>

namespace OpenCMM {

    class OpenCmmMcpPluginPrivate;

    class OpenCmmMcpPlugin : public QObject, public IMcpServerPlugin {
        Q_OBJECT
            Q_PLUGIN_METADATA(IID OpenCMM_IMcpServerPlugin_iid)
            Q_INTERFACES(OpenCMM::IMcpServerPlugin)

    public:
        explicit OpenCmmMcpPlugin(QObject* parent = nullptr);
        ~OpenCmmMcpPlugin() override;

        bool startServer(IMcpHostBridge* bridge, quint16 port = 8765) override;
        void stopServer() override;
        bool isRunning() const override;
        quint16 serverPort() const override;

        // Internal helper accessed by private implementation
        QJsonObject handleJsonRpcRequest(const QJsonObject& req);
        QJsonObject buildToolsListResult() const;
        QJsonObject handleToolCall(const QJsonObject& params);

    private:
        QScopedPointer<OpenCmmMcpPluginPrivate> d;
    };

} // namespace OpenCMM