// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include "OpenCmmMcpPlugin.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QCoreApplication>
#include <QThread>
#include <thread>
#include <atomic>
#include <cstring>

namespace OpenCMM {

    class OpenCmmMcpPluginPrivate {
    public:
        OpenCmmMcpPluginPrivate(OpenCmmMcpPlugin* owner) : m_owner(owner) {
            WSADATA wsaData{};
            if (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0) {
                m_wsaInitialized = true;
            }
        }

        ~OpenCmmMcpPluginPrivate() {
            stopServer();
            if (m_wsaInitialized) {
                WSACleanup();
            }
        }

        bool startServer(IMcpHostBridge* bridge, quint16 port) {
            stopServer();
            if (!m_wsaInitialized) return false;

            m_bridge = bridge;
            m_port = port;

            SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (s == INVALID_SOCKET) return false;

            BOOL reuse = TRUE;
            setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1 only
            addr.sin_port = htons(m_port);

            if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
                closesocket(s);
                return false;
            }

            if (listen(s, SOMAXCONN) == SOCKET_ERROR) {
                closesocket(s);
                return false;
            }

            m_listenSocket = s;
            m_running = true;
            m_acceptThread = std::thread(&OpenCmmMcpPluginPrivate::acceptLoop, this);
            return true;
        }

        void stopServer() {
            m_running = false;
            if (m_listenSocket != INVALID_SOCKET) {
                closesocket(m_listenSocket);
                m_listenSocket = INVALID_SOCKET;
            }
            if (m_acceptThread.joinable()) {
                m_acceptThread.join();
            }
        }

        bool isRunning() const {
            return m_running;
        }

        quint16 serverPort() const {
            return m_port;
        }

        void acceptLoop() {
            while (m_running) {
                SOCKET clientSock = accept(m_listenSocket, nullptr, nullptr);
                if (clientSock == INVALID_SOCKET) {
                    break;
                }

                // Handle each client in a dedicated detached thread
                std::thread([this, clientSock]() {
                    handleClient(clientSock);
                    }).detach();
            }
        }

        void handleClient(SOCKET clientSock) {
            char buffer[16384];
            int bytesRead = recv(clientSock, buffer, sizeof(buffer) - 1, 0);
            if (bytesRead > 0) {
                buffer[bytesRead] = '\0';
                QByteArray rawData(buffer, bytesRead);

                int jsonStart = rawData.indexOf("\r\n\r\n");
                const bool isHttp = rawData.startsWith("POST ") || rawData.startsWith("GET ");
                QByteArray payload;

                if (isHttp && jsonStart >= 0) {
                    payload = rawData.mid(jsonStart + 4);

                    // Case-insensitive search for Content-Length
                    int clIndex = rawData.indexOf("Content-Length:");
                    if (clIndex == -1) {
                        clIndex = rawData.indexOf("content-length:");
                    }

                    if (clIndex != -1) {
                        int lineEnd = rawData.indexOf("\r\n", clIndex);
                        if (lineEnd != -1) {
                            int contentLength = rawData.mid(clIndex + 15, lineEnd - (clIndex + 15)).trimmed().toInt();
                            while (payload.size() < contentLength) {
                                int moreBytes = recv(clientSock, buffer, sizeof(buffer) - 1, 0);
                                if (moreBytes > 0) {
                                    payload.append(buffer, moreBytes);
                                }
                                else {
                                    break;
                                }
                            }
                        }
                    }
                }
                else {
                    payload = rawData;
                }

                // Trim trailing null bytes, newlines, or whitespace
                payload = payload.trimmed();

                QJsonParseError err;
                QJsonDocument doc = QJsonDocument::fromJson(payload, &err);
                QJsonObject responseObj;

                if (err.error == QJsonParseError::NoError && doc.isObject()) {
                    // Safely marshal execution onto the main Qt GUI thread
                    if (QThread::currentThread() != QCoreApplication::instance()->thread()) {
                        QMetaObject::invokeMethod(qApp, [this, reqObj = doc.object(), &responseObj]() {
                            responseObj = m_owner->handleJsonRpcRequest(reqObj);
                            }, Qt::BlockingQueuedConnection);
                    }
                    else {
                        responseObj = m_owner->handleJsonRpcRequest(doc.object());
                    }
                }
                else {
                    responseObj = QJsonObject();
                    responseObj["jsonrpc"] = "2.0";
                    responseObj["id"] = QJsonValue::Null;
                    QJsonObject errObj;
                    errObj["code"] = -32700;
                    errObj["message"] = "Parse error: " + err.errorString();
                    responseObj["error"] = errObj;
                }

                // JSON-RPC notifications return an empty QJsonObject and must not emit a JSON payload
                if (responseObj.isEmpty()) {
                    if (isHttp) {
                        const char* noContent =
                            "HTTP/1.1 204 No Content\r\n"
                            "Connection: close\r\n"
                            "Content-Length: 0\r\n\r\n";
                        send(clientSock, noContent, static_cast<int>(std::strlen(noContent)), 0);
                    }
                    shutdown(clientSock, SD_SEND);
                    closesocket(clientSock);
                    return;
                }

                QByteArray outBytes = QJsonDocument(responseObj).toJson(QJsonDocument::Compact);

                if (isHttp) {
                    QByteArray httpPacket;
                    httpPacket.append("HTTP/1.1 200 OK\r\n");
                    httpPacket.append("Content-Type: application/json; charset=utf-8\r\n");
                    httpPacket.append("Access-Control-Allow-Origin: *\r\n");
                    httpPacket.append("Connection: close\r\n");
                    httpPacket.append("Content-Length: " + QByteArray::number(outBytes.size()) + "\r\n");
                    httpPacket.append("\r\n");
                    httpPacket.append(outBytes);

                    send(clientSock, httpPacket.constData(), static_cast<int>(httpPacket.size()), 0);
                }
                else {
                    outBytes.append('\n');
                    send(clientSock, outBytes.constData(), static_cast<int>(outBytes.size()), 0);
                }
            }

            shutdown(clientSock, SD_SEND);
            closesocket(clientSock);
        }

        OpenCmmMcpPlugin* m_owner = nullptr;
        IMcpHostBridge* m_bridge = nullptr;
        SOCKET m_listenSocket = INVALID_SOCKET;
        std::thread m_acceptThread;
        std::atomic<bool> m_running{ false };
        quint16 m_port = 8765;
        bool m_wsaInitialized = false;
    };

    OpenCmmMcpPlugin::OpenCmmMcpPlugin(QObject* parent)
        : QObject(parent), d(new OpenCmmMcpPluginPrivate(this)) {
    }

    OpenCmmMcpPlugin::~OpenCmmMcpPlugin() = default;

    bool OpenCmmMcpPlugin::startServer(IMcpHostBridge* bridge, quint16 port) {
        return d->startServer(bridge, port);
    }

    void OpenCmmMcpPlugin::stopServer() {
        d->stopServer();
    }

    bool OpenCmmMcpPlugin::isRunning() const {
        return d->isRunning();
    }

    quint16 OpenCmmMcpPlugin::serverPort() const {
        return d->serverPort();
    }

    QJsonObject OpenCmmMcpPlugin::handleJsonRpcRequest(const QJsonObject& req) {
        const QString method = req.value("method").toString();

        // Ignore JSON-RPC notifications (e.g., "notifications/initialized", "notifications/cancelled")
        if (!req.contains("id") || method.startsWith("notifications/")) {
            return QJsonObject{};
        }

        const QJsonValue idVal = req.value("id");
        const QJsonObject params = req.value("params").toObject();

        QJsonObject resp;
        resp["jsonrpc"] = "2.0";
        resp["id"] = idVal;

        if (method == "initialize") {
            QJsonObject result;
            result["protocolVersion"] = "2024-11-05";
            result["serverInfo"] = QJsonObject{
                { "name", "OpenCMM-MCP-Server" },
                { "version", "1.0.0" }
            };
            result["capabilities"] = QJsonObject{
                { "tools", QJsonObject{} }
            };
            resp["result"] = result;
        }
        else if (method == "ping") {
            resp["result"] = QJsonObject{};
        }
        else if (method == "tools/list") {
            resp["result"] = buildToolsListResult();
        }
        else if (method == "tools/call") {
            resp["result"] = handleToolCall(params);
        }
        else {
            resp["error"] = QJsonObject{
                { "code", -32601 },
                { "message", "Method not found: " + method }
            };
        }
        return resp;
    }

    QJsonObject OpenCmmMcpPlugin::buildToolsListResult() const {
        QJsonArray tools;

        tools.append(QJsonObject{
            { "name", "get_cmm_status" },
            { "description", "Get live CMM DRO coordinates, active stylus calibration radius, units, and 3-2-1 PCS alignment state." },
            { "inputSchema", QJsonObject{ { "type", "object" },{ "properties", QJsonObject{} } } }
            });

        tools.append(QJsonObject{
            { "name", "list_inspection_features" },
            { "description", "List all solved and planned DMIS 5.3 inspection features, nominals, actuals, and PASS/FAIL status." },
            { "inputSchema", QJsonObject{ { "type", "object" },{ "properties", QJsonObject{} } } }
            });

        tools.append(QJsonObject{
            { "name", "run_preflight_check" },
            { "description", "Run CAD collision and standoff clearance validation across all loaded DMIS 5.3 features." },
            { "inputSchema", QJsonObject{ { "type", "object" },{ "properties", QJsonObject{} } } }
            });

        tools.append(QJsonObject{
            { "name", "create_dmis_feature" },
            { "description", "Queue a planned offline DMIS 5.3 feature. If faceIndex (from list_cad_faces) is provided, nominals and valid interior UV touch points (TopAbs_IN) are automatically extracted from the STEP B-Rep face." },
            { "inputSchema", QJsonObject{
                { "type", "object" },
                { "properties", QJsonObject{
                    { "label", QJsonObject{ { "type", "string" } } },
                    { "faceIndex", QJsonObject{ { "type", "integer" },{ "description", "Optional 0-based faceIndex from list_cad_faces to sample valid interior UV points and auto-fill nominals" } } },
                    { "featureType", QJsonObject{ { "type", "string" },{ "description", "PLANE, CYLNDR, CIRCLE, SPHERE, or LINE (auto-detected if faceIndex is set)" } } },
                    { "boundarySub", QJsonObject{ { "type", "string" },{ "description", "INNER, OUTER, or UNBND (auto-detected if faceIndex is set)" } } },
                    { "datumLetter", QJsonObject{ { "type", "string" },{ "description", "Optional 3-2-1 datum letter: A, B, or C" } } },
                    { "nomX", QJsonObject{ { "type", "number" } } },
                    { "nomY", QJsonObject{ { "type", "number" } } },
                    { "nomZ", QJsonObject{ { "type", "number" } } },
                    { "nomI", QJsonObject{ { "type", "number" } } },
                    { "nomJ", QJsonObject{ { "type", "number" } } },
                    { "nomK", QJsonObject{ { "type", "number" } } },
                    { "nominalDiameter", QJsonObject{ { "type", "number" } } },
                    { "hitCount", QJsonObject{ { "type", "integer" },{ "description", "Number of PTMEAS hits to generate (default 4 for planes/circles, 6 for cylinders)" } } }
                } },
                { "required", QJsonArray{ "label" } }
            } }
            });

        tools.append(QJsonObject{
            { "name", "list_cad_faces" },
            { "description", "Extract geometric faces (PLANE, CYLNDR, SPHERE, CONE) from the loaded STEP CAD model including centroids, outward normals/axes, INNER/OUTER bore orientation, and nominal diameters." },
            { "inputSchema", QJsonObject{
                { "type", "object" },
                { "properties", QJsonObject{
                    { "surfaceType", QJsonObject{
                        { "type", "string" },
                        { "description", "Optional filter: ALL, PLANE, CYLNDR, SPHERE, or CONE (default ALL)" }
                    } },
                    { "maxFaces", QJsonObject{
                        { "type", "integer" },
                        { "description", "Maximum number of faces to return (default 100)" }
                    } }
                } }
            } }
            });

        tools.append(QJsonObject{
            { "name", "run_dmis_program" },
            { "description", "Start, stop, or reset offline DMIS 5.3 path simulation playback in the 3D viewport." },
            { "inputSchema", QJsonObject{
                { "type", "object" },
                { "properties", QJsonObject{
                    { "action", QJsonObject{
                        { "type", "string" },
                        { "description", "Playback command: start, stop, or reset (default start)" }
                    } }
                } }
            } }
            });

        tools.append(QJsonObject{
            { "name", "create_dmis_feature" },
            { "description", "Queue a planned offline DMIS 5.3 feature with optional per-feature tolerances and Datum A/B/C assignment." },
            { "inputSchema", QJsonObject{
                { "type", "object" },
                { "properties", QJsonObject{
                    { "label", QJsonObject{ { "type", "string" } } },
                    { "faceIndex", QJsonObject{ { "type", "integer" },{ "description", "Optional faceIndex from list_cad_faces" } } },
                    { "featureType", QJsonObject{ { "type", "string" },{ "description", "PLANE, CYLNDR, CIRCLE, SPHERE, or LINE" } } },
                    { "boundarySub", QJsonObject{ { "type", "string" },{ "description", "INNER, OUTER, or UNBND" } } },
                    { "datumLetter", QJsonObject{ { "type", "string" },{ "description", "Optional 3-2-1 datum letter: A, B, or C" } } },
                    { "plusToleranceMm", QJsonObject{ { "type", "number" },{ "description", "Upper tolerance limit in mm (default app default)" } } },
                    { "minusToleranceMm", QJsonObject{ { "type", "number" },{ "description", "Lower tolerance limit in mm (default app default)" } } },
                    { "nomX", QJsonObject{ { "type", "number" } } },
                    { "nomY", QJsonObject{ { "type", "number" } } },
                    { "nomZ", QJsonObject{ { "type", "number" } } },
                    { "nomI", QJsonObject{ { "type", "number" } } },
                    { "nomJ", QJsonObject{ { "type", "number" } } },
                    { "nomK", QJsonObject{ { "type", "number" } } },
                    { "nominalDiameter", QJsonObject{ { "type", "number" } } },
                    { "hitCount", QJsonObject{ { "type", "integer" },{ "description", "Number of PTMEAS hits (default 4 for planes/circles, 6 for cylinders)" } } }
                } },
                { "required", QJsonArray{ "label" } }
            } }
            });

        tools.append(QJsonObject{
            { "name", "solve_321_alignment" },
            { "description", "Compute and activate an orthogonal 3-2-1 Part Coordinate System (PCS) alignment from Datums A (Level +Z), B (Rotate +X), and C (Origin 0,0,0)." },
            { "inputSchema", QJsonObject{
                { "type", "object" },
                { "properties", QJsonObject{
                    { "datumAFeatureId", QJsonObject{ { "type", "integer" },{ "description", "Feature ID or index for primary Datum A plane" } } },
                    { "datumBFeatureId", QJsonObject{ { "type", "integer" },{ "description", "Feature ID or index for secondary Datum B line/axis" } } },
                    { "datumCFeatureId", QJsonObject{ { "type", "integer" },{ "description", "Feature ID or index for tertiary Datum C point/circle" } } }
                } }
            } }
            });
        tools.append(QJsonObject{
            { "name", "export_inspection_file" },
            { "description", "Headlessly export inspection program/results to disk as a DMIS, QIF, CSV, HTML, or professional PDF inspection certificate with custom metadata and template styles." },
            { "inputSchema", QJsonObject{
                { "type", "object" },
                { "properties", QJsonObject{
                    { "format", QJsonObject{ { "type", "string" },{ "description", "Export format: dmis, qif, pdf, csv, or html" } } },
                    { "templateStyle", QJsonObject{ { "type", "string" },{ "description", "PDF template layout: standard, aerospace, or compact (default standard)" } } },
                    { "partName", QJsonObject{ { "type", "string" },{ "description", "Part name or job number" } } },
                    { "customerName", QJsonObject{ { "type", "string" },{ "description", "Customer name / client" } } },
                    { "poNumber", QJsonObject{ { "type", "string" },{ "description", "Customer Purchase Order (PO) number" } } },
                    { "inspectorName", QJsonObject{ { "type", "string" },{ "description", "Quality inspector name or ID" } } },
                    { "batchNumber", QJsonObject{ { "type", "string" },{ "description", "Lot / serial / batch number" } } },
                    { "filePath", QJsonObject{ { "type", "string" },{ "description", "Optional output file path" } } }
                } },
                { "required", QJsonArray{ "format" } }
            } }
            });

        return QJsonObject{ { "tools", tools } };
    }

    QJsonObject OpenCmmMcpPlugin::handleToolCall(const QJsonObject& params) {
        if (!d->m_bridge) {
            return QJsonObject{ { "isError", true } };
        }

        const QString toolName = params.value("name").toString();
        const QJsonObject args = params.value("arguments").toObject();
        QJsonObject payload;

        if (toolName == "get_cmm_status") {
            payload = d->m_bridge->getMachineStatusJson();
        }
        else if (toolName == "list_inspection_features") {
            payload["solvedFeatures"] = d->m_bridge->getSolvedFeaturesJson();
            payload["plannedDmisFeatures"] = d->m_bridge->getPlannedDmisFeaturesJson();
        }
        else if (toolName == "run_preflight_check") {
            payload = d->m_bridge->runPreFlightCheckJson();
        }
        else if (toolName == "create_dmis_feature") {
            payload = d->m_bridge->createOfflineDmisFeatureJson(args);
        }
        else if (toolName == "list_cad_faces") {
            payload = d->m_bridge->getCadFacesJson(args);
        }
        else if (toolName == "run_dmis_program") {
            payload = d->m_bridge->runDmisPlaybackJson(args);
        }
        else if (toolName == "export_inspection_file") {
            payload = d->m_bridge->exportInspectionFileJson(args);
        }
        else if (toolName == "solve_321_alignment") {
            payload = d->m_bridge->solve321AlignmentJson(args);
        }
        else {
            payload["error"] = "Unknown tool: " + toolName;
        }

        QJsonArray content;
        content.append(QJsonObject{
            { "type", "text" },
            { "text", QString::fromUtf8(QJsonDocument(payload).toJson(QJsonDocument::Indented)) }
            });

        return QJsonObject{ { "content", content } };
    }

} // namespace OpenCMM

#include "OpenCmmMcpPlugin.moc"