// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#ifdef OPENCMM_BRIDGE_STANDALONE

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <io.h>
#include <fcntl.h>

namespace {

    std::string postToOpenCmm(const std::string& jsonLine, unsigned short port) {
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s == INVALID_SOCKET) return {};

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1 only
        addr.sin_port = htons(port);

        if (connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
            closesocket(s);
            return {};
        }

        std::ostringstream req;
        req << "POST / HTTP/1.1\r\n"
            << "Host: 127.0.0.1:" << port << "\r\n"
            << "Content-Type: application/json\r\n"
            << "Content-Length: " << jsonLine.size() << "\r\n"
            << "Connection: close\r\n\r\n"
            << jsonLine;

        const std::string packet = req.str();
        send(s, packet.data(), static_cast<int>(packet.size()), 0);

        std::string rawResponse;
        char buffer[8192];
        int bytesRead = 0;
        while ((bytesRead = recv(s, buffer, sizeof(buffer), 0)) > 0) {
            rawResponse.append(buffer, bytesRead);
        }
        closesocket(s);

        const std::size_t headerEnd = rawResponse.find("\r\n\r\n");
        if (headerEnd == std::string::npos) return {};

        std::string body = rawResponse.substr(headerEnd + 4);
        while (!body.empty() && (body.back() == '\r' || body.back() == '\n' || body.back() == ' ')) {
            body.pop_back();
        }
        return body;
    }

} // namespace

int main(int argc, char* argv[]) {
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);

    unsigned short port = 8765;
    if (argc > 1) {
        const int parsed = std::atoi(argv[1]);
        if (parsed > 0 && parsed <= 65535) {
            port = static_cast<unsigned short>(parsed);
        }
    }

    WSADATA wsaData{};
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        return 1;
    }

    std::string line;
    while (std::getline(std::cin, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' ')) {
            line.pop_back();
        }
        if (line.empty()) continue;

        const bool isNotification = (line.find("\"notifications/") != std::string::npos) ||
            (line.find("\"id\"") == std::string::npos);

        const std::string reply = postToOpenCmm(line, port);
        if (!reply.empty()) {
            std::cout << reply << "\n" << std::flush;
        }
        else if (!isNotification) {
            std::cout << "{\"jsonrpc\":\"2.0\",\"id\":null,\"error\":{\"code\":-32000,\"message\":\"OpenCMM MCP server is offline or disabled on port "
                << port << "\"}}\n" << std::flush;
        }
    }

    WSACleanup();
    return 0;
}

#endif // OPENCMM_BRIDGE_STANDALONE