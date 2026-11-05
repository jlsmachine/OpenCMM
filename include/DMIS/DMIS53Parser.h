// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>

namespace OpenCMM {

    struct RefIdMapping {
        std::string targetObject; // e.g., "F(CYLN1)"
        std::string idType;       // "NAME" or "QIF"
        std::string keyOrDocId;   // e.g., "QPId" or QIF Document UUID
        std::string valueOrObjId; // e.g., UUID string or external database ID
    };

    class DMIS53Parser {
    public:
        DMIS53Parser();
        ~DMIS53Parser();
        static std::string validateAndTrimIdentifier(const std::string& rawIdent);
        static RefIdMapping parseRefIdStatement(const std::string& statement);
        static bool runSelfTest();
    };
    
} // namespace OpenCMM