// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace OpenCMM {

    struct CharacterizationProfile {
        std::string profileType; // "INPUT" or "OUTPUT"
        bool supportsQif = false;
        bool supportsQpId = false;
        int maxIdentifierLength = 16; // Default standard, upgraded to 64 for 5.3
        std::unordered_map<int, std::string> algDefs;
        std::unordered_map<int, std::string> errorCodes;
    };

    class ChFileParser {
    public:
        // Parses a DMIS .chf characterization file from disk
        static CharacterizationProfile loadCheckmasterProfile(const std::string& filePath);
    };

} // namespace OpenCMM