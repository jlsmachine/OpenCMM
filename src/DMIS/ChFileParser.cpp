// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "dmis/ChFileParser.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace OpenCMM {

    CharacterizationProfile ChFileParser::loadCheckmasterProfile(const std::string& filePath) {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open DMIS characterization file: " + filePath);
        }

        CharacterizationProfile profile;
        std::string line;
        std::string currentSection = "";
        bool parsingAlgDefs = false;
        bool parsingErrors = false;

        while (std::getline(file, line)) {
            // Trim whitespace
            line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](unsigned char ch) { return !std::isspace(ch); }));
            line.erase(std::find_if(line.rbegin(), line.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), line.end());

            // Skip comments and empty lines
            if (line.empty() || line.rfind("$$", 0) == 0) {
                continue;
            }

            if (line.rfind("CHFILE/", 0) == 0) {
                profile.profileType = line.substr(7);
                continue;
            }

            if (line == "CHFIL2") {
                continue;
            }
            else if (line == "ENDAT") {
                parsingAlgDefs = false;
                parsingErrors = false;
                continue;
            }

            if (line.rfind("ALGDEF/CODE", 0) == 0) {
                parsingAlgDefs = true;
                parsingErrors = false;
                continue;
            }

            if (line.rfind("ERROR/CODE", 0) == 0) {
                parsingErrors = true;
                parsingAlgDefs = false;
                continue;
            }

            if (parsingAlgDefs || parsingErrors) {
                size_t commaPos = line.find(',');
                if (commaPos != std::string::npos) {
                    try {
                        int code = std::stoi(line.substr(0, commaPos));
                        std::string desc = line.substr(commaPos + 1);
                        // Clean up quotes around description if present
                        if (!desc.empty() && desc.front() == '\'') {
                            desc = desc.substr(1);
                        }
                        if (!desc.empty() && desc.back() == '\'') {
                            desc.pop_back();
                        }

                        if (parsingAlgDefs) {
                            profile.algDefs[code] = desc;
                        }
                        else if (parsingErrors) {
                            profile.errorCodes[code] = desc;
                        }
                    }
                    catch (...) {
                        // Skip malformed entries safely
                    }
                }
            }

            // Check metadata flags in CHFIL3
            if (line.rfind("QIF_SUPPORT = TRUE", 0) == 0) {
                profile.supportsQif = true;
            }
            if (line.rfind("QPID_SUPPORT = TRUE", 0) == 0) {
                profile.supportsQpId = true;
            }
            if (line.rfind("MAX_IDENTIFIER_LENGTH =", 0) == 0) {
                size_t eqPos = line.find('=');
                if (eqPos != std::string::npos) {
                    try {
                        profile.maxIdentifierLength = std::stoi(line.substr(eqPos + 1));
                    }
                    catch (...) {}
                }
            }
        }

        return profile;
    }

} // namespace OpenCMM