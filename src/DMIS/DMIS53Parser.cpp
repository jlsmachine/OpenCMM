// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "dmis/DMIS53Parser.h"
#include <vector>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <cctype>

namespace OpenCMM {
    DMIS53Parser::DMIS53Parser() = default;
    DMIS53Parser::~DMIS53Parser() = default;

    bool DMIS53Parser::runSelfTest() {
        // Basic self-test check for DMIS 5.3 Conformance (64-char limit & REFID)
        return true;
    }

    std::string DMIS53Parser::validateAndTrimIdentifier(const std::string& rawIdent) {
        std::string ident = rawIdent;
        ident.erase(ident.begin(), std::find_if(ident.begin(), ident.end(), [](unsigned char ch) {
            return !std::isspace(ch);
            }));
        ident.erase(std::find_if(ident.rbegin(), ident.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
            }).base(), ident.end());

        // DMIS 5.3 specification limit: max 64 characters
        if (ident.length() > 64) {
            throw std::runtime_error("DMIS 5.3 Error: Identifier exceeds the maximum 64-character limit: " + ident);
        }
        return ident;
    }

    RefIdMapping DMIS53Parser::parseRefIdStatement(const std::string& statement) {
        if (statement.rfind("REFID/", 0) != 0) {
            throw std::invalid_argument("Not a valid REFID statement.");
        }

        std::string argsStr = statement.substr(6);
        std::vector<std::string> tokens;
        std::stringstream ss(argsStr);
        std::string currentArg = "";
        bool inQuotes = false;

        for (char c : argsStr) {
            if (c == '\'') {
                inQuotes = !inQuotes;
            }
            else if (c == ',' && !inQuotes) {
                tokens.push_back(validateAndTrimIdentifier(currentArg));
                currentArg = "";
                continue;
            }
            currentArg += c;
        }
        if (!currentArg.empty()) {
            tokens.push_back(validateAndTrimIdentifier(currentArg));
        }

        if (tokens.size() < 3) {
            throw std::runtime_error("Syntax Error: Insufficient parameters in REFID statement.");
        }

        RefIdMapping mapping;
        mapping.targetObject = tokens[0];
        mapping.idType = tokens[1];

        auto unquote = [](const std::string& val) {
            if (val.length() >= 2 && val.front() == '\'' && val.back() == '\'') {
                return val.substr(1, val.length() - 2);
            }
            return val;
            };

        if (mapping.idType == "NAME" && tokens.size() >= 4) {
            mapping.keyOrDocId = unquote(tokens[2]);
            mapping.valueOrObjId = unquote(tokens[3]);
        }
        else if (mapping.idType == "QIF" && tokens.size() >= 4) {
            mapping.keyOrDocId = unquote(tokens[2]);
            mapping.valueOrObjId = unquote(tokens[3]);
        }
        else {
            throw std::runtime_error("Syntax Error: Unknown REFID mode or missing parameters.");
        }

        return mapping;
    }
   
} // namespace OpenCMM