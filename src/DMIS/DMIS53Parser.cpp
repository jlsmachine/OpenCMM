// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "dmis/DMIS53Parser.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <cctype>
#include <cmath>

namespace OpenCMM {
    namespace {
        std::string trimStr(const std::string& s) {
            auto start = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
            auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c); }).base();
            return (start < end) ? std::string(start, end) : std::string();
        }

        std::string toUpperStr(std::string s) {
            std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            return s;
        }

        std::string unquoteStr(const std::string& val) {
            std::string t = trimStr(val);
            if (t.length() >= 2 && ((t.front() == '\'' && t.back() == '\'') || (t.front() == '"' && t.back() == '"'))) {
                return t.substr(1, t.length() - 2);
            }
            return t;
        }

        std::string extractParenLabel(const std::string& token, const std::string& prefix) {
            std::string t = trimStr(token);
            std::string u = toUpperStr(t);
            std::string p = toUpperStr(prefix) + "(";
            if (u.rfind(p, 0) == 0 && u.back() == ')' && t.size() > p.size() + 1) {
                return trimStr(t.substr(p.size(), t.size() - p.size() - 1));
            }
            return t;
        }

        std::string extractFeatureLabel(const std::string& token) {
            std::string t = trimStr(token);
            std::string u = toUpperStr(t);
            if (u.rfind("F(", 0) == 0) return extractParenLabel(t, "F");
            if (u.rfind("FA(", 0) == 0) return extractParenLabel(t, "FA");
            return t;
        }

        std::vector<std::string> splitCsvRespectingQuotes(const std::string& s) {
            std::vector<std::string> out;
            std::string cur;
            bool inQuotes = false;
            int parenDepth = 0;
            for (char c : s) {
                if (c == '\'') {
                    inQuotes = !inQuotes;
                    cur += c;
                }
                else if (!inQuotes && c == '(') {
                    parenDepth++;
                    cur += c;
                }
                else if (!inQuotes && c == ')') {
                    if (parenDepth > 0) parenDepth--;
                    cur += c;
                }
                else if (!inQuotes && parenDepth == 0 && c == ',') {
                    out.push_back(trimStr(cur));
                    cur.clear();
                }
                else {
                    cur += c;
                }
            }
            if (!cur.empty()) {
                out.push_back(trimStr(cur));
            }
            return out;
        }

        bool tryParseDouble(const std::string& s, double& outVal) {
            try {
                size_t idx = 0;
                double v = std::stod(trimStr(s), &idx);
                if (idx > 0) {
                    outVal = v;
                    return true;
                }
            }
            catch (...) {}
            return false;
        }
    }

    DMIS53Parser::DMIS53Parser() = default;
    DMIS53Parser::~DMIS53Parser() = default;

    bool DMIS53Parser::runSelfTest() {
        return true;
    }

    std::string DMIS53Parser::validateAndTrimIdentifier(const std::string& rawIdent) {
        std::string ident = trimStr(rawIdent);
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
        std::vector<std::string> rawTokens = splitCsvRespectingQuotes(argsStr);
        std::vector<std::string> tokens;
        for (const auto& rt : rawTokens) {
            tokens.push_back(validateAndTrimIdentifier(rt));
        }

        if (tokens.size() < 3) {
            throw std::runtime_error("Syntax Error: Insufficient parameters in REFID statement.");
        }

        RefIdMapping mapping;
        mapping.targetObject = tokens[0];
        mapping.idType = tokens[1];

        if ((mapping.idType == "NAME" || mapping.idType == "QIF") && tokens.size() >= 4) {
            mapping.keyOrDocId = unquoteStr(tokens[2]);
            mapping.valueOrObjId = unquoteStr(tokens[3]);
        }
        else {
            throw std::runtime_error("Syntax Error: Unknown REFID mode or missing parameters.");
        }

        return mapping;
    }

    DmisProgramData DMIS53Parser::parseDmisProgramFile(const std::string& filePath) {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            DmisProgramData prog;
            prog.valid = false;
            prog.errorMessage = "Could not open DMIS file: " + filePath;
            return prog;
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return parseDmisProgramText(buffer.str());
    }

    DmisProgramData DMIS53Parser::parseDmisProgramText(const std::string& sourceText) {
        DmisProgramData prog;
        std::istringstream stream(sourceText);
        std::string rawLine;
        std::string accumulatedLine;

        std::unordered_map<std::string, size_t> featureIndexByLabel;
        std::unordered_map<std::string, std::string> pendingRefIdByLabel;

        bool insideMeasBlock = false;
        size_t currentMeasFeatureIdx = 0;
        double unitScaleToMm = 1.0;

        auto getOrCreateFeature = [&](const std::string& rawLabel, const std::string& defaultType) -> DmisPlannedFeature& {
            std::string cleanLbl = toUpperStr(extractFeatureLabel(rawLabel));
            validateAndTrimIdentifier(cleanLbl);
            auto it = featureIndexByLabel.find(cleanLbl);
            if (it != featureIndexByLabel.end()) {
                return prog.features[it->second];
            }
            DmisPlannedFeature feat;
            feat.label = cleanLbl;
            feat.featureType = defaultType;
            if (pendingRefIdByLabel.count(cleanLbl)) {
                feat.refId = pendingRefIdByLabel[cleanLbl];
            }
            size_t newIdx = prog.features.size();
            prog.features.push_back(feat);
            featureIndexByLabel[cleanLbl] = newIdx;
            return prog.features.back();
        };

        while (std::getline(stream, rawLine)) {
            std::string line = trimStr(rawLine);
            if (line.empty() || line.rfind("$$", 0) == 0) {
                continue;
            }

            if (!line.empty() && line.back() == '&') {
                accumulatedLine += line.substr(0, line.size() - 1) + " ";
                continue;
            }
            if (!accumulatedLine.empty()) {
                line = accumulatedLine + line;
                accumulatedLine.clear();
            }

            if (!line.empty() && line.back() == '$') {
                line.pop_back();
                line = trimStr(line);
            }

            std::string upperLine = toUpperStr(line);

            // 1. FILNAM/'PartName', 05.3
            if (upperLine.rfind("FILNAM/", 0) == 0) {
                auto args = splitCsvRespectingQuotes(line.substr(7));
                if (!args.empty()) {
                    prog.partName = unquoteStr(args[0]);
                }
                continue;
            }

            // 2. UNITS/MM or UNITS/INCH
            if (upperLine.rfind("UNITS/", 0) == 0) {
                auto args = splitCsvRespectingQuotes(upperLine.substr(6));
                if (!args.empty()) {
                    if (args[0].find("IN") != std::string::npos) {
                        prog.units = "INCH";
                        unitScaleToMm = 25.4;
                    }
                    else {
                        prog.units = "MM";
                        unitScaleToMm = 1.0;
                    }
                }
                continue;
            }

            // 3. SNSTYP/INDEX, 1, <diam>
            if (upperLine.rfind("SNSTYP/", 0) == 0) {
                auto args = splitCsvRespectingQuotes(line.substr(7));
                double diam = 0.0;
                if (args.size() >= 2 && tryParseDouble(args.back(), diam) && diam > 0.0) {
                    prog.stylusDiameterMm = diam * unitScaleToMm;
                }
                continue;
            }

            // 4. CLEARP/ZPLUS, <z>
            if (upperLine.rfind("CLEARP/", 0) == 0) {
                auto args = splitCsvRespectingQuotes(line.substr(7));
                double zVal = 0.0;
                if (args.size() >= 2 && tryParseDouble(args[1], zVal)) {
                    prog.clearanceZMm = zVal * unitScaleToMm;
                }
                else if (args.size() == 1 && tryParseDouble(args[0], zVal)) {
                    prog.clearanceZMm = zVal * unitScaleToMm;
                }
                continue;
            }

            // 5. REFID/F(FEAT_1), QIF, 'QPId', '<uuid>'
            if (upperLine.rfind("REFID/", 0) == 0) {
                try {
                    RefIdMapping refMap = parseRefIdStatement(line);
                    std::string cleanLbl = toUpperStr(extractFeatureLabel(refMap.targetObject));
                    pendingRefIdByLabel[cleanLbl] = refMap.valueOrObjId;
                    auto it = featureIndexByLabel.find(cleanLbl);
                    if (it != featureIndexByLabel.end()) {
                        prog.features[it->second].refId = refMap.valueOrObjId;
                    }
                }
                catch (...) {}
                continue;
            }

            // 6. DATDEF/FA(DATUM_A), DAT(A)  or  DATDEF/F(DATUM_A), DAT(A)
            if (upperLine.rfind("DATDEF/", 0) == 0) {
                auto tokens = splitCsvRespectingQuotes(upperLine.substr(7));
                if (tokens.size() >= 2) {
                    std::string featLbl = extractFeatureLabel(tokens[0]);
                    std::string datLetter = extractParenLabel(tokens[1], "DAT");
                    if (!featLbl.empty() && !datLetter.empty()) {
                        prog.datumLetterToFeature[datLetter] = featLbl;
                        auto it = featureIndexByLabel.find(featLbl);
                        if (it != featureIndexByLabel.end()) {
                            prog.features[it->second].datumLetter = datLetter;
                        }
                    }
                }
                continue;
            }

            // 7. DATSET/DAT(A), ZDIR, ZORIG, DAT(B), XDIR, YORIG, DAT(C), XORIG
            if (upperLine.find("DATSET/") != std::string::npos) {
                size_t dsPos = upperLine.find("DATSET/");
                auto tokens = splitCsvRespectingQuotes(upperLine.substr(dsPos + 7));
                std::vector<std::string> datLetters;
                for (const auto& tk : tokens) {
                    if (tk.rfind("DAT(", 0) == 0) {
                        std::string dl = extractParenLabel(tk, "DAT");
                        if (!dl.empty() && std::find(datLetters.begin(), datLetters.end(), dl) == datLetters.end()) {
                            datLetters.push_back(dl);
                        }
                    }
                }
                if (datLetters.size() >= 3) {
                    prog.datumAlignment.has321Alignment = true;
                    prog.datumAlignment.primaryDatumLetter = datLetters[0];
                    prog.datumAlignment.secondaryDatumLetter = datLetters[1];
                    prog.datumAlignment.tertiaryDatumLetter = datLetters[2];
                }
                continue;
            }

            // 8. Feature Definition: F(LABEL) = FEAT/TYPE, ...
            if (upperLine.rfind("F(", 0) == 0 && upperLine.find("FEAT/") != std::string::npos) {
                size_t eqPos = line.find('=');
                size_t featPos = upperLine.find("FEAT/");
                if (eqPos != std::string::npos && featPos != std::string::npos) {
                    std::string lhsLabel = line.substr(0, eqPos);
                    std::string rhsBody = line.substr(featPos + 5);
                    auto tokens = splitCsvRespectingQuotes(rhsBody);
                    if (!tokens.empty()) {
                        std::string fType = toUpperStr(tokens[0]);
                        DmisPlannedFeature& feat = getOrCreateFeature(lhsLabel, fType);
                        feat.featureType = fType;

                        std::vector<double> numericParams;
                        for (size_t i = 1; i < tokens.size(); ++i) {
                            double val = 0.0;
                            if (tryParseDouble(tokens[i], val)) {
                                numericParams.push_back(val);
                            }
                            else {
                                std::string sub = toUpperStr(tokens[i]);
                                if (sub == "INNER" || sub == "OUTER" || sub == "BORE" || sub == "BOSS" || sub == "UNBND" || sub == "BND") {
                                    feat.boundarySub = sub;
                                }
                            }
                        }

                        if (fType == "SPHERE") {
                            if (numericParams.size() >= 3) {
                                feat.nomX = numericParams[0] * unitScaleToMm;
                                feat.nomY = numericParams[1] * unitScaleToMm;
                                feat.nomZ = numericParams[2] * unitScaleToMm;
                            }
                            if (numericParams.size() >= 4) {
                                feat.nominalDiameter = numericParams[3] * unitScaleToMm;
                            }
                        }
                        else {
                            if (numericParams.size() >= 3) {
                                feat.nomX = numericParams[0] * unitScaleToMm;
                                feat.nomY = numericParams[1] * unitScaleToMm;
                                feat.nomZ = numericParams[2] * unitScaleToMm;
                            }
                            if (numericParams.size() >= 6) {
                                feat.nomI = numericParams[3];
                                feat.nomJ = numericParams[4];
                                feat.nomK = numericParams[5];
                            }
                            if (numericParams.size() >= 7 && (fType == "CIRCLE" || fType == "CYLNDR" || fType == "CYL")) {
                                feat.nominalDiameter = numericParams[6] * unitScaleToMm;
                            }
                        }
                    }
                }
                continue;
            }

            // 9. MEAS/TYPE, F(LABEL), COUNT
            if (upperLine.rfind("MEAS/", 0) == 0) {
                auto tokens = splitCsvRespectingQuotes(line.substr(5));
                if (tokens.size() >= 2) {
                    std::string mType = toUpperStr(tokens[0]);
                    std::string fLbl = tokens[1];
                    DmisPlannedFeature& feat = getOrCreateFeature(fLbl, mType);
                    if (feat.featureType.empty()) {
                        feat.featureType = mType;
                    }
                    currentMeasFeatureIdx = featureIndexByLabel[toUpperStr(extractFeatureLabel(fLbl))];
                    insideMeasBlock = true;
                }
                continue;
            }

            // 10. ENDMES
            if (upperLine.rfind("ENDMES", 0) == 0) {
                insideMeasBlock = false;
                continue;
            }

            // 11. PTMEAS/CART, X, Y, Z, I, J, K
            if (insideMeasBlock && upperLine.rfind("PTMEAS/", 0) == 0 && currentMeasFeatureIdx < prog.features.size()) {
                auto tokens = splitCsvRespectingQuotes(line.substr(7));
                std::vector<double> nums;
                for (const auto& tk : tokens) {
                    double v = 0.0;
                    if (tryParseDouble(tk, v)) {
                        nums.push_back(v);
                    }
                }
                if (nums.size() >= 3) {
                    DmisTargetPoint pt;
                    pt.x = nums[0] * unitScaleToMm;
                    pt.y = nums[1] * unitScaleToMm;
                    pt.z = nums[2] * unitScaleToMm;
                    if (nums.size() >= 6) {
                        double len = std::hypot(nums[3], std::hypot(nums[4], nums[5]));
                        if (len > 1e-9) {
                            pt.i = nums[3] / len;
                            pt.j = nums[4] / len;
                            pt.k = nums[5] / len;
                        }
                    }
                    prog.features[currentMeasFeatureIdx].points.push_back(pt);
                }
                continue;
            }
        }

        // Resolve DATDEF feature labels into DmisPlannedFeature::datumLetter and DmisDatumSetRule
        for (const auto& kv : prog.datumLetterToFeature) {
            auto it = featureIndexByLabel.find(kv.second);
            if (it != featureIndexByLabel.end()) {
                prog.features[it->second].datumLetter = kv.first;
            }
        }
        if (prog.datumAlignment.has321Alignment) {
            prog.datumAlignment.primaryFeatureLabel = prog.datumLetterToFeature[prog.datumAlignment.primaryDatumLetter];
            prog.datumAlignment.secondaryFeatureLabel = prog.datumLetterToFeature[prog.datumAlignment.secondaryDatumLetter];
            prog.datumAlignment.tertiaryFeatureLabel = prog.datumLetterToFeature[prog.datumAlignment.tertiaryDatumLetter];
        }
        else if (prog.datumLetterToFeature.count("A") && prog.datumLetterToFeature.count("B") && prog.datumLetterToFeature.count("C")) {
            prog.datumAlignment.has321Alignment = true;
            prog.datumAlignment.primaryDatumLetter = "A";
            prog.datumAlignment.secondaryDatumLetter = "B";
            prog.datumAlignment.tertiaryDatumLetter = "C";
            prog.datumAlignment.primaryFeatureLabel = prog.datumLetterToFeature["A"];
            prog.datumAlignment.secondaryFeatureLabel = prog.datumLetterToFeature["B"];
            prog.datumAlignment.tertiaryFeatureLabel = prog.datumLetterToFeature["C"];
        }

        prog.valid = !prog.features.empty();
        if (!prog.valid && prog.errorMessage.empty()) {
            prog.errorMessage = "No valid FEAT/ or MEAS/ blocks found in DMIS program.";
        }
        return prog;
    }
   
} // namespace OpenCMM
