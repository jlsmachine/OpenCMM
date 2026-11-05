// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "dmis/DMIS53Parser.h"
#include <iostream>
#include <cassert>
#include <stdexcept>

namespace OpenCMM {

    void runDMIS53ParserTests() {
        std::cout << "[Test] Running DMIS 5.3 Parser Verification...\n";

        // Test 1: Valid REFID Statement with NAME format
        {
            std::string cmd = "REFID/F(CYLN1), NAME, 'QPId', '53827D36-8C6C-420f-88A8-B15361AD6D19'";
            RefIdMapping map = DMIS53Parser::parseRefIdStatement(cmd);
            assert(map.targetObject == "F(CYLN1)");
            assert(map.idType == "NAME");
            assert(map.keyOrDocId == "QPId");
            assert(map.valueOrObjId == "53827D36-8C6C-420f-88A8-B15361AD6D19");
            std::cout << "  [PASS] REFID NAME format parsed successfully.\n";
        }

        // Test 2: Valid REFID Statement with QIF format
        {
            std::string cmd = "REFID/F(CYLN1), QIF, '43083E4F-D3E2-48cd-A7E0-62AC6940E3AC', '47'";
            RefIdMapping map = DMIS53Parser::parseRefIdStatement(cmd);
            assert(map.targetObject == "F(CYLN1)");
            assert(map.idType == "QIF");
            assert(map.keyOrDocId == "43083E4F-D3E2-48cd-A7E0-62AC6940E3AC");
            assert(map.valueOrObjId == "47");
            std::cout << "  [PASS] REFID QIF format parsed successfully.\n";
        }

        // Test 3: 64-Character Identifier Limit Check (Valid)
        {
            // Exactly 64 characters long identifier name
            std::string validIdent = "F(ThisIsAnExtremelyLongFeatureLabelThatReachesExactly64C)";
            std::string result = DMIS53Parser::validateAndTrimIdentifier(validIdent);
            assert(result == validIdent);
            std::cout << "  [PASS] 64-character limit accepted valid identifier.\n";
        }

        // Test 4: 64-Character Identifier Limit Check (Exceeded)
        {
            // 65 characters long identifier name
            std::string invalidIdent = "F(ThisIsAnExtremelyLongFeatureLabelThatReachesExactly65Ch)";
            bool exceptionThrown = false;
            try {
                DMIS53Parser::validateAndTrimIdentifier(invalidIdent);
            }
            catch (const std::runtime_error& e) {
                exceptionThrown = true;
            }
            assert(exceptionThrown);
            std::cout << "  [PASS] Over-64-character identifier correctly rejected with exception.\n";
        }

        std::cout << "[Test] All DMIS 5.3 parser tests completed successfully!\n";
    }

} // namespace OpenCMM