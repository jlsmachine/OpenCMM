// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "dmis/DMIS53Parser.h"
#include <iostream>
#include <cstring>

int main(int argc, char* argv[]) {
    bool runTestFlag = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--test-dmis") == 0) {
            runTestFlag = true;
            break;
        }
    }

    std::cout << "[OpenCMM Test Runner] Initializing DMIS 5.3 parser validation..." << std::endl;

    if (runTestFlag) {
        // Execute DMIS 5.3 Conformance tests (64-char limit & REFID mappings)
        bool passed = OpenCMM::DMIS53Parser::runSelfTest();
        if (passed) {
            std::cout << "[SUCCESS] DMIS 5.3 validation tests passed successfully!" << std::endl;
            return 0;
        }
        else {
            std::cerr << "[FAILURE] DMIS 5.3 validation tests failed." << std::endl;
            return 1;
        }
    }
    else {
        std::cout << "Usage: dmis_parser_test --test-dmis" << std::endl;
        return 0;
    }
}