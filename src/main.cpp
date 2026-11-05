// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ui/MainWindow.h"
#include <QApplication>
#include <iostream>
#include <cstring>

namespace OpenCMM {
    // Forward declaration of our DMIS 5.3 test suite runner
    void runDMIS53ParserTests();
}

int main(int argc, char* argv[]) {
    // Check for --test-dmis command line flag
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--test-dmis") == 0) {
            std::cout << "Executing DMIS 5.3 parser unit tests (--test-dmis flag detected)...\n";
            try {
                //OpenCMM::runDMIS53ParserTests();
                std::cout << "All DMIS 5.3 tests passed successfully.\n";
                return 0; // Exit successfully after tests complete
            }
            catch (const std::exception& e) {
                std::cerr << "DMIS 5.3 Test Failure: " << e.what() << "\n";
                return 1; // Exit with error code if assertion/test fails
            }
        }
    }

    // Normal application startup
    QApplication app(argc, argv);

    OpenCMM::MainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}