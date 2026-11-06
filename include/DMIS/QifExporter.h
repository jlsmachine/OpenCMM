// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <QString>
#include <vector>
#include "math/MetrologyMath.h"

namespace OpenCMM {

    struct InspectionResultItem {
        QString featureName;
        QString refId;          // Parsed DMIS 5.3 REFID / QIF identifier
        QString featureType;    // "Bore", "Plane", "Sphere", etc.
        double nominalMm;
        double actualMm;
        double plusTolMm;
        double minusTolMm;
        bool passed;
    };

    class QifExporter {
    public:
        // Export solved inspection tree to QIF 2.1 XML
        static bool exportToQif(const QString& filePath,
            const QString& partId,
            const std::vector<InspectionResultItem>& results);
    };

} // namespace OpenCMM