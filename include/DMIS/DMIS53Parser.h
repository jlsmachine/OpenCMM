// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>
#include <vector>

namespace OpenCMM {

    struct RefIdMapping {
        std::string targetObject; // e.g., "F(CYLN1)"
        std::string idType;       // "NAME" or "QIF"
        std::string keyOrDocId;   // e.g., "QPId" or QIF Document UUID
        std::string valueOrObjId; // e.g., UUID string or external database ID
    };

    struct DmisTargetPoint {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double i = 0.0;
        double j = 0.0;
        double k = 1.0;
    };

    struct DmisPlannedFeature {
        std::string label;        // e.g., "FEAT_1"
        std::string refId;        // Persistent QIF QPId UUID from REFID/
        std::string featureType;  // "PLANE", "CIRCLE", "CYLNDR", "SPHERE", "LINE"
        std::string boundarySub;  // "INNER", "OUTER", "CART", "UNBND"
        double nomX = 0.0;
        double nomY = 0.0;
        double nomZ = 0.0;
        double nomI = 0.0;
        double nomJ = 0.0;
        double nomK = 1.0;
        double nominalDiameter = 0.0;
        double plusTolMm = 0.0500;
        double minusTolMm = 0.0500;
        std::vector<DmisTargetPoint> points;
    };

    struct DmisProgramData {
        bool valid = false;
        std::string partName = "DMIS-PART";
        std::string units = "MM";
        double stylusDiameterMm = 3.0;
        double clearanceZMm = 100.0;
        std::vector<DmisPlannedFeature> features;
        std::string errorMessage;
    };

    class DMIS53Parser {
    public:
        DMIS53Parser();
        ~DMIS53Parser();
        static std::string validateAndTrimIdentifier(const std::string& rawIdent);
        static RefIdMapping parseRefIdStatement(const std::string& statement);
        static DmisProgramData parseDmisProgramText(const std::string& sourceText);
        static DmisProgramData parseDmisProgramFile(const std::string& filePath);
        static bool runSelfTest();
    };
    
} // namespace OpenCMM
