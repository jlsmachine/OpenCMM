// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QString>
#include <vector>
#include <memory>

namespace OpenCMM {

    struct DmisVector3d {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        DmisVector3d() = default;
        DmisVector3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    enum class DmisNodeType {
        FeatureDeclaration,
        MeasureBlock,
        PointMeasurement,
        GotoMotion,
        ToleranceCommand,
        Assignment
    };

    class DmisNode {
    public:
        virtual ~DmisNode() = default;
        virtual DmisNodeType nodeType() const = 0;
    };

    class DmisFeatureDeclNode : public DmisNode {
    public:
        DmisNodeType nodeType() const override { return DmisNodeType::FeatureDeclaration; }

        QString labelId;
        QString featureType;
        QString subtype;
        std::vector<double> parameters;
    };

    class DmisPointMeasureNode : public DmisNode {
    public:
        DmisNodeType nodeType() const override { return DmisNodeType::PointMeasurement; }

        DmisVector3d point{ 0.0, 0.0, 0.0 };
        DmisVector3d vector{ 0.0, 0.0, 1.0 };
    };

    class DmisMeasureBlockNode : public DmisNode {
    public:
        DmisNodeType nodeType() const override { return DmisNodeType::MeasureBlock; }

        QString featureLabel;
        std::vector<std::unique_ptr<DmisPointMeasureNode>> points;
    };

    class DmisGotoNode : public DmisNode {
    public:
        DmisNodeType nodeType() const override { return DmisNodeType::GotoMotion; }

        DmisVector3d target{ 0.0, 0.0, 0.0 };
    };

} // namespace OpenCMM