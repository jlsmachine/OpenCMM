// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QString>
#include <QHash>
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace OpenCMM {

    struct MeasuredFeatureData {
        QString label;
        QString type;
        Eigen::Vector3d center{ 0, 0, 0 };
        Eigen::Vector3d axis{ 0, 0, 1 };
        double diameter = 0.0;
        std::vector<Eigen::Vector3d> measuredPoints;
    };

    class DmisContext {
    public:
        DmisContext();

        void registerFeature(const QString& label, const MeasuredFeatureData& data);
        bool getFeature(const QString& label, MeasuredFeatureData& outData) const;

        void pushTransformation(const Eigen::Isometry3d& transform);
        void popTransformation();
        Eigen::Isometry3d getCurrentTransformation() const;

        void setActiveStylusRadius(double radius);
        double getActiveStylusRadius() const;

    private:
        QHash<QString, MeasuredFeatureData> m_featureRegistry;
        std::vector<Eigen::Isometry3d> m_transformStack;
        double m_activeStylusRadius = 1.0;
    };

} // namespace OpenCMM