// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dmis/DmisContext.h"

namespace OpenCMM {

    DmisContext::DmisContext() {
        m_transformStack.push_back(Eigen::Isometry3d::Identity());
    }

    void DmisContext::registerFeature(const QString& label, const MeasuredFeatureData& data) {
        m_featureRegistry[label.toUpper()] = data;
    }

    bool DmisContext::getFeature(const QString& label, MeasuredFeatureData& outData) const {
        auto it = m_featureRegistry.constFind(label.toUpper());
        if (it != m_featureRegistry.constEnd()) {
            outData = it.value();
            return true;
        }
        return false;
    }

    void DmisContext::pushTransformation(const Eigen::Isometry3d& transform) {
        Eigen::Isometry3d current = getCurrentTransformation();
        m_transformStack.push_back(current * transform);
    }

    void DmisContext::popTransformation() {
        if (m_transformStack.size() > 1) {
            m_transformStack.pop_back();
        }
    }

    Eigen::Isometry3d DmisContext::getCurrentTransformation() const {
        if (m_transformStack.empty()) {
            return Eigen::Isometry3d::Identity();
        }
        return m_transformStack.back();
    }

    void DmisContext::setActiveStylusRadius(double radius) {
        m_activeStylusRadius = radius;
    }

    double DmisContext::getActiveStylusRadius() const {
        return m_activeStylusRadius;
    }

} // namespace OpenCMM