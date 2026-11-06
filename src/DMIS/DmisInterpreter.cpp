// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dmis/DmisInterpreter.h"
#include "ui/OcctViewport.hpp"
#include <QDebug>
#include <cmath>

namespace OpenCMM {

    DmisInterpreter::DmisInterpreter(DmisContext& context, OcctViewport* viewport, QObject* parent)
        : QObject(parent), m_context(context), m_viewport(viewport) {
    }

    void DmisInterpreter::executeProgram(const std::vector<std::unique_ptr<DmisNode>>& programNodes) {
        emit executionStarted();

        int index = 0;
        for (const auto& node : programNodes) {
            if (!node) continue;
            try {
                executeNode(node.get(), index);
            }
            catch (const std::exception& e) {
                emit errorOccurred(QString("Runtime Error at node %1: %2").arg(index).arg(e.what()));
                break;
            }
            index++;
        }

        emit executionFinished();
    }

    void DmisInterpreter::executeNode(const DmisNode* node, int index) {
        switch (node->nodeType()) {
        case DmisNodeType::FeatureDeclaration:
            executeFeatureDecl(static_cast<const DmisFeatureDeclNode*>(node));
            emit lineExecuted(index, "Declared Feature: " + static_cast<const DmisFeatureDeclNode*>(node)->labelId);
            break;
        case DmisNodeType::MeasureBlock:
            executeMeasureBlock(static_cast<const DmisMeasureBlockNode*>(node));
            emit lineExecuted(index, "Measured Feature: " + static_cast<const DmisMeasureBlockNode*>(node)->featureLabel);
            break;
        case DmisNodeType::GotoMotion:
            executeGoto(static_cast<const DmisGotoNode*>(node));
            emit lineExecuted(index, "Motion GOTO executed");
            break;
        default:
            break;
        }
    }

    void DmisInterpreter::executeFeatureDecl(const DmisFeatureDeclNode* node) {
        MeasuredFeatureData data;
        data.label = node->labelId;
        data.type = node->featureType;

        if (node->parameters.size() >= 3) {
            data.center = Eigen::Vector3d(node->parameters[0], node->parameters[1], node->parameters[2]);
        }
        if (node->parameters.size() >= 6) {
            data.axis = Eigen::Vector3d(node->parameters[3], node->parameters[4], node->parameters[5]);
        }
        if (node->parameters.size() >= 7 && (node->featureType == "CYL" || node->featureType == "CIRCLE")) {
            data.diameter = node->parameters[6];
        }

        m_context.registerFeature(node->labelId, data);
    }

    void DmisInterpreter::executeMeasureBlock(const DmisMeasureBlockNode* node) {
        MeasuredFeatureData data;
        data.label = node->featureLabel;

        // Aggregate point cloud collected during measurement routine
        Eigen::Vector3d sum(0, 0, 0);
        for (const auto& ptNode : node->points) {
            data.measuredPoints.push_back(ptNode->point);
            sum += ptNode->point;
        }

        // Basic least-squares centroid estimation for simulation/fallback
        if (!data.measuredPoints.empty()) {
            data.center = sum / static_cast<double>(data.measuredPoints.size());
        }

        m_context.registerFeature(node->featureLabel, data);

        // Bridge to OcctViewport to update 3D digital twin visualization if viewport is available
        if (m_viewport) {
            // e.g., m_viewport->addFittedFeatureVisual(data);
            qDebug() << "Visualizing fitted feature in OpenCMM viewport:" << node->featureLabel;
        }
    }

    void DmisInterpreter::executeGoto(const DmisGotoNode* node) {
        qDebug() << "Executing GOTO motion target:" << node->target.x() << node->target.y() << node->target.z();
        // Here you can stream coordinates or interpolate motion toward target via hardware HAL / Digital Twin
    }

} // namespace OpenCMM