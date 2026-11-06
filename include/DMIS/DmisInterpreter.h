// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "dmis/DmisAst.h"
#include "dmis/DmisContext.h"
#include <vector>
#include <memory>
#include <QObject>

namespace OpenCMM {

    class OcctViewport; // Forward declaration

    class DmisInterpreter : public QObject {
        Q_OBJECT
    public:
        explicit DmisInterpreter(DmisContext& context, OcctViewport* viewport, QObject* parent = nullptr);

        void executeProgram(const std::vector<std::unique_ptr<DmisNode>>& programNodes);

    signals:
        void executionStarted();
        void lineExecuted(int nodeIndex, QString description);
        void executionFinished();
        void errorOccurred(QString errorMessage);

    private:
        void executeNode(const DmisNode* node, int index);
        void executeFeatureDecl(const DmisFeatureDeclNode* node);
        void executeMeasureBlock(const DmisMeasureBlockNode* node);
        void executeGoto(const DmisGotoNode* node);

        DmisContext& m_context;
        OcctViewport* m_viewport;
    };

} // namespace OpenCMM