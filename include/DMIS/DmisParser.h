// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "dmis/DmisToken.h"
#include "dmis/DmisAst.h"
#include <vector>
#include <memory>

namespace OpenCMM {

    class DmisParser {
    public:
        explicit DmisParser(const std::vector<DmisToken>& tokens);
        std::vector<std::unique_ptr<DmisNode>> parseProgram();

    private:
        std::vector<DmisToken> m_tokens;
        size_t m_index = 0;

        const DmisToken& peek() const;
        const DmisToken& advance();
        bool match(DmisTokenType type);
        bool check(DmisTokenType type) const;
        bool isAtEnd() const;

        std::unique_ptr<DmisNode> parseStatement();
        std::unique_ptr<DmisFeatureDeclNode> parseFeatureDeclaration();
        std::unique_ptr<DmisMeasureBlockNode> parseMeasureBlock();
        std::unique_ptr<DmisGotoNode> parseGotoCommand();
    };

} // namespace OpenCMM