// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QString>
#include <vector>

namespace OpenCMM {

    enum class DmisTokenType {
        Unknown,
        Identifier,    // e.g., F(CYL-1), L(PATH-1)
        Keyword,       // e.g., FEAT, MEAS, ENDMES, GOTO, TOL, UNITS
        Number,        // e.g., 12.500, -0.001
        StringLiteral, // e.g., "PART_1"
        Slash,         // /
        Equal,         // =
        Comma,         // ,
        LeftParen,     // (
        RightParen,    // )
        Colon,         // :
        EndStatement,  // newline or $
        EndOfFile
    };

    struct DmisToken {
        DmisTokenType type = DmisTokenType::Unknown;
        QString text;
        int line = 0;
        int column = 0;
    };

    class DmisLexer {
    public:
        explicit DmisLexer(const QString& sourceCode);
        std::vector<DmisToken> tokenize();

    private:
        QString m_src;
        int m_pos = 0;
        int m_line = 1;
        int m_col = 1;

        QChar peek() const;
        QChar advance();
        bool isAtEnd() const;
        void skipWhitespaceAndComments();
        DmisToken lexToken();
    };

} // namespace OpenCMM