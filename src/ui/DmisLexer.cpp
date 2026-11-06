// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dmis/DmisToken.h"
#include <cctype>

namespace OpenCMM {

    DmisLexer::DmisLexer(const QString& sourceCode)
        : m_src(sourceCode), m_pos(0), m_line(1), m_col(1) {}

    QChar DmisLexer::peek() const {
        if (m_pos >= m_src.length()) return QChar();
        return m_src.at(m_pos);
    }

    QChar DmisLexer::advance() {
        if (m_pos >= m_src.length()) return QChar();
        QChar c = m_src.at(m_pos++);
        if (c == '\n') {
            m_line++;
            m_col = 1;
        } else {
            m_col++;
        }
        return c;
    }

    bool DmisLexer::isAtEnd() const {
        return m_pos >= m_src.length();
    }

    void DmisLexer::skipWhitespaceAndComments() {
        while (!isAtEnd()) {
            QChar c = peek();
            if (c.isSpace()) {
                advance();
            } else if (c == '$' && m_pos + 1 < m_src.length() && m_src.at(m_pos + 1) == '$') {
                // DMIS comment line: $$ ...
                while (!isAtEnd() && peek() != '\n') {
                    advance();
                }
            } else {
                break;
            }
        }
    }

    std::vector<DmisToken> DmisLexer::tokenize() {
        std::vector<DmisToken> tokens;
        while (!isAtEnd()) {
            skipWhitespaceAndComments();
            if (isAtEnd()) break;

            int startLine = m_line;
            int startCol = m_col;
            QChar c = peek();

            // Identifiers or Keywords e.g. F(CYL-1), FEAT, MEAS, GOTO
            if (c.isLetter() || c == '_' || c == '(' || c == ')') {
                QString text;
                while (!isAtEnd()) {
                    QChar nc = peek();
                    if (nc.isLetterOrNumber() || nc == '_' || nc == '-' || nc == '(' || nc == ')') {
                        text.append(advance());
                    } else {
                        break;
                    }
                }

                DmisTokenType type = DmisTokenType::Identifier;
                QString upper = text.toUpper();
                if (upper == "FEAT" || upper == "MEAS" || upper == "ENDMES" || 
                    upper == "GOTO" || upper == "TOL" || upper == "UNITS" ||
                    upper == "PTMEAS" || upper == "CART") {
                    type = DmisTokenType::Keyword;
                }
                tokens.push_back({type, text, startLine, startCol});
            }
            // Numbers
            else if (c.isDigit() || c == '.' || c == '-' || c == '+') {
                // Check if sign is part of a number
                QString text;
                text.append(advance());
                
                bool hasDecimal = (text == ".");
                while (!isAtEnd()) {
                    QChar nc = peek();
                    if (nc.isDigit()) {
                        text.append(advance());
                    } else if (nc == '.' && !hasDecimal) {
                        hasDecimal = true;
                        text.append(advance());
                    } else {
                        break;
                    }
                }
                
                // If it's just a single '+' or '-' operator without digits following, treat as symbol if needed
                if (text == "-" || text == "+") {
                    tokens.push_back({DmisTokenType::Unknown, text, startLine, startCol});
                } else {
                    tokens.push_back({DmisTokenType::Number, text, startLine, startCol});
                }
            }
            // Symbols
            else {
                advance();
                QString text(c);
                DmisTokenType type = DmisTokenType::Unknown;
                if (c == '/') type = DmisTokenType::Slash;
                else if (c == '=') type = DmisTokenType::Equal;
                else if (c == ',') type = DmisTokenType::Comma;
                else if (c == '(') type = DmisTokenType::LeftParen;
                else if (c == ')') type = DmisTokenType::RightParen;
                else if (c == ':') type = DmisTokenType::Colon;
                else if (c == '\n' || c == '$') type = DmisTokenType::EndStatement;

                tokens.push_back({type, text, startLine, startCol});
            }
        }
        tokens.push_back({DmisTokenType::EndOfFile, "", m_line, m_col});
        return tokens;
    }

} // namespace OpenCMM
