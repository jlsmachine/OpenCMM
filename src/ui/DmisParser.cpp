// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dmis/DmisParser.h"
#include <stdexcept>

namespace OpenCMM {

    DmisParser::DmisParser(const std::vector<DmisToken>& tokens)
        : m_tokens(tokens), m_index(0) {}

    const DmisToken& DmisParser::peek() const {
        if (m_index >= m_tokens.size()) {
            return m_tokens.back();
        }
        return m_tokens[m_index];
    }

    const DmisToken& DmisParser::advance() {
        if (!isAtEnd()) {
            m_index++;
        }
        return m_tokens[m_index - 1];
    }

    bool DmisParser::check(DmisTokenType type) const {
        if (isAtEnd()) return false;
        return peek().type == type;
    }

    bool DmisParser::match(DmisTokenType type) {
        if (check(type)) {
            advance();
            return true;
        }
        return false;
    }

    bool DmisParser::isAtEnd() const {
        return peek().type == DmisTokenType::EndOfFile;
    }

    std::vector<std::unique_ptr<DmisNode>> DmisParser::parseProgram() {
        std::vector<std::pair<std::string, std::unique_ptr<DmisNode>>> statements; // placeholder for AST
        std::vector<std::unique_ptr<DmisNode>> programNodes;
        while (!isAtEnd()) {
            if (peek().type == DmisTokenType::EndStatement) {
                advance();
                continue;
            }
            auto stmt = parseStatement();
            if (stmt) {
                programNodes.push_back(std::move(stmt));
            } else {
                advance(); // Skip unparsable token to prevent infinite loops
            }
        }
        return programNodes;
    }

    std::unique_ptr<DmisNode> DmisParser::parseStatement() {
        if (check(DmisTokenType::Identifier) || check(DmisTokenType::Keyword)) {
            QString text = peek().text.toUpper();
            
            // Check for Feature Declaration: F(label) = FEAT/...
            if (text.startsWith("F(") || text == "FEAT") {
                return parseFeatureDeclaration();
            }
            // Check for MEAS block
            else if (text == "MEAS") {
                return parseMeasureBlock();
            }
            // Check for GOTO motion
            else if (text == "GOTO") {
                return parseGotoCommand();
            }
        }
        advance();
        return nullptr;
    }

    std::unique_ptr<DmisFeatureDeclNode> DmisParser::parseFeatureDeclaration() {
        auto node = std::make_unique<DmisFeatureDeclNode>();
        
        // Parse label (e.g. F(CYL-1))
        node->labelId = advance().text;
        
        if (match(DmisTokenType::Equal)) {
            // Expect FEAT / type
            if (peek().text.toUpper() == "FEAT") {
                advance();
            }
            if (match(DmisTokenType::Slash)) {
                node->featureType = advance().text.toUpper();
            }
            // Parse remaining comma-separated parameters
            while (!isAtEnd() && !check(DmisTokenType::EndStatement)) {
                if (match(DmisTokenType::Comma)) {
                    if (check(DmisTokenType::Number) || check(DmisTokenType::Identifier)) {
                        bool negative = false;
                        if (peek().text == "-") {
                            advance();
                            negative = true;
                        }
                        if (check(DmisTokenType::Number)) {
                            double val = advance().text.toDouble();
                            node->parameters.push_back(negative ? -val : val);
                        } else {
                            node->subtype = advance().text.toUpper();
                        }
                    }
                } else {
                    advance();
                }
            }
        }
        return node;
    }

    std::unique_ptr<DmisMeasureBlockNode> DmisParser::parseMeasureBlock() {
        auto node = std::make_unique<DmisMeasureBlockNode>();
        advance(); // consume MEAS
        if (match(DmisTokenType::Slash)) {
            advance(); // e.g. CYL
        }
        if (match(DmisTokenType::Comma)) {
            node->featureLabel = advance().text;
        }

        // Parse points until ENDMES
        while (!isAtEnd()) {
            if (peek().text.toUpper() == "ENDMES") {
                advance();
                break;
            }
            if (peek().text.toUpper() == "PTMEAS") {
                advance(); // consume PTMEAS
                if (match(DmisTokenType::Slash)) {
                    advance(); // e.g. CART
                }
                auto ptNode = std::make_unique<DmisPointMeasureNode>();
                std::vector<double> coords;
                while (!isAtEnd() && !check(DmisTokenType::EndStatement) && peek().text.toUpper() != "ENDMES") {
                    if (match(DmisTokenType::Comma)) {
                        if (check(DmisTokenType::Number)) {
                            coords.push_back(advance().text.toDouble());
                        }
                    } else {
                        advance();
                    }
                }
                if (coords.size() >= 3) {
                    ptNode->point = Eigen::Vector3d(coords[0], coords[1], coords[2]);
                }
                if (coords.size() >= 6) {
                    ptNode->vector = Eigen::Vector3d(coords[3], coords[4], coords[5]);
                }
                node->points.push_back(std::move(ptNode));
            } else {
                advance();
            }
        }
        return node;
    }

    std::unique_ptr<DmisGotoNode> DmisParser::parseGotoCommand() {
        auto node = std::make_unique<DmisGotoNode>();
        advance(); // consume GOTO
        if (match(DmisTokenType::Slash)) {
            std::vector<double> coords;
            while (!isAtEnd() && !check(DmisTokenType::EndStatement)) {
                if (match(DmisTokenType::Comma)) {
                    if (check(DmisTokenType::Number)) {
                        coords.push_back(advance().text.toDouble());
                    }
                } else {
                    advance();
                }
            }
            if (coords.size() >= 3) {
                node->target = Eigen::Vector3d(coords[0], coords[1], coords[2]);
            }
        }
        return node;
    }

} // namespace OpenCMM
