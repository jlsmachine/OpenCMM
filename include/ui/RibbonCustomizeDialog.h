// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QComboBox>
#include <QString>
#include <QStringList>
#include <vector>

namespace OpenCMM {

    struct RibbonCommandMeta {
        QString id;          // Unique command ID, e.g. "cad_load_step"
        QString label;       // Display label, e.g. "Load STEP"
        QString iconSymbol;  // Icon glyph, e.g. "📦"
        QString groupTitle;  // Ribbon group header, e.g. "3D CAD Model"
    };

    struct RibbonTabConfig {
        QString name;
        bool visible = true;
        bool isCustom = false;
        QStringList commandIds;
    };

    class RibbonCustomizeDialog : public QDialog {
        Q_OBJECT
    public:
        explicit RibbonCustomizeDialog(bool darkMode, QWidget* parent = nullptr);
        ~RibbonCustomizeDialog() override = default;

        static std::vector<RibbonCommandMeta> availableCommandsCatalog();
        static std::vector<RibbonTabConfig> defaultRibbonLayout();
        static std::vector<RibbonTabConfig> loadSavedLayout();
        static void saveLayout(const std::vector<RibbonTabConfig>& layout);
        static const RibbonCommandMeta* findCommandMeta(const QString& cmdId);

    private slots:
        void onSelectedTabChanged(int row);
        void onTabItemChanged(QListWidgetItem* item);

        void onAddTabClicked();
        void onRenameTabClicked();
        void onDeleteTabClicked();
        void onMoveTabUpClicked();
        void onMoveTabDownClicked();

        void onAddCommandToTabClicked();
        void onRemoveCommandFromTabClicked();
        void onMoveCommandUpClicked();
        void onMoveCommandDownClicked();
        void onCategoryFilterChanged(int index);

        void onResetDefaultsClicked();
        void onSaveClicked();

    private:
        void setupUi(bool darkMode);
        void populateTabList(int selectRow = 0);
        void populateTabCommandsList(int tabIndex);
        void populateCatalogList(const QString& categoryFilter = QString());
        void syncCurrentTabCommandsFromList();

        std::vector<RibbonTabConfig> m_tabs;
        int m_currentTabIndex = -1;
        bool m_updatingUi = false;

        QListWidget* m_tabListWidget = nullptr;
        QListWidget* m_tabCommandsListWidget = nullptr;
        QListWidget* m_catalogListWidget = nullptr;
        QComboBox* m_categoryCombo = nullptr;

        QPushButton* m_renameTabBtn = nullptr;
        QPushButton* m_deleteTabBtn = nullptr;
        QPushButton* m_moveTabUpBtn = nullptr;
        QPushButton* m_moveTabDownBtn = nullptr;

        QPushButton* m_addCmdBtn = nullptr;
        QPushButton* m_removeCmdBtn = nullptr;
        QPushButton* m_moveCmdUpBtn = nullptr;
        QPushButton* m_moveCmdDownBtn = nullptr;
    };

} // namespace OpenCMM
