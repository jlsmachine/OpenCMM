// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/RibbonCustomizeDialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QInputDialog>
#include <QMessageBox>
#include <QDialogButtonBox>
#include <QSettings>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QAbstractItemView>
#include <algorithm>

namespace OpenCMM {

    std::vector<RibbonCommandMeta> RibbonCustomizeDialog::availableCommandsCatalog() {
        return {
            { "sys_settings",        "Settings...",        "⚙",  "System & Hardware" },
            { "sys_about",           "About",              "ℹ",  "System & Hardware" },
            { "sys_customize",       "Customize Ribbon",   "🛠", "System & Hardware" },
            { "dro_zero",            "Zero XYZ",           "🎯", "Machine DRO" },
            { "dro_units",           "Units (mm/in)",      "📏", "Machine DRO" },
            { "buf_clear_points",    "Clear Points",       "🗑", "Point Buffer" },
            { "sim_target_combo",    "Virtual Target",     "🎯", "Offline Simulator & CAD" },
            { "sim_hit",             "Simulate Hit",       "⚡", "Offline Simulator & CAD" },
            { "cad_auto_probe",      "Auto-Probe CAD",     "🎯", "Offline Simulator & CAD" },
            { "prog_create_feature", "Create Feature...",  "➕", "Offline Simulator & CAD" },
            { "dmis_save",           "Save DMIS",          "💾", "DMIS 5.3 Program" },
            { "dmis_load",           "Load DMIS",          "📂", "DMIS 5.3 Program" },
            { "dmis_run",            "Run DMIS",           "▶",  "DMIS 5.3 Program" },
            { "dmis_preflight",      "Pre-Flight Check",   "🔍", "DMIS 5.3 Program" },
            { "probe_comp_combo",    "Tip Compensation",   "🧭", "Stylus Compensation" },
            { "probe_calibrate",     "Calibrate Tip",      "🔴", "Stylus Compensation" },
            { "datum_set_a",         "Set Datum A",        "🅰", "CAD / Feature Datums (A-B-C)" },
            { "datum_set_b",         "Set Datum B",        "🅱", "CAD / Feature Datums (A-B-C)" },
            { "datum_set_c",         "Set Datum C",        "🅲", "CAD / Feature Datums (A-B-C)" },
            { "pcs_align_321",       "3-2-1 Align...",     "📐", "Part Coordinate System (PCS)" },
            { "pcs_coord_toggle",    "Coord: MCS / PCS",   "🌐", "Part Coordinate System (PCS)" },
            { "meas_workplane",      "2D Workplane",       "📐", "Active Projection" },
            { "meas_style",          "Feature Style",      "👁", "Feature Display" },
            { "meas_fit_plane",      "Fit Plane",          "▱",  "3D Features" },
            { "meas_fit_sphere",     "Fit Sphere",         "🌐", "3D Features" },
            { "meas_fit_cylinder",   "Fit Cylinder",       "⌭",  "3D Features" },
            { "meas_fit_circle",     "Fit 2D Circle",      "⭕", "2D Workplane Features" },
            { "meas_fit_line",       "Fit 2D Line",        "╱",  "2D Workplane Features" },
            { "meas_relation",       "Distance / Angle",   "↔",  "Dimensions & Relations" },
            { "cad_load_step",       "Load STEP",          "📦", "3D CAD Model" },
            { "cad_orient",          "Orient CAD...",      "🧭", "3D CAD Model" },
            { "cad_surface_dev",     "Surface Dev",        "🌡", "Point Cloud vs. CAD" },
            { "rep_edit_tol",        "Edit Tol...",        "±",  "Feature Evaluation" },
            { "rep_clear_all",       "Clear All",          "🧹", "Manage Features" },
            { "rep_export_report",   "Export Report",      "📄", "Quality & DMIS Export" },
            { "sys_settings",        "Settings...",        "⚙",  "System & Hardware" },
            { "sys_about",           "About",              "ℹ",  "System & Hardware" },
            { "sys_customize",       "Customize Ribbon",   "🛠", "System & Hardware" },
            { "sys_mcp_toggle",      "MCP Server Toggle",  "🤖", "System & Hardware" }
        };
    }

    const RibbonCommandMeta* RibbonCustomizeDialog::findCommandMeta(const QString& cmdId) {
        static const std::vector<RibbonCommandMeta> catalog = availableCommandsCatalog();
        for (const auto& item : catalog) {
            if (item.id == cmdId) {
                return &item;
            }
        }
        return nullptr;
    }

    std::vector<RibbonTabConfig> RibbonCustomizeDialog::defaultRibbonLayout() {
        return {
          {
                "Home", true, false,
                { "sys_settings", "sys_about", "sys_mcp_toggle", "dro_zero", "dro_units", "buf_clear_points",
                  "sim_target_combo", "sim_hit", "cad_auto_probe", "prog_create_feature",
                  "dmis_save", "dmis_load", "dmis_run", "dmis_preflight" }
            },
            {
                "Probe & Alignment", true, false,
                { "probe_comp_combo", "probe_calibrate", "datum_set_a", "datum_set_b", "datum_set_c",
                  "pcs_align_321", "pcs_coord_toggle", "buf_clear_points" }
            },
            {
                "Measure", true, false,
                { "meas_workplane", "meas_style", "prog_create_feature", "meas_fit_plane", "meas_fit_sphere",
                  "meas_fit_cylinder", "meas_fit_circle", "meas_fit_line", "meas_relation" }
            },
            {
                "CAD & Deviation", true, false,
                { "cad_load_step", "cad_orient", "cad_auto_probe", "prog_create_feature",
                  "dmis_save", "dmis_load", "dmis_run", "dmis_preflight", // <--- Or here
                  "datum_set_a", "datum_set_b", "datum_set_c", "cad_surface_dev" }
            },
            {
                "Reporting", true, false,
                { "rep_edit_tol", "rep_clear_all", "dmis_save", "rep_export_report" }
            }
        };
    }

    std::vector<RibbonTabConfig> RibbonCustomizeDialog::loadSavedLayout() {
        QSettings settings("JLS Machine", "OpenCMM");
        settings.beginGroup("RibbonCustomization");
        const QByteArray rawJson = settings.value("LayoutJsonV2").toByteArray();
        settings.endGroup();

        if (!rawJson.isEmpty()) {
            QJsonDocument doc = QJsonDocument::fromJson(rawJson);
            if (doc.isArray()) {
                std::vector<RibbonTabConfig> loaded;
                QJsonArray arr = doc.array();
                for (const QJsonValue& val : arr) {
                    if (!val.isObject()) continue;
                    QJsonObject obj = val.toObject();
                    RibbonTabConfig tab;
                    tab.name = obj.value("name").toString().trimmed();
                    if (tab.name.isEmpty()) continue;
                    tab.visible = obj.value("visible").toBool(true);
                    tab.isCustom = obj.value("isCustom").toBool(false);

                    QJsonArray cmds = obj.value("commands").toArray();
                    for (const QJsonValue& cVal : cmds) {
                        QString cid = cVal.toString();
                        if (findCommandMeta(cid) != nullptr) {
                            tab.commandIds.append(cid);
                        }
                    }
                    loaded.push_back(tab);
                }
                if (!loaded.empty()) {
                    return loaded;
                }
            }
        }

        std::vector<RibbonTabConfig> defaults = RibbonCustomizeDialog::defaultRibbonLayout();
        settings.beginGroup("RibbonCustomization");
        for (auto& tab : defaults) {
            const QString legacyKey = "Tab_" + tab.name;
            if (settings.contains(legacyKey)) {
                tab.visible = settings.value(legacyKey, true).toBool();
            }
        }
        settings.endGroup();
        return defaults;
    }

    void RibbonCustomizeDialog::saveLayout(const std::vector<RibbonTabConfig>& layout) {
        QJsonArray arr;
        for (const auto& tab : layout) {
            QJsonObject obj;
            obj["name"] = tab.name;
            obj["visible"] = tab.visible;
            obj["isCustom"] = tab.isCustom;

            QJsonArray cmds;
            for (const QString& cid : tab.commandIds) {
                cmds.append(cid);
            }
            obj["commands"] = cmds;
            arr.append(obj);
        }

        QJsonDocument doc(arr);
        QSettings settings("JLS Machine", "OpenCMM");
        settings.beginGroup("RibbonCustomization");
        settings.setValue("LayoutJsonV2", doc.toJson(QJsonDocument::Compact));
        settings.endGroup();
        settings.sync();
    }

    RibbonCustomizeDialog::RibbonCustomizeDialog(bool darkMode, QWidget* parent)
        : QDialog(parent),
        m_tabs(loadSavedLayout()) {
        setWindowTitle("Customize Ribbon Tabs & Commands");
        resize(880, 540);
        setupUi(darkMode);
        populateCatalogList();
        populateTabList(0);
    }

    void RibbonCustomizeDialog::setupUi(bool darkMode) {
        setStyleSheet(darkMode
            ? "QDialog { background-color: #1e2227; color: #e0e0e0; }"
            "QGroupBox { border: 1px solid #3a3f4b; border-radius: 4px; margin-top: 10px; padding-top: 12px; color: #abb2bf; font-weight: bold; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
            "QListWidget { background-color: #181a1f; color: #ffffff; border: 1px solid #3a3f4b; border-radius: 4px; padding: 4px; font-size: 12px; }"
            "QListWidget::item { padding: 5px 6px; border-bottom: 1px solid #23272e; }"
            "QListWidget::item:selected { background-color: #2d5a88; color: #ffffff; }"
            "QComboBox { background-color: #181a1f; color: #ffffff; border: 1px solid #4b5263; border-radius: 3px; padding: 4px 8px; font-weight: bold; }"
            "QPushButton { background-color: #282c34; color: #abb2bf; border: 1px solid #4b5263; border-radius: 4px; padding: 5px 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #3e4451; color: #ffffff; border: 1px solid #61afef; }"
            "QPushButton:disabled { color: #5c6370; border-color: #2c313a; }"
            : "QDialog { background-color: #f1f5f9; color: #1e293b; }"
            "QGroupBox { border: 1px solid #cbd5e1; border-radius: 4px; margin-top: 10px; padding-top: 12px; color: #334155; font-weight: bold; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
            "QListWidget { background-color: #ffffff; color: #0f172a; border: 1px solid #cbd5e1; border-radius: 4px; padding: 4px; font-size: 12px; }"
            "QListWidget::item { padding: 5px 6px; border-bottom: 1px solid #f1f5f9; }"
            "QListWidget::item:selected { background-color: #0284c7; color: #ffffff; }"
            "QComboBox { background-color: #ffffff; color: #0f172a; border: 1px solid #94a3b8; border-radius: 3px; padding: 4px 8px; font-weight: bold; }"
            "QPushButton { background-color: #ffffff; color: #334155; border: 1px solid #cbd5e1; border-radius: 4px; padding: 5px 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #e2e8f0; color: #0f172a; border: 1px solid #0284c7; }"
            "QPushButton:disabled { color: #94a3b8; border-color: #e2e8f0; }"
        );

        auto* mainLayout = new QVBoxLayout(this);
        mainLayout->setSpacing(10);

        auto* headerLbl = new QLabel(
            "Create custom tabs, toggle tab visibility, and add, remove, or reorder commands on any tab. "
            "You can also drag-and-drop commands directly inside the middle list.",
            this
        );
        headerLbl->setWordWrap(true);
        headerLbl->setStyleSheet("color: #7f848e; font-size: 11px;");
        mainLayout->addWidget(headerLbl);

        auto* columnsLayout = new QHBoxLayout();
        columnsLayout->setSpacing(12);

        // --- Column 1: Ribbon Tabs ---
        auto* tabGroup = new QGroupBox("1. Ribbon Tabs", this);
        auto* tabLayout = new QVBoxLayout(tabGroup);

        m_tabListWidget = new QListWidget(tabGroup);
        connect(m_tabListWidget, &QListWidget::currentRowChanged, this, &RibbonCustomizeDialog::onSelectedTabChanged);
        connect(m_tabListWidget, &QListWidget::itemChanged, this, &RibbonCustomizeDialog::onTabItemChanged);
        tabLayout->addWidget(m_tabListWidget, 1);

        auto* tabBtnRow1 = new QHBoxLayout();
        auto* addTabBtn = new QPushButton("➕ New Tab", tabGroup);
        m_renameTabBtn = new QPushButton("✏ Rename", tabGroup);
        m_deleteTabBtn = new QPushButton("🗑 Delete", tabGroup);
        connect(addTabBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onAddTabClicked);
        connect(m_renameTabBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onRenameTabClicked);
        connect(m_deleteTabBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onDeleteTabClicked);
        tabBtnRow1->addWidget(addTabBtn);
        tabBtnRow1->addWidget(m_renameTabBtn);
        tabBtnRow1->addWidget(m_deleteTabBtn);
        tabLayout->addLayout(tabBtnRow1);

        auto* tabBtnRow2 = new QHBoxLayout();
        m_moveTabUpBtn = new QPushButton("▲ Move Tab Left", tabGroup);
        m_moveTabDownBtn = new QPushButton("▼ Move Tab Right", tabGroup);
        connect(m_moveTabUpBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onMoveTabUpClicked);
        connect(m_moveTabDownBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onMoveTabDownClicked);
        tabBtnRow2->addWidget(m_moveTabUpBtn);
        tabBtnRow2->addWidget(m_moveTabDownBtn);
        tabLayout->addLayout(tabBtnRow2);

        columnsLayout->addWidget(tabGroup, 3);

        // --- Column 2: Commands on Selected Tab ---
        auto* cmdGroup = new QGroupBox("2. Commands on Selected Tab (Order: Left → Right)", this);
        auto* cmdLayout = new QVBoxLayout(cmdGroup);

        m_tabCommandsListWidget = new QListWidget(cmdGroup);
        m_tabCommandsListWidget->setDragDropMode(QAbstractItemView::InternalMove);
        m_tabCommandsListWidget->setDefaultDropAction(Qt::MoveAction);
        connect(m_tabCommandsListWidget->model(), &QAbstractItemModel::rowsMoved, this, [this]() {
            syncCurrentTabCommandsFromList();
            });
        cmdLayout->addWidget(m_tabCommandsListWidget, 1);

        auto* cmdBtnRow = new QHBoxLayout();
        m_moveCmdUpBtn = new QPushButton("▲ Move Up", cmdGroup);
        m_moveCmdDownBtn = new QPushButton("▼ Move Down", cmdGroup);
        m_removeCmdBtn = new QPushButton("✖ Remove", cmdGroup);
        connect(m_moveCmdUpBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onMoveCommandUpClicked);
        connect(m_moveCmdDownBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onMoveCommandDownClicked);
        connect(m_removeCmdBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onRemoveCommandFromTabClicked);
        cmdBtnRow->addWidget(m_moveCmdUpBtn);
        cmdBtnRow->addWidget(m_moveCmdDownBtn);
        cmdBtnRow->addWidget(m_removeCmdBtn);
        cmdLayout->addLayout(cmdBtnRow);

        columnsLayout->addWidget(cmdGroup, 4);

        // --- Column 3: Available Commands Catalog ---
        auto* catGroup = new QGroupBox("3. Available Commands Catalog", this);
        auto* catLayout = new QVBoxLayout(catGroup);

        auto* filterRow = new QHBoxLayout();
        filterRow->addWidget(new QLabel("Category:", catGroup));
        m_categoryCombo = new QComboBox(catGroup);
        m_categoryCombo->addItem("All Categories", "");

        QStringList categories;
        for (const auto& meta : availableCommandsCatalog()) {
            if (!categories.contains(meta.groupTitle)) {
                categories.append(meta.groupTitle);
                m_categoryCombo->addItem(meta.groupTitle, meta.groupTitle);
            }
        }
        connect(m_categoryCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &RibbonCustomizeDialog::onCategoryFilterChanged);
        filterRow->addWidget(m_categoryCombo, 1);
        catLayout->addLayout(filterRow);

        m_catalogListWidget = new QListWidget(catGroup);
        connect(m_catalogListWidget, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) {
            onAddCommandToTabClicked();
            });
        catLayout->addWidget(m_catalogListWidget, 1);

        m_addCmdBtn = new QPushButton("⬅ Add Selected Command to Tab", catGroup);
        m_addCmdBtn->setStyleSheet("background-color: #2b6cb0; color: #ffffff; border: 1px solid #61afef; padding: 6px 12px; font-weight: bold;");
        connect(m_addCmdBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onAddCommandToTabClicked);
        catLayout->addWidget(m_addCmdBtn);

        columnsLayout->addWidget(catGroup, 4);
        mainLayout->addLayout(columnsLayout, 1);

        // --- Bottom Bar: Reset Defaults + Save / Cancel ---
        auto* bottomRow = new QHBoxLayout();
        auto* resetBtn = new QPushButton("↺ Reset Ribbon to Factory Defaults", this);
        connect(resetBtn, &QPushButton::clicked, this, &RibbonCustomizeDialog::onResetDefaultsClicked);
        bottomRow->addWidget(resetBtn);
        bottomRow->addStretch(1);

        auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &RibbonCustomizeDialog::onSaveClicked);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &RibbonCustomizeDialog::reject);
        bottomRow->addWidget(buttonBox);

        mainLayout->addLayout(bottomRow);
    }

    void RibbonCustomizeDialog::populateTabList(int selectRow) {
        m_updatingUi = true;
        m_tabListWidget->clear();

        for (const auto& tab : m_tabs) {
            const QString label = tab.isCustom ? (tab.name + "  [Custom]") : tab.name;
            auto* item = new QListWidgetItem(label, m_tabListWidget);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(tab.visible ? Qt::Checked : Qt::Unchecked);
        }
        m_updatingUi = false;

        if (!m_tabs.empty()) {
            const int clamped = std::clamp(selectRow, 0, static_cast<int>(m_tabs.size()) - 1);
            m_tabListWidget->setCurrentRow(clamped);
        }
        else {
            m_currentTabIndex = -1;
            m_tabCommandsListWidget->clear();
        }
    }

    void RibbonCustomizeDialog::populateTabCommandsList(int tabIndex) {
        m_updatingUi = true;
        m_tabCommandsListWidget->clear();

        if (tabIndex >= 0 && tabIndex < static_cast<int>(m_tabs.size())) {
            const auto& cmdIds = m_tabs[tabIndex].commandIds;
            for (const QString& cid : cmdIds) {
                const RibbonCommandMeta* meta = findCommandMeta(cid);
                if (!meta) continue;

                const QString text = QString("%1  %2   —   [%3]")
                    .arg(meta->iconSymbol)
                    .arg(meta->label)
                    .arg(meta->groupTitle);

                auto* item = new QListWidgetItem(text, m_tabCommandsListWidget);
                item->setData(Qt::UserRole, meta->id);
            }
        }
        m_updatingUi = false;
    }

    void RibbonCustomizeDialog::populateCatalogList(const QString& categoryFilter) {
        m_catalogListWidget->clear();
        for (const auto& meta : availableCommandsCatalog()) {
            if (!categoryFilter.isEmpty() && meta.groupTitle != categoryFilter) {
                continue;
            }
            const QString text = QString("%1  %2   [%3]")
                .arg(meta.iconSymbol, meta.label, meta.groupTitle);
            auto* item = new QListWidgetItem(text, m_catalogListWidget);
            item->setData(Qt::UserRole, meta.id);
        }
        if (m_catalogListWidget->count() > 0) {
            m_catalogListWidget->setCurrentRow(0);
        }
    }

    void RibbonCustomizeDialog::syncCurrentTabCommandsFromList() {
        if (m_updatingUi || m_currentTabIndex < 0 || m_currentTabIndex >= static_cast<int>(m_tabs.size())) {
            return;
        }
        QStringList updated;
        for (int i = 0; i < m_tabCommandsListWidget->count(); ++i) {
            QListWidgetItem* item = m_tabCommandsListWidget->item(i);
            if (item) {
                updated.append(item->data(Qt::UserRole).toString());
            }
        }
        m_tabs[m_currentTabIndex].commandIds = updated;
    }

    void RibbonCustomizeDialog::onSelectedTabChanged(int row) {
        m_currentTabIndex = row;
        populateTabCommandsList(row);

        const bool valid = (row >= 0 && row < static_cast<int>(m_tabs.size()));
        m_renameTabBtn->setEnabled(valid);
        m_deleteTabBtn->setEnabled(valid && m_tabs.size() > 1);
        m_moveTabUpBtn->setEnabled(valid && row > 0);
        m_moveTabDownBtn->setEnabled(valid && row + 1 < static_cast<int>(m_tabs.size()));
        m_addCmdBtn->setEnabled(valid);
    }

    void RibbonCustomizeDialog::onTabItemChanged(QListWidgetItem* item) {
        if (m_updatingUi || !item) return;
        const int row = m_tabListWidget->row(item);
        if (row >= 0 && row < static_cast<int>(m_tabs.size())) {
            m_tabs[row].visible = (item->checkState() == Qt::Checked);
        }
    }

    void RibbonCustomizeDialog::onAddTabClicked() {
        bool ok = false;
        QString name = QInputDialog::getText(
            this,
            "New Custom Ribbon Tab",
            "Enter a name for the new Ribbon Tab:",
            QLineEdit::Normal,
            "Shop Floor",
            &ok
        ).trimmed();

        if (!ok || name.isEmpty()) return;

        RibbonTabConfig newTab;
        newTab.name = name;
        newTab.visible = true;
        newTab.isCustom = true;
        newTab.commandIds = { "dro_zero", "cad_auto_probe", "prog_create_feature", "dmis_run", "rep_export_report" };

        m_tabs.push_back(newTab);
        populateTabList(static_cast<int>(m_tabs.size()) - 1);
    }

    void RibbonCustomizeDialog::onRenameTabClicked() {
        if (m_currentTabIndex < 0 || m_currentTabIndex >= static_cast<int>(m_tabs.size())) return;

        bool ok = false;
        QString newName = QInputDialog::getText(
            this,
            "Rename Ribbon Tab",
            "Enter new tab title:",
            QLineEdit::Normal,
            m_tabs[m_currentTabIndex].name,
            &ok
        ).trimmed();

        if (!ok || newName.isEmpty()) return;

        m_tabs[m_currentTabIndex].name = newName;
        populateTabList(m_currentTabIndex);
    }

    void RibbonCustomizeDialog::onDeleteTabClicked() {
        if (m_currentTabIndex < 0 || m_currentTabIndex >= static_cast<int>(m_tabs.size())) return;
        if (m_tabs.size() <= 1) {
            QMessageBox::warning(this, "Cannot Delete Tab", "At least one tab must remain on the Ribbon Bar.");
            return;
        }

        const QString tabName = m_tabs[m_currentTabIndex].name;
        const auto resp = QMessageBox::question(
            this,
            "Delete Ribbon Tab",
            QString("Remove the '%1' tab from the Ribbon Bar?").arg(tabName),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        if (resp != QMessageBox::Yes) return;

        m_tabs.erase(m_tabs.begin() + m_currentTabIndex);
        populateTabList(std::max(0, m_currentTabIndex - 1));
    }

    void RibbonCustomizeDialog::onMoveTabUpClicked() {
        if (m_currentTabIndex <= 0 || m_currentTabIndex >= static_cast<int>(m_tabs.size())) return;
        std::swap(m_tabs[m_currentTabIndex], m_tabs[m_currentTabIndex - 1]);
        populateTabList(m_currentTabIndex - 1);
    }

    void RibbonCustomizeDialog::onMoveTabDownClicked() {
        if (m_currentTabIndex < 0 || m_currentTabIndex + 1 >= static_cast<int>(m_tabs.size())) return;
        std::swap(m_tabs[m_currentTabIndex], m_tabs[m_currentTabIndex + 1]);
        populateTabList(m_currentTabIndex + 1);
    }

    void RibbonCustomizeDialog::onAddCommandToTabClicked() {
        if (m_currentTabIndex < 0 || m_currentTabIndex >= static_cast<int>(m_tabs.size())) return;
        QListWidgetItem* catItem = m_catalogListWidget->currentItem();
        if (!catItem) return;

        const QString cmdId = catItem->data(Qt::UserRole).toString();
        if (cmdId.isEmpty()) return;

        int insertPos = m_tabCommandsListWidget->currentRow();
        if (insertPos >= 0 && insertPos < m_tabs[m_currentTabIndex].commandIds.size()) {
            m_tabs[m_currentTabIndex].commandIds.insert(insertPos + 1, cmdId);
            populateTabCommandsList(m_currentTabIndex);
            m_tabCommandsListWidget->setCurrentRow(insertPos + 1);
        }
        else {
            m_tabs[m_currentTabIndex].commandIds.append(cmdId);
            populateTabCommandsList(m_currentTabIndex);
            m_tabCommandsListWidget->setCurrentRow(m_tabCommandsListWidget->count() - 1);
        }
    }

    void RibbonCustomizeDialog::onRemoveCommandFromTabClicked() {
        if (m_currentTabIndex < 0 || m_currentTabIndex >= static_cast<int>(m_tabs.size())) return;
        const int row = m_tabCommandsListWidget->currentRow();
        if (row < 0 || row >= m_tabs[m_currentTabIndex].commandIds.size()) return;

        m_tabs[m_currentTabIndex].commandIds.removeAt(row);
        populateTabCommandsList(m_currentTabIndex);
        if (m_tabCommandsListWidget->count() > 0) {
            m_tabCommandsListWidget->setCurrentRow(std::min(row, m_tabCommandsListWidget->count() - 1));
        }
    }

    void RibbonCustomizeDialog::onMoveCommandUpClicked() {
        if (m_currentTabIndex < 0 || m_currentTabIndex >= static_cast<int>(m_tabs.size())) return;
        const int row = m_tabCommandsListWidget->currentRow();
        if (row <= 0 || row >= m_tabs[m_currentTabIndex].commandIds.size()) return;

        m_tabs[m_currentTabIndex].commandIds.swapItemsAt(row, row - 1);
        populateTabCommandsList(m_currentTabIndex);
        m_tabCommandsListWidget->setCurrentRow(row - 1);
    }

    void RibbonCustomizeDialog::onMoveCommandDownClicked() {
        if (m_currentTabIndex < 0 || m_currentTabIndex >= static_cast<int>(m_tabs.size())) return;
        const int row = m_tabCommandsListWidget->currentRow();
        if (row < 0 || row + 1 >= m_tabs[m_currentTabIndex].commandIds.size()) return;

        m_tabs[m_currentTabIndex].commandIds.swapItemsAt(row, row + 1);
        populateTabCommandsList(m_currentTabIndex);
        m_tabCommandsListWidget->setCurrentRow(row + 1);
    }

    void RibbonCustomizeDialog::onCategoryFilterChanged(int index) {
        if (!m_categoryCombo) return;
        populateCatalogList(m_categoryCombo->itemData(index).toString());
    }

    void RibbonCustomizeDialog::onResetDefaultsClicked() {
        const auto resp = QMessageBox::question(
            this,
            "Reset Ribbon to Defaults",
            "Restore all default Ribbon tabs and command ordering?",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        if (resp != QMessageBox::Yes) return;

        m_tabs = RibbonCustomizeDialog::defaultRibbonLayout();
        populateTabList(0);
    }

    void RibbonCustomizeDialog::onSaveClicked() {
        syncCurrentTabCommandsFromList();

        bool hasVisible = false;
        for (const auto& t : m_tabs) {
            if (t.visible) {
                hasVisible = true;
                break;
            }
        }
        if (!hasVisible && !m_tabs.empty()) {
            m_tabs.front().visible = true;
        }

        saveLayout(m_tabs);
        accept();
    }

} // namespace OpenCMM