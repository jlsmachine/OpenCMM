#include "cad/CmmDigitalTwin.h"

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTemporaryDir>
#include <QProcess>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QRegularExpression>

#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>
#include <Quantity_Color.hxx>
#include <algorithm>

namespace OpenCMM {

    namespace {

        std::vector<std::string> extractStepSolidNames(const QString& stepFilePath) {
            std::vector<std::string> names;
            QFile file(stepFilePath);
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                return names;
            }

            const QString content = QString::fromUtf8(file.readAll());
            static const QRegularExpression re(R"(MANIFOLD_SOLID_BREP\s*\(\s*'([^']*)')");
            QRegularExpressionMatchIterator it = re.globalMatch(content);
            while (it.hasNext()) {
                QRegularExpressionMatch match = it.next();
                names.push_back(match.captured(1).toStdString());
            }
            return names;
        }

        bool pickAutomaticSolidColor(const std::string& solidName, Eigen::Vector3d& outRgb) {
            const QString s = QString::fromStdString(solidName).toLower();

            if (s.contains("ruby")) {
                outRgb = Eigen::Vector3d(0.92, 0.12, 0.18); // Ruby Red tip
                return true;
            }
            if (s.contains("blue") || s.contains("badge") || s.contains("fascia") || s.contains("stripe")) {
                outRgb = Eigen::Vector3d(0.14, 0.42, 0.72); // Helmel Blue trim
                return true;
            }
            if (s.contains("renishaw") || s.contains("tp_es") || s.contains("knob") ||
                s.contains("mount") || s.contains("cap") || s.contains("slot") || s.contains("scale_housing")) {
                outRgb = Eigen::Vector3d(0.16, 0.18, 0.21); // Dark Anodized / Matte Black hardware
                return true;
            }
            if (s.contains("insert")) {
                outRgb = Eigen::Vector3d(0.76, 0.62, 0.28); // Brass threaded inserts
                return true;
            }
            if (s.contains("way") || s.contains("stem") || s.contains("shank") || s.contains("ram_column")) {
                outRgb = Eigen::Vector3d(0.88, 0.90, 0.92); // Precision ground steel
                return true;
            }
            if (s.contains("granite")) {
                outRgb = Eigen::Vector3d(0.20, 0.22, 0.24); // Charcoal granite
                return true;
            }
            return false;
        }

    } // namespace

    void CmmDigitalTwin::clearScene() {
        if (!m_context.IsNull()) {
            for (auto& node : m_nodes) {
                for (auto& ais : node.aisShapes) {
                    if (!ais.IsNull()) {
                        m_context->Remove(ais, Standard_False);
                    }
                }
            }
        }
        m_nodes.clear();
    }

    void CmmDigitalTwin::buildDefaultBridgeMachine() {
        m_config = MachinePostConfig{};
        m_config.machineName = "Default Moving-Bridge CMM";
        m_config.manufacturer = "OpenCMM";
        m_config.probeMountOffsetMm = Eigen::Vector3d(0.0, 0.0, 0.0);

        KinematicLinkDef base;
        base.id = "granite_base";
        base.parentId = "";
        base.axis = "NONE";
        base.colorRgb = Eigen::Vector3d(0.22, 0.24, 0.26);
        base.primitives.push_back({ "box", Eigen::Vector3d(-300.0, -200.0, -80.0), Eigen::Vector3d(600.0, 400.0, 80.0) });
        m_config.links.push_back(base);

        KinematicLinkDef bridge;
        bridge.id = "bridge_y";
        bridge.parentId = "granite_base";
        bridge.axis = "Y";
        bridge.colorRgb = Eigen::Vector3d(0.75, 0.77, 0.80);
        bridge.primitives.push_back({ "box", Eigen::Vector3d(-290.0, -25.0, 0.0), Eigen::Vector3d(35.0, 50.0, 320.0) });
        bridge.primitives.push_back({ "box", Eigen::Vector3d(255.0, -25.0, 0.0), Eigen::Vector3d(35.0, 50.0, 320.0) });
        bridge.primitives.push_back({ "box", Eigen::Vector3d(-290.0, -30.0, 320.0), Eigen::Vector3d(580.0, 60.0, 55.0) });
        m_config.links.push_back(bridge);

        KinematicLinkDef carriage;
        carriage.id = "carriage_x";
        carriage.parentId = "bridge_y";
        carriage.axis = "X";
        carriage.colorRgb = Eigen::Vector3d(0.28, 0.32, 0.38);
        carriage.primitives.push_back({ "box", Eigen::Vector3d(-35.0, -40.0, 310.0), Eigen::Vector3d(70.0, 80.0, 75.0) });
        m_config.links.push_back(carriage);

        KinematicLinkDef quill;
        quill.id = "quill_z";
        quill.parentId = "carriage_x";
        quill.axis = "Z";
        quill.colorRgb = Eigen::Vector3d(0.82, 0.84, 0.86);
        quill.primitives.push_back({ "box", Eigen::Vector3d(-18.0, -18.0, 50.0), Eigen::Vector3d(36.0, 36.0, 360.0) });
        m_config.links.push_back(quill);

        rebuildAisNodes("");
        updateMachinePosition(m_lastPosMm.x(), m_lastPosMm.y(), m_lastPosMm.z());
    }

    bool CmmDigitalTwin::loadMachinePackage(const QString& packagePath) {
        QFileInfo fi(packagePath);
        if (!fi.exists()) {
            return false;
        }

        if (fi.suffix().compare("json", Qt::CaseInsensitive) == 0) {
            QFile f(packagePath);
            if (!f.open(QIODevice::ReadOnly)) return false;
            return parseManifestJson(f.readAll(), fi.absolutePath());
        }

        QTemporaryDir tempDir;
        if (!tempDir.isValid()) {
            return false;
        }

        QProcess tarProc;
        tarProc.start("tar", { "-xf", QDir::toNativeSeparators(packagePath), "-C", QDir::toNativeSeparators(tempDir.path()) });
        if (!tarProc.waitForFinished(10000) || tarProc.exitCode() != 0) {
            return false;
        }

        QDir dir(tempDir.path());
        QString jsonPath = dir.filePath("machine.cmm.json");
        if (!QFileInfo::exists(jsonPath)) {
            QStringList jsonFiles = dir.entryList({ "*.json" }, QDir::Files);
            if (jsonFiles.isEmpty()) return false;
            jsonPath = dir.filePath(jsonFiles.first());
        }

        QFile f(jsonPath);
        if (!f.open(QIODevice::ReadOnly)) {
            return false;
        }

        return parseManifestJson(f.readAll(), tempDir.path());
    }

    bool CmmDigitalTwin::parseManifestJson(const QByteArray& jsonBytes, const QString& baseDir) {
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(jsonBytes, &err);
        if (err.error != QJsonParseError::NoError || !doc.isObject()) {
            return false;
        }

        QJsonObject root = doc.object();
        MachinePostConfig cfg;
        cfg.machineName = root.value("machine_name").toString("Custom CMM").toStdString();
        cfg.manufacturer = root.value("manufacturer").toString("Unknown").toStdString();
        cfg.controllerType = root.value("controller_type").toString("PicoSerial").toStdString();

        if (root.contains("axes") && root["axes"].isObject()) {
            QJsonObject axesObj = root["axes"].toObject();
            auto readVec3d = [](const QJsonArray& arr, const Eigen::Vector3d& defVal) {
                if (arr.size() < 3) return defVal;
                return Eigen::Vector3d(arr[0].toDouble(), arr[1].toDouble(), arr[2].toDouble());
                };
            cfg.scaleMm = readVec3d(axesObj["scale_mm"].toArray(), cfg.scaleMm);
            cfg.travelMinMm = readVec3d(axesObj["travel_min_mm"].toArray(), cfg.travelMinMm);
            cfg.travelMaxMm = readVec3d(axesObj["travel_max_mm"].toArray(), cfg.travelMaxMm);

            QJsonArray signArr = axesObj["sign"].toArray();
            if (signArr.size() >= 3) {
                cfg.sign = Eigen::Vector3i(signArr[0].toInt(1), signArr[1].toInt(1), signArr[2].toInt(1));
            }
        }

        if (root.contains("probe_mount_offset_mm") && root["probe_mount_offset_mm"].isArray()) {
            QJsonArray arr = root["probe_mount_offset_mm"].toArray();
            if (arr.size() >= 3) {
                cfg.probeMountOffsetMm = Eigen::Vector3d(arr[0].toDouble(), arr[1].toDouble(), arr[2].toDouble());
            }
        }

        QJsonArray chainArr = root["kinematic_chain"].toArray();
        for (const QJsonValue& val : chainArr) {
            if (!val.isObject()) continue;
            QJsonObject obj = val.toObject();

            KinematicLinkDef link;
            link.id = obj.value("id").toString().toStdString();
            link.parentId = obj.value("parent").toString().toStdString();
            link.axis = obj.value("axis").toString("NONE").toUpper().toStdString();
            link.stepFile = obj.value("step_file").toString().toStdString();

            QJsonArray colArr = obj["color_rgb"].toArray();
            if (colArr.size() >= 3) {
                link.colorRgb = Eigen::Vector3d(colArr[0].toDouble(), colArr[1].toDouble(), colArr[2].toDouble());
            }

            if (obj.contains("part_colors") && obj["part_colors"].isObject()) {
                QJsonObject pcObj = obj["part_colors"].toObject();
                for (auto it = pcObj.begin(); it != pcObj.end(); ++it) {
                    if (it.value().isArray()) {
                        QJsonArray cArr = it.value().toArray();
                        if (cArr.size() >= 3) {
                            link.partColors[it.key().toStdString()] = Eigen::Vector3d(
                                cArr[0].toDouble(), cArr[1].toDouble(), cArr[2].toDouble()
                            );
                        }
                    }
                }
            }

            QJsonArray primsArr = obj["primitives"].toArray();
            for (const QJsonValue& pVal : primsArr) {
                if (!pVal.isObject()) continue;
                QJsonObject pObj = pVal.toObject();
                KinematicPrimitive prim;
                prim.type = pObj.value("type").toString("box").toLower().toStdString();
                QJsonArray oArr = pObj["origin"].toArray();
                QJsonArray sArr = pObj["size"].toArray();
                if (oArr.size() >= 3) prim.origin = Eigen::Vector3d(oArr[0].toDouble(), oArr[1].toDouble(), oArr[2].toDouble());
                if (sArr.size() >= 3) prim.size = Eigen::Vector3d(sArr[0].toDouble(), sArr[1].toDouble(), sArr[2].toDouble());
                link.primitives.push_back(prim);
            }

            cfg.links.push_back(link);
        }

        if (cfg.links.empty()) {
            return false;
        }

        m_config = cfg;
        rebuildAisNodes(baseDir);
        updateMachinePosition(m_lastPosMm.x(), m_lastPosMm.y(), m_lastPosMm.z());
        return true;
    }

    TopoDS_Shape CmmDigitalTwin::buildLinkShape(
        const KinematicLinkDef& def,
        const QString& baseDir,
        std::vector<std::string>& outSolidNames) {

        outSolidNames.clear();

        if (!def.stepFile.empty() && !baseDir.isEmpty()) {
            QString fullStepPath = QDir(baseDir).filePath(QString::fromStdString(def.stepFile));
            if (QFileInfo::exists(fullStepPath)) {
                outSolidNames = extractStepSolidNames(fullStepPath);

                STEPControl_Reader reader;
                if (reader.ReadFile(fullStepPath.toUtf8().constData()) == IFSelect_RetDone) {
                    reader.TransferRoots();
                    TopoDS_Shape stepShape = reader.OneShape();
                    if (!stepShape.IsNull()) {
                        return stepShape;
                    }
                }
            }
        }

        BRep_Builder builder;
        TopoDS_Compound comp;
        builder.MakeCompound(comp);
        bool hasShapes = false;

        for (const auto& prim : def.primitives) {
            if (prim.type == "box" && prim.size.x() > 0 && prim.size.y() > 0 && prim.size.z() > 0) {
                TopoDS_Shape box = BRepPrimAPI_MakeBox(
                    gp_Pnt(prim.origin.x(), prim.origin.y(), prim.origin.z()),
                    prim.size.x(), prim.size.y(), prim.size.z()
                ).Shape();
                builder.Add(comp, box);
                hasShapes = true;
            }
            else if (prim.type == "cylinder" && prim.size.x() > 0 && prim.size.y() > 0) {
                gp_Ax2 ax(gp_Pnt(prim.origin.x(), prim.origin.y(), prim.origin.z()), gp_Dir(0, 0, 1));
                TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(ax, prim.size.x(), prim.size.y()).Shape();
                builder.Add(comp, cyl);
                hasShapes = true;
            }
        }

        return hasShapes ? comp : TopoDS_Shape();
    }

    void CmmDigitalTwin::rebuildAisNodes(const QString& baseDir) {
        clearScene();

        for (size_t i = 0; i < m_config.links.size(); ++i) {
            RuntimeNode node;
            node.def = m_config.links[i];
            node.visible = true;

            if (!node.def.parentId.empty()) {
                for (size_t j = 0; j < i; ++j) {
                    if (m_config.links[j].id == node.def.parentId) {
                        node.parentIndex = static_cast<int>(j);
                        break;
                    }
                }
            }

            std::vector<std::string> solidNames;
            TopoDS_Shape shape = buildLinkShape(node.def, baseDir, solidNames);
            if (!shape.IsNull() && !m_context.IsNull()) {
                size_t solidIdx = 0;
                for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next(), ++solidIdx) {
                    TopoDS_Shape solid = exp.Current();
                    Handle(AIS_Shape) aisSolid = new AIS_Shape(solid);

                    Eigen::Vector3d rgb = node.def.colorRgb;
                    bool hasCustom = false;
                    std::string sName = (solidIdx < solidNames.size()) ? solidNames[solidIdx] : "";

                    for (const auto& kv : node.def.partColors) {
                        if (sName == kv.first || sName.find(kv.first) != std::string::npos) {
                            rgb = kv.second;
                            hasCustom = true;
                            break;
                        }
                    }
                    if (!hasCustom && !sName.empty()) {
                        hasCustom = pickAutomaticSolidColor(sName, rgb);
                    }

                    Quantity_Color col(
                        std::clamp(rgb.x(), 0.0, 1.0),
                        std::clamp(rgb.y(), 0.0, 1.0),
                        std::clamp(rgb.z(), 0.0, 1.0),
                        Quantity_TOC_RGB
                    );
                    aisSolid->SetColor(col);
                    aisSolid->Attributes()->SetFaceBoundaryDraw(Standard_True);

                    if (m_visible && node.visible) {
                        m_context->Display(aisSolid, AIS_Shaded, -1, Standard_False);
                    }
                    node.aisShapes.push_back(aisSolid);
                }

                // Fallback if shape has no explicit solids (e.g. primitives or generic compound)
                if (node.aisShapes.empty()) {
                    Handle(AIS_Shape) aisAll = new AIS_Shape(shape);
                    Quantity_Color col(
                        std::clamp(node.def.colorRgb.x(), 0.0, 1.0),
                        std::clamp(node.def.colorRgb.y(), 0.0, 1.0),
                        std::clamp(node.def.colorRgb.z(), 0.0, 1.0),
                        Quantity_TOC_RGB
                    );
                    aisAll->SetColor(col);
                    aisAll->Attributes()->SetFaceBoundaryDraw(Standard_True);
                    if (m_visible && node.visible) {
                        m_context->Display(aisAll, AIS_Shaded, -1, Standard_False);
                    }
                    node.aisShapes.push_back(aisAll);
                }
            }

            m_nodes.push_back(node);
        }

        if (!m_context.IsNull()) {
            m_context->UpdateCurrentViewer();
        }
    }

    void CmmDigitalTwin::updateMachinePosition(double xMm, double yMm, double zMm) {
        m_lastPosMm = Eigen::Vector3d(xMm, yMm, zMm);
        if (m_context.IsNull() || m_nodes.empty()) {
            return;
        }

        const Eigen::Vector3d& mountOff = m_config.probeMountOffsetMm;
        std::vector<Eigen::Vector3d> worldTranslations(m_nodes.size(), Eigen::Vector3d::Zero());

        for (size_t i = 0; i < m_nodes.size(); ++i) {
            Eigen::Vector3d localDelta = Eigen::Vector3d::Zero();
            const std::string& ax = m_nodes[i].def.axis;
            if (ax == "X") {
                localDelta.x() = xMm - mountOff.x();
            }
            else if (ax == "Y") {
                localDelta.y() = yMm - mountOff.y();
            }
            else if (ax == "Z") {
                localDelta.z() = zMm - mountOff.z();
            }

            int pIdx = m_nodes[i].parentIndex;
            if (pIdx >= 0 && pIdx < static_cast<int>(i)) {
                worldTranslations[i] = worldTranslations[pIdx] + localDelta;
            }
            else {
                worldTranslations[i] = localDelta;
            }

            for (auto& ais : m_nodes[i].aisShapes) {
                if (!ais.IsNull()) {
                    gp_Trsf trsf;
                    trsf.SetTranslation(gp_Vec(
                        worldTranslations[i].x(),
                        worldTranslations[i].y(),
                        worldTranslations[i].z()
                    ));
                    m_context->SetLocation(ais, TopLoc_Location(trsf));
                }
            }
        }

        m_context->UpdateCurrentViewer();
    }

    void CmmDigitalTwin::setMachineVisible(bool visible) {
        m_visible = visible;
        if (m_context.IsNull()) return;

        for (auto& node : m_nodes) {
            for (auto& ais : node.aisShapes) {
                if (ais.IsNull()) continue;
                if (m_visible && node.visible) {
                    m_context->Display(ais, AIS_Shaded, -1, Standard_False);
                }
                else {
                    m_context->Erase(ais, Standard_False);
                }
            }
        }
        m_context->UpdateCurrentViewer();
    }

    QStringList CmmDigitalTwin::getLinkIds() const {
        QStringList ids;
        for (const auto& node : m_nodes) {
            ids.append(QString::fromStdString(node.def.id));
        }
        return ids;
    }

    bool CmmDigitalTwin::isLinkVisible(const QString& linkId) const {
        const std::string target = linkId.toStdString();
        for (const auto& node : m_nodes) {
            if (node.def.id == target) {
                return node.visible;
            }
        }
        return false;
    }

    void CmmDigitalTwin::setLinkVisible(const QString& linkId, bool visible) {
        const std::string target = linkId.toStdString();
        for (auto& node : m_nodes) {
            if (node.def.id == target) {
                node.visible = visible;
                if (!m_context.IsNull()) {
                    for (auto& ais : node.aisShapes) {
                        if (ais.IsNull()) continue;
                        if (m_visible && node.visible) {
                            m_context->Display(ais, AIS_Shaded, -1, Standard_True);
                        }
                        else {
                            m_context->Erase(ais, Standard_True);
                        }
                    }
                }
                break;
            }
        }
    }

} // namespace OpenCMM