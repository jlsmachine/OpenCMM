#include "cad/CmmDigitalTwin.h"

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTemporaryDir>
#include <QProcess>
#include <nlohmann/json.hpp>

#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopLoc_Location.hxx>
#include <Quantity_Color.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>

namespace OpenCMM {

    void CmmDigitalTwin::clearLinks() {
        if (!m_ctx.IsNull()) {
            for (auto& link : m_links) {
                if (!link.aisShape.IsNull()) {
                    m_ctx->Remove(link.aisShape, Standard_False);
                }
            }
        }
        m_links.clear();
    }

    void CmmDigitalTwin::resolveParentIndices() {
        for (size_t i = 0; i < m_links.size(); ++i) {
            m_links[i].parentIndex = -1;
            if (m_links[i].parentId.empty()) continue;
            for (size_t j = 0; j < m_links.size(); ++j) {
                if (m_links[j].id == m_links[i].parentId) {
                    m_links[i].parentIndex = static_cast<int>(j);
                    break;
                }
            }
        }
    }

    bool CmmDigitalTwin::loadMachinePackage(const QString& packagePath) {
        if (m_ctx.IsNull() || packagePath.isEmpty()) return false;

        QTemporaryDir tempDir;
        QString workingDir;
        QString jsonPath = packagePath;

        // Extract .cmm zip archive into tempDir using Windows system tar
        if (packagePath.endsWith(".cmm", Qt::CaseInsensitive) ||
            packagePath.endsWith(".zip", Qt::CaseInsensitive)) {
            if (!tempDir.isValid()) return false;

            QProcess proc;
            proc.start("tar", { "-xf", QDir::toNativeSeparators(packagePath), "-C", QDir::toNativeSeparators(tempDir.path()) });
            if (!proc.waitForFinished(10000) || proc.exitCode() != 0) {
                return false;
            }

            workingDir = tempDir.path();
            QDir dir(workingDir);
            QStringList foundJson = dir.entryList({ "*.cmm.json", "*.json" }, QDir::Files);
            if (foundJson.isEmpty()) {
                return false;
            }
            jsonPath = dir.filePath(foundJson.first());
        }
        else {
            workingDir = QFileInfo(packagePath).absolutePath();
        }

        QFile file(jsonPath);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            return false;
        }
        QByteArray rawData = file.readAll();
        file.close();

        return parseJsonConfig(rawData, workingDir);
    }

    bool CmmDigitalTwin::parseJsonConfig(const QByteArray& jsonData, const QString& workingDir) {
        nlohmann::json j;
        try {
            j = nlohmann::json::parse(jsonData.constBegin(), jsonData.constEnd());
        }
        catch (...) {
            return false;
        }

        clearLinks();

        m_config.machineName = QString::fromStdString(j.value("machine_name", "Custom CMM"));
        m_config.manufacturer = QString::fromStdString(j.value("manufacturer", "OpenCMM"));

        if (j.contains("axes") && j["axes"].is_object()) {
            const auto& ax = j["axes"];
            if (ax.contains("scale_mm") && ax["scale_mm"].is_array() && ax["scale_mm"].size() == 3) {
                m_config.encoderScaleMm = Eigen::Vector3d(ax["scale_mm"][0], ax["scale_mm"][1], ax["scale_mm"][2]);
            }
            if (ax.contains("sign") && ax["sign"].is_array() && ax["sign"].size() == 3) {
                m_config.axisSign = Eigen::Vector3i(ax["sign"][0], ax["sign"][1], ax["sign"][2]);
            }
            if (ax.contains("travel_min_mm") && ax["travel_min_mm"].is_array() && ax["travel_min_mm"].size() == 3) {
                m_config.travelMinMm = Eigen::Vector3d(ax["travel_min_mm"][0], ax["travel_min_mm"][1], ax["travel_min_mm"][2]);
            }
            if (ax.contains("travel_max_mm") && ax["travel_max_mm"].is_array() && ax["travel_max_mm"].size() == 3) {
                m_config.travelMaxMm = Eigen::Vector3d(ax["travel_max_mm"][0], ax["travel_max_mm"][1], ax["travel_max_mm"][2]);
            }
        }

        QDir baseDir(workingDir);

        if (j.contains("kinematic_chain") && j["kinematic_chain"].is_array()) {
            for (const auto& node : j["kinematic_chain"]) {
                KinematicLink link;
                link.id = node.value("id", "link");
                link.parentId = node.value("parent", "");

                std::string axisStr = node.value("axis", "NONE");
                if (axisStr == "X") link.axis = KinematicAxis::AxisX;
                else if (axisStr == "Y") link.axis = KinematicAxis::AxisY;
                else if (axisStr == "Z") link.axis = KinematicAxis::AxisZ;
                else link.axis = KinematicAxis::None;

                TopoDS_Shape linkShape;
                bool loadedStep = false;

                std::string stepRelPath = node.value("step_file", "");
                if (!stepRelPath.empty()) {
                    QString fullStepPath = baseDir.filePath(QString::fromStdString(stepRelPath));
                    if (QFileInfo::exists(fullStepPath)) {
                        STEPControl_Reader reader;
                        if (reader.ReadFile(fullStepPath.toUtf8().constData()) == IFSelect_RetDone) {
                            reader.TransferRoots();
                            linkShape = reader.OneShape();
                            loadedStep = !linkShape.IsNull();
                        }
                    }
                }

                if (!loadedStep && node.contains("primitives") && node["primitives"].is_array()) {
                    BRep_Builder builder;
                    TopoDS_Compound comp;
                    builder.MakeCompound(comp);

                    for (const auto& prim : node["primitives"]) {
                        std::string pType = prim.value("type", "box");
                        auto orig = prim.value("origin", std::vector<double>{ 0.0, 0.0, 0.0 });
                        if (orig.size() < 3) continue;

                        if (pType == "box") {
                            auto sz = prim.value("size", std::vector<double>{ 10.0, 10.0, 10.0 });
                            if (sz.size() == 3) {
                                TopoDS_Shape box = BRepPrimAPI_MakeBox(
                                    gp_Pnt(orig[0], orig[1], orig[2]), sz[0], sz[1], sz[2]
                                ).Shape();
                                builder.Add(comp, box);
                            }
                        }
                        else if (pType == "cylinder") {
                            double rad = prim.value("radius", 5.0);
                            double h = prim.value("height", 20.0);
                            gp_Ax2 cylAx(gp_Pnt(orig[0], orig[1], orig[2]), gp_Dir(0.0, 0.0, 1.0));
                            TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(cylAx, rad, h).Shape();
                            builder.Add(comp, cyl);
                        }
                    }
                    linkShape = comp;
                }

                if (!linkShape.IsNull()) {
                    link.aisShape = new AIS_Shape(linkShape);
                    auto rgb = node.value("color_rgb", std::vector<double>{ 0.65, 0.68, 0.72 });
                    if (rgb.size() == 3) {
                        link.aisShape->SetColor(Quantity_Color(rgb[0], rgb[1], rgb[2], Quantity_TOC_RGB));
                    }
                    link.aisShape->Attributes()->SetFaceBoundaryDraw(Standard_True);
                    if (m_visible) {
                        m_ctx->Display(link.aisShape, AIS_Shaded, -1, Standard_False);
                    }
                    m_links.push_back(link);
                }
            }
        }

        resolveParentIndices();
        updateMachinePosition(m_lastX, m_lastY, m_lastZ);
        return !m_links.empty();
    }

    void CmmDigitalTwin::buildDefaultBridgeMachine() {
        if (m_ctx.IsNull()) return;
        clearLinks();

        m_config = MachinePostConfig{};
        BRep_Builder builder;

        // 1. Moving Y-Bridge (Left/Right columns + top X-beam)
        {
            KinematicLink bridge;
            bridge.id = "bridge_y";
            bridge.parentId = "";
            bridge.axis = KinematicAxis::AxisY;

            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            builder.Add(comp, BRepPrimAPI_MakeBox(gp_Pnt(-300.0, -30.0, 0.0), 35.0, 60.0, 260.0).Shape());
            builder.Add(comp, BRepPrimAPI_MakeBox(gp_Pnt(265.0, -30.0, 0.0), 35.0, 60.0, 260.0).Shape());
            builder.Add(comp, BRepPrimAPI_MakeBox(gp_Pnt(-300.0, -35.0, 260.0), 600.0, 70.0, 50.0).Shape());

            bridge.aisShape = new AIS_Shape(comp);
            bridge.aisShape->SetColor(Quantity_Color(0.72, 0.75, 0.78, Quantity_TOC_RGB));
            bridge.aisShape->Attributes()->SetFaceBoundaryDraw(Standard_True);
            if (m_visible) {
                m_ctx->Display(bridge.aisShape, AIS_Shaded, -1, Standard_False);
            }
            m_links.push_back(bridge);
        }

        // 2. X-Carriage riding along the bridge crossbeam
        {
            KinematicLink carriage;
            carriage.id = "carriage_x";
            carriage.parentId = "bridge_y";
            carriage.axis = KinematicAxis::AxisX;

            TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(-35.0, -42.0, 250.0), 70.0, 84.0, 75.0).Shape();
            carriage.aisShape = new AIS_Shape(box);
            carriage.aisShape->SetColor(Quantity_Color(0.25, 0.30, 0.36, Quantity_TOC_RGB));
            carriage.aisShape->Attributes()->SetFaceBoundaryDraw(Standard_True);
            if (m_visible) {
                m_ctx->Display(carriage.aisShape, AIS_Shaded, -1, Standard_False);
            }
            m_links.push_back(carriage);
        }

        // 3. Z-Quill & Probe Head (bottom meets stylus stem at Z = 50.0)
        {
            KinematicLink quill;
            quill.id = "quill_z";
            quill.parentId = "carriage_x";
            quill.axis = KinematicAxis::AxisZ;

            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            builder.Add(comp, BRepPrimAPI_MakeBox(gp_Pnt(-18.0, -18.0, 62.0), 36.0, 36.0, 310.0).Shape());
            gp_Ax2 headAx(gp_Pnt(0.0, 0.0, 50.0), gp_Dir(0.0, 0.0, 1.0));
            builder.Add(comp, BRepPrimAPI_MakeCylinder(headAx, 9.0, 12.0).Shape());

            quill.aisShape = new AIS_Shape(comp);
            quill.aisShape->SetColor(Quantity_Color(0.85, 0.87, 0.90, Quantity_TOC_RGB));
            quill.aisShape->Attributes()->SetFaceBoundaryDraw(Standard_True);
            if (m_visible) {
                m_ctx->Display(quill.aisShape, AIS_Shaded, -1, Standard_False);
            }
            m_links.push_back(quill);
        }

        resolveParentIndices();
        updateMachinePosition(m_lastX, m_lastY, m_lastZ);
    }

    void CmmDigitalTwin::updateMachinePosition(double xMm, double yMm, double zMm) {
        m_lastX = xMm;
        m_lastY = yMm;
        m_lastZ = zMm;

        if (m_ctx.IsNull() || m_links.empty()) return;

        for (size_t i = 0; i < m_links.size(); ++i) {
            gp_Trsf localTrsf;
            if (m_links[i].axis == KinematicAxis::AxisX) {
                localTrsf.SetTranslation(gp_Vec(xMm, 0.0, 0.0));
            }
            else if (m_links[i].axis == KinematicAxis::AxisY) {
                localTrsf.SetTranslation(gp_Vec(0.0, yMm, 0.0));
            }
            else if (m_links[i].axis == KinematicAxis::AxisZ) {
                localTrsf.SetTranslation(gp_Vec(0.0, 0.0, zMm));
            }

            if (m_links[i].parentIndex >= 0 && m_links[i].parentIndex < static_cast<int>(i)) {
                m_links[i].worldTrsf = m_links[m_links[i].parentIndex].worldTrsf * localTrsf;
            }
            else {
                m_links[i].worldTrsf = localTrsf;
            }

            if (!m_links[i].aisShape.IsNull()) {
                m_ctx->SetLocation(m_links[i].aisShape, TopLoc_Location(m_links[i].worldTrsf));
            }
        }

        m_ctx->UpdateCurrentViewer();
    }

    void CmmDigitalTwin::setMachineVisible(bool visible) {
        m_visible = visible;
        if (m_ctx.IsNull()) return;

        for (auto& link : m_links) {
            if (link.aisShape.IsNull()) continue;
            if (visible) {
                m_ctx->Display(link.aisShape, AIS_Shaded, -1, Standard_False);
            }
            else {
                m_ctx->Erase(link.aisShape, Standard_False);
            }
        }
        m_ctx->UpdateCurrentViewer();
    }

} // namespace OpenCMM