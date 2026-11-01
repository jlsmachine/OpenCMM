#pragma once
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <QString>

namespace OpenCMM {

class CmmDigitalTwin {
public:
    void setContext(const Handle(AIS_InteractiveContext)& ctx) { m_ctx = ctx; }

    // Load individual STEP components for the Helmel Checkmaster
    bool loadMachineModels(const QString& baseStepPath,
                           const QString& bridgeYStepPath,
                           const QString& carriageXStepPath,
                           const QString& quillZStepPath,
                           const QString& probeStepPath);

    // Called at 30Hz by PicoSerialController::droUpdated(xMm, yMm, zMm)
    void updateMachinePosition(double xMm, double yMm, double zMm) {
        if (m_ctx.IsNull()) return;

        if (!m_bridgeY.IsNull()) {
            gp_Trsf trsfY;
            trsfY.SetTranslation(gp_Vec(0.0, yMm, 0.0));
            m_ctx->SetLocation(m_bridgeY, TopLoc_Location(trsfY));
        }

        if (!m_carriageX.IsNull()) {
            gp_Trsf trsfXY;
            trsfXY.SetTranslation(gp_Vec(xMm, yMm, 0.0));
            m_ctx->SetLocation(m_carriageX, TopLoc_Location(trsfXY));
        }

        if (!m_quillZ.IsNull()) {
            gp_Trsf trsfXYZ;
            trsfXYZ.SetTranslation(gp_Vec(xMm, yMm, zMm));
            m_ctx->SetLocation(m_quillZ, TopLoc_Location(trsfXYZ));
        }

        if (!m_probe.IsNull()) {
            gp_Trsf trsfXYZ;
            trsfXYZ.SetTranslation(gp_Vec(xMm, yMm, zMm));
            m_ctx->SetLocation(m_probe, TopLoc_Location(trsfXYZ));
        }

        m_ctx->UpdateCurrentViewer();
    }

private:
    Handle(AIS_InteractiveContext) m_ctx;
    Handle(AIS_Shape) m_base;
    Handle(AIS_Shape) m_bridgeY;
    Handle(AIS_Shape) m_carriageX;
    Handle(AIS_Shape) m_quillZ;
    Handle(AIS_Shape) m_probe;
};

} // namespace OpenCMM