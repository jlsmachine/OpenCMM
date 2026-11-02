# OpenCMM Machine Package Specification (`.cmm`)

OpenCMM uses a modular, CAM-style machine definition architecture. A `.cmm` package is a standard **ZIP archive** containing a JSON manifest (`.cmm.json`), optional STEP 3D link meshes, and configuration metadata. This allows you to swap physical CMM configurations, axis travels, and kinematic hierarchies without modifying any C++ code.

---

## 1. Directory Structure Inside a `.cmm` Archive

When you create a `.cmm` package (which is just a `.zip` file renamed to `.cmm`), its internal layout should follow this structure:

```text
MyMachine.cmm (ZIP Archive)
│
├── machine.cmm.json       <-- Main manifest (required)
└── meshes/                <-- Folder for 3D link geometry (optional)
    ├── granite_base.step
    ├── bridge_y.step
    ├── carriage_x.step
    └── quill_z.step