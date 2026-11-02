# OpenCMM Machine Package Specification (.cmm)

OpenCMM uses a modular, CAM-style machine definition architecture. A `.cmm` package is a standard **ZIP archive** containing a JSON manifest (`.cmm.json`), optional STEP/STL 3D link meshes, and configuration metadata. This allows you to swap physical CMM configurations, axis travels, and kinematic hierarchies without modifying any C++ code.

---

## 1. Directory Structure Inside a `.cmm` Archive

When you create a `.cmm` package (which is just a `.zip` file renamed to `.cmm`), its internal layout should follow this structure:

```text
MyMachine.cmm (ZIP Archive)
│
├── machine.cmm.json       <-- Main manifest (required)
└── meshes/                <-- Folder for 3D link geometry (optional)
    ├── bridge_y.step
    ├── carriage_x.step
    └── quill_z.step
```

---

## 2. Manifest Schema (`machine.cmm.json`)

The manifest defines the machine's display name, manufacturer, controller type, encoder scaling factors, axis travel limits, and the hierarchical kinematic chain.

### Full Example Manifest (`machine.cmm.json`)
```json
{
  "machine_name": "Helmel Checkmaster 216-142",
  "manufacturer": "Helmel Engineering",
  "controller_type": "PicoSerial",
  "axes": {
    "scale_mm": [0.0005, 0.0005, 0.0005],
    "sign": [1, 1, 1],
    "travel_min_mm": [0.0, 0.0, -350.0],
    "travel_max_mm": [400.0, 600.0, 0.0]
  },
  "probe_mount_offset_mm": [0.0, 0.0, 0.0],
  "kinematic_chain": [
    {
      "id": "granite_base",
      "parent": "",
      "axis": "NONE",
      "color_rgb": [0.22, 0.24, 0.26],
      "step_file": "",
      "primitives": [
        { "type": "box", "origin": [-100, -100, -120], "size": [600, 800, 120] }
      ]
    },
    {
      "id": "bridge_y",
      "parent": "granite_base",
      "axis": "Y",
      "color_rgb": [0.75, 0.77, 0.80],
      "step_file": "meshes/bridge_y.step",
      "primitives": []
    },
    {
      "id": "carriage_x",
      "parent": "bridge_y",
      "axis": "X",
      "color_rgb": [0.28, 0.32, 0.38],
      "step_file": "meshes/carriage_x.step",
      "primitives": []
    },
    {
      "id": "quill_z",
      "parent": "carriage_x",
      "axis": "Z",
      "color_rgb": [0.82, 0.84, 0.86],
      "step_file": "meshes/quill_z.step",
      "primitives": []
    }
  ]
}
```

---

## 3. Key Property Definitions

### Root Properties
* **`machine_name`** *(string)*: Display name shown in the UI and window title.
* **`manufacturer`** *(string)*: Builder or brand of the CMM.
* **`controller_type`** *(string)*: Hardware interface driver (`PicoSerial`, `Simulation`).

### Axis & Limit Properties (`axes`)
* **`scale_mm`** *[X, Y, Z float]*: Encoder resolution per step/pulse in millimeters.
* **`sign`** *[X, Y, Z int]*: Direction multipliers (`1` or `-1`) to match motor phasing.
* **`travel_min_mm` / `travel_max_mm`** *[X, Y, Z float]*: Soft travel limits of the machine envelope.

### Kinematic Chain Nodes (`kinematic_chain`)
Each physical moving assembly or fixed base is defined as a node in the kinematic tree:
* **`id`** *(string)*: Unique identifier for the link.
* **`parent`** *(string)*: The `id` of the parent link this part rides on (leave empty `""` for world/granite base).
* **`axis`** *(string)*: Motion axis controlled by the DRO (`"NONE"`, `"X"`, `"Y"`, or `"Z"`).
* **`color_rgb`** *[R, G, B float]*: RGB color values between `0.0` and `1.0` (applied if no STEP file is used or as a base tint).
* **`step_file`** *(string)*: Relative path to an external `.step` or `.stp` CAD file for this link's geometry.
* **`primitives`** *(array)*: Inline procedural geometry (boxes/cylinders) used as a fallback if no STEP file is specified. Useful for quick prototyping or parametric components.

---

## 4. How to Package a Machine (.cmm)

1. Create a working folder on your computer (e.g., `Helmel_Checkmaster/`).
2. Place your `machine.cmm.json` file inside the root of that folder.
3. Create a subfolder named `meshes/` and drop your `.step` files inside.
4. Select all contents of the folder (`machine.cmm.json` and the `meshes/` directory), right-click, and compress them into a **ZIP archive**.
5. Rename the file extension from `.zip` to **`.cmm`** (e.g., `Helmel_Checkmaster.cmm`).

---

## 5. Loading into OpenCMM

1. Launch OpenCMM and click **⚙ Settings...** in the top-left toolbar.
2. Go to the **Machine & Hardware** tab.
3. Click **Browse .cmm...** and select your packed `.cmm` archive (or an unzipped `.cmm.json` file).
4. Click **OK**—OpenCMM will automatically unpack the archive, load the kinematic hierarchy, display the 3D machine model, and animate it in real time at 60 FPS as you jog or simulate probe hits.