# OpenCMM Machine Package Specification (`.cmm`)

OpenCMM uses a modular, CAM-style machine definition architecture. A `.cmm` package is a standard **ZIP archive** (renamed to `.cmm`) containing a JSON manifest (`machine.cmm.json`), optional STEP 3D link meshes, configuration metadata, and a **DMIS 5.3 Characterization File (`.chf`)**. This allows you to swap physical CMM configurations, axis travels, kinematic hierarchies, and metrology standards support without modifying any C++ code.

---

## 1. Directory Structure Inside a `.cmm` Archive

When you create a `.cmm` package (which is just a `.zip` file renamed to `.cmm`), its internal layout should follow this structure:

```text
MyMachine.cmm (ZIP Archive)
│
├── machine.cmm.json           <-- Main manifest (required)
├── Helmel_Checkmaster.chf     <-- DMIS 5.3 characterization file (required for metrology)
└── meshes/                    <-- Folder for 3D link geometry (optional)
    ├── granite_base.step
    ├── bridge_y.step
    ├── carriage_x.step
    └── quill_z.step
```

---

## 2. Setting Up CAD Origins

To ensure your STEP models align and animate correctly in OpenCMM without manual offset tweaking, set up your CAD assembly (in SolidWorks, Fusion 360, Inventor, etc.) using these coordinate conventions before exporting:

### The Granite Table Base
* **`Z = 0`:** Set this precisely on the **top flat surface of the granite plate**. All part heights and probe tips measure upward from this reference plane.
* **`X = 0` and `Y = 0`:** Set this at the **center of the granite work area** or at the front-left corner of the usable measuring volume.

### Individual Link STEP Files
When exporting individual component STEP files, export them relative to the **assembly's global origin** while the machine is at its **home / zero position**:
* **Granite Base (`granite_base.step`):** Modeled with the top surface at $Z=0$.
* **Bridge / Y-Axis (`bridge_y.step`):** Modeled at its nominal home position along Y travel.
* **Carriage / X-Axis (`carriage_x.step`):** Modeled centered on the bridge crossbeam at X = 0.
* **Quill / Z-Axis (`quill_z.step`):** Modeled at its **fully retracted (up) home position** ($Z_{\max}$).

---

## 3. Manifest Schema (`machine.cmm.json`)

### Full Example Manifest (`machine.cmm.json`)
```json
{
  "machine_name": "Helmel Checkmaster 112-102",
  "manufacturer": "Helmel Engineering",
  "controller_type": "PicoSerial",
  "axes": {
    "scale_mm": [0.0005, 0.0005, 0.0005],
    "sign": [1, 1, 1],
    "travel_min_mm": [-150.0, -150.0, -250.0],
    "travel_max_mm": [150.0, 150.0, 0.0]
  },
  "probe_mount_offset_mm": [0.0, -93.0, 252.0],
  "kinematic_chain": [
    {
      "id": "granite_base",
      "parent": "",
      "axis": "NONE",
      "color_rgb": [0.22, 0.24, 0.26],
      "step_file": "meshes/granite_base.step",
      "part_colors": {
        "Granite_Surface_Plate": [0.22, 0.24, 0.26],
        "Helmel_Front_Badge": [0.14, 0.42, 0.72]
      },
      "primitives": [
        { "type": "box", "origin": [-265, -342.5, -100], "size": [530, 685, 100] }
      ]
    },
    {
      "id": "bridge_y",
      "parent": "granite_base",
      "axis": "Y",
      "color_rgb": [0.86, 0.88, 0.89],
      "step_file": "meshes/bridge_y.step",
      "part_colors": {
        "Checkmaster_Blue_Fascia": [0.14, 0.42, 0.72],
        "Y_Axis_Lock_Knob": [0.16, 0.18, 0.21]
      },
      "primitives": []
    },
    {
      "id": "carriage_x",
      "parent": "bridge_y",
      "axis": "X",
      "color_rgb": [0.80, 0.82, 0.84],
      "step_file": "meshes/carriage_x.step",
      "part_colors": {
        "Housing_Blue_Stripe": [0.14, 0.42, 0.72],
        "X_Axis_Clamp_Knob": [0.16, 0.18, 0.21]
      },
      "primitives": []
    },
    {
      "id": "quill_z",
      "parent": "carriage_x",
      "axis": "Z",
      "color_rgb": [0.82, 0.84, 0.86],
      "step_file": "meshes/quill_z.step",
      "part_colors": {
        "Renishaw_TP_ES_Body": [0.15, 0.16, 0.18],
        "Stylus_10mm_Stem": [0.88, 0.90, 0.92],
        "Stylus_Ruby_Tip": [0.92, 0.12, 0.18]
      },
      "primitives": []
    }
  ]
}
```

---

## 4. DMIS 5.3 Characterization File (`.chf`)

In compliance with **DMIS 5.3 Conformance specifications (Clause 5.5)**, machine packages include a characterization file declaring supported EBNF command subsets, machine-dependent algorithms (`ALGDEF`), error handling profiles (`ERROR`), and persistent identifier rules (`QIF` / `QPId`).

### Example Characterization File (`Helmel_Checkmaster.chf`)
```text
CHFILE/INPUT
$$ =====================================================================
$$ OpenCMM - Helmel Checkmaster 1112-102 Characterization File (DMIS 5.3)
$$ Copyright (C) 2026 JLS Machine LLC
$$ SPDX-License-Identifier: GPL-3.0-or-later
$$ =====================================================================

CHFIL1
(*-------------------------------------------------------------*)
(* DMIS 5.3 INPUT Grammar Subset supported by OpenCMM          *)
(*-------------------------------------------------------------*)
inputFile = dmisFirstStatement , [dmisStatementList] , endfilStm ;
dmisFirstStatement = dmismnStm ;

dmisStatement = 
  aclratStm     (* FULL *)
  | assignStm   (* FULL *)
  | boundStm    (* FULL *)
  | calibStm    (* FULL *)
  | clearStm    (* FULL *)
  | constStm    (* PART *)
  | datdefStm   (* FULL *)
  | datsetStm   (* FULL *)
  | declStm     (* FULL *)
  | displyStm   (* FULL *)
  | dmisStm     (* FULL *)
  | endmesStm   (* FULL *)
  | endfilStm   (* FULL *)
  | evalStm     (* FULL *)
  | featStm     (* PART *)
  | fedratStm   (* FULL *)
  | filnamStm   (* FULL *)
  | gotoStm     (* FULL *)
  | ifStm       (* FULL *)
  | measStm     (* FULL *)
  | modeStm     (* FULL *)
  | outputStm   (* FULL *)
  | ptmeasStm   (* FULL *)
  | refidStm    (* EXTENSION - DMIS 5.3 QPId / REFID *)
  | rotateStm   (* FULL *)
  | saveStm     (* FULL *)
  | snsdefStm   (* PART *)
  | snslctStm   (* FULL *)
  | tecompStm   (* FULL *)
  | textStm     (* FULL *)
  | tolStm      (* PART *)
  | transStm    (* FULL *)
  | unitsStm    (* FULL *)
  | wkplanStm   (* FULL *)
  ;
ENDCH1

CHFIL2
$$ Machine-dependent parameters for OpenCMM & Helmel Checkmaster Hardware
ALGDEF/CODE,INTGR
  1, 'Least Squares (LSTSQR)'
  2, 'Min-Max (MINMAX)'
  3, 'Maximum Inscribed (MAXINS)'
  4, 'Minimum Circumscribed (MINCIR)'
ENDAT

ERROR/CODE,INTGR
  101, 'ILLEGALTOUCH: Unexpected or unintended probe trigger'
  102, 'NOTOUCH: Probe failed to trigger within search distance'
  103, 'ENCODER_COMM_ERROR: Pico 2 serial communications lost'
ENDAT

FILDEF/CODE,INTGR
  1, 'DEFAULT_FILTER'
ENDAT
ENDCH2

CHFIL3
$$ User-defined extensions and QIF 2.1 interoperability metadata
QIF_SUPPORT = TRUE
QPID_SUPPORT = TRUE
MAX_IDENTIFIER_LENGTH = 64
ENDCH3

ENDCHF


CHFILE/OUTPUT
CHFIL1
outputFile = dmisProg ;
dmisProg = filnamStm , [dmisOutputList] , endfilStm ;
dmisOutputList = [dmisOutputList] , dmisOutputStatement ;
dmisOutputStatement = 
  faFeatureStatement
  | taToleranceStatement
  | textResStatement
  ;
ENDCH1
ENDCHF
```

---

## 5. Key Property Definitions

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
* **`color_rgb`** *[R, G, B float]*: Default fallback RGB color values between `0.0` and `1.0`.
* **`step_file`** *(string)*: Relative path to an external `.step` or `.stp` CAD file for this link's geometry.
* **`part_colors`** *(dictionary)*: Maps individual solid body names (parsed from `MANIFOLD_SOLID_BREP` entries inside STEP files) to custom RGB color arrays `[R, G, B]`.
* **`primitives`** *(array)*: Inline procedural geometry (boxes/cylinders) used as a fallback if no STEP file is specified.
* **`probe_mount_offset_mm`** *[X, Y, Z float]*: The offset from the quill home position to the active ruby probe tip, ensuring the CAD-modeled probe tip aligns perfectly with the machine's DRO coordinates.

---

## 6. How to Package and Load a Machine

1. Create a working folder on your computer (e.g., `Helmel_Checkmaster/`).
2. Place your `machine.cmm.json` manifest and your `Helmel_Checkmaster.chf` characterization file inside the root of that folder.
3. Create a subfolder named `meshes/` and drop your exported STEP files inside.
4. Select the contents of the folder, compress them into a **ZIP archive**, and rename the extension from `.zip` to **`.cmm`** (e.g., `Helmel_Checkmaster.cmm`).
5. In OpenCMM, open **⚙ Settings...** $\rightarrow$ **Machine & Hardware**, click **Browse .cmm...**, and select your package. OpenCMM will automatically parse the archive, load the kinematic digital twin, and validate the DMIS 5.3 characterization profile.