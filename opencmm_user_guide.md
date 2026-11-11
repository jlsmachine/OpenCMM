# OpenCMM Studio — Offline CAD Programming & DMIS 5.3 Execution Guide

**Copyright (C) 2026 JLS Machine LLC**

**License:** GPL-3.0-or-later

This guide explains how to use OpenCMM's **Offline CAD-to-DMIS Authoring**, **3D Virtual Raycast Probing**, **Digital Twin Trajectory Playback**, **AI Model Context Protocol (MCP) Integration**, and **Guided Shop-Floor Execution** workflows.

## 1. Workflow Overview

OpenCMM supports a complete, standards-compliant metrology lifecycle whether you are programming at your desk without hardware or inspecting physical parts on the granite table:

1. **Import & Orient CAD:** Load a customer `.step` / `.stp` model and position it inside the virtual machine volume.

2. **Author Inspection Offline:** Click CAD faces to auto-sample features (`Auto-Probe CAD`), pick individual points (`Ctrl + Left-Click`), or prompt an AI assistant via Model Context Protocol (MCP) to extract CAD faces and build inspection routines automatically.

3. **Export DMIS 5.3 Program & Reports:** Headlessly export or save inspection routines to `.dmis`, `.qif`, `.pdf`, `.csv`, or `.html` files complete with custom metadata, professional template layouts, and Certificate of Conformance sign-off blocks.

4. **Verify & Execute:** Load the `.dmis` program to run a full 3D Digital Twin trajectory simulation (`Run DMIS`) or guide a shop-floor operator point-by-point using live 3D bullseye markers and Raspberry Pi Pico 2 touch-trigger hardware.

---

## 2. Loading & Orienting Your 3D STEP Model

Before programming features offline, load your part model and place it on the virtual CMM granite table:

1. Navigate to the **CAD & Deviation** ribbon tab and click **Load STEP**.

2. Select your part file (`*.stp`, `*.step`). The model will appear in the 3D OpenCASCADE viewport with face-selection mode active.

3. Click **Orient CAD...** to open the part positioning dialog:

   * **Drop to Granite (**$Z=0$**):** Translates the part vertically so its lowest bounding-box point rests directly on the granite surface ($Z_{\min} = 0\text{ mm}$).

   * **Center XY:** Centers the part's bounding box around $(0, 0)$ in the machine XY plane.

   * **Min Corner** $\rightarrow (0,0,0)$**:** Snaps the minimum $(X, Y, Z)$ bounding-box corner to the machine origin.

   * **Level Selected Face** $+Z$**:** Left-click any planar face on the CAD model first, then click this button to rotate the entire part so that the selected face's normal points straight up along $+Z$.

   * **Manual Translate / Rotate:** Enter exact $\Delta X, \Delta Y, \Delta Z$ offsets (in $\text{mm}$) or $r_X, r_Y, r_Z$ rotations (in degrees) and click **Apply**.

### Using the Floating 3D Viewport HUD

In the top-right corner of the 3D viewport, use the floating **CAD DISPLAY & MACHINE LINKS** overlay panel to customize visibility:

* **Display Modes:** Switch the part between **Shaded + Edges**, **Shaded Only**, and **Wireframe**.

* **Machine Links:** Toggle individual CMM structural links (e.g., `granite_base`, `bridge_y`, `carriage_x`, `quill_z`) on or off. Active links highlight in blue.

* **Built-in Stylus:** Toggle the procedural red ruby tip and vertical stem on or off.

* **CAD Trans Slider:** Adjust CAD model transparency from $0\%$ to $90\%$ so you can clearly see internal bores, deep pockets, and probe hit markers inside the part.

---

## 3. Offline CAD Probing (Programming Without a CMM)

You can program complete dimensional inspections directly from the 3D STEP model using three complementary methods.

### Method A: One-Click Auto-Probe CAD Face (`Auto-Probe CAD`)

Best for rapidly programming planes, internal bores, external bosses, and spheres:

1. **Left-Click** any face on the 3D STEP model.

   * OpenCMM inspects the underlying B-Rep surface (`GeomAbs_Plane`, `GeomAbs_Cylinder`, `GeomAbs_Sphere`).

   * The status bar displays the detected surface type, nominal diameter (for cylinders/spheres), or centroid coordinates.

   * **Automatic Setup:** OpenCMM automatically sets **Tip Compensation** to `Internal (+R)` for bores/cavities or `External (-R)` for bosses/planes, and automatically switches the **2D Workplane** (`XY (+Z)`, `YZ (+X)`, `ZX (+Y)`, or `Last Fitted Plane` for tilted features) to match the feature's axis.

2. Click **Auto-Probe CAD** (available on both the **Home** and **CAD & Deviation** tabs).

3. OpenCMM automatically:

   * Samples the face's parametric $(U, V)$ space using `BRepTopAdaptor_FClass2d` (`TopAbs_IN`), automatically avoiding cross-holes, bolt holes, chamfers, and pockets.

   * Generates $5$ circumferential points for cylinders/spheres or $4$ farthest-point-sampled hits for planes.

   * Offsets each contact point along the outward surface normal by the effective stylus radius ($R_{\text{eff}}$):
     

     $$
     \mathbf{P}_{\text{ruby}} = \mathbf{P}_{\text{surf}} + R_{\text{eff}}\mathbf{n}
     $$

   * Solves the geometric feature (`Plane`, `2D Circle`, or `Sphere`), renders the fitted 3D feature visual in the viewport, and logs the pass/fail evaluation in the **Inspection Feature Tree**.

### Method B: Manual Virtual Raycast Probing (`Ctrl + Left-Click`)

Best for custom point distributions, 2D edge lines, or freeform surface deviation checks:

1. Hold down **Ctrl** and **Left-Click** anywhere on a CAD face in the 3D viewport.

2. OpenCMM casts an analytical 3D view ray (`IntCurvesFace_ShapeIntersector`) onto the exact B-Rep face under your cursor:

   * Extracts the true 3D surface contact coordinate $(X, Y, Z)$ and outward unit normal vector $\mathbf{n} = (I, J, K)$.

   * Computes the compensated ruby center position and moves the Digital Twin CMM quill/stylus to that point.

   * Adds the hit to the **Active Probe Buffer** annotated with its DMIS normal vector `N(I, J, K)` and status `CAD PTMEAS`.

3. Once you have placed the desired number of hits on the feature, go to the **Measure** tab and click **Fit Plane**, **Fit Sphere**, **Fit 2D Circle**, **Fit 2D Line**, or **Surface Dev**.

### Method C: AI Agent Integration via Model Context Protocol (MCP)

OpenCMM includes a modular local MCP server plugin (`opencmm_mcp.dll`) and zero-dependency `stdio`-to-TCP bridge (`opencmm_mcp_bridge.exe`) that connects AI assistants like Claude Desktop directly to your local session on port `8765`:

* **`get_cmm_status`**: Queries live DRO coordinates, units, stylus radius, and alignment state.
* **`list_cad_faces`**: Extracts topology (`PLANE`, `CYLNDR`, `SPHERE`, `CONE`), centroids, outward normals, bore orientation (`INNER`/`OUTER`), and nominal diameters from the loaded STEP model.
* **`create_dmis_feature`**: Queues planned offline DMIS inspection features—accepting an optional `faceIndex` to sample valid interior UV touch points automatically, plus custom per-feature tolerances (`plusTolMm`, `minusTolMm`) and Datum A/B/C tags.
* **`run_preflight_check`**: Runs CAD collision and standoff clearance verification across all planned features.
* **`solve_321_alignment`**: Computes and activates an orthogonal 3-2-1 Part Coordinate System (PCS) alignment from measured datums.
* **`run_dmis_program`**: Starts, stops, or resets 3D trajectory simulation playback.
* **`export_inspection_file`**: Headlessly exports inspection files (`.dmis`, `.qif`, `.pdf`, `.csv`, `.html`) with custom metadata and template styles.

---

## 4. Automated 3-2-1 Part Coordinate System (PCS) Alignment

To evaluate part geometry in drawing-specified coordinates rather than Machine Coordinate System (MCS) raw axes:

1. Probe or select features to serve as your primary datums:
   * **Datum A:** Primary leveling datum (typically the top flat clamping plane with a $+Z$ normal).
   * **Datum B:** Secondary orientation datum (typically a perpendicular side edge line or cylinder axis).
   * **Datum C:** Tertiary origin datum (typically a bore center or corner point).
2. Right-click any solved feature in the **Inspection Feature Tree** and choose **Set as Primary Datum A**, **Secondary Datum B**, or **Tertiary Datum C**, or use the **3-2-1 Align...** wizard on the **Home** tab.
3. Once all three datums are assigned, OpenCMM automatically computes the orthogonal transformation matrix (`m_pcsMatrix`), renders the 3D PCS coordinate triad in the viewport, and switches the DRO to aligned PCS coordinates.

---

## 5. Custom Per-Feature Tolerances & Reports

* **Editing Tolerances:** Right-click or select any solved feature in the **Inspection Feature Tree** and click **Edit Tol...** (or click the `±` button on the **Reporting** tab) to adjust exact nominal dimensions and asymmetric upper/lower tolerance limits (`+Tol`, `-Tol`).
* **Professional PDF Report Templates:** When exporting inspection certificates (`Export Report` or via MCP `export_inspection_file`), you can specify template layouts (`standard`, `aerospace`, or `compact`) along with professional header metadata:
  * Customer name and Purchase Order (PO) number.
  * Batch, lot, or serial numbers.
  * Quality inspector ID and Certificate of Conformance sign-off compliance blocks.

---

## 6. Exporting DMIS 5.3 Programs & Inspection Reports

Once you have solved your features in the **Inspection Feature Tree**:

1. Navigate to the **Reporting** tab.

2. Click **Export Report**.

3. Enter your **Part / Job Number** (e.g., `JLS-PART-001`).

4. In the Save dialog's **Save as type** dropdown, choose from five formats:

   * **DMIS 5.3 Inspection Program (`*.dmis`)**

   * **QIF 2.1 Quality Information Framework (`*.qif`)**

   * **PDF Printable Inspection Certificate (`*.pdf`)**

   * **HTML Printable Inspection Report (`*.html`)**

   * **CSV Spreadsheet (`*.csv`)**

### What Is Written in a `.dmis` 5.3 Export

When you export as `*.dmis`, OpenCMM generates a standard DMIS 5.3 program containing:

* Header & configuration statements (`DMISMN`, `FILNAM`, `UNITS/MM, ANGDEC`, `WKPLAN/XYPLAN`, `MODE/PROG, MAN`).

* Active stylus definition (`SNSTYP/INDEX, 1, <diameter>`).

* Automatic safe Z-clearance plane (`CLEARP/ZPLUS, <zMax + 25.0>`) computed from your CAD model's bounding box.

* Persistent QIF 2.1 traceability mappings (`REFID/F(FEAT_1), QIF, 'QPId', '<uuid>'`).

* Nominal feature declarations (`F(FEAT_1) = FEAT/PLANE...`, `FEAT/CIRCLE...`, `FEAT/SPHERE...`, `FEAT/LINE...`).

* Measurement blocks (`MEAS/... ENDMES`) containing every planned touch point and surface normal (`PTMEAS/CART, X, Y, Z, I, J, K`).

---

## 7. Loading & Executing DMIS 5.3 Programs

You can load any `.dmis` program back into OpenCMM for offline trajectory verification or guided shop-floor inspection on the physical CMM.

### Step 1: Load the `.dmis` File

1. Click **Load DMIS** on the **Home** or **CAD & Deviation** tab and select a `.dmis` file.

2. OpenCMM parses all `FEAT/`, `MEAS/`, `PTMEAS/`, `REFID/`, `SNSTYP/`, and `CLEARP/` blocks:

   * Every feature in the program is added to the **Inspection Feature Tree** with a yellow `PLANNED (0/N pts)` status.

   * The first feature becomes `ACTIVE` and its nominal value, probe compensation mode (`Internal` vs. `External`), and 2D workplane are armed automatically.

### Step 2: Follow the 3D Target Guide Markers

In the 3D viewport, OpenCMM renders visual probing guides for the active feature:

* **Gold Bullseye Sphere + Approach Vector Rod:** Marks the exact coordinate where the probe tip should touch next, with an $18\text{ mm}$ gold rod extending along the outward surface normal $(I, J, K)$ to show the approach direction.

* **Orange Spheres + Short Rods:** Show upcoming touch points queued for the current feature.

* **Green Spheres:** Mark completed touch points on the current feature.

### Step 3: Choose Your Execution Mode

#### Mode 1: Automated Offline Digital Twin Playback (`Run DMIS`)

* Click **Run DMIS** (`▶`) on the **Home** or **CAD & Deviation** tab (or invoke via MCP `run_dmis_program`).

* The Digital Twin CMM animates smoothly at $30\text{ FPS}$ through the entire inspection program:

  1. Moves through the Z-clearance plane (`CLEARP`) to the $18\text{ mm}$ normal standoff position.

  2. Approaches along the surface normal vector $(I, J, K)$ to make contact with the surface.

  3. Records the hit, retracts along the normal vector, and advances to the next target point.

  4. Automatically solves each feature (`PASS` / `FAIL`), renders the fitted 3D geometry, and advances to the next feature until the entire program finishes.

* Click **Stop DMIS** (`⏹`) at any time to pause playback.

#### Mode 2: Single-Step Virtual Probing (`Simulate Hit`)

* While a DMIS program is loaded, click **Simulate Hit** (`⚡`) on the **Home** tab to fire one planned `PTMEAS` point at a time.

* This allows you to inspect each touch point, normal vector, and feature fit step-by-step.

#### Mode 3: Guided Shop-Floor CMM Probing (Connected Hardware)

* Connect your physical CMM (e.g., Helmel Checkmaster with Raspberry Pi Pico 2 controller) via **Settings...**.

* Load the part's `.dmis` program using **Load DMIS**.

* As you move the physical CMM bridge/quill, the status bar displays a live 3D proximity guide to the active gold target bullseye:
  

  $$
  \text{Guide to Target: } \Delta X, \Delta Y, \Delta Z \quad (\text{3D Dist})
  $$

* Touch the physical part at the gold target marker. When the touch-trigger probe fires:

  * OpenCMM records the actual machine coordinate, pairs it with the planned DMIS surface normal, turns the marker green, and highlights the next gold target.

  * As soon as the last point of the `MEAS/` block is triggered, OpenCMM automatically solves the feature in-place in the tree, plays the pass/fail chime, and advances the 3D gold target markers to the next feature in the program.

---

## 8. Keyboard Shortcuts & UI Customization

* **Live Keyboard Shortcuts:** OpenCMM includes fully customizable keybindings managed via **Configure Keyboard Shortcuts** (accessible by right-clicking anywhere on the Ribbon bar). Key bindings such as `Space` for `Simulate Hit`, `Ctrl+0` for `Zero Axes`, `Ctrl+U` for `Toggle Units`, and `F5` for `Run / Stop DMIS` are saved persistently in `QSettings`.
* **Window Size & Layout Persistence:** OpenCMM automatically saves and restores your window size, monitor position, dock layout, and **Maximized** state whenever you close and reopen the application.
* **Dark & Light Themes:** Open **Settings...** (`⚙`) on the **Home** tab and toggle **Enable Dark Metrology Theme** to switch all toolbars, DRO readouts, feature trees, floating HUD controls, and the 3D OpenCASCADE background between Dark and Light modes.
* **Reset to Factory Settings:** Inside **Settings...**, click **Reset to Factory Settings** in the bottom-left corner to clear all stored `QSettings` preferences, reset axis scales/polarities, restore default tolerances, and return the Digital Twin to the default moving-bridge CMM.