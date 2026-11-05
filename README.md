# OpenCMM

**OpenCMM** is a modern, open-source CMM (Coordinate Measuring Machine) acquisition, inspection, and metrology software suite. Built for high-performance precision measurement, it bridges custom hardware (such as manual CMMs retrofitted with Raspberry Pi Pico 2 encoders and touch probes) with professional-grade CAD rendering and least-squares metrology analytics.

---

## Key Features

- **Live Open CASCADE 3D Viewport (`OcctViewport`)**: 
  - Direct STEP file CAD import and topological B-Ref face picking (`TopAbs_FACE`).
  - Real-time 3D ruby probe stylus kinematic tracking and interactive hit-point marker visualization.
  - Interactive 3D orbit, pan, and zoom powered by Open CASCADE's OpenGL graphic driver and WNT window integration.
- **Hardware Abstraction Layer (HAL)**:
  - Streams raw 32-bit signed quadrature encoder counts from a Raspberry Pi Pico 2 via JSON over serial (`qtserialport`).
  - Configured for sub-micron accuracy ($0.0005\text{ mm}$ per quadrature count internally).
  - Live DRO (Digital Readout) in Millimeters and Inches with zeroing and live trigger detection.
- **Advanced Metrology Math Engine**:
  - Least-squares sphere fitting (Master Ball calibration & form error analysis).
  - Least-squares plane fitting (SVD-based normal vector and flatness evaluation).
  - 3-2-1 Part Alignment and SVD/Kabsch Best-Fit nominal matching.
- **Inspection Feature Tree**:
  - Structured QTreeWidget UI organizing active probe point buffers and solved geometric features.

---

## Tech Stack

- **Language:** C++20
- **Build System:** CMake (with vcpkg manifest mode)
- **GUI Framework:** Qt6 (`qtbase`, `qtwidgets`, `qtserialport`, OpenGL modules)
- **CAD Kernel:** Open CASCADE Technology (OCCT)
- **Math Library:** Eigen3
- **Serialization:** nlohmann-json

---

## Project Directory Structure

```text
OpenCMM/
├── CMakeLists.txt              # Root CMake configuration
├── vcpkg.json                  # vcpkg manifest dependencies
├── include/
│   ├── core/
│   │   └── Units.h             # Unit conversion and Position3D structures
│   ├── hal/
│   │   ├── ICmmController.h    # Abstract hardware interface & signals
│   │   └── PicoSerialController.h # Pico 2 JSON serial streaming HAL
│   ├── math/
│   │   └── MetrologyMath.h     # Least-squares sphere/plane fitting & alignments
│   └── ui/
│       ├── MainWindow.h        # Main application window layout & docks
│       └── OcctViewport.h      # Open CASCADE 3D CAD & stylus viewport
└── src/
    ├── main.cpp                # Application entry point
    ├── hal/
    │   └── PicoSerialController.cpp
    ├── math/
    │   └── MetrologyMath.cpp
    └── ui/
        ├── MainWindow.cpp
        └── OcctViewport.cpp
```

---

## Getting Started & Building

### Prerequisites
- **Visual Studio 2026** (or MSVC 2022 toolchain with C++20 support)
- **CMake** (3.24 or higher)
- **vcpkg** (configured in manifest mode)
- **Qt6 & Open CASCADE** (automatically provisioned and built via `vcpkg.json`)

### Build Instructions

1. Clone the repository:
   ```powershell
   git clone https://github.com/your-username/OpenCMM.git
   cd OpenCMM
   ```
2. Configure and build using CMake / Visual Studio:
   - Open the repository folder as a CMake project in Visual Studio.
   - Allow vcpkg to manifest-install dependencies (`eigen3`, `nlohmann-json`, `opencascade`, `qtbase`, `qtserialport`).
   - Press **F5** to build and launch OpenCMM.

---

## License & Copyright

Copyright (C) 2026 JLS Machine LLC

**OpenCMM** (including the C++ desktop studio and Raspberry Pi Pico 2 firmware) is free software: you can redistribute it and/or modify it under the terms of the **GNU General Public License as published by the Free Software Foundation**, either version 3 of the License, or (at your option) any later version (`GPL-3.0-or-later`).

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the [LICENSE](LICENSE) file for the full license text.

### Machine Post Packages & 3D CAD Assets (`Machines/`)
The digital twin machine manifests (`.cmm.json`), `.cmm` archives, and 3D STEP meshes located in the `Machines/` directory are licensed under the **Creative Commons Attribution 4.0 International License (CC-BY-4.0)**, allowing anyone to freely share, adapt, and author custom CMM machine packages.

### Third-Party Libraries & Acknowledgments
OpenCMM dynamically links against and builds upon the following open-source libraries:
* **Qt 6** (`QWidgets`, `QSerialPort`, `QGui`, `QCore`) — Licensed under the [GNU LGPLv3](https://www.gnu.org/licenses/lgpl-3.0.html) / GPLv3.
* **OpenCASCADE Technology (OCCT)** — Licensed under the [GNU LGPLv2.1](https://dev.opencascade.org/resources/licensing).
* **Eigen 3** — Licensed under the [Mozilla Public License 2.0 (MPL-2.0)](https://www.mozilla.org/en-US/MPL/2.0/).
* **MicroPython** (RP2350 / Pico 2 runtime) — Licensed under the [MIT License](https://opensource.org/licenses/MIT).