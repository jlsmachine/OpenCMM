# OpenCMM Hardware & Firmware Interface Guide

**Target Hardware:** Helmel Checkmaster CMM + Renishaw Optical Scales + Renishaw Touch-Trigger Probe + Raspberry Pi Pico 2 (RP2350)

This directory contains the MicroPython PIO quadrature decoder firmware (`main.py`) and hardware wiring documentation required to interface physical CMM linear optical encoders and a touch-trigger probe with OpenCMM over USB CDC Serial (`115200` baud).

## 1. DB15 Scale Breakout to Pico 2 Wiring Schematic (X-Axis Reference)

Each linear optical encoder readhead connects through a DB15 breakout terminal block using shielded KWANGIL multi-conductor cable to the Raspberry Pi Pico 2 screw terminal shield.

| DB15 Breakout Terminal | Renishaw Signal Connected | KWANGIL Cable Wire | Pico 2 Screw Shield Terminal | 
| ----- | ----- | ----- | ----- | 
| **Terminal 7 & 8** | $+5\text{V}$ (Brown wire from factory cable) | Red | **VBUS (**$5\text{V}$**)** | 
| **Terminal 2 & 9** | $0\text{V}$ / GND (White wire from factory cable) | Black | **GND** | 
| **Terminal 14** | $\text{A+}$ Signal (Green wire from factory cable) | Green | **GP2 (X-Axis Phase A)** | 
| **Terminal 13** | $\text{B+}$ Signal (Blue wire from factory cable) | White | **GP3 (X-Axis Phase B)** | 
| **Shield Clamp** | Braided Drain / Outer Shield | Bare Drain Wire | **GND** *(Connected at Pico end only)* | 

> **Note on Grounding & EMI Shielding:** Connect the bare braided shield drain wire to `GND` **only at the Pico 2 screw shield end**. Leaving the DB15 breakout end isolated prevents ground loops while draining ambient shop-floor electrical noise away from the $\text{A+}/\text{B+}$ quadrature lines.

## 2. DB9 Touch-Trigger Probe to Pico 2 Wiring Schematic

The Renishaw touch-trigger probe connects via a DB9 breakout connector directly to the Pico 2 screw shield. The internal kinematic switch operates as a Normally Closed (NC) circuit when seated.

| DB9 Pin | Wire Color | Official Renishaw Function | Electrical Role | Pico 2 Connection |
| ----- | ----- | ----- | ----- | ----- |
| **Pin 1** | Red | LED Cathode ($-$) | Probe Status LED Negative | Optional *(See LED Note)* |
| **Pin 2** | Thick Black (Coax) | Screen / Shield | Outer Cable Shield / Ground | **GND** |
| **Pin 3** | White | LED Anode ($+$) | Probe Status LED Positive | Optional *(See LED Note)* |
| **Pin 4** | Thin Black | Probe Circuit | Kinematic Switch Contact A | **GND** |
| **Pin 5** | Green | Probe Circuit | Kinematic Switch Contact B | **GP14 (Probe Input)** |

> **Probe Status LED Note:** Pins 1 (Red, Cathode $-$) and 3 (White, Anode $+$) drive the built-in status LED inside the Renishaw probe head. If you wish to illuminate the LED, connect **Pin 1 (Red)** to **GND** and **Pin 3 (White)** to **$3.3\text{V}$** (or a dedicated GPIO output pin) through a current-limiting resistor (typically $220\,\Omega$ to $330\,\Omega$). Never connect the LED pins directly to $+5\text{V}$ (`VBUS`) or $3.3\text{V}$ without a series resistor, or you will burn out the internal probe LED.

## 3. Complete Pico 2 GPIO Pin Allocation (All Axes & Probe)

When wiring the remaining Y and Z axes (using the same DB15 terminal pinout for power and $\text{A+}/\text{B+}$ signals) and the Renishaw touch-trigger probe, use the following GPIO map configured in `main.py`:

| Function / Axis | Signal | Pico 2 Pin | Internal Pull-Up | Notes | 
| ----- | ----- | ----- | ----- | ----- | 
| **Power Supply** | $+5\text{V}$ Readhead Power | `VBUS` | N/A | Supplied directly from USB $5\text{V}$ | 
| **Common Ground** | $0\text{V}$ / Signal Ground | `GND` | N/A | Shared across all 3 scales, shield drains, and probe | 
| **X-Axis Scale** | Phase $\text{A+}$ / Phase $\text{B+}$ | `GP2` / `GP3` | `Pin.PULL_UP` | PIO StateMachine `0` | 
| **Y-Axis Scale** | Phase $\text{A+}$ / Phase $\text{B+}$ | `GP6` / `GP7` | `Pin.PULL_UP` | PIO StateMachine `1` | 
| **Z-Axis Scale** | Phase $\text{A+}$ / Phase $\text{B+}$ | `GP10` / `GP11` | `Pin.PULL_UP` | PIO StateMachine `2` | 
| **Touch Probe** | Seated / Hit Trigger | `GP14` | `Pin.PULL_UP` | Normally Closed (NC) to `GND` when seated; `HIGH` ($1$) on hit | 

## 4. Serial Protocol Specification (`115200` Baud NDJSON)

The Pico 2 streams Newline-Delimited JSON (NDJSON) packets at $60\text{ Hz}$ and responds to host commands instantaneously:

### Outgoing Packets (Pico 2 $\rightarrow$ OpenCMM Host)

* **Autonomous** $60\text{ Hz}$ **DRO Stream:**

  ```json
  {"type":"dro","x":14520,"y":-8310,"z":4102,"probe":0}
  ```

* **Hardware Interrupt Probe Hit Event (**$60\text{ ms}$ **hardware debounce):**

  ```json
  {"type":"hit","event":"hit","id":1,"x":14522,"y":-8310,"z":4100,"probe":1}
  ```

* **Ping Diagnostic Response:**

  ```json
  {"type":"pong"}
  ```

### Incoming Host Commands (OpenCMM Host $\rightarrow$ Pico 2)

* `PING\n` — Returns `{"type":"pong"}` to verify connection health.

* `ZERO\n` (or `Z\n`) — Resets all three hardware PIO state machine $X$ registers (`sm_x`, `sm_y`, `sm_z`) to `0`.