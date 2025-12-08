# ⚙️ Two-Stage CNC PCB Milling Workflow (KiCad Gerber)

This repository contains two Python scripts designed to transform **KiCad Gerber** and **Excellon drill files** into optimized **G-Code** for PCB milling machines, focusing on trace isolation, drilling, and bed leveling compensation.

The script is based on Matthias Wandel script:
https://github.com/Matthias-Wandel/Gerber2nc

Big thanks on GOOGLE GEMINI, who supported me a lot in the improvement of the scripts.

The workflow is divided into two distinct stages: **Preparation** (`pcb_preparation_v2.py`) and **Machining** (`gerber2nc_v3.py`).


> 📚 For a comprehensive, step-by-step guide on running the CNC process (including Z-Referencing and tool changes), please refer to the detailed documentation: **[README_CNC'ing.md](README_CNC'ing.md)**



## ⚠️ Disclaimer

**THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.**

**BY USING THIS SCRIPT AND THE GENERATED G-CODE, YOU AGREE TO ASSUME ALL RISKS AND RESPONSIBILITIES FOR ANY PERSONAL INJURY, EQUIPMENT DAMAGE, OR OTHER LOSSES THAT MAY RESULT FROM THE OPERATION OF YOUR CNC MACHINE.**

**The user is solely responsible for:**
* Verifying all generated G-Code before execution.
* Ensuring the machine's work area is clear and safe.
* Confirming that all tool offsets, depths, feed rates, and RPM values are safe and appropriate for their specific CNC machine and materials.

---

## 1. ✨ Workflow Overview & Features

### Stage 1: Preparation (`pcb_preparation_v2.py`)
This script focuses on physically aligning the PCB and generating the **Bed Mesh** file using the `Edge_Cuts` contour.

* **Automatic Centering:** Automatically shifts and aligns the PCB to the defined center of the bed (`BED_MID_X`, `BED_MID_Y`) for machining.
* **Tool Head Positioning:** Generate G-code to move the tool head at PCB's bottom-left corner to facilitate PCB placement.
* **Contour Mapping:** Generates G-code to safely trace the perimeter of the placed PCB for visual verification.
* **Bed Mesh Area:** Calculates the exact area for the `G29` probing routine based on the PCB outline.
* **Visualization:** Outputs a `.png` image showing the PCB placement, bed-mesh area and safety margin area.

### Stage 2: Machining (`gerber2nc_v3.py`)
This script generates the final cutting toolpaths and drilling holes.

* **Trace Isolation:** Converts copper traces into offset paths for isolation milling, supporting single or multi-pass routing.
* **Drilling Separation:** Separates holes into **"small"** and **"large"** groups for dedicated drilling files.
* **Edge Cut Clearance:** Generates a final edge cut G-code with a customizable **clearance offset** around the board outline.
* **Visualization:** Outputs a `.png` image showing all toolpaths, holes, and the PCB contour. 

---

## 🚀 Getting Started

### Prerequisites

Both scripts require the `numpy` and `matplotlib` libraries, and `gerber2nc_v3.py` additionally requires `shapely` and `Pillow`.

```bash
pip install numpy matplotlib shapely Pillow
```

# ⚙️ KiCad Gerber to CNC G-Code Generator (Python)

This Python script is a robust tool designed to transform **KiCad Gerber** and **Excellon drill files** into optimized **G-Code** for PCB milling machines. It handles trace isolation routing, drilling, and incorporates advanced features like **edge cut clearance**  generation for clean board separation.

## ✨ Features

* **Automatic Toolpath Generation:** Converts copper traces into offset paths for isolation milling using `shapely`.
* **Single or Multi-Pass Isolation:** Supports single or multiple passes with defined spacing for wider isolation areas.
* **Two-Group Drilling:** Separates holes into "small" and "large" groups for optimized drilling with different tools.
* **Contour Separation:** Generates a final edge cut G-code file with a customizable **clearance offset**.
* **Visualization:** Outputs a `.png` image showing all toolpaths, holes, and the PCB contour with clearance and tabs.

## 🚀 Getting Started

### Prerequisites

This script requires the `shapely`, `Pillow`, and `numpy` libraries.

```bash
pip install shapely Pillow numpy
````

### Usage

1.  **Export KiCad Files:** Ensure you have the following files generated from KiCad in the same directory as the script:

      * `[ProjectName]-F_Cu.gbr` (Front Copper Layer)
      * `[ProjectName]-Edge_Cuts.gbr` (Board Outline)
      * `[ProjectName]-PTH.drl` (Plated Through Holes/Drill File)

2.  **Run Stage 1 (Preparation):** Execute `pcb_preparation_v2.py` to get the necessary alignment and bed mesh files.

    ```bash
    python3 pcb_preparation_v2.py
    # Enter base file name (e.g., 'MyBoard')
    ```

3.  **Run Stage 2 (Machining):** Execute `gerber2nc_v3.py` to generate the final toolpaths.

    ```bash
    python3 gerber2nc_v3.py MyBoard
    ```

### Output Files

| Script | File Name Pattern | Description | Bed Mesh |
| :--- | :--- | :--- | :--- |
| `pcb_preparation_v2.py` | `xx_BedMesh_G29_CNC.gcode` | The file to run the G29 Bed Mesh routine. | N/A |
| `pcb_preparation_v2.py` | `xx_CheckPos_CNC.png` | G-code to move the tool head for PCB alignment (moves to Start Point at bottom-left corner) | N/A |
| `pcb_preparation_v2.py` | `xx_ContourMapping_CNC.gcode` | G-Code to trace the perimeter for alignment verification. | N/A |
| `pcb_preparation_v2.py` | `xx_ContourMapping_Visualization.png` | Visualization of PCB placement and mesh area. | N/A |
| `gerber2nc_v3.py` | `xx_Pre_Drill.gcode` | G-Code for hole center marking (Pre-drill). | ENABLED |
| `gerber2nc_v3.py` | `xx_Engraving.gcode` | G-Code for trace isolation milling. | ENABLED |
| `gerber2nc_v3.py` | `xx_Small_Drill.gcode` | G-Code for small diameter holes ($\le$ 0.85mm). | **DISABLED** |
| `gerber2nc_v3.py` | `xx_Large_Drill.gcode` | G-Code for large diameter holes. | **DISABLED** |
| `gerber2nc_v3.py` | `xx_Edge_Cut.gcode` | G-Code for final board contour marking. | ENABLED |
| `gerber2nc_v3.py` | `xx_Visualization.png` | Visual map of all toolpaths and holes. | N/A |
-----

## 🛠️ Configuration Parameters

The milling behavior and path generation are controlled by parameters set at the top of the Python script. **Adjust these values** based on your machine, tooling, and material.

| Script | Parameter | Default Value | Description |
| :--- | :--- | :--- | :--- |
| **Positioning** | `BED_MID_X` (`gerber2nc_v3.py`) | `100.0` | Target X coordinate (mm) for the PCB **center**. |
| **Positioning** | `BED_MID_Y` (`gerber2nc_v3.py`) | `140.0` | Target Y coordinate (mm) for the PCB **center**. |
| **Positioning** | `BED_MAX_X` (`gerber2nc_v3.py`) | `240.0` | Max bed width (mm) used to delimit the Bed Mesh. |
| **Positioning** | `BED_MAX_Y` (`gerber2nc_v3.py`) | `220.0` | Max bed depth (mm) used to delimit the Bed Mesh. |
| **Milling** | `SPINDLE_SPEED` (`gerber2nc_v3.py`) | `254` | Spindle speed (S-value in G-code). |
| **Milling** | `SAFE_HEIGHT` (`gerber2nc_v3.py`) | `5.0` | Z-axis height (mm) for rapid moves (G0). |
| **Milling** | `MOVE_FEED_RATE` (`gerber2nc_v3.py`) | `2000` | X/Y speed for non-cutting moves (G0). |
| **Milling** | `PLUNGE_FEED_RATE` (`gerber2nc_v3.py`) | `50` | Z-axis feed rate (mm/min). |
| **Isolation** | `CUT_DEPTH` (`gerber2nc_v3.py`) | `-0.06` | **Trace isolation depth (mm).** |
| **Isolation** | `ISOLATION_OFFSET` (`gerber2nc_v3.py`) | `0.22` | **1st pass distance (mm)** from the trace centerline. |
| **Isolation** | `ISOLATION_TYPE` (`gerber2nc_v3.py`) | `'single'` | Set to `'multi'` for 3 passes; otherwise, 1 pass. |
| **Isolation** | `PASS_SPACING` (`gerber2nc_v3.py`) | `0.04` | Lateral spacing (mm) between passes if `ISOLATION_TYPE` is `'multi'`. |
| **Drilling** | `SMALL_HOLE_MAX_DIAMETER` (`gerber2nc_v3.py`) | `0.85` | Max diameter (mm) for holes to be grouped as `Small_Drill`. |
| **Drilling** | `HOLE_START_DEPTH` (`gerber2nc_v3.py`) | `0.5` | Z height (mm) where drilling starts (above material). |
| **Drilling** | `HOLE_FINAL_DEPTH` (`gerber2nc_v3.py`) | `-1.8` | Final drilling depth (mm). |
| **Drilling** | `Z_DRILL_ADJUST` (`gerber2nc_v3.py`) | `0.0` | Manual offset to be added to drilling Z-positions. |
| **Outline** | `FINAL_CUT_DEPTH` (`gerber2nc_v3.py`) | `-0.1` | Edge cut depth (mm) for the final contour marking. |
| **Outline** | `EDGE_CUT_CLEARANCE_OFFSET` (`gerber2nc_v3.py`) | `1.0` | **Clearance offset (mm)** added around the board outline for the final cut path. |
-----

## 🖼️ Visualization Output

The `_visualization.png` file provides a critical pre-flight check of all toolpaths.

  * **Dark Green:** The original PCB area defined by Edge Cuts.
  * **White Lines:** Trace Isolation Toolpaths.
  * **Green Lines:** Shallow Cut (`TAB_HEIGHT`) segments corresponding to the tab areas.
  * **Red/Blue/Black:** Traces, Pads, and Holes (for context).

<!-- end list -->







