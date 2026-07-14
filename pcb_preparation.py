#!/usr/bin/python3
# Generates G-code for CNC bed mesh leveling and contour mapping from a KiCad PCB.
#
# Output files:
#   1) xx_BedMesh_G29_CNC.gcode    -> restricted G29 bed mesh over PCB area
#   2) xx_ContourMapping_CNC.gcode -> CNC head traces PCB perimeter N times
#   3) xx_ContourMapping_Visualization.png -> visual of the positioned contour
#
# Requires: numpy, matplotlib  (pip install numpy matplotlib)
# Written by Alessandro Miertschink 2025

import os
import re
import sys
from typing import List, Tuple

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: 'matplotlib' not installed. Run: pip install matplotlib")
    sys.exit(1)

# --- CNC MACHINE (MARLIN) SETTINGS ---
BED_SIZE_X          = 240.0   # Bed width (mm)
BED_SIZE_Y          = 220.0   # Bed depth (mm)
SAFETY_PADDING      = 10.0    # PCB corner offset from origin (mm)

Z_HOMING_FINAL      = 30.0    # Z height after homing (mm)
Z_MAPPING_HEIGHT    = 5.0     # Z height during contour mapping (mm)
FAST_SPEED          = 10000   # G0 rapid speed (mm/min)
CONTOUR_SPEED       = 5000    # G1 feed speed (mm/min)
CONTOUR_REPETITIONS = 5       # Number of contour loops
BED_MESH_COMMAND    = "M420 S1 ; Enable Saved Bed Leveling"

# --- GERBER PARSING ---
COORD_REGEX        = re.compile(r'X([-]?\d+)Y([-]?\d+)')
GERBER_SCALE       = 1_000_000.0  # Raw Gerber integer → mm
FLOAT_TOLERANCE    = 1e-4


# ── Helpers ───────────────────────────────────────────────────────────────────

def _is_origin(x: float, y: float) -> bool:
    return abs(x) < FLOAT_TOLERANCE and abs(y) < FLOAT_TOLERANCE


# ── Core processing ───────────────────────────────────────────────────────────

def extract_contour_from_gerber(filepath: str) -> List[Tuple[float, float]]:
    """Read a Gerber edge-cuts file and return coordinates in mm."""
    print(f"PARSING: {filepath}")
    coords = []
    try:
        with open(filepath, 'r') as f:
            content = f.read()

        for m in COORD_REGEX.finditer(content):
            x = int(m.group(1)) / GERBER_SCALE
            y = int(m.group(2)) / GERBER_SCALE
            p = (round(x, 4), round(y, 4))

            if _is_origin(*p):
                continue
            if coords and abs(p[0] - coords[-1][0]) < FLOAT_TOLERANCE \
                      and abs(p[1] - coords[-1][1]) < FLOAT_TOLERANCE:
                continue
            coords.append(p)

    except Exception as e:
        print(f"ERROR during parsing: {e}")

    return coords


def optimize_contour_points(contour_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Position, circularly sequence, and deduplicate contour points."""
    if not contour_coords:
        return []

    arr = np.array(contour_coords)

    # 1. Shift so bottom-left lands at (SAFETY_PADDING, SAFETY_PADDING)
    offset_x = SAFETY_PADDING - arr[:, 0].min()
    offset_y = SAFETY_PADDING - arr[:, 1].min()
    print(f"CORNER POS: original min ({arr[:, 0].min():.2f}, {arr[:, 1].min():.2f})")
    print(f"CORNER POS: offset X={offset_x:.2f}, Y={offset_y:.2f} mm")
    arr[:, 0] += offset_x
    arr[:, 1] += offset_y

    # 2. Sort by angle around centroid
    cx, cy = arr.mean(axis=0)
    arr = arr[np.argsort(np.arctan2(arr[:, 1] - cy, arr[:, 0] - cx))]

    # 3. Roll so start = point closest to (min_x, min_y)
    min_x, min_y = arr[:, 0].min(), arr[:, 1].min()
    arr = np.roll(arr, -np.hypot(arr[:, 0] - min_x, arr[:, 1] - min_y).argmin(), axis=0)

    # 4. Deduplicate consecutive points
    pts = [(round(float(x), 4), round(float(y), 4)) for x, y in arr]
    clean = [pts[0]]
    for p in pts[1:]:
        if not (abs(p[0] - clean[-1][0]) < FLOAT_TOLERANCE and
                abs(p[1] - clean[-1][1]) < FLOAT_TOLERANCE):
            clean.append(p)

    # 5. Remove out-of-bounds anomalies
    valid = []
    for x, y in clean:
        if x < -5.0 or x > BED_SIZE_X + 5.0 or y < -5.0 or y > BED_SIZE_Y + 5.0:
            print(f"WARNING: Anomaly ({x:.2f}, {y:.2f}) removed.")
        else:
            valid.append((x, y))

    # 6. Close loop
    if len(valid) >= 2 and valid[0] != valid[-1]:
        valid.append(valid[0])

    print(f"OPTIMIZATION: {len(valid)} contour points")
    return valid


def calculate_mesh_bounds(coords: List[Tuple[float, float]],
                          padding: float = 5.0) -> Tuple[float, float, float, float]:
    """Return G29 bounds (X_start, Y_start, X_end, Y_end) clamped to bed size."""
    if not coords:
        raise ValueError("No coordinates for mesh bounds.")

    arr = np.array(coords)
    X_start = max(0.0,        arr[:, 0].min() - padding)
    Y_start = max(0.0,        arr[:, 1].min() - padding)
    X_end   = min(BED_SIZE_X, arr[:, 0].max() + padding)
    Y_end   = min(BED_SIZE_Y, arr[:, 1].max() + padding)

    print(f"\nBED MESH BOUNDS (G29): X:{X_start:.2f}-{X_end:.2f}, Y:{Y_start:.2f}-{Y_end:.2f} mm")
    return X_start, Y_start, X_end, Y_end


# ── G-code generators ─────────────────────────────────────────────────────────

def generate_bed_mesh_gcode(base_name: str,
                            bounds: Tuple[float, float, float, float]) -> str:
    """G-code for restricted G29 bed mesh leveling."""
    X_start, Y_start, X_end, Y_end = bounds
    lines = [
        f"; CNC Bed Mesh GCODE for PCB: {base_name} (G29)",
        "G21 ; Units set to Millimeters",
        "G90 ; Absolute Positioning",
        "\n; --- Homing and Preparation ---",
        "G28 ; Home X, Y, and Z axes",
        f"G0 Z{Z_HOMING_FINAL:.2f} F{FAST_SPEED} ; Raise Z to safety height",
        "\n; --- Execute Bed Mesh (G29) on PCB area ---",
        f"G29 L{X_start:.2f} R{X_end:.2f} F{Y_start:.2f} B{Y_end:.2f} ; Restricted probing",
        "\n; --- Finalization ---",
        "M500 ; Save Bed Mesh to EEPROM",
        f"G0 Z{Z_HOMING_FINAL:.2f} F{FAST_SPEED} ; Raise Z for safety",
        "M84 ; Disable Motors",
    ]
    return "\n".join(lines)


def generate_contour_gcode(base_name: str,
                           contour: List[Tuple[float, float]]) -> str:
    """G-code for repetitive contour mapping path."""
    lines = [
        f"; CNC Contour Mapping GCODE for PCB: {base_name}",
        "G21 ; Units set to Millimeters",
        "G90 ; Absolute Positioning",
        "\n; --- Homing and Safety Height ---",
        "G28 ; Home X, Y, and Z axes",
        f"G0 Z{Z_HOMING_FINAL:.2f} F{FAST_SPEED} ; Raise Z after Homing",
        "\n; --- Bed Leveling Compensation ---",
        BED_MESH_COMMAND,
        f"\n; --- Circular Mapping Contour ({CONTOUR_REPETITIONS} Loops) ---",
    ]

    if len(contour) < 2:
        lines.append("; ERROR: Empty contour. Aborting.")
        return "\n".join(lines)

    start_x, start_y = contour[0]
    lines.append(f"G0 X{start_x:.2f} Y{start_y:.2f} F{FAST_SPEED}")
    lines.append(f"G1 Z{Z_MAPPING_HEIGHT:.2f} F{CONTOUR_SPEED}")

    for i in range(CONTOUR_REPETITIONS):
        lines.append(f"\n; START OF LOOP {i + 1}")
        for x, y in contour[1:]:
            lines.append(f"G1 X{x:.2f} Y{y:.2f} F{CONTOUR_SPEED}")
        lines.append(f"; END OF LOOP {i + 1}")

    lines += [
        "\n; --- Finalization ---",
        f"G0 Z{Z_HOMING_FINAL:.2f} F{FAST_SPEED} ; Raise Z to safety height",
        "M84 ; Disable Motors",
    ]
    return "\n".join(lines)


# ── Visualization ─────────────────────────────────────────────────────────────

def create_contour_png(contour: List[Tuple[float, float]], base_name: str):
    """Save a 2D plot of the optimized contour as PNG."""
    if not contour:
        print("WARNING: Empty contour. Cannot generate PNG.")
        return

    xs = [p[0] for p in contour]
    ys = [p[1] for p in contour]

    fig, ax = plt.subplots()
    ax.plot(xs, ys, marker='o', linestyle='-', color='blue',
            linewidth=2, markersize=4, label='Sequenced Contour')
    ax.plot(xs[0], ys[0], marker='o', color='red', markersize=6, label='Start Point')
    ax.set_title(f"Optimized Contour: {base_name}")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend()

    output = f"{base_name}_ContourMapping_Visualization.png"
    plt.savefig(output, bbox_inches='tight', dpi=300)
    plt.close(fig)
    print(f"VISUALIZATION: saved as '{output}'")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    base_name = input("Enter base file name (e.g., 'ProDOS ROM-Drive'): ").strip()
    if not base_name:
        return

    filepath = os.path.join(os.getcwd(), f"{base_name}-Edge_Cuts.gbr")
    if not os.path.exists(filepath):
        print(f"ERROR: '{filepath}' not found.")
        return

    # 1. Extract raw contour
    raw_coords = extract_contour_from_gerber(filepath)
    if not raw_coords:
        print("ERROR: Failed to extract contour coordinates.")
        return

    # 2. Optimize (position, sequence, clean)
    optimized = optimize_contour_points(raw_coords)

    # 3. Bed Mesh G-code (G29)
    try:
        bounds   = calculate_mesh_bounds(optimized)
        gcode    = generate_bed_mesh_gcode(base_name, bounds)
        out_file = f"{base_name}_BedMesh_G29_CNC.gcode"
        with open(out_file, 'w') as f:
            f.write(gcode)
        print(f"\nSUCCESS: Bed Mesh G-Code saved as '{out_file}'")
    except ValueError as e:
        print(f"ERROR: {e}")
        return

    # 4. Contour Mapping G-code
    gcode    = generate_contour_gcode(base_name, optimized)
    out_file = f"{base_name}_ContourMapping_CNC.gcode"
    try:
        with open(out_file, 'w') as f:
            f.write(gcode)
        print(f"SUCCESS: Contour G-Code saved as '{out_file}' ({len(optimized)} points)")
    except Exception as e:
        print(f"ERROR saving contour G-code: {e}")
        return

    # 5. PNG visualization
    create_contour_png(optimized, base_name)


if __name__ == "__main__":
    main()
