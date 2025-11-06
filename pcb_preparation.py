#!/usr/bin/python3
# Generates CNC tool paths (G-code) from a single-sided KiCad PCB layout.
#
# Tree Files will be exported:
#
# 1) xx_BedMesh_G29_CNC -> with G-code to 'bed mesh' the area of PCB
# 2) xx_CountourMapping_CNC -> with G-code to move the CNC head around the perimeter of the PCB
# 3) xx_ContourMapping_Visualization -> a PNG to visualized the PCB size and placement on the CNC
#
# Requires the 'numpy' library (pip install numpy).
# Written by Alessandro Miertschink 2025, with use of Google-Gemini

import os
import numpy as np
import re
from typing import List, Tuple

# --- DEPENDENCIES CHECK (LIBRARIES MUST BE INSTALLED) ---
try:
    import numpy
except ImportError:
    print("ERROR: 'numpy' package not installed. Install with 'pip install numpy'.")
    raise SystemExit(1)

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: 'matplotlib' package not installed. Install with 'pip install matplotlib'.")
    raise SystemExit(1)

# --- CNC MACHINE (MARLIN) SETTINGS ---
BED_SIZE_X = 240.0  # Bed width in mm
BED_SIZE_Y = 220.0  # Bed depth in mm
SAFETY_PADDING = 10.0  # Used for corner positioning (e.g., use 10 for x10,y10 start)

Z_HOMING_FINAL = 30.0  # Z height after homing
Z_MAPPING_HEIGHT = 5.0  # Z height for contour mapping
FAST_SPEED = 10000  # G0 (rapid) movement speed
CONTOUR_SPEED = 5000  # G1 (feed) movement speed
CONTOUR_REPETITIONS = 5  # Number of times to loop the contour
BED_MESH_COMMAND = "M420 S1 ; Enable Saved Bed Leveling"
MM_PER_UNIT = 1.0  # Scale factor, usually 1.0

# --- GERBER PARSING SETTINGS ---
COORD_REGEX = re.compile(r'X([-]?\d+)Y([-]?\d+)')
GERBER_SCALE_FACTOR = 1000000.0  # CRITICAL: Factor to convert raw Gerber integers to MM
FLOAT_TOLERANCE = 1e-4  # Tolerance for float comparison


def is_close_to_zero(x, y):
    """Checks if the coordinate (x, y) is close to (0, 0) within tolerance."""
    return abs(x) < FLOAT_TOLERANCE and abs(y) < FLOAT_TOLERANCE


def optimize_contour_points(contour_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Positions, sequences, and cleans the contour points. Input coords MUST be in MM.
    """
    if not contour_coords:
        return []

    coords_array = np.array(contour_coords)

    # --- 0. CALCULATE OFFSET FOR CORNER POSITIONING ---
    min_x_raw = np.min(coords_array[:, 0])
    min_y_raw = np.min(coords_array[:, 1])

    start_x_target = 0.0
    start_y_target = 0.0

    offset_x = start_x_target - min_x_raw + SAFETY_PADDING
    offset_y = start_y_target - min_y_raw + SAFETY_PADDING

    print(f"CORNER POS: Original min X/Y: ({min_x_raw:.2f}, {min_y_raw:.2f})")
    print(f"CORNER POS: Applying offset X: {offset_x:.2f}, Y: {offset_y:.2f} mm")

    # Apply the offset
    coords_array[:, 0] += offset_x
    coords_array[:, 1] += offset_y

    # --- 1. Circular Ordering ---
    center_x_new = np.mean(coords_array[:, 0])
    center_y_new = np.mean(coords_array[:, 1])
    angles = np.arctan2(coords_array[:, 1] - center_y_new, coords_array[:, 0] - center_x_new)
    sorted_indices = np.argsort(angles)
    circular_coords_array = coords_array[sorted_indices]

    # --- 2. Normalize start point to the closest corner ---
    min_x_new = np.min(circular_coords_array[:, 0])
    min_y_new = np.min(circular_coords_array[:, 1])
    distances = np.sqrt((circular_coords_array[:, 0] - min_x_new) ** 2 + (circular_coords_array[:, 1] - min_y_new) ** 2)
    start_index = np.argmin(distances)
    optimized_list_array = np.roll(circular_coords_array, -start_index, axis=0)

    # --- 3. Filter Duplicates and Anomalies ---
    optimized_list = [(np.round(x, 4), np.round(y, 4)) for x, y in optimized_list_array.tolist()]

    temp_filtered_list = []
    if optimized_list:
        temp_filtered_list.append(optimized_list[0])
        for coord in optimized_list[1:]:
            # Check for close proximity duplicates
            if not (np.isclose(coord[0], temp_filtered_list[-1][0], atol=FLOAT_TOLERANCE) and
                    np.isclose(coord[1], temp_filtered_list[-1][1], atol=FLOAT_TOLERANCE)):
                temp_filtered_list.append(coord)

    # Final removal of points outside expected bounds
    final_clean_list = []
    for x, y in temp_filtered_list:
        if x < -5.0 or x > BED_SIZE_X + 5.0 or y < -5.0 or y > BED_SIZE_Y + 5.0:
            print(f"WARNING: Anomaly ({x:.2f}, {y:.2f}) removed.")
        else:
            final_clean_list.append((x, y))

    # --- 4. Close the Loop ---
    if len(final_clean_list) < 2:
        return final_clean_list

    start_point = final_clean_list[0]
    end_point = final_clean_list[-1]

    # Close the loop if start and end points are not close
    if not (np.isclose(start_point[0], end_point[0], atol=FLOAT_TOLERANCE) and
            np.isclose(start_point[1], end_point[1], atol=FLOAT_TOLERANCE)):
        final_clean_list.append(start_point)

    print(f"OPTIMIZATION: Contour cleaned and closed. Total points: {len(final_clean_list)}")
    return final_clean_list


def extract_contour_from_gerber(contour_filepath: str) -> List[Tuple[float, float]]:
    """
    Reads the Gerber file, extracts X/Y coords, and SCALES THEM TO MM.
    """
    print(f"PARSING: Reading file: {contour_filepath}")
    contour_coords = []
    try:
        with open(contour_filepath, 'r') as f:
            content = f.read()

        for match in COORD_REGEX.finditer(content):
            # Convert raw integer from Gerber to MM
            x = int(match.group(1)) / GERBER_SCALE_FACTOR * MM_PER_UNIT
            y = int(match.group(2)) / GERBER_SCALE_FACTOR * MM_PER_UNIT

            coord = (np.round(x, 4), np.round(y, 4))

            # Filter (0.0, 0.0) and consecutive duplicates
            if not is_close_to_zero(coord[0], coord[1]):
                if not contour_coords or not (np.isclose(coord[0], contour_coords[-1][0], atol=FLOAT_TOLERANCE) and
                                              np.isclose(coord[1], contour_coords[-1][1], atol=FLOAT_TOLERANCE)):
                    contour_coords.append(coord)

        return contour_coords

    except Exception as e:
        print(f"ERROR during parsing: {e}")
        return []


def create_contour_png(contour: List[Tuple[float, float]], base_name: str):
    """
    Creates a 2D plot of the optimized contour and saves it as a PNG file.
    """
    if not contour:
        print("WARNING: Empty contour. Cannot generate PNG.")
        return

    x_coords = [coord[0] for coord in contour]
    y_coords = [coord[1] for coord in contour]

    fig, ax = plt.subplots()

    # Plot the contour path
    ax.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue', linewidth=2, markersize=4,
            label='Sequenced Contour')

    # Highlight the start point
    ax.plot(x_coords[0], y_coords[0], marker='o', color='red', markersize=6, label='Start Point')

    ax.set_title(f"Optimized Contour Visualization: {base_name}")
    ax.set_xlabel("X Axis (mm)")
    ax.set_ylabel("Y Axis (mm)")

    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend()

    output_png = f"{base_name}_ContourMapping_Visualization.png"
    plt.savefig(output_png, bbox_inches='tight', dpi=300)
    plt.close(fig)

    print(f"VISUALIZATION: Contour PNG saved as: {output_png}")


def calculate_mesh_bounds(optimized_coords: List[Tuple[float, float]], mesh_padding: float = 5.0) -> Tuple[
    float, float, float, float]:
    """
    Calculates G29 bounds (L, F, R, B) using positioned coordinates plus padding.
    """
    if not optimized_coords:
        raise ValueError("No coordinates found for mesh bounds.")

    coords_array = np.array(optimized_coords)

    # Calculate min/max of the positioned PCB
    min_x = np.min(coords_array[:, 0])
    max_x = np.max(coords_array[:, 0])
    min_y = np.min(coords_array[:, 1])
    max_y = np.max(coords_array[:, 1])

    # Apply padding and clamp within bed limits
    X_start = max(0.0, min_x - mesh_padding)
    Y_start = max(0.0, min_y - mesh_padding)
    X_end = min(BED_SIZE_X, max_x + mesh_padding)
    Y_end = min(BED_SIZE_Y, max_y + mesh_padding)

    print(f"\nBED MESH BOUNDS (G29): X: {X_start:.2f}-{X_end:.2f}, Y: {Y_start:.2f}-{Y_end:.2f} mm")

    return X_start, Y_start, X_end, Y_end


def generate_bed_mesh_gcode(base_name: str, bounds: Tuple[float, float, float, float]) -> str:
    """
    Generates G-Code for restricted G29 Bed Mesh leveling.
    """
    X_start, Y_start, X_end, Y_end = bounds
    gcode_lines = []

    gcode_lines.append(f"; CNC Bed Mesh GCODE for PCB: {base_name} (G29)")
    gcode_lines.append("G21 ; Units set to Millimeters")
    gcode_lines.append("G90 ; Absolute Positioning")

    gcode_lines.append("\n; --- Homing and Preparation ---")
    gcode_lines.append("G28 ; Home X, Y, and Z axes")
    gcode_lines.append(f"G0 Z{Z_HOMING_FINAL:.2f} F{FAST_SPEED} ; Raise Z to safety height")

    gcode_lines.append("\n; --- Execute Bed Mesh (G29) on PCB area ---")

    # G29 command with bounds (Left, Right, Front, Back)
    gcode_lines.append(
        f"G29 L{X_start:.2f} R{X_end:.2f} F{Y_start:.2f} B{Y_end:.2f} ; Perform restricted probing"
    )

    gcode_lines.append("\n; --- Finalization ---")
    gcode_lines.append("M500 ; Save Bed Mesh to EEPROM")
    gcode_lines.append(f"G0 Z{Z_HOMING_FINAL:.2f} F{FAST_SPEED} ; Raise Z for safety")
    gcode_lines.append("M84 ; Disable Motors")

    return "\n".join(gcode_lines)


def generate_circular_contour_gcode(base_name: str, contour: List[Tuple[float, float]]) -> str:
    """
    Generates G-Code for the repetitive contour mapping path.
    """
    gcode_lines = []
    gcode_lines.append(f"; CNC Contour Mapping GCODE for PCB: {base_name}")
    gcode_lines.append("G21 ; Units set to Millimeters")
    gcode_lines.append("G90 ; Absolute Positioning")
    gcode_lines.append("\n; --- Homing and Safety Height ---")
    gcode_lines.append("G28 ; Home X, Y, and Z axes")
    gcode_lines.append(f"G0 Z{Z_HOMING_FINAL:.2f} F{FAST_SPEED} ; Raise Z after Homing")
    gcode_lines.append("\n; --- Bed Leveling Compensation ---")
    gcode_lines.append(BED_MESH_COMMAND)
    gcode_lines.append(f"\n; --- Circular Mapping Contour ({CONTOUR_REPETITIONS} Loops) ---")

    if not contour or len(contour) < 2:
        gcode_lines.append("; ERROR: Empty contour. Aborting.")
        return "\n".join(gcode_lines)

    start_x, start_y = contour[0]

    # 1. Rapid move to the start corner (P1)
    gcode_lines.append(f"G0 X{start_x:.2f} Y{start_y:.2f} F{FAST_SPEED}")
    # 2. Lower Z to mapping height
    gcode_lines.append(f"G1 Z{Z_MAPPING_HEIGHT:.2f} F{CONTOUR_SPEED}")

    # Loop for repetition
    for i in range(CONTOUR_REPETITIONS):
        gcode_lines.append(f"\n; START OF LOOP {i + 1}")

        # Start from index 1 (P2)
        for x, y in contour[1:]:
            # Movement command (XNN.NN YNN.NN in MM)
            gcode_lines.append(f"G1 X{x:.2f} Y{y:.2f} F{CONTOUR_SPEED}")

        gcode_lines.append(f"; END OF LOOP {i + 1}")

    # Finalization
    gcode_lines.append("\n; --- Finalization ---")
    gcode_lines.append(f"G0 Z{Z_HOMING_FINAL:.2f} F{FAST_SPEED} ; Raise Z to safety height")
    gcode_lines.append("M84 ; Disable Motors")

    return "\n".join(gcode_lines)


# ----------------- MAIN ROUTINE -----------------

def main_routine():
    """
    Main entry point for the routine.
    """
    base_name = input("Enter base file name (e.g., 'ProDOS ROM-Drive'): ").strip()

    if not base_name: return

    contour_filename = f"{base_name}-Edge_Cuts.gbr"
    contour_filepath = os.path.join(os.getcwd(), contour_filename)

    if not os.path.exists(contour_filepath):
        print(f"ERROR: Contour file '{contour_filename}' not found.")
        return

    # 1. Extract raw points
    raw_coords = extract_contour_from_gerber(contour_filepath)

    if not raw_coords:
        print("ERROR: Failed to extract contour coordinates.")
        return

    # 2. OPTIMIZE (Positioning, Sequencing, Cleaning)
    optimized_coords = optimize_contour_points(raw_coords)

    # 3. GENERATE BED MESH G-CODE (G29)
    try:
        mesh_bounds = calculate_mesh_bounds(optimized_coords)
        mesh_gcode = generate_bed_mesh_gcode(base_name, mesh_bounds)
        mesh_output_filename = f"{base_name}_BedMesh_G29_CNC.gcode"

        with open(mesh_output_filename, 'w') as f:
            f.write(mesh_gcode)
        print(f"\nSUCCESS: Bed Mesh G-Code saved as: {mesh_output_filename}")

    except ValueError as e:
        print(f"ERROR: Failed to generate Bed Mesh G-Code: {e}")
        return

    # 4. GENERATE CONTOUR MAPPING G-CODE
    final_gcode = generate_circular_contour_gcode(base_name, optimized_coords)
    contour_output_filename = f"{base_name}_ContourMapping_CNC.gcode"

    try:
        with open(contour_output_filename, 'w') as f:
            f.write(final_gcode)
        print(f"\nSUCCESS: CNC Mapping G-Code saved as: {contour_output_filename}")
        print(f"Points used: {len(optimized_coords)}")

        # 5. EXPORT VISUALIZATION (PNG)
        create_contour_png(optimized_coords, base_name)

    except Exception as e:
        print(f"\nERROR saving file or generating PNG: {e}")


if __name__ == "__main__":
    main_routine()