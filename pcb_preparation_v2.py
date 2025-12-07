#!/usr/bin/python3
# Generates CNC tool paths (G-code) from a single-sided KiCad PCB layout.
# CENTERED VERSION with VERIFICATION GCODE & PADDED MESH VISUALIZATION.
#
# Four Files will be exported:
# 1) xx_BedMesh_G29_CNC -> G-code to 'bed mesh' the area of PCB (+2mm padding)
# 2) xx_CountourMapping_CNC -> G-code to move the CNC head around the perimeter
# 3) xx_CheckPos_CNC -> G-code to verify PCB alignment (moves to Start Point)
# 4) xx_ContourMapping_Visualization -> PNG visualizing placement + Mesh Area + Start Point
#
# Requires 'numpy' and 'matplotlib' libraries.

import os
import numpy as np
import re
from typing import List, Tuple
import matplotlib.pyplot as plt

# --- DEPENDENCIES CHECK ---
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

# --- CNC MACHINE & PCB PLACEMENT SETTINGS ---

# Target CENTER point for the PCB placement.
BED_MID_X = 100.0
BED_MID_Y = 140.0

# Bed Mesh padding around the PCB area (for the G29 command)
MESH_PADDING = 2.0  # mm (Added 2mm margin as requested)

Z_HOMING_FINAL_HEIGHT = 20.0  # Z height after homing
Z_MAPPING_HEIGHT = 20.0  # Z height for contour mapping and Z parking on start point
FAST_SPEED = 5000  # G0 speed
CONTOUR_SPEED = 2000  # G1 speed
CONTOUR_REPETITIONS = 5  # Loop count
BED_MESH_COMMAND = "M420 S1 ; Enable Saved Bed Leveling"
MM_PER_UNIT = 1.0  # Scale factor

# --- GERBER PARSING SETTINGS ---
COORD_REGEX = re.compile(r'X([-]?\d+)Y([-]?\d+)')
GERBER_SCALE_FACTOR = 1000000.0
FLOAT_TOLERANCE = 1e-4


def is_close_to_zero(x, y):
    return abs(x) < FLOAT_TOLERANCE and abs(y) < FLOAT_TOLERANCE


def optimize_contour_points(contour_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Positions the PCB centered at BED_MID_X/Y.
    """
    if not contour_coords:
        return []

    coords_array = np.array(contour_coords)

    # --- 0. CALCULATE OFFSET FOR CENTERING ---
    min_x_raw = np.min(coords_array[:, 0])
    max_x_raw = np.max(coords_array[:, 0])
    min_y_raw = np.min(coords_array[:, 1])
    max_y_raw = np.max(coords_array[:, 1])

    raw_center_x = (min_x_raw + max_x_raw) / 2.0
    raw_center_y = (min_y_raw + max_y_raw) / 2.0

    offset_x = BED_MID_X - raw_center_x
    offset_y = BED_MID_Y - raw_center_y

    print(f"CENTERING: Applying Offset X: {offset_x:.2f}, Y: {offset_y:.2f} mm")

    coords_array[:, 0] += offset_x
    coords_array[:, 1] += offset_y

    # --- 1. Circular Ordering ---
    center_x_new = np.mean(coords_array[:, 0])
    center_y_new = np.mean(coords_array[:, 1])
    angles = np.arctan2(coords_array[:, 1] - center_y_new, coords_array[:, 0] - center_x_new)
    sorted_indices = np.argsort(angles)
    circular_coords_array = coords_array[sorted_indices]

    # --- 2. Normalize start point ---
    distances = np.sqrt(circular_coords_array[:, 0] ** 2 + circular_coords_array[:, 1] ** 2)
    start_index = np.argmin(distances)
    optimized_list_array = np.roll(circular_coords_array, -start_index, axis=0)

    # --- 3. Filter Duplicates ---
    optimized_list = [(np.round(x, 4), np.round(y, 4)) for x, y in optimized_list_array.tolist()]
    temp_filtered_list = []
    if optimized_list:
        temp_filtered_list.append(optimized_list[0])
        for coord in optimized_list[1:]:
            if not (np.isclose(coord[0], temp_filtered_list[-1][0], atol=FLOAT_TOLERANCE) and
                    np.isclose(coord[1], temp_filtered_list[-1][1], atol=FLOAT_TOLERANCE)):
                temp_filtered_list.append(coord)

    # --- 4. Close the Loop ---
    final_clean_list = list(temp_filtered_list)
    if len(final_clean_list) >= 2:
        start_point = final_clean_list[0]
        end_point = final_clean_list[-1]
        if not (np.isclose(start_point[0], end_point[0], atol=FLOAT_TOLERANCE) and
                np.isclose(start_point[1], end_point[1], atol=FLOAT_TOLERANCE)):
            final_clean_list.append(start_point)

    return final_clean_list


def extract_contour_from_gerber(contour_filepath: str) -> List[Tuple[float, float]]:
    print(f"PARSING: Reading file: {contour_filepath}")
    contour_coords = []
    try:
        with open(contour_filepath, 'r') as f:
            content = f.read()

        for match in COORD_REGEX.finditer(content):
            x = int(match.group(1)) / GERBER_SCALE_FACTOR * MM_PER_UNIT
            y = int(match.group(2)) / GERBER_SCALE_FACTOR * MM_PER_UNIT
            coord = (np.round(x, 4), np.round(y, 4))

            if not is_close_to_zero(coord[0], coord[1]):
                if not contour_coords or not (np.isclose(coord[0], contour_coords[-1][0], atol=FLOAT_TOLERANCE) and
                                              np.isclose(coord[1], contour_coords[-1][1], atol=FLOAT_TOLERANCE)):
                    contour_coords.append(coord)
        return contour_coords
    except Exception as e:
        print(f"ERROR during parsing: {e}")
        return []


def calculate_mesh_bounds(optimized_coords: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    """
    Calculates G29 bounds with PADDING.
    """
    coords_array = np.array(optimized_coords)
    min_x = np.min(coords_array[:, 0])
    max_x = np.max(coords_array[:, 0])
    min_y = np.min(coords_array[:, 1])
    max_y = np.max(coords_array[:, 1])

    # Apply padding (clamped to 0.0)
    X_start = max(0.0, min_x - MESH_PADDING)
    Y_start = max(0.0, min_y - MESH_PADDING)
    X_end = max_x + MESH_PADDING
    Y_end = max_y + MESH_PADDING

    print(f"BED MESH BOUNDS (+{MESH_PADDING}mm): X: {X_start:.2f}-{X_end:.2f}, Y: {Y_start:.2f}-{Y_end:.2f}")
    return X_start, Y_start, X_end, Y_end


def create_contour_png(contour: List[Tuple[float, float]], mesh_bounds: Tuple[float, float, float, float],
                       base_name: str):
    """
    Creates a PNG visualizing the PCB, Start Point, and the BED MESH AREA.
    """
    if not contour: return

    x_coords = [coord[0] for coord in contour]
    y_coords = [coord[1] for coord in contour]
    start_x, start_y = contour[0]

    # Unpack mesh bounds for drawing
    mx_start, my_start, mx_end, my_end = mesh_bounds

    fig, ax = plt.subplots(figsize=(10, 10))

    # 1. Plot PCB Contour
    ax.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue', linewidth=2, markersize=3, label='PCB Contour')

    # 2. Plot Start Point
    ax.plot(start_x, start_y, marker='o', color='red', markersize=8, label='Start Point')

    # 3. Plot Bed Center
    ax.plot(BED_MID_X, BED_MID_Y, marker='x', color='green', markersize=10, markeredgewidth=2, label='Bed Center')

    # 4. Plot BED MESH AREA (The requested Rectangle)
    # We define the 5 points to close the rectangle: BL -> TL -> TR -> BR -> BL
    rect_x = [mx_start, mx_start, mx_end, mx_end, mx_start]
    rect_y = [my_start, my_end, my_end, my_start, my_start]
    ax.plot(rect_x, rect_y, color='orange', linestyle='--', linewidth=2, label=f'Bed Mesh Area (+{MESH_PADDING}mm)')

    # Axis Limits Calculation
    min_x = np.min(x_coords)
    max_x = np.max(x_coords)
    min_y = np.min(y_coords)
    max_y = np.max(y_coords)

    # Ensure bounds are wide enough to see the mesh box and origin
    plot_min_x = min(0, mx_start) - 20
    plot_min_y = min(0, my_start) - 20
    plot_max_x = max(BED_MID_X * 1.5, mx_end) + 20
    plot_max_y = max(BED_MID_Y * 1.5, my_end) + 20

    ax.set_xlim(plot_min_x, plot_max_x)
    ax.set_ylim(plot_min_y, plot_max_y)

    # Reference lines
    ax.axvline(0, color='gray', linestyle='--', linewidth=1)
    ax.axhline(0, color='gray', linestyle='--', linewidth=1, label='Origin (0,0)')

    ax.set_title(f"PCB Visualization: {base_name}\nCenter Target: ({BED_MID_X}, {BED_MID_Y})")
    ax.set_xlabel("X Axis (mm)")
    ax.set_ylabel("Y Axis (mm)")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='upper right')

    # Footer Text
    footer_text = f"START POINT: X={start_x:.2f}, Y={start_y:.2f} | MESH: X{mx_start:.1f}-{mx_end:.1f} Y{my_start:.1f}-{my_end:.1f}"
    fig.text(0.5, 0.02, footer_text, ha='center', fontsize=11,
             bbox=dict(facecolor='#f0f0f0', edgecolor='black', boxstyle='round,pad=0.5'))

    plt.subplots_adjust(bottom=0.1)
    output_png = f"{base_name}_ContourMapping_Visualization.png"
    plt.savefig(output_png, bbox_inches='tight', dpi=150)
    plt.close(fig)
    print(f"VISUALIZATION: Saved {output_png} (Shows Mesh Area)")


def generate_bed_mesh_gcode(base_name: str, bounds: Tuple[float, float, float, float]) -> str:
    X_start, Y_start, X_end, Y_end = bounds
    gcode_lines = [
        f"; CNC Bed Mesh GCODE for PCB: {base_name}",
        "M502" # Factory Default
        "M500" # Save Eeprom
        "G21", "G90",
        "G28 ; Home All",
        f"G0 Z{Z_HOMING_FINAL_HEIGHT:.2f} F{FAST_SPEED}",
        f"G29 L{X_start:.2f} R{X_end:.2f} F{Y_start:.2f} B{Y_end:.2f} ; Probing Area (+{MESH_PADDING}mm)",
        "M500",
        f"G0 Z{Z_HOMING_FINAL_HEIGHT:.2f} F{FAST_SPEED}",
        "M84 ; Disable Steppers",
        "M30 ; Program End"
    ]
    return "\n".join(gcode_lines)


def generate_circular_contour_gcode(base_name: str, contour: List[Tuple[float, float]]) -> str:
    if not contour: return ""
    start_x, start_y = contour[0]
    gcode_lines = [
        f"; CNC Contour GCODE: {base_name}",
        "G21", "G90",
        "G28 ; Home All",
        f"G0 Z{Z_HOMING_FINAL_HEIGHT:.2f} F{FAST_SPEED}",
        BED_MESH_COMMAND,
        f"G0 X{start_x:.2f} Y{start_y:.2f} F{FAST_SPEED}",
        f"G1 Z{Z_MAPPING_HEIGHT:.2f} F{CONTOUR_SPEED}"
    ]
    for i in range(CONTOUR_REPETITIONS):
        gcode_lines.append(f"; Loop {i + 1}")
        for x, y in contour[1:]:
            gcode_lines.append(f"G1 X{x:.2f} Y{y:.2f} F{CONTOUR_SPEED}")

    gcode_lines.extend([
        f"G0 Z{Z_HOMING_FINAL_HEIGHT:.2f} F{FAST_SPEED}",
        "M84 ; Disable Steppers",
        "M30 ; Program End"
    ])
    return "\n".join(gcode_lines)


def generate_verify_gcode(base_name: str, start_point: Tuple[float, float]) -> str:
    """
    Generates G-code to verify PCB start position.
    """
    x, y = start_point
    gcode_lines = [
        f"; Verification GCODE for: {base_name}",
        "G21",
        "G90",
        "G28 X Y Z      ; Home all axes",
        f"G0 X{x:.2f} Y{y:.2f} ; Move to Start Point",
        f"G0 Z{Z_MAPPING_HEIGHT:.2f}        ; Drop Z to verification height",
        "M84            ; Disable Steppers",
        "M30            ; Program End"
    ]
    return "\n".join(gcode_lines)


def main_routine():
    base_name = input("Enter base file name (e.g., 'MyBoard'): ").strip()
    if not base_name: return

    contour_filename = f"{base_name}-Edge_Cuts.gbr"
    contour_filepath = os.path.join(os.getcwd(), contour_filename)

    if not os.path.exists(contour_filepath):
        print(f"ERROR: Contour file '{contour_filename}' not found.")
        return

    # 1. Extract and Optimize
    raw_coords = extract_contour_from_gerber(contour_filepath)
    if not raw_coords: return
    optimized_coords = optimize_contour_points(raw_coords)

    # 2. Calculate Mesh Bounds (Needed for G-code AND PNG)
    try:
        mesh_bounds = calculate_mesh_bounds(optimized_coords)
    except ValueError as e:
        print(f"Error calculating bounds: {e}")
        return

    # 3. Generate Bed Mesh File
    try:
        with open(f"{base_name}_BedMesh_G29_CNC.gcode", 'w') as f:
            f.write(generate_bed_mesh_gcode(base_name, mesh_bounds))
    except Exception as e:
        print(f"Error saving Mesh Gcode: {e}")

    # 4. Generate Contour Map File
    try:
        final_gcode = generate_circular_contour_gcode(base_name, optimized_coords)
        with open(f"{base_name}_ContourMapping_CNC.gcode", 'w') as f:
            f.write(final_gcode)
    except Exception as e:
        print(f"Error saving Contour Gcode: {e}")

    # 5. Generate Verification File
    try:
        if optimized_coords:
            start_point = optimized_coords[0]
            verify_gcode = generate_verify_gcode(base_name, start_point)
            with open(f"{base_name}_CheckPos_CNC.gcode", 'w') as f:
                f.write(verify_gcode)
    except Exception as e:
        print(f"Error saving Verify Gcode: {e}")

    # 6. Generate Visualization (Now includes Mesh Bounds)
    create_contour_png(optimized_coords, mesh_bounds, base_name)
    print("\nDONE: All files generated.")


if __name__ == "__main__":
    main_routine()