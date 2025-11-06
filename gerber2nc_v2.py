#!/usr/bin/python3
# Generates CNC tool paths (G-code) from a single-sided KiCad PCB layout.
# Supports traces, holes, and pads (no rotated pads).
# Requires the 'shapely' and 'numpy' library (pip install shapely / pip install numpy).
#
# Based on Matthias Wandel August 2025
# Improved by Alessandro Miertschink 2025, with use of Google-Gemini

import sys, re
from shapely.geometry import LineString, MultiLineString, Point, box
from shapely.ops import unary_union
import os
from PIL import Image, ImageDraw
import numpy as np
from typing import List, Tuple

# --- CONFIGURATION PARAMETERS ---

# PCB & Optimization
SAFETY_PADDING = 5.0  # Space from origin (0,0) for the PCB's bottom-left corner.
FLOAT_TOLERANCE = 1e-4  # Tolerance for float comparison (e.g., duplicate points)
SMALL_HOLE_MAX_DIAMETER = 0.85  # Max diameter (mm) for 'small' drill group

# CNC Milling Parameters
SPINDLE_SPEED = 12000  # RPM
CUT_DEPTH = -0.1  # mm (Trace isolation depth)
FINAL_CUT_DEPTH = -0.5  # mm (Final edge cut depth)
SAFE_HEIGHT = 5.0  # mm above workpiece
PLUNGE_FEED_RATE = 100  # mm/min (Z axis feed rate)
FEED_RATE = 300  # mm/min (XY trace feed rate)
CUT_FEED_RATE = 200  # mm/min (XY final cut feed rate)
HOLE_START_DEPTH = 0.1  # Depth before slow drilling
HOLE_FINAL_DEPTH = -1.8  # Final hole depth

# Trace Isolation Path Calculation
ISOLATION_OFFSET = 0.22  # mm (Distance from trace center for first pass)
ISOLATION_PASSES = 3  # Number of passes for isolation
PASS_SPACING = 0.2  # mm (Spacing between isolation passes)

# NEW PARAMETER FOR EDGE CUT
EDGE_CUT_CLEARANCE_OFFSET = 1.0  # mm (Positive value to mill the outline bigger)


# --------------------------------

class GerberTracesParser:
    def __init__(self, filename: str):
        self.apertures: dict = {}
        self.current_aperture: int = -1
        self.traces: list = []
        self.pads: list = []
        self.unit_mult: float = 1.0
        self.current_x: float = -1000.0
        self.current_y: float = -1000.0

        try:
            self._parse_gerber_file(filename)
        except FileNotFoundError:
            print(f"Error: Trace file '{filename}' not found.")
            sys.exit(1)

    def _process_extended_command(self, line: str):
        # Process extended commands (starting with %)
        if not line: return

        # Aperture definition
        aperture_match = re.match(r'%ADD(\d+)([^,]+),([^*]+)\*%', line)
        if aperture_match:
            aperture_num = int(aperture_match.group(1))
            aperture_type = aperture_match.group(2)
            params = aperture_match.group(3).split('X')

            if aperture_type == 'C':  # Circle
                diameter = float(params[0])
                self.apertures[aperture_num] = {'type': 'circle', 'diameter': diameter}
            elif aperture_type == 'R':  # Rectangle
                width = float(params[0])
                height = float(params[1]) if len(params) > 1 else width
                self.apertures[aperture_num] = {'type': 'rectangle', 'width': width, 'height': height}
            elif aperture_type == 'RoundRect':  # Rounded Rectangle - treated as rectangle
                width = abs(float(params[1])) + abs(float(params[3])) + float(params[0])
                height = abs(float(params[2])) + abs(float(params[4])) + float(params[0])
                self.apertures[aperture_num] = {'type': 'rectangle', 'width': width, 'height': height}

        # Units
        if 'MOMM*%' in line:
            self.unit_mult = 1
        elif 'MOIN*%' in line:
            self.unit_mult = 25.4

    def _process_command(self, line: str):
        # Process regular Gerber commands
        global x_min, x_max, y_min, y_max

        line = line.rstrip('*')

        # Aperture selection
        aperture_match = re.match(r'D(\d+)', line)
        if aperture_match:
            aperture_num = int(aperture_match.group(1))
            if aperture_num >= 10:  # D10+ are user-defined apertures
                self.current_aperture = aperture_num
            return

        # Coordinate and operation commands
        coord_match = re.match(r'X(-?[0-9.]+)Y(-?[0-9.]+)D0([0123])?', line)
        if coord_match:
            # Parse coords. KiCad uses 1 million units per mm!
            x = float(coord_match.group(1)) * 0.000001 * self.unit_mult
            y = float(coord_match.group(2)) * 0.000001 * self.unit_mult
            operation = int(coord_match.group(3))

            # Compute extents
            m = 1.5 if operation == 3 else 0.6
            if x - m < x_min: x_min = x - m
            if x + m > x_max: x_max = x + m
            if y - m < y_min: y_min = y - m
            if y + m > y_max: y_max = y + m

            # Process operation
            if operation == 1:  # Linear interpolation with exposure on (Trace)
                if self.current_aperture in self.apertures:
                    aperture = self.apertures[self.current_aperture]
                    width = aperture.get('diameter', aperture.get('width', 0.1))
                    self.traces.append([(self.current_x, self.current_y), (x, y), width])
            elif operation == 3:  # Flash (Pad)
                if self.current_aperture in self.apertures:
                    aperture = self.apertures[self.current_aperture]
                    self.pads.append([(x, y), aperture])

            # Update current position
            self.current_x, self.current_y = x, y

    def _parse_gerber_file(self, filename: str):
        # Parse a Gerber file
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()

        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('%'):
                self._process_extended_command(line)
            else:
                self._process_command(line)

    def shift(self, x_shift: float, y_shift: float):
        # Shift all coordinates relative to the new origin
        for trace in self.traces:
            for startstop in range(2):
                x, y = trace[startstop]
                trace[startstop] = [x - x_shift, y - y_shift]

        for pad in self.pads:
            x, y = pad[0]
            pad[0] = [x - x_shift, y - y_shift]


# =========================================================================================
class GerberEdgeCutsParser:
    def __init__(self, filename: str):
        self.raw_outline: list[tuple[float, float]] = []  # Raw coords
        self.outline: list[tuple[float, float]] = []  # Final optimized coords
        self.unit_mult: float = 1.0
        global x_min, x_max, y_min, y_max

        try:
            f = open(filename, 'r', encoding='utf-8')
        except:
            print("No edge cuts defined, that's OK.")
            return

        for line in f:
            line = line.strip()

            # Units
            if 'MOMM*%' in line:
                self.unit_mult = 1.0
            elif 'MOIN*%' in line:
                self.unit_mult = 25.4

            # Coordinate and operation commands
            coord_match = re.match(r'X(-?[0-9.]+)Y(-?[0-9.]+)D0([0123])?', line)
            if coord_match:
                # Parse coords. KiCad uses 1 million units per mm!
                x = float(coord_match.group(1)) * 0.000001 * self.unit_mult
                y = float(coord_match.group(2)) * 0.000001 * self.unit_mult

                m = 0.2  # Margin for extents calculation
                if x - m < x_min: x_min = x - m
                if x + m > x_max: x_max = x + m
                if y - m < y_min: y_min = y - m
                if y + m > y_max: y_max = y + y

                self.raw_outline.append((x, y))

        f.close()
        self.outline = self.raw_outline[:]

    def shift(self, x_base, y_base):
        # Redundant: optimize_contour_points handles absolute positioning.
        pass


# =========================================================================================
class DrillfileParser:
    def __init__(self, filename: str):
        self.tool_diameters: dict = {}
        self.holes: list[tuple[float, float, float]] = []
        self.small_holes: list[tuple[float, float, float]] = []
        self.large_holes: list[tuple[float, float, float]] = []
        current_tool = None
        self.units_mult = 1.0
        global x_min, x_max, y_min, y_max

        try:
            f = open(filename, 'r', encoding='utf-8')
        except:
            print("No drill file, that's OK.")
            return

        for line in f:
            line = line.strip()
            if not line or line.startswith(";"): continue

            # Detect units
            if "METRIC" in line.upper():
                self.units_mult = 1.0
            elif "INCH" in line.upper():
                self.units_mult = 25.4

            # Tool definition: e.g., T01C0.800
            match_tool = re.match(r"^T(\d+)C([\d\.]+)", line)
            if match_tool:
                tool_num = match_tool.group(1)
                diameter = float(match_tool.group(2)) * self.units_mult
                self.tool_diameters[tool_num] = diameter
                continue

            # Tool change: e.g., T01
            match_tool_change = re.match(r"^T(\d+)$", line)
            if match_tool_change:
                current_tool = match_tool_change.group(1)
                continue

            # Drill coordinates: e.g., X012345Y067890
            match_coord = re.match(r"^X(-?\d+(\.\d+)?)Y(-?\d+(\.\d+)?)", line)
            if match_coord and current_tool:
                x = float(match_coord.group(1)) * self.units_mult
                y = float(match_coord.group(3)) * self.units_mult

                dia = self.tool_diameters[current_tool]
                hole_data = (x, y, dia)
                self.holes.append(hole_data)

                if dia <= SMALL_HOLE_MAX_DIAMETER:
                    self.small_holes.append(hole_data)
                else:
                    self.large_holes.append(hole_data)

                print("Hole (%5.1f,%5.1f),%5.2f" % (x, y, dia))

                if x < x_min: x_min = x
                if x > x_max: x_max = x
                if y < y_min: y_min = y
                if y > y_max: y_max = y

        f.close()

    def shift(self, x_shift: float, y_shift: float):
        # Offset same way as traces were offset.
        def apply_shift(hole_list):
            return [(h[0] - x_shift, h[1] - y_shift, h[2]) for h in hole_list]

        self.holes = apply_shift(self.holes)
        self.small_holes = apply_shift(self.small_holes)
        self.large_holes = apply_shift(self.large_holes)


# =========================================================================================

class ShapelyBases:
    def __init__(self, parser: GerberTracesParser):
        # Init and Add the geometries
        traces = []
        pads = []

        # Add traces
        for trace in parser.traces:
            width_mm = trace[2]
            traces.append(LineString([trace[0], trace[1]]).buffer(width_mm / 2))

        # Add pads
        for pad in parser.pads:
            x_mm, y_mm = pad[0]
            aperture = pad[1]

            if aperture['type'] == 'circle':
                radius = (aperture['diameter'] / 2)
                pads.append(Point(x_mm, y_mm).buffer(radius))
            elif aperture['type'] == 'rectangle':
                width = aperture['width'] / 2
                height = aperture['height'] / 2
                pads.append(box(x_mm - width, y_mm - height, x_mm + width, y_mm + height))
            else:
                print("Pad type :", aperture['type'], "ignored")

        # Combine shapes into one geometry
        self.combined_geometry = unary_union(traces + pads)

    def compute_trace_toolpaths(self, offset_distance: float, num_passes: int, path_spacing: float):
        all_passes = []
        for passnum in range(0, num_passes):
            offset = offset_distance + path_spacing * passnum
            thispath = self.combined_geometry.buffer(offset).simplify(0.03).boundary
            if isinstance(thispath, LineString): thispath = MultiLineString([thispath])
            if hasattr(thispath, 'geoms'):
                all_passes += list(thispath.geoms)

        return MultiLineString(all_passes)


# =========================================================================================
class OutputVisualizer:
    def __init__(self):
        self.holes = []
        self.scale = 25  # pixels per mm

    def load_trace_geometries(self, traces):
        self.traces = traces

    def load_holes(self, p_holes: list):
        self.holes = p_holes

    def load_trace_mill_geometry(self, offsets):
        self.trace_mill_geometry = offsets

    def load_edge_cut_geometry(self, edgecuts):
        self.edgecuts = edgecuts

    def save_png_visualization(self, filename="pcb_visualization.png"):
        global x_min, x_max, y_min, y_max
        global base_name

        # Use optimized edge cut bounds
        x_min_v, x_max_v, y_min_v, y_max_v = x_min, x_max, y_min, y_max

        width_mm = x_max_v - x_min_v
        height_mm = y_max_v - y_min_v

        # Extra margin around the PCB
        margin_mm = 5

        # Calculate image dimensions in pixels, including the margins
        width_px = int((width_mm + 2 * margin_mm) * self.scale)
        height_px = int((height_mm + 2 * margin_mm) * self.scale)

        img = Image.new('RGB', (width_px, height_px), color='black')
        draw = ImageDraw.Draw(img)

        # --- FINAL ALIGNMENT CORRECTION ---
        # x_base_offset_px: Shifts PCB's x_min_v (should be SAFETY_PADDING) to 'margin_mm' in the PNG.
        x_base_offset_px = int((margin_mm - x_min_v) * self.scale)

        # y_base_offset_px: Shift y_min_v (should be SAFETY_PADDING).
        y_base_offset_px = int((margin_mm - y_min_v) * self.scale)

        pcb_color = (0, 80, 0)  # Dark green PCB color

        # Draw Edge Cuts (Contour) - Polygon
        if self.edgecuts:
            # Generate the buffered/cleared outline for visualization
            outline_geom = LineString(self.edgecuts).buffer(EDGE_CUT_CLEARANCE_OFFSET, cap_style=3, join_style=2)

            # The boundary of the buffered polygon represents the actual cut path
            coords_px = []
            if hasattr(outline_geom.exterior, 'coords'):
                for x, y in outline_geom.exterior.coords:
                    screen_x = int(x * self.scale + x_base_offset_px)
                    # Y Inversion: Y_screen = H_px - (Y_pcb * scale + Y_base_offset_px)
                    screen_y = int(height_px - (y * self.scale + y_base_offset_px))
                    coords_px.append((screen_x, screen_y))

            if len(coords_px) >= 3:
                # Use the original outline to draw the PCB polygon (without buffer)
                original_coords_px = []
                for x, y in self.edgecuts:
                    screen_x = int(x * self.scale + x_base_offset_px)
                    screen_y = int(height_px - (y * self.scale + y_base_offset_px))
                    original_coords_px.append((screen_x, screen_y))

                draw.polygon(original_coords_px, fill=pcb_color, outline=(255, 255, 255), width=1)

                # Draw the *cleared* cut path as an outline (optional, for visibility)
                draw.line(coords_px, fill=(255, 165, 0), width=2)  # Orange for Clearance path

            elif len(coords_px) == 2:
                draw.line(coords_px, fill=(255, 255, 255), width=2)
            else:
                print("Warning: Edge cuts definition insufficient to draw.")

        # Draw Traces (Red)
        for trace in self.traces.traces:
            start_x, start_y = trace[0]
            end_x, end_y = trace[1]
            width_mm = trace[2]

            x1_px = int(start_x * self.scale + x_base_offset_px)
            y1_px = int(height_px - (start_y * self.scale + y_base_offset_px))
            x2_px = int(end_x * self.scale + x_base_offset_px)
            y2_px = int(height_px - (end_y * self.scale + y_base_offset_px))

            line_width_px = max(1, int(width_mm * self.scale))
            draw.line((x1_px, y1_px, x2_px, y2_px), fill=(255, 0, 0), width=line_width_px)

        # Draw Pads (Blue)
        for pad in self.traces.pads:
            x_mm, y_mm = pad[0]
            aperture = pad[1]

            x_center_px = int(x_mm * self.scale + x_base_offset_px)
            y_center_px = int(height_px - (y_mm * self.scale + y_base_offset_px))

            if aperture['type'] == 'circle':
                radius_px = int((aperture['diameter'] / 2) * self.scale)
                draw.ellipse((x_center_px - radius_px, y_center_px - radius_px,
                              x_center_px + radius_px, y_center_px + radius_px),
                             fill=(0, 0, 255), outline=(173, 216, 230))
            elif aperture['type'] == 'rectangle':
                width_px = int(aperture['width'] * self.scale / 2)
                height_px_half = int(aperture['height'] * self.scale / 2)
                draw.rectangle((x_center_px - width_px, y_center_px - height_px_half,
                                x_center_px + width_px, y_center_px + height_px_half),
                               fill=(0, 0, 255), outline=(173, 216, 230))

        # Draw Toolpaths (White) - Line
        if hasattr(self, 'trace_mill_geometry') and self.trace_mill_geometry:
            for shape in self.trace_mill_geometry.geoms:
                coords_px = []
                for x, y in shape.coords:
                    screen_x = int(x * self.scale + x_base_offset_px)
                    screen_y = int(height_px - (y * self.scale + y_base_offset_px))
                    coords_px.append((screen_x, screen_y))
                if len(coords_px) > 1:
                    draw.line(coords_px, fill=(255, 255, 255), width=2)

        # Draw Holes (Black) - Ellipse
        for hole in self.holes:
            x, y, diameter = hole
            x_center_px = int(x * self.scale + x_base_offset_px)
            y_center_px = int(height_px - (y * self.scale + y_base_offset_px))
            radius_px = int(diameter / 2 * self.scale)
            draw.ellipse((x_center_px - radius_px, y_center_px - radius_px,
                          x_center_px + radius_px, y_center_px + radius_px),
                         fill=(0, 0, 0), outline=(255, 255, 255), width=1)

        img.save(filename)
        print(f"Visualization saved to '{filename}'")
    # ------------------------------------------------------------------


# =========================================================================================
class GcodeGenerator:
    # ... (other methods remain the same) ...

    def _write_header(self, f, description: str, tool_num: int, tool_name: str, speed_override=None):
        f.write("%\n")
        f.write(f"; GCODE FILE: {description}\n")
        f.write("G21  ; Set units to mm\n")
        f.write("G90  ; Absolute positioning\n")
        f.write("G28  ; Homing\n")
        f.write("G420 S1  ; Activate bed leveling\n")
        f.write(f"G0 Z{SAFE_HEIGHT:.2f}  ; Move to safe height\n")
        f.write(f"(load {tool_name})\nT{tool_num} M06\n")

        speed = speed_override if speed_override is not None else SPINDLE_SPEED
        f.write(f"S{speed} M3  ; Start spindle clockwise\n")
        f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")

    def _write_footer(self, f):
        y_home = 50.0  # Fixed Y return position
        f.write("M5  ; Stop spindle\n")
        f.write(f"G0 X0 Y{y_home:.1f} Z50  ; Return home, raise spindle\n")
        f.write("M30 ; End of program\n")
        f.write("%\n")
        f.close()

    def OutputEngravingGcode(self, filename: str, edgecuts: list, trace_mill_geometry):
        f = open(filename, "w")
        self._write_header(f, "Trace Isolation Milling (Engraving)", 1, "0.2 mm engraving tool (T1)")

        f.write("\n; --- TRACE ISOLATION MILLING ---\n")
        for path in trace_mill_geometry.geoms:
            started = False
            for x, y in path.coords:
                if not started:
                    f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
                    f.write(f"G0 Z0.1\n")
                    f.write(f"G1 Z{CUT_DEPTH:.3f} F{PLUNGE_FEED_RATE:d}\n")
                    f.write(f"G1 F{FEED_RATE:d}\n")
                    started = True
                else:
                    f.write(f"G1 X{x:.2f} Y{y:.2f}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")

        self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    # --- REVISED METHOD FOR EDGE CUT GCODE ---
    def OutputEdgeCutGcode(self, filename: str, edgecuts: list):
        if not edgecuts:
            print(f"Warning: No edge cuts found. Skipping file generation: {filename}")
            return

        # 1. Use shapely to create an outer buffer based on the clearance offset.
        # LineString(edgecuts) converts the list of coordinates to a line (contour).
        # buffer(x) creates a polygon offset outward by x.
        # boundary extracts the outer edge (the final path).
        # cap_style=3 (square), join_style=2 (miter) for sharp corners.

        # Since the list of coordinates is already a closed loop, LineString works.
        outline_geom = LineString(edgecuts).buffer(EDGE_CUT_CLEARANCE_OFFSET, cap_style=3, join_style=2).exterior.coords

        # Extract coordinates from the boundary of the buffer
        cleared_coords = list(outline_geom)

        # 2. Write G-code using the cleared coordinates
        f = open(filename, "w")
        self._write_header(f, f"PCB Final Edge Cut (Cleared by {EDGE_CUT_CLEARANCE_OFFSET}mm)", 4,
                           "Edge Cut Tool (T4 - 1.0mm or more)")

        f.write("\n; --- PCB FINAL EDGE CUT (WITH CLEARANCE) ---\n")
        f.write(f"; Toolpath offset applied: {EDGE_CUT_CLEARANCE_OFFSET} mm\n")
        started = False

        for x, y in cleared_coords:
            if not started:
                f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
                f.write(f"G0 Z0.1\n")
                f.write(f"G1 Z{FINAL_CUT_DEPTH:.3f} F{PLUNGE_FEED_RATE:d}\n")
                f.write(f"G1 F{CUT_FEED_RATE:.1f}\n")
                started = True
            else:
                f.write(f"G1 X{x:.2f} Y{y:.2f}\n")

        f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")
        self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    # ------------------------------------------------------------------

    def OutputSmallDrillGcode(self, filename: str, small_holes: list):
        if not small_holes:
            print(f"Warning: No small holes found. Skipping file generation: {filename}")
            return

        f = open(filename, "w")
        self._write_header(f, "Small Hole Drilling", 2, "small drill (T2)")

        f.write(f"\n; --- DRILLING OPERATIONS (Small Holes) ---\n")

        for x, y, _ in small_holes:
            f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
            f.write(f"G0 Z{HOLE_START_DEPTH:.2f}\n")
            f.write(f"G1 Z{HOLE_FINAL_DEPTH:.2f} F{PLUNGE_FEED_RATE:d}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")

        f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")
        self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    def OutputLargeDrillGcode(self, filename: str, large_holes: list):
        if not large_holes:
            print(f"Warning: No large holes found. Skipping file generation: {filename}")
            return

        f = open(filename, "w")
        self._write_header(f, "Large Hole Drilling", 3, "large drill (T3)")

        f.write(f"\n; --- DRILLING OPERATIONS (Large Holes) ---\n")

        for x, y, _ in large_holes:
            f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
            f.write(f"G0 Z{HOLE_START_DEPTH:.2f}\n")
            f.write(f"G1 Z{HOLE_FINAL_DEPTH:.2f} F{PLUNGE_FEED_RATE:d}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")

        f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")
        self._write_footer(f)
        print(f"G-code generated in '{filename}'")


# =========================================================================================
# EDGE CUT OPTIMIZATION FUNCTION
def optimize_contour_points(contour_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Positions, circularly sequences, and cleans the contour points.
    Input coords MUST be in MM (raw Gerber coordinates).
    The lowest X/Y point is moved to (SAFETY_PADDING, SAFETY_PADDING).
    """
    if not contour_coords:
        return []

    coords_array = np.array(contour_coords)

    # 1. Calculate and Apply Offset for Corner Positioning
    min_x_raw = np.min(coords_array[:, 0])
    min_y_raw = np.min(coords_array[:, 1])

    start_x_target = SAFETY_PADDING
    start_y_target = SAFETY_PADDING

    offset_x = start_x_target - min_x_raw
    offset_y = start_y_target - min_y_raw

    print(f"CORNER POS: Original min X/Y: ({min_x_raw:.2f}, {min_y_raw:.2f})")
    print(f"CORNER POS: Applying offset X: {offset_x:.2f}, Y: {offset_y:.2f} mm")

    coords_array[:, 0] += offset_x
    coords_array[:, 1] += offset_y

    # 2. Circular Ordering
    center_x_new = np.mean(coords_array[:, 0])
    center_y_new = np.mean(coords_array[:, 1])
    angles = np.arctan2(coords_array[:, 1] - center_y_new, coords_array[:, 0] - center_x_new)
    sorted_indices = np.argsort(angles)
    circular_coords_array = coords_array[sorted_indices]

    # Normalize start point to the closest corner (X/Y min)
    min_x_new = np.min(circular_coords_array[:, 0])
    min_y_new = np.min(circular_coords_array[:, 1])
    distances = np.sqrt((circular_coords_array[:, 0] - min_x_new) ** 2 + (circular_coords_array[:, 1] - min_y_new) ** 2)
    start_index = np.argmin(distances)
    optimized_list_array = np.roll(circular_coords_array, -start_index, axis=0)

    # 3. Filter Duplicates
    optimized_list = [(np.round(x, 4), np.round(y, 4)) for x, y in optimized_list_array.tolist()]

    final_clean_list = []
    if optimized_list:
        final_clean_list.append(optimized_list[0])
        for coord in optimized_list[1:]:
            # Check for close proximity duplicates
            if not (np.isclose(coord[0], final_clean_list[-1][0], atol=FLOAT_TOLERANCE) and
                    np.isclose(coord[1], final_clean_list[-1][1], atol=FLOAT_TOLERANCE)):
                final_clean_list.append(coord)

    # 4. Close the Loop
    if len(final_clean_list) < 2:
        return final_clean_list

    start_point = final_clean_list[0]
    end_point = final_clean_list[-1]

    # Close the loop if start and end points are not close
    if not (np.isclose(start_point[0], end_point[0], atol=FLOAT_TOLERANCE) and
            np.isclose(start_point[1], end_point[1], atol=FLOAT_TOLERANCE)):
        final_clean_list.append(start_point)

    print(f"OPTIMIZATION: Contour cleaned, positioned, and closed. Total points: {len(final_clean_list)}")
    return final_clean_list


# ----------------- MAIN EXECUTION FLOW -----------------

if len(sys.argv) < 2:
    print("\n--- KiCad Gerber to CNC G-Code Generator ---")
    base_name = input("Enter the project base name (e.g., 'MyBoard'): ").strip().replace("\\", "/")

    if not base_name:
        print("Error: Project name cannot be empty.")
        sys.exit(1)

    if not os.path.exists(base_name + "-F_Cu.gbr"):
        print(f"Error: Main Gerber file '{base_name}-F_Cu.gbr' not found.")
        sys.exit(1)

    outname = base_name.split("/")[-1]
else:
    base_name = sys.argv[1].replace("\\", "/")
    outname = base_name.split("/")[-1]

# Global extents (used for shifting other data relative to edge cuts)
x_min: float = 1000000.0
x_max: float = -1000000.0
y_min: float = 1000000.0
y_max: float = -1000000.0

print(f"Loading files with base name: '{base_name}'")
# First pass reads global raw bounds (x_min, y_min)
gerber_traces = GerberTracesParser(base_name + "-F_Cu.gbr")
gerber_edgecuts = GerberEdgeCutsParser(base_name + "-Edge_cuts.gbr")
drilldata = DrillfileParser(base_name + "-PTH.drl")

# --- ALIGNMENT/SHIFTING CORRECTION ---

# 1. Capture the raw minimum (bottom-left corner of the combined area)
x_shift_base = x_min
y_shift_base = y_min

print(f"SHIFT: Raw X: {x_shift_base:.2f}, Raw Y: {y_shift_base:.2f}")

# Translation offset to be applied: (SAFETY_PADDING - x_shift_base)
shift_for_subtraction_x = x_shift_base - SAFETY_PADDING
shift_for_subtraction_y = y_shift_base - SAFETY_PADDING

# 2. Apply the offset to all elements
gerber_traces.shift(shift_for_subtraction_x, shift_for_subtraction_y)
drilldata.shift(shift_for_subtraction_x, shift_for_subtraction_y)

# 3. Optimize the contour. This function applies ABSOLUTE repositioning.
optimized_edge_cuts = optimize_contour_points(gerber_edgecuts.raw_outline)

if optimized_edge_cuts:
    gerber_edgecuts.outline = optimized_edge_cuts

    # 4. Update global bounds based on the ALREADY POSITIONED Edge Cuts
    x_coords = [c[0] for c in optimized_edge_cuts]
    y_coords = [c[1] for c in optimized_edge_cuts]

    # We need to calculate bounds *after* applying the clearance to correctly set visualization limits
    # We will use the original bounds here and rely on the visualizer to handle the cleared path display.
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    print(f"Final Bounds: X:{x_min:.2f}-{x_max:.2f}, Y:{y_min:.2f}-{y_max:.2f}")

# ------------------------------------------

# 5. Compute trace isolation toolpaths
sh_base = ShapelyBases(gerber_traces)
trace_mill_geometry = sh_base.compute_trace_toolpaths(ISOLATION_OFFSET, ISOLATION_PASSES, PASS_SPACING)

# 6. Generate Visualization
visualizer = OutputVisualizer()
visualizer.load_trace_geometries(gerber_traces)
visualizer.load_trace_mill_geometry(trace_mill_geometry)
visualizer.load_edge_cut_geometry(gerber_edgecuts.outline)
visualizer.load_holes(drilldata.holes)
visualizer.save_png_visualization(f"{outname}_Visualization.png")

# 7. Generate G-Code Files
gcode = GcodeGenerator()

# Trace Isolation
gcode.OutputEngravingGcode(f"{outname}_Engraving.gcode",
                           gerber_edgecuts.outline,
                           trace_mill_geometry)

# Small Hole Drilling
gcode.OutputSmallDrillGcode(f"{outname}_Small_Drill.gcode",
                            drilldata.small_holes)

# Large Hole Drilling
gcode.OutputLargeDrillGcode(f"{outname}_Large_Drill.gcode",
                            drilldata.large_holes)

# Final Edge Cut - NOW USES CLEARANCE
gcode.OutputEdgeCutGcode(f"{outname}_Edge_Cut.gcode",
                         gerber_edgecuts.outline)