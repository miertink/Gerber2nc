#!/usr/bin/python3
# Generates CNC tool paths (G-code) from a KiCad PCB layout (F_Cu, Edge_Cuts, Drill).
# The PCB is automatically CENTERED on the bed target (BED_MID_X, BED_MID_Y).
#
# Six G-Code files and one Visualization PNG will be exported:
#
# 1) xx_Pre_Drill.gcode (Center marks - Uses Bed Mesh)
# 2) xx_Engraving.gcode (Trace Isolation - Uses Bed Mesh)
# 3) xx_Small_Drill.gcode (Small Holes - Bed Mesh DISABLED for precision)
# 4) xx_Large_Drill.gcode (Large Holes - Bed Mesh DISABLED for precision)
# 5) xx_Edge_Cut.gcode (Final Contour Cut - Uses Bed Mesh)
# 6) xx_Visualization.png (Visualizes PCB Placement + Bed Mesh Area)
#
# Requires 'shapely' and 'numpy'.

import sys, re
from shapely.geometry import LineString, MultiLineString, Point, box
from shapely.ops import unary_union
import os
from PIL import Image, ImageDraw
import numpy as np
from typing import List, Tuple

# --- CONFIGURATION PARAMETERS ---

# PCB & Positioning
BED_MID_X = 100.0
BED_MID_Y = 140.0
BED_MAX_X = 240.0
BED_MAX_Y = 220.0

FLOAT_TOLERANCE = 1e-4
SMALL_HOLE_MAX_DIAMETER = 0.85

# CNC Milling Parameters
SPINDLE_SPEED = 254
SAFE_HEIGHT = 5.0

# Trace Isolation Path Calculation
CUT_DEPTH = -0.05
ISOLATION_OFFSET = 0.22
PASS_SPACING = 0.04
# 'single' or 'multi'
ISOLATION_TYPE = 'single'

# Rate Parameters (mm/min)
MOVE_FEED_RATE = 1000
PLUNGE_FEED_RATE = 10
CUT_FEED_RATE = 50
FEED_RATE = 100

# Drilling Parameters (mm)
HOLE_START_DEPTH = 0.5
HOLE_PRE_DEPTH = -0.2
HOLE_FINAL_DEPTH = -1.8
Z_DRILL_ADJUST = 0.0  # Manual Z zero adjustment (applies to all drill passes)

# Edge Cut Parameters (mm)
FINAL_CUT_DEPTH = -0.1
EDGE_CUT_CLEARANCE_OFFSET = 1.0
MESH_PADDING = 5.0


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
            print(f"ERROR: Trace file '{filename}' not found.")
            sys.exit(1)

    def _process_extended_command(self, line: str):
        if not line: return

        aperture_match = re.match(r'%ADD(\d+)([^,]+),([^*]+)\*%', line)
        if aperture_match:
            aperture_num = int(aperture_match.group(1))
            aperture_type = aperture_match.group(2)
            params = aperture_match.group(3).split('X')

            if aperture_type == 'C':
                diameter = float(params[0])
                self.apertures[aperture_num] = {'type': 'circle', 'diameter': diameter}
            elif aperture_type == 'R':
                width = float(params[0])
                height = float(params[1]) if len(params) > 1 else width
                self.apertures[aperture_num] = {'type': 'rectangle', 'width': width, 'height': height}
            elif aperture_type == 'RoundRect':
                width = abs(float(params[1])) + abs(float(params[3])) + float(params[0])
                height = abs(float(params[2])) + abs(float(params[4])) + float(params[0])
                self.apertures[aperture_num] = {'type': 'rectangle', 'width': width, 'height': height}

        if 'MOMM*%' in line:
            self.unit_mult = 1
        elif 'MOIN*%' in line:
            self.unit_mult = 25.4

    def _process_command(self, line: str):
        global x_min, x_max, y_min, y_max

        line = line.rstrip('*')

        aperture_match = re.match(r'D(\d+)', line)
        if aperture_match:
            aperture_num = int(aperture_match.group(1))
            if aperture_num >= 10:
                self.current_aperture = aperture_num
            return

        coord_match = re.match(r'X(-?[0-9.]+)Y(-?[0-9.]+)D0([0123])?', line)
        if coord_match:
            x = float(coord_match.group(1)) * 0.000001 * self.unit_mult
            y = float(coord_match.group(2)) * 0.000001 * self.unit_mult
            operation = int(coord_match.group(3))

            m = 1.5 if operation == 3 else 0.6
            if x - m < x_min: x_min = x - m
            if x + m > x_max: x_max = x + m
            if y - m < y_min: y_min = y - m
            if y + m > y_max: y_max = y + m

            if operation == 1:
                if self.current_aperture in self.apertures:
                    aperture = self.apertures[self.current_aperture]
                    width = aperture.get('diameter', aperture.get('width', 0.1))
                    self.traces.append([(self.current_x, self.current_y), (x, y), width])
            elif operation == 3:
                if self.current_aperture in self.apertures:
                    aperture = self.apertures[self.current_aperture]
                    self.pads.append([(x, y), aperture])

            self.current_x, self.current_y = x, y

    def _parse_gerber_file(self, filename: str):
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()

        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('%'):
                self._process_extended_command(line)
            else:
                self._process_command(line)

    def shift(self, x_shift: float, y_shift: float):
        # Shift coordinates
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
        self.raw_outline: list[tuple[float, float]] = []
        self.outline: list[tuple[float, float]] = []
        self.unit_mult: float = 1.0
        global x_min, x_max, y_min, y_max

        try:
            f = open(filename, 'r', encoding='utf-8')
        except:
            print("No edge cuts defined. OK.")
            return

        for line in f:
            line = line.strip()

            if 'MOMM*%' in line:
                self.unit_mult = 1.0
            elif 'MOIN*%' in line:
                self.unit_mult = 25.4

            coord_match = re.match(r'X(-?[0-9.]+)Y(-?[0-9.]+)D0([0123])?', line)
            if coord_match:
                x = float(coord_match.group(1)) * 0.000001 * self.unit_mult
                y = float(coord_match.group(2)) * 0.000001 * self.unit_mult

                m = 0.2
                if x - m < x_min: x_min = x - m
                if x + m > x_max: x_max = x + m
                if y - m < y_min: y_min = y - m
                if y + m > y_max: y_max = y + y

                self.raw_outline.append((x, y))

        f.close()
        self.outline = self.raw_outline[:]

    def shift(self, x_shift, y_shift):
        self.raw_outline = [(x - x_shift, y - y_shift) for x, y in self.raw_outline]
        self.outline = self.raw_outline[:]


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
            print("No drill file. OK.")
            return

        for line in f:
            line = line.strip()
            if not line or line.startswith(";"): continue

            if "METRIC" in line.upper():
                self.units_mult = 1.0
            elif "INCH" in line.upper():
                self.units_mult = 25.4

            match_tool = re.match(r"^T(\d+)C([\d\.]+)", line)
            if match_tool:
                tool_num = match_tool.group(1)
                diameter = float(match_tool.group(2)) * self.units_mult
                self.tool_diameters[tool_num] = diameter
                continue

            match_tool_change = re.match(r"^T(\d+)$", line)
            if match_tool_change:
                current_tool = match_tool_change.group(1)
                continue

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

                print(f"Hole ({x:.1f},{y:.1f}),{dia:.2f}")

                if x < x_min: x_min = x
                if x > x_max: x_max = x
                if y < y_min: y_min = y
                if y > y_max: y_max = y

        f.close()

    def shift(self, x_shift: float, y_shift: float):
        def apply_shift(hole_list):
            return [(h[0] - x_shift, h[1] - y_shift, h[2]) for h in hole_list]

        self.holes = apply_shift(self.holes)
        self.small_holes = apply_shift(self.small_holes)
        self.large_holes = apply_shift(self.large_holes)


# =========================================================================================

class ShapelyBases:
    def __init__(self, parser: GerberTracesParser):
        traces = []
        pads = []

        for trace in parser.traces:
            width_mm = trace[2]
            traces.append(LineString([trace[0], trace[1]]).buffer(width_mm / 2))

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

        self.combined_geometry = unary_union(traces + pads)

    def compute_trace_toolpaths(self, offset_distance: float, pass_spacing: float, isolation_type: str):
        all_passes = []

        # Primary Pass (Climb Milling) is always included
        pass_definitions = [
            (offset_distance, True, "Primary Pass (Climb Milling)"),
        ]

        # Add secondary passes if isolation_type is 'multi'
        if isolation_type.lower() == 'multi':
            pass_definitions += [
                (offset_distance + pass_spacing, False, "Secondary Pass 1"),
                (offset_distance - pass_spacing, False, "Secondary Pass 2"),
            ]
            print("Generating 3 passes (Primary + 2 Secondary).")
        else:
            print("Generating 1 pass (Primary Only).")

        for offset, reverse_direction, description in pass_definitions:
            print(f"Generating {description} at offset {offset:.3f} mm")

            thispath = self.combined_geometry.buffer(offset).simplify(0.03).boundary

            if isinstance(thispath, LineString):
                thispath = MultiLineString([thispath])

            if hasattr(thispath, 'geoms'):
                paths_for_this_pass = []

                if reverse_direction:
                    # Apply inversion (Climb Milling)
                    for geom in thispath.geoms:
                        if geom.geom_type == 'LineString':
                            reversed_coords = list(geom.coords)[::-1]
                            paths_for_this_pass.append(LineString(reversed_coords))
                        else:
                            paths_for_this_pass.append(geom)
                else:
                    # Conventional Milling
                    paths_for_this_pass = list(thispath.geoms)

                all_passes += paths_for_this_pass

        return MultiLineString(all_passes)


# =========================================================================================
class OutputVisualizer:
    def __init__(self):
        self.holes = []
        self.scale = 25

    def load_trace_geometries(self, traces):
        self.traces = traces

    def load_holes(self, p_holes: list):
        self.holes = p_holes

    def load_trace_mill_geometry(self, offsets):
        self.trace_mill_geometry = offsets

    def load_edge_cut_geometry(self, edgecuts):
        self.edgecuts = edgecuts

    def save_png_visualization(self, filename: str, mesh_bounds: Tuple[float, float, float, float]):
        global x_min, x_max, y_min, y_max
        global outname

        x_min_v, x_max_v, y_min_v, y_max_v = x_min, x_max, y_min, y_max

        margin_mm = 10

        all_x_coords = [x_min_v, x_max_v, BED_MID_X, mesh_bounds[0], mesh_bounds[2]]
        all_y_coords = [y_min_v, y_max_v, BED_MID_Y, mesh_bounds[1], mesh_bounds[3]]

        x_min_plot = min(min(all_x_coords), 0.0) - margin_mm
        x_max_plot = max(max(all_x_coords), BED_MAX_X) + margin_mm
        y_min_plot = min(min(all_y_coords), 0.0) - margin_mm
        y_max_plot = max(max(all_y_coords), BED_MAX_Y) + margin_mm

        width_px = int((x_max_plot - x_min_plot) * self.scale)
        height_px = int((y_max_plot - y_min_plot) * self.scale)

        img = Image.new('RGB', (width_px, height_px), color='black')
        draw = ImageDraw.Draw(img)

        # Calculate pixel offset
        x_base_offset_px = int((-x_min_plot) * self.scale)
        y_base_offset_px = int((-y_min_plot) * self.scale)

        # Helper function to convert MM to inverted Y pixel coordinates
        def mm_to_px(x, y):
            screen_x = int(x * self.scale + x_base_offset_px)
            # Y Inversion
            screen_y = int(height_px - (y * self.scale + y_base_offset_px))
            return (screen_x, screen_y)

        pcb_color = (0, 80, 0)

        # Draw CNC Origin (0,0)
        origin_x, origin_y = mm_to_px(0.0, 0.0)
        draw.line([(origin_x - 5, origin_y), (origin_x + 5, origin_y)], fill=(100, 100, 100), width=1)
        draw.line([(origin_x, origin_y - 5), (origin_x, origin_y + 5)], fill=(100, 100, 100), width=1)

        # Draw Target Center
        center_x, center_y = mm_to_px(BED_MID_X, BED_MID_Y)
        draw.line([(center_x - 10, center_y), (center_x + 10, center_y)], fill=(0, 255, 0), width=2)
        draw.line([(center_x, center_y - 10), (center_x, center_y + 10)], fill=(0, 255, 0), width=2)

        # Draw Edge Cuts (Contour)
        if self.edgecuts:
            outline_geom = LineString(self.edgecuts).buffer(EDGE_CUT_CLEARANCE_OFFSET, cap_style=3,
                                                            join_style=2).exterior

            coords_px = []
            if hasattr(outline_geom, 'coords'):
                for x, y in outline_geom.coords:
                    coords_px.append(mm_to_px(x, y))

            if len(coords_px) >= 3:
                original_coords_px = [mm_to_px(x, y) for x, y in self.edgecuts]

                draw.polygon(original_coords_px, fill=pcb_color, outline=(255, 255, 255), width=1)
                # Draw final edge cut path
                draw.line(coords_px, fill=(255, 165, 0), width=2)

        # Draw Bed Mesh Area
        mx_start, my_start, mx_end, my_end = mesh_bounds
        rect_coords_mm = [
            (mx_start, my_start), (mx_start, my_end), (mx_end, my_end), (mx_end, my_start), (mx_start, my_start)
        ]
        rect_coords_px = [mm_to_px(x, y) for x, y in rect_coords_mm]

        # Draw dashed mesh boundary
        if len(rect_coords_px) == 5:
            for i in range(4):
                x1, y1 = rect_coords_px[i]
                x2, y2 = rect_coords_px[i + 1]
                for t in np.linspace(0, 1, 40):
                    if int(t * 40) % 2 == 0:
                        draw.line([(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t),
                                   (x1 + (x2 - x1) * (t + 0.025), y1 + (y2 - y1) * (t + 0.025))],
                                  fill=(255, 140, 0), width=3)

                        # Draw Traces (Red)
        for trace in self.traces.traces:
            start_x, start_y = trace[0]
            end_x, end_y = trace[1]
            width_mm = trace[2]

            x1_px, y1_px = mm_to_px(start_x, start_y)
            x2_px, y2_px = mm_to_px(end_x, end_y)

            line_width_px = max(1, int(width_mm * self.scale))
            draw.line([(x1_px, y1_px), (x2_px, y2_px)], fill=(255, 0, 0), width=line_width_px)

        # Draw Pads (Blue)
        for pad in self.traces.pads:
            x_mm, y_mm = pad[0]
            aperture = pad[1]

            x_center_px, y_center_px = mm_to_px(x_mm, y_mm)

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

        # Draw Toolpaths (White)
        if hasattr(self, 'trace_mill_geometry') and self.trace_mill_geometry:
            for shape in self.trace_mill_geometry.geoms:
                coords_px = [mm_to_px(x, y) for x, y in shape.coords]
                if len(coords_px) > 1:
                    draw.line(coords_px, fill=(255, 255, 255), width=2)

        # Draw Holes (Black)
        for hole in self.holes:
            x, y, diameter = hole
            x_center_px, y_center_px = mm_to_px(x, y)
            radius_px = int(diameter / 2 * self.scale)
            draw.ellipse((x_center_px - radius_px, y_center_px - radius_px,
                          x_center_px + radius_px, y_center_px + radius_px),
                         fill=(0, 0, 0), outline=(255, 255, 255), width=1)

        img.save(filename)
        print(f"Visualization saved to '{filename}'")
    # ------------------------------------------------------------------


# =========================================================================================
class GcodeGenerator:
    def _write_header(self, f, description: str, tool_num: int, tool_name: str, use_bed_mesh: bool,
                      speed_override=None):
        f.write("%\n")
        f.write(f"; GCODE FILE: {description}\n")
        f.write("G21  ; Set units to mm\n")
        f.write("G90  ; Absolute positioning\n")
        f.write("G28  ; Homing\n")
        if use_bed_mesh:
            f.write("G420 S1  ; Activate bed leveling\n")

        f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE} ; Move to safe height\n")
        f.write(f"(load {tool_name})\nT{tool_num} M06\n")

        speed = speed_override if speed_override is not None else SPINDLE_SPEED
        f.write(f"M3 S{speed} ; Start spindle clockwise\n")
        f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE}\n")

    def _write_footer(self, f):
        """Finalization."""
        y_home = 50.0
        f.write("\n; --- FINALIZATION ---\n")
        f.write("M5  ; Stop spindle\n")
        f.write(f"G0 X0 Y{y_home:.1f} Z50 F{MOVE_FEED_RATE} ; Return home\n")
        f.write("M84 ; Disable Steppers\n")
        f.write("M30 ; End of program\n")
        f.write("%\n")
        f.close()

    def OutputEngravingGcode(self, filename: str, edgecuts: list, trace_mill_geometry):
        f = open(filename, "w")
        self._write_header(f, "Trace Isolation Milling (Engraving)", 1, "0.2 mm engraving tool (T1)", use_bed_mesh=True)

        f.write("\n; --- TRACE ISOLATION MILLING ---\n")
        for path in trace_mill_geometry.geoms:
            started = False
            for x, y in path.coords:
                if not started:
                    f.write(f"G0 X{x:.2f} Y{y:.2f} F{MOVE_FEED_RATE}\n")
                    f.write(f"G0 Z0.1 F{MOVE_FEED_RATE}\n")
                    f.write(f"G1 Z{CUT_DEPTH:.3f} F{PLUNGE_FEED_RATE:d}\n")
                    f.write(f"G1 F{FEED_RATE:d}\n")
                    started = True
                else:
                    f.write(f"G1 X{x:.2f} Y{y:.2f}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE}\n")

        self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    def OutputPreDrillGcode(self, filename: str, all_holes: list):
        if not all_holes:
            print(f"Warning: No holes found for pre-drilling. Skipping file generation: {filename}")
            return

        f = open(filename, "w")
        self._write_header(f, "Hole Center Marking (Pre-Drill)", 1, "Pre-Drill Tool (T1)", use_bed_mesh=True)

        f.write(f"\n; --- CENTER MARKING OPERATIONS ---\n")

        z_pre = HOLE_PRE_DEPTH + Z_DRILL_ADJUST

        for x, y, _ in all_holes:
            f.write(f"G0 X{x:.2f} Y{y:.2f} F{MOVE_FEED_RATE}\n")
            f.write(f"G0 Z0.1 F{MOVE_FEED_RATE}\n")
            f.write(f"G1 Z{z_pre:.2f} F{PLUNGE_FEED_RATE:d}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE}\n")

        self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    def OutputEdgeCutGcode(self, filename: str, edgecuts: list):
        if not edgecuts:
            print(f"Warning: No edge cuts found. Skipping file generation: {filename}")
            return

        outline_geom = LineString(edgecuts).buffer(EDGE_CUT_CLEARANCE_OFFSET, cap_style=3, join_style=2).exterior.coords

        cleared_coords = list(outline_geom)

        f = open(filename, "w")
        self._write_header(f, f"PCB Final Edge Cut (Cleared by {EDGE_CUT_CLEARANCE_OFFSET}mm)", 4,
                           "Edge Cut Tool (T4 - 1.0mm or more)", use_bed_mesh=True)

        f.write("\n; --- PCB FINAL EDGE CUT ---\n")
        started = False

        for x, y in cleared_coords:
            if not started:
                f.write(f"G0 X{x:.2f} Y{y:.2f} F{MOVE_FEED_RATE}\n")
                f.write(f"G0 Z0.1 F{MOVE_FEED_RATE}\n")
                f.write(f"G1 Z{FINAL_CUT_DEPTH:.3f} F{PLUNGE_FEED_RATE:d}\n")
                f.write(f"G1 F{CUT_FEED_RATE:.1f}\n")
                started = True
            else:
                f.write(f"G1 X{x:.2f} Y{y:.2f}\n")

        f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE}\n")
        self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    def OutputSmallDrillGcode(self, filename: str, small_holes: list):
        if not small_holes:
            print(f"Warning: No small holes found. Skipping file generation: {filename}")
            return

        f = open(filename, "w")
        # Bed Mesh Disabled
        self._write_header(f, "Small Hole Drilling", 2, "small drill (T2)", use_bed_mesh=False)

        f.write(f"\n; --- SMALL HOLE DRILLING ---\n")

        z_start = HOLE_START_DEPTH + Z_DRILL_ADJUST
        z_final = HOLE_FINAL_DEPTH + Z_DRILL_ADJUST

        for x, y, _ in small_holes:
            f.write(f"G0 X{x:.2f} Y{y:.2f} F{MOVE_FEED_RATE}\n")
            f.write(f"G0 Z{z_start:.2f} F{MOVE_FEED_RATE}\n")
            f.write(f"G1 Z{z_final:.2f} F{PLUNGE_FEED_RATE:d}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE}\n")

        f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE}\n")
        self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    def OutputLargeDrillGcode(self, filename: str, large_holes: list):
        if not large_holes:
            print(f"Warning: No large holes found. Skipping file generation: {filename}")
            return

        f = open(filename, "w")
        # Bed Mesh Disabled
        self._write_header(f, "Large Hole Drilling", 3, "large drill (T3)", use_bed_mesh=False)

        f.write(f"\n; --- LARGE HOLE DRILLING ---\n")

        z_start = HOLE_START_DEPTH + Z_DRILL_ADJUST
        z_final = HOLE_FINAL_DEPTH + Z_DRILL_ADJUST

        for x, y, _ in large_holes:
            f.write(f"G0 X{x:.2f} Y{y:.2f} F{MOVE_FEED_RATE}\n")
            f.write(f"G0 Z{z_start:.2f} F{MOVE_FEED_RATE}\n")
            f.write(f"G1 Z{z_final:.2f} F{PLUNGE_FEED_RATE:d}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE}\n")

        f.write(f"G0 Z{SAFE_HEIGHT:.2f} F{MOVE_FEED_RATE}\n")
        self._write_footer(f)
        print(f"G-code generated in '{filename}'")


# =========================================================================================
def calculate_mesh_bounds(optimized_coords: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    """Calculates G29 bounds using positioned coordinates plus padding."""
    if not optimized_coords:
        raise ValueError("No coordinates found for mesh bounds.")

    coords_array = np.array(optimized_coords)

    min_x = np.min(coords_array[:, 0])
    max_x = np.max(coords_array[:, 0])
    min_y = np.min(coords_array[:, 1])
    max_y = np.max(coords_array[:, 1])

    # Apply padding and clamp
    X_start = max(0.0, min_x - MESH_PADDING)
    Y_start = max(0.0, min_y - MESH_PADDING)
    X_end = min(BED_MAX_X, max_x + MESH_PADDING)
    Y_end = min(BED_MAX_Y, max_y + MESH_PADDING)

    print(
        f"\nBED MESH BOUNDS (G29 for visualization - {MESH_PADDING:.2f}mm padding): X: {X_start:.2f}-{X_end:.2f}, Y: {Y_start:.2f}-{Y_end:.2f} mm")

    return X_start, Y_start, X_end, Y_end


# EDGE CUT OPTIMIZATION FUNCTION
def optimize_contour_points(contour_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Sequences and cleans the contour points."""
    if not contour_coords:
        return []

    coords_array = np.array(contour_coords)

    # Circular Ordering
    center_x_new = np.mean(coords_array[:, 0])
    center_y_new = np.mean(coords_array[:, 1])
    angles = np.arctan2(coords_array[:, 1] - center_y_new, coords_array[:, 0] - center_x_new)
    sorted_indices = np.argsort(angles)
    circular_coords_array = coords_array[sorted_indices]

    # Normalize start point (closest to X/Y min corner)
    min_x_new = np.min(circular_coords_array[:, 0])
    min_y_new = np.min(circular_coords_array[:, 1])

    distances = np.sqrt((circular_coords_array[:, 0] - min_x_new) ** 2 + (circular_coords_array[:, 1] - min_y_new) ** 2)
    start_index = np.argmin(distances)
    optimized_list_array = np.roll(circular_coords_array, -start_index, axis=0)

    # Filter Duplicates
    optimized_list = [(np.round(x, 4), np.round(y, 4)) for x, y in optimized_list_array.tolist()]

    final_clean_list = []
    if optimized_list:
        final_clean_list.append(optimized_list[0])
        for coord in optimized_list[1:]:
            if not (np.isclose(coord[0], final_clean_list[-1][0], atol=FLOAT_TOLERANCE) and
                    np.isclose(coord[1], final_clean_list[-1][1], atol=FLOAT_TOLERANCE)):
                final_clean_list.append(coord)

    # Close the Loop
    if len(final_clean_list) < 2:
        return final_clean_list

    start_point = final_clean_list[0]
    end_point = final_clean_list[-1]

    if not (np.isclose(start_point[0], end_point[0], atol=FLOAT_TOLERANCE) and
            np.isclose(start_point[1], end_point[1], atol=FLOAT_TOLERANCE)):
        final_clean_list.append(start_point)

    print(f"OPTIMIZATION: Contour cleaned and positioned. Total points: {len(final_clean_list)}")
    return final_clean_list


# ----------------- MAIN EXECUTION FLOW -----------------

if len(sys.argv) < 2:
    print("\n--- KiCad Gerber to CNC G-Code Generator ---")
    base_name = input("Enter the project base name (e.g., 'MyBoard'): ").strip().replace("\\", "/")

    if not base_name:
        print("ERROR: Project name cannot be empty.")
        sys.exit(1)

    if not os.path.exists(base_name + "-F_Cu.gbr"):
        print(f"ERROR: Main Gerber file '{base_name}-F_Cu.gbr' not found.")
        sys.exit(1)

    outname = base_name.split("/")[-1]
else:
    base_name = sys.argv[1].replace("\\", "/")
    outname = base_name.split("/")[-1]

# Global extents (initial bounds for raw parsing)
x_min: float = 1000000.0
x_max: float = -1000000.0
y_min: float = 1000000.0
y_max: float = -1000000.0

print(f"Loading files with base name: '{base_name}'")

# Read files and capture raw bounds
gerber_traces = GerberTracesParser(base_name + "-F_Cu.gbr")
gerber_edgecuts = GerberEdgeCutsParser(base_name + "-Edge_cuts.gbr")
drilldata = DrillfileParser(base_name + "-PTH.drl")

# --- ALIGNMENT/SHIFTING CORRECTION (CENTERING) ---

# Capture RAW extents
x_raw_min = x_min
x_raw_max = x_max
y_raw_min = y_min
y_raw_max = y_max

print(f"SHIFT: Raw Bounds X:{x_raw_min:.2f}-{x_raw_max:.2f}, Y:{y_raw_min:.2f}-{y_raw_max:.2f}")

# Calculate the RAW center and required shift
x_raw_center = (x_raw_min + x_raw_max) / 2.0
y_raw_center = (y_raw_min + y_raw_max) / 2.0

shift_for_subtraction_x = x_raw_center - BED_MID_X
shift_for_subtraction_y = y_raw_center - BED_MID_Y

print(f"SHIFT: Raw Center: ({x_raw_center:.2f}, {y_raw_center:.2f})")
print(f"SHIFT: Target Center: ({BED_MID_X:.2f}, {BED_MID_Y:.2f})")
print(f"SHIFT: Applying Offset X: {shift_for_subtraction_x * -1:.2f}, Y: {shift_for_subtraction_y * -1:.2f} mm")

# Apply the offset to all elements
gerber_traces.shift(shift_for_subtraction_x, shift_for_subtraction_y)
drilldata.shift(shift_for_subtraction_x, shift_for_subtraction_y)
gerber_edgecuts.shift(shift_for_subtraction_x, shift_for_subtraction_y)

# Optimize the contour
optimized_edge_cuts = optimize_contour_points(gerber_edgecuts.raw_outline)

if optimized_edge_cuts:
    gerber_edgecuts.outline = optimized_edge_cuts

    # Update global bounds based on the POSITIONED Edge Cuts
    x_coords = [c[0] for c in optimized_edge_cuts]
    y_coords = [c[1] for c in optimized_edge_cuts]

    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    print(
        f"Final Bounds (PCB Centered @ {BED_MID_X:.2f},{BED_MID_Y:.2f}): X:{x_min:.2f}-{x_max:.2f}, Y:{y_min:.2f}-{y_max:.2f}")

# Compute trace isolation toolpaths
sh_base = ShapelyBases(gerber_traces)
trace_mill_geometry = sh_base.compute_trace_toolpaths(ISOLATION_OFFSET, PASS_SPACING, ISOLATION_TYPE)

# Calculate Bed Mesh Bounds
mesh_bounds = calculate_mesh_bounds(optimized_edge_cuts)

# Generate Visualization
visualizer = OutputVisualizer()
visualizer.load_trace_geometries(gerber_traces)
visualizer.load_trace_mill_geometry(trace_mill_geometry)
visualizer.load_edge_cut_geometry(gerber_edgecuts.outline)
visualizer.load_holes(drilldata.holes)
visualizer.save_png_visualization(f"{outname}_Visualization.png", mesh_bounds)

# Generate G-Code Files
gcode = GcodeGenerator()

# Pre-Drill Center Marking (Uses Bed Mesh)
gcode.OutputPreDrillGcode(f"{outname}_Pre_Drill.gcode",
                          drilldata.holes)

# Trace Isolation (Uses Bed Mesh)
gcode.OutputEngravingGcode(f"{outname}_Engraving.gcode",
                           gerber_edgecuts.outline,
                           trace_mill_geometry)

# Small Hole Drilling (Bed Mesh Disabled)
gcode.OutputSmallDrillGcode(f"{outname}_Small_Drill.gcode",
                            drilldata.small_holes)

# Large Hole Drilling (Bed Mesh Disabled)
gcode.OutputLargeDrillGcode(f"{outname}_Large_Drill.gcode",
                            drilldata.large_holes)

# Final Edge Cut (Uses Bed Mesh)
gcode.OutputEdgeCutGcode(f"{outname}_Edge_Cut.gcode",
                         gerber_edgecuts.outline)