#!/usr/bin/python3
# Generates CNC tool paths (G-code) from a single-sided KiCad PCB layout.
# Supports traces, holes, pads (circle/rectangle/obround/RoundRect/aperture
# macros, including rotated free-form polygon pads), and copper zone fills.
# Requires: shapely, numpy, Pillow  (pip install shapely numpy Pillow)
#
# Based on Matthias Wandel August 2025
# Improved by Alessandro Miertschink 2025

import sys
import re
import os
import math
from dataclasses import dataclass
from typing import List, Tuple

from shapely.geometry import LineString, MultiLineString, Point, Polygon, box
from shapely.ops import unary_union
from PIL import Image, ImageDraw
import numpy as np

# --- CONFIGURATION PARAMETERS ---

SAFETY_PADDING          = 5.0    # Space from origin for PCB bottom-left corner (mm)
FLOAT_TOLERANCE         = 1e-4   # Tolerance for float comparison
SMALL_HOLE_MAX_DIAMETER = 0.85   # Max diameter (mm) for 'small' drill group

SPINDLE_SPEED           = 12000  # RPM
CUT_DEPTH               = -0.1   # mm (trace isolation depth)
FINAL_CUT_DEPTH         = -1.8   # mm (final edge cut depth)
SAFE_HEIGHT             = 3.0    # mm above workpiece
PLUNGE_FEED_RATE        = 200    # mm/min (Z axis)
FEED_RATE               = 450    # mm/min (XY trace)
CUT_FEED_RATE           = 300    # mm/min (XY final cut)
HOLE_START_DEPTH        = 0.1    # mm before slow drilling
HOLE_FINAL_DEPTH        = -1.8   # mm final hole depth

ISOLATION_OFFSET        = 0.1   # mm (distance from trace center, first pass)
ISOLATION_PASSES        = 3      # number of isolation passes
PASS_SPACING            = 0.02    # mm between passes

EDGE_CUT_CLEARANCE_OFFSET = 1.0  # mm (mill outline this much larger)


# ── Bounds ────────────────────────────────────────────────────────────────────

@dataclass
class Bounds:
    x_min: float = 1e6
    x_max: float = -1e6
    y_min: float = 1e6
    y_max: float = -1e6

    def update(self, x: float, y: float, margin: float = 0.0):
        self.x_min = min(self.x_min, x - margin)
        self.x_max = max(self.x_max, x + margin)
        self.y_min = min(self.y_min, y - margin)
        self.y_max = max(self.y_max, y + margin)

    def merge(self, other: "Bounds"):
        self.x_min = min(self.x_min, other.x_min)
        self.x_max = max(self.x_max, other.x_max)
        self.y_min = min(self.y_min, other.y_min)
        self.y_max = max(self.y_max, other.y_max)


# ── GerberTracesParser ────────────────────────────────────────────────────────

class GerberTracesParser:
    def __init__(self, filename: str):
        self.apertures: dict = {}
        self.macros: dict = {}
        self.current_aperture: int = -1
        self.traces: list = []
        self.pads: list = []
        self.regions: list = []
        self.unit_mult: float = 1.0
        self.current_x: float = -1000.0
        self.current_y: float = -1000.0
        self.bounds = Bounds()
        self.in_region: bool = False
        self._region_contours: list = []
        self._region_current: list = []

        try:
            self._parse_gerber_file(filename)
        except FileNotFoundError:
            print(f"Error: Trace file '{filename}' not found.")
            sys.exit(1)

    def _process_extended_command(self, line: str):
        if not line:
            return

        m = re.match(r'%ADD(\d+)([^,]+),([^*]+)\*%', line)
        if m:
            num   = int(m.group(1))
            atype = m.group(2)
            params = m.group(3).split('X')

            if atype == 'C':
                self.apertures[num] = {'type': 'circle', 'diameter': float(params[0])}
            elif atype == 'R':
                w = float(params[0])
                h = float(params[1]) if len(params) > 1 else w
                self.apertures[num] = {'type': 'rectangle', 'width': w, 'height': h}
            elif atype == 'O':
                w = float(params[0])
                h = float(params[1]) if len(params) > 1 else w
                self.apertures[num] = {'type': 'obround', 'width': w, 'height': h}
            elif atype == 'RoundRect':
                # KiCad RoundRect params: corner_radius, -half_w, -half_h, half_w, half_h
                w = abs(float(params[1])) + abs(float(params[3]))
                h = abs(float(params[2])) + abs(float(params[4]))
                self.apertures[num] = {'type': 'rectangle', 'width': w, 'height': h}
            elif atype in self.macros:
                shape = self._eval_macro(atype, params)
                if shape is not None:
                    self.apertures[num] = shape
                else:
                    print(f"Aperture macro '{atype}' not supported, pad ignored")

        if 'MOMM*%' in line:
            self.unit_mult = 1.0
        elif 'MOIN*%' in line:
            self.unit_mult = 25.4

    def _process_command(self, line: str):
        line = line.rstrip('*')

        # Region (zone fill) mode: G36 starts a filled-polygon block, G37 ends it.
        # Coordinates inside must NOT be treated as stroked traces (no aperture
        # width applies to a region outline) — otherwise they get drawn as thin
        # zig-zag traces following the fill's boundary using whatever aperture
        # happened to be last selected.
        if line == 'G36':
            self.in_region = True
            self._region_contours = []
            self._region_current = []
            return
        if line == 'G37':
            if self._region_current:
                self._region_contours.append(self._region_current)
            self.regions.extend(c for c in self._region_contours if len(c) >= 3)
            self.in_region = False
            self._region_contours = []
            self._region_current = []
            return

        # Aperture selection (D10+)
        m = re.match(r'D(\d+)$', line)
        if m:
            num = int(m.group(1))
            if num >= 10:
                self.current_aperture = num
            return

        m = re.match(r'X(-?[0-9.]+)Y(-?[0-9.]+)D0([123])?', line)
        if not m:
            return

        x  = float(m.group(1)) * 1e-6 * self.unit_mult
        y  = float(m.group(2)) * 1e-6 * self.unit_mult
        op = int(m.group(3)) if m.group(3) else 2  # default: move (D02)

        if self.in_region:
            self.bounds.update(x, y, margin=0.6)
            if op == 2:
                if self._region_current:
                    self._region_contours.append(self._region_current)
                self._region_current = [(x, y)]
            else:
                self._region_current.append((x, y))
            self.current_x, self.current_y = x, y
            return

        self.bounds.update(x, y, margin=1.5 if op == 3 else 0.6)

        if op == 1 and self.current_aperture in self.apertures:
            ap    = self.apertures[self.current_aperture]
            width = ap.get('diameter', ap.get('width', 0.1))
            self.traces.append([(self.current_x, self.current_y), (x, y), width])
        elif op == 3 and self.current_aperture in self.apertures:
            self.pads.append([(x, y), self.apertures[self.current_aperture]])

        self.current_x, self.current_y = x, y

    def _parse_gerber_file(self, filename: str):
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
        lines = [line.strip() for line in content.split('\n')]

        i = 0
        while i < len(lines):
            line = lines[i]
            if line.startswith('%AM'):
                # Aperture macro definitions can span several physical lines;
                # keep consuming until the block's closing '*%' is reached.
                block = [line]
                while not block[-1].endswith('*%') and i + 1 < len(lines):
                    i += 1
                    block.append(lines[i])
                self._process_macro_definition(''.join(block))
            elif line.startswith('%') or not line:
                self._process_extended_command(line)
            else:
                self._process_command(line)
            i += 1

    def _process_macro_definition(self, text: str):
        m = re.match(r'%AM([^*]+)\*(.*)%$', text)
        if not m:
            return
        name, body = m.group(1), m.group(2)
        self.macros[name] = [s for s in body.split('*') if s.strip()]

    def _resolve_macro_values(self, tokens: list, params: list) -> list:
        values = []
        for tok in tokens:
            expr = re.sub(r'\$(\d+)',
                           lambda mm: params[int(mm.group(1)) - 1], tok)
            try:
                values.append(float(eval(expr, {"__builtins__": {}}, {})))
            except Exception:
                values.append(0.0)
        return values

    def _eval_macro(self, name: str, params: list):
        """Evaluate a Gerber aperture macro (currently: outline/polygon
        primitive '4' only) into a pad shape usable by ShapelyBases /
        OutputVisualizer. Returns None if the macro can't be represented."""
        polygons = []
        for stmt in self.macros.get(name, []):
            tokens = [t.strip() for t in stmt.split(',')]
            code = tokens[0]
            if code != '4':
                continue  # other primitives (circle/line/comment) not supported
            values = self._resolve_macro_values(tokens[1:], params)
            if len(values) < 2:
                continue
            # Gerber outline primitive: n = vertex count MINUS 1 — the polygon
            # is closed by repeating the first point, so n+1 coordinate pairs
            # follow before the trailing rotation value.
            num_points = int(values[1])
            n_coords = 2 * (num_points + 1)
            coords = values[2:2 + n_coords]
            rotation = values[2 + n_coords] if len(values) > 2 + n_coords else 0.0
            rot = math.radians(rotation)
            cos_r, sin_r = math.cos(rot), math.sin(rot)
            pts = []
            for j in range(0, len(coords) - 1, 2):
                x, y = coords[j], coords[j + 1]
                pts.append((x * cos_r - y * sin_r, x * sin_r + y * cos_r))
            if len(pts) >= 3:
                polygons.append(pts)

        if len(polygons) == 1:
            return {'type': 'polygon', 'points': polygons[0]}
        return None

    def shift(self, dx: float, dy: float):
        for trace in self.traces:
            for i in range(2):
                x, y = trace[i]
                trace[i] = [x - dx, y - dy]
        for pad in self.pads:
            x, y = pad[0]
            pad[0] = [x - dx, y - dy]
        for region in self.regions:
            for i in range(len(region)):
                x, y = region[i]
                region[i] = (x - dx, y - dy)


# ── GerberEdgeCutsParser ──────────────────────────────────────────────────────

class GerberEdgeCutsParser:
    def __init__(self, filename: str):
        self.raw_outline: list = []
        self.outline: list = []
        self.unit_mult: float = 1.0
        self.bounds = Bounds()

        try:
            with open(filename, 'r', encoding='utf-8') as f:
                self._parse(f)
        except FileNotFoundError:
            print("No edge cuts defined, that's OK.")
            return

        self.outline = self.raw_outline[:]

    def _parse(self, f):
        for line in f:
            line = line.strip()
            if 'MOMM*%' in line:
                self.unit_mult = 1.0
            elif 'MOIN*%' in line:
                self.unit_mult = 25.4

            m = re.match(r'X(-?[0-9.]+)Y(-?[0-9.]+)D0([123])?', line)
            if m:
                x = float(m.group(1)) * 1e-6 * self.unit_mult
                y = float(m.group(2)) * 1e-6 * self.unit_mult
                self.bounds.update(x, y, margin=0.2)
                self.raw_outline.append((x, y))


# ── DrillfileParser ───────────────────────────────────────────────────────────

class DrillfileParser:
    def __init__(self, filename: str):
        self.tool_diameters: dict = {}
        self.holes: list = []
        self.small_holes: list = []
        self.large_holes: list = []
        self.bounds = Bounds()
        self.units_mult: float = 1.0

        try:
            with open(filename, 'r', encoding='utf-8') as f:
                self._parse(f)
        except FileNotFoundError:
            print("No drill file, that's OK.")

    def _parse(self, f):
        current_tool = None
        for line in f:
            line = line.strip()
            if not line or line.startswith(';'):
                continue

            if 'METRIC' in line.upper():
                self.units_mult = 1.0
            elif 'INCH' in line.upper():
                self.units_mult = 25.4

            m = re.match(r'^T(\d+)C([\d.]+)', line)
            if m:
                self.tool_diameters[m.group(1)] = float(m.group(2)) * self.units_mult
                continue

            m = re.match(r'^T(\d+)$', line)
            if m:
                current_tool = m.group(1)
                continue

            m = re.match(r'^X(-?\d+(?:\.\d+)?)Y(-?\d+(?:\.\d+)?)', line)
            if m and current_tool:
                x   = float(m.group(1)) * self.units_mult
                y   = float(m.group(2)) * self.units_mult
                dia = self.tool_diameters[current_tool]
                hole = (x, y, dia)

                self.holes.append(hole)
                (self.small_holes if dia <= SMALL_HOLE_MAX_DIAMETER else self.large_holes).append(hole)
                self.bounds.update(x, y)
                print(f"Hole ({x:5.1f},{y:5.1f}), dia={dia:5.2f}")

    def shift(self, dx: float, dy: float):
        def _s(lst):
            return [(h[0] - dx, h[1] - dy, h[2]) for h in lst]
        self.holes       = _s(self.holes)
        self.small_holes = _s(self.small_holes)
        self.large_holes = _s(self.large_holes)


# ── ShapelyBases ──────────────────────────────────────────────────────────────

class ShapelyBases:
    def __init__(self, parser: GerberTracesParser):
        shapes = []
        for trace in parser.traces:
            shapes.append(LineString([trace[0], trace[1]]).buffer(trace[2] / 2))
        for pad in parser.pads:
            x, y = pad[0]
            ap   = pad[1]
            if ap['type'] == 'circle':
                shapes.append(Point(x, y).buffer(ap['diameter'] / 2))
            elif ap['type'] == 'rectangle':
                hw, hh = ap['width'] / 2, ap['height'] / 2
                shapes.append(box(x - hw, y - hh, x + hw, y + hh))
            elif ap['type'] == 'obround':
                w, h = ap['width'], ap['height']
                radius = min(w, h) / 2
                if w >= h:
                    dx = (w - h) / 2
                    axis = LineString([(x - dx, y), (x + dx, y)])
                else:
                    dy = (h - w) / 2
                    axis = LineString([(x, y - dy), (x, y + dy)])
                shapes.append(axis.buffer(radius))
            elif ap['type'] == 'polygon':
                pts = [(x + px, y + py) for px, py in ap['points']]
                poly = Polygon(pts)
                if not poly.is_valid:
                    poly = poly.buffer(0)
                shapes.append(poly)
            else:
                print(f"Pad type '{ap['type']}' ignored")
        for region in parser.regions:
            poly = Polygon(region)
            if not poly.is_valid:
                poly = poly.buffer(0)
            shapes.append(poly)

        self.combined_geometry = unary_union(shapes)

    def compute_trace_toolpaths(self, offset: float, num_passes: int, spacing: float) -> MultiLineString:
        all_paths = []
        for i in range(num_passes):
            path = self.combined_geometry.buffer(offset + spacing * i).simplify(0.03).boundary
            if isinstance(path, LineString):
                path = MultiLineString([path])
            if hasattr(path, 'geoms'):
                all_paths.extend(path.geoms)
        return MultiLineString(all_paths)


# ── OutputVisualizer ──────────────────────────────────────────────────────────

class OutputVisualizer:
    SCALE = 25  # pixels per mm

    # KiCad-ish palette: gray board, red copper, yellow board outline,
    # orange fill for copper islands (zone/region fills).
    COLOR_BACKGROUND  = (0, 0, 0)
    COLOR_BOARD       = (60, 60, 60)
    COLOR_OUTLINE     = (255, 255, 0)
    COLOR_CLEARANCE   = (255, 165, 0)
    COLOR_COPPER      = (200, 0, 0)
    COLOR_ISLAND_FILL = (200, 80, 80)
    COLOR_TOOLPATH    = (20, 20, 20)
    COLOR_HOLE        = (0, 0, 0)

    def __init__(self):
        self.holes: list = []
        self.traces: GerberTracesParser = None
        self.trace_mill_geometry = None
        self.edgecuts: list = []

    def load_trace_geometries(self, parser: GerberTracesParser):
        self.traces = parser

    def load_holes(self, holes: list):
        self.holes = holes

    def load_trace_mill_geometry(self, geom):
        self.trace_mill_geometry = geom

    def load_edge_cut_geometry(self, coords: list):
        self.edgecuts = coords

    def save_png_visualization(self, filename: str, bounds: Bounds):
        s          = self.SCALE
        margin_mm  = 5
        width_px   = int((bounds.x_max - bounds.x_min + 2 * margin_mm) * s)
        height_px  = int((bounds.y_max - bounds.y_min + 2 * margin_mm) * s)
        x_off      = int((margin_mm - bounds.x_min) * s)
        y_off      = int((margin_mm - bounds.y_min) * s)

        def to_px(x, y):
            return int(x * s + x_off), int(height_px - (y * s + y_off))

        img  = Image.new('RGB', (width_px, height_px), color=self.COLOR_BACKGROUND)
        draw = ImageDraw.Draw(img)

        # Edge cuts
        if self.edgecuts:
            buffered   = LineString(self.edgecuts).buffer(EDGE_CUT_CLEARANCE_OFFSET, cap_style=3, join_style=2)
            cleared_px = [to_px(x, y) for x, y in buffered.exterior.coords]
            original_px = [to_px(x, y) for x, y in self.edgecuts]

            if len(original_px) >= 3:
                draw.polygon(original_px, fill=self.COLOR_BOARD, outline=self.COLOR_OUTLINE, width=1)
                if len(cleared_px) >= 2:
                    draw.line(cleared_px, fill=self.COLOR_CLEARANCE, width=2)
            elif len(original_px) == 2:
                draw.line(original_px, fill=self.COLOR_OUTLINE, width=2)
            else:
                print("Warning: Edge cuts insufficient to draw.")

        # Traces
        if self.traces:
            for trace in self.traces.traces:
                x1, y1 = to_px(*trace[0])
                x2, y2 = to_px(*trace[1])
                w_px   = max(1, int(trace[2] * s))
                draw.line((x1, y1, x2, y2), fill=self.COLOR_COPPER, width=w_px)

            for pad in self.traces.pads:
                cx, cy = to_px(*pad[0])
                ap     = pad[1]
                if ap['type'] == 'circle':
                    r = int(ap['diameter'] / 2 * s)
                    draw.ellipse((cx - r, cy - r, cx + r, cy + r),
                                 fill=self.COLOR_COPPER, outline=self.COLOR_COPPER)
                elif ap['type'] == 'rectangle':
                    hw = int(ap['width']  / 2 * s)
                    hh = int(ap['height'] / 2 * s)
                    draw.rectangle((cx - hw, cy - hh, cx + hw, cy + hh),
                                   fill=self.COLOR_COPPER, outline=self.COLOR_COPPER)
                elif ap['type'] == 'obround':
                    hw = int(ap['width']  / 2 * s)
                    hh = int(ap['height'] / 2 * s)
                    draw.rounded_rectangle((cx - hw, cy - hh, cx + hw, cy + hh),
                                            radius=min(hw, hh),
                                            fill=self.COLOR_COPPER, outline=self.COLOR_COPPER)
                elif ap['type'] == 'polygon':
                    ox, oy = pad[0]
                    pts_px = [to_px(ox + px, oy + py) for px, py in ap['points']]
                    if len(pts_px) >= 3:
                        draw.polygon(pts_px, fill=self.COLOR_COPPER, outline=self.COLOR_COPPER)

            for region in self.traces.regions:
                region_px = [to_px(x, y) for x, y in region]
                if len(region_px) >= 3:
                    draw.polygon(region_px, fill=self.COLOR_ISLAND_FILL, outline=self.COLOR_COPPER)

        # Toolpaths
        if self.trace_mill_geometry:
            for shape in self.trace_mill_geometry.geoms:
                pts = [to_px(x, y) for x, y in shape.coords]
                if len(pts) > 1:
                    draw.line(pts, fill=self.COLOR_TOOLPATH, width=2)

        # Holes
        for x, y, dia in self.holes:
            cx, cy = to_px(x, y)
            r      = int(dia / 2 * s)
            draw.ellipse((cx - r, cy - r, cx + r, cy + r),
                         fill=self.COLOR_HOLE, outline=self.COLOR_OUTLINE, width=1)

        img.save(filename)
        print(f"Visualization saved to '{filename}'")


# ── GcodeGenerator ────────────────────────────────────────────────────────────

class GcodeGenerator:
    def _write_header(self, f, description: str, tool_num: int, tool_name: str, speed: int = SPINDLE_SPEED):
        f.write("%\n")
        f.write(f"; GCODE FILE: {description}\n")
        f.write("G21  ; Set units to mm\n")
        f.write("G90  ; Absolute positioning\n")
        f.write(f"G0 Z{SAFE_HEIGHT:.2f}  ; Move to safe height\n")
        f.write(f"(load {tool_name})\nT{tool_num} M06\n")
        f.write(f"S{speed} M3  ; Start spindle clockwise\n")
        f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")

    def _write_footer(self, f):
        f.write("M5  ; Stop spindle\n")
        f.write("G0 X0 Y50.0 Z50  ; Return home\n")
        f.write("M30 ; End of program\n")
        f.write("%\n")

    def OutputEngravingGcode(self, filename: str, trace_mill_geometry):
        with open(filename, 'w') as f:
            self._write_header(f, "Trace Isolation Milling", 1, "0.2 mm engraving tool (T1)")
            f.write("\n; --- TRACE ISOLATION MILLING ---\n")
            for path in trace_mill_geometry.geoms:
                started = False
                for x, y in path.coords:
                    if not started:
                        f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
                        f.write(f"G0 Z0.1\n")
                        f.write(f"G1 Z{CUT_DEPTH:.3f} F{PLUNGE_FEED_RATE}\n")
                        f.write(f"G1 F{FEED_RATE}\n")
                        started = True
                    else:
                        f.write(f"G1 X{x:.2f} Y{y:.2f}\n")
                f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")
            self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    def OutputEdgeCutGcode(self, filename: str, edgecuts: list):
        if not edgecuts:
            print(f"Warning: No edge cuts found. Skipping '{filename}'")
            return

        cleared = list(
            LineString(edgecuts)
            .buffer(EDGE_CUT_CLEARANCE_OFFSET, cap_style=3, join_style=2)
            .exterior.coords
        )

        with open(filename, 'w') as f:
            self._write_header(f, f"PCB Edge Cut (offset +{EDGE_CUT_CLEARANCE_OFFSET}mm)",
                               4, "Edge Cut Tool (T4 - 1.0mm+)")
            f.write("\n; --- PCB FINAL EDGE CUT ---\n")
            f.write(f"; Toolpath offset: {EDGE_CUT_CLEARANCE_OFFSET} mm\n")
            started = False
            for x, y in cleared:
                if not started:
                    f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
                    f.write(f"G0 Z0.1\n")
                    f.write(f"G1 Z{FINAL_CUT_DEPTH:.3f} F{PLUNGE_FEED_RATE}\n")
                    f.write(f"G1 F{CUT_FEED_RATE:.1f}\n")
                    started = True
                else:
                    f.write(f"G1 X{x:.2f} Y{y:.2f}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")
            self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    def _output_drill_gcode(self, filename: str, holes: list,
                            tool_num: int, tool_name: str, description: str):
        if not holes:
            print(f"Warning: No holes found. Skipping '{filename}'")
            return
        with open(filename, 'w') as f:
            self._write_header(f, description, tool_num, tool_name)
            f.write(f"\n; --- DRILLING ({description}) ---\n")
            for x, y, _ in holes:
                f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
                f.write(f"G0 Z{HOLE_START_DEPTH:.2f}\n")
                f.write(f"G1 Z{HOLE_FINAL_DEPTH:.2f} F{PLUNGE_FEED_RATE}\n")
                f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")
            f.write(f"G0 Z{SAFE_HEIGHT:.2f}\n")
            self._write_footer(f)
        print(f"G-code generated in '{filename}'")

    def OutputSmallDrillGcode(self, filename: str, holes: list):
        self._output_drill_gcode(filename, holes, 2, "small drill (T2)", "Small Hole Drilling")

    def OutputLargeDrillGcode(self, filename: str, holes: list):
        self._output_drill_gcode(filename, holes, 3, "large drill (T3)", "Large Hole Drilling")


# ── Contour optimization ──────────────────────────────────────────────────────

def optimize_contour_points(contour_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Position, circularly sequence, and deduplicate contour points."""
    if not contour_coords:
        return []

    arr = np.array(contour_coords)

    # 1. Shift so bottom-left corner lands at (SAFETY_PADDING, SAFETY_PADDING)
    offset_x = SAFETY_PADDING - arr[:, 0].min()
    offset_y = SAFETY_PADDING - arr[:, 1].min()
    print(f"CORNER POS: offset X={offset_x:.2f}, Y={offset_y:.2f} mm")
    arr[:, 0] += offset_x
    arr[:, 1] += offset_y

    # 2. Sort by angle around centroid
    cx, cy = arr.mean(axis=0)
    arr = arr[np.argsort(np.arctan2(arr[:, 1] - cy, arr[:, 0] - cx))]

    # 3. Roll so start = point closest to (min_x, min_y)
    min_x, min_y = arr[:, 0].min(), arr[:, 1].min()
    arr = np.roll(arr, -np.hypot(arr[:, 0] - min_x, arr[:, 1] - min_y).argmin(), axis=0)

    # 4. Deduplicate
    pts = [(round(float(x), 4), round(float(y), 4)) for x, y in arr]
    clean = [pts[0]]
    for p in pts[1:]:
        if not (abs(p[0] - clean[-1][0]) < FLOAT_TOLERANCE and
                abs(p[1] - clean[-1][1]) < FLOAT_TOLERANCE):
            clean.append(p)

    # 5. Close loop
    if len(clean) >= 2 and clean[0] != clean[-1]:
        clean.append(clean[0])

    print(f"OPTIMIZATION: {len(clean)} contour points")
    return clean


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("\n--- KiCad Gerber to CNC G-Code Generator ---")
        base_name = input("Enter the project base name (e.g., 'MyBoard'): ").strip().replace("\\", "/")
        if not base_name:
            print("Error: Project name cannot be empty.")
            sys.exit(1)
        if not os.path.exists(base_name + "-F_Cu.gbr"):
            print(f"Error: '{base_name}-F_Cu.gbr' not found.")
            sys.exit(1)
    else:
        base_name = sys.argv[1].replace("\\", "/")

    out_name = base_name.split("/")[-1]
    print(f"Loading files with base name: '{base_name}'")

    # Parse all input files
    gerber_traces   = GerberTracesParser(base_name + "-F_Cu.gbr")
    gerber_edgecuts = GerberEdgeCutsParser(base_name + "-Edge_cuts.gbr")
    drilldata       = DrillfileParser(base_name + "-PTH.drl")

    # Aggregate bounds from all parsers
    global_bounds = Bounds()
    global_bounds.merge(gerber_traces.bounds)
    global_bounds.merge(gerber_edgecuts.bounds)
    global_bounds.merge(drilldata.bounds)

    # Shift traces and holes so PCB starts at SAFETY_PADDING
    dx = global_bounds.x_min - SAFETY_PADDING
    dy = global_bounds.y_min - SAFETY_PADDING
    print(f"SHIFT: dx={dx:.2f}, dy={dy:.2f}")
    gerber_traces.shift(dx, dy)
    drilldata.shift(dx, dy)

    # Optimize edge cut contour (applies its own absolute positioning)
    optimized_edge_cuts = optimize_contour_points(gerber_edgecuts.raw_outline)
    if optimized_edge_cuts:
        gerber_edgecuts.outline = optimized_edge_cuts
        ec_arr = np.array(optimized_edge_cuts)
        final_bounds = Bounds(
            x_min=float(ec_arr[:, 0].min()), x_max=float(ec_arr[:, 0].max()),
            y_min=float(ec_arr[:, 1].min()), y_max=float(ec_arr[:, 1].max()),
        )
        print(f"Final Bounds: X:{final_bounds.x_min:.2f}-{final_bounds.x_max:.2f}, "
              f"Y:{final_bounds.y_min:.2f}-{final_bounds.y_max:.2f}")
    else:
        final_bounds = global_bounds

    # Compute trace isolation toolpaths
    sh_base             = ShapelyBases(gerber_traces)
    trace_mill_geometry = sh_base.compute_trace_toolpaths(ISOLATION_OFFSET, ISOLATION_PASSES, PASS_SPACING)

    # Visualization
    viz = OutputVisualizer()
    viz.load_trace_geometries(gerber_traces)
    viz.load_trace_mill_geometry(trace_mill_geometry)
    viz.load_edge_cut_geometry(gerber_edgecuts.outline)
    viz.load_holes(drilldata.holes)
    viz.save_png_visualization(f"{out_name}_visualization.png", final_bounds)

    # G-code output
    gcode = GcodeGenerator()
    gcode.OutputEngravingGcode(f"{out_name}_Engraving.nc",    trace_mill_geometry)
    gcode.OutputSmallDrillGcode(f"{out_name}_Small_Drill.nc", drilldata.small_holes)
    gcode.OutputLargeDrillGcode(f"{out_name}_Large_Drill.nc", drilldata.large_holes)
    gcode.OutputEdgeCutGcode(f"{out_name}_Edge_Cut.nc",       gerber_edgecuts.outline)


if __name__ == "__main__":
    main()
