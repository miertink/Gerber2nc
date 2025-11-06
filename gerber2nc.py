#!/usr/bin/python3
# Program for taking output from a simple single sided KiCad PCB layout
# and turn it into tool paths for milling out on a CNC.
#
# Single layer front side only, traces, holes and pads only, no rotated pads.
# KiCad output only (have not tried it with any other program)
#
# To change various parameters, search for 'parameters' in the code
#
# requires shapely library (pip install shapely)
#
# Matthias Wandel August 2025

import sys, re, math
from shapely.geometry import LineString, MultiLineString, Point, box
from shapely.ops import unary_union


class Gerber_Traces_Parser:
    def __init__(self, filename: str):
        self.apertures:dict = {}
        self.current_aperture:int = -1
        self.traces:list = []

        self.pads:list = []
        self.unit_mult:float = 1.0
        self.current_x:float = -1000.0
        self.current_y:float = -1000.0

        self._parse_gerber_file(filename)

    def _process_extended_command(self, line:str):
        # Process extended commands (those starting with %)
        if not line:
            return

        # Aperture definition - handle various types
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
            elif aperture_type == 'RoundRect':  # Rounded Rectangle
                corner_radius = float(params[0])
                x1, y1 = float(params[1]), float(params[2])
                x2, y2 = float(params[3]), float(params[4])

                # Calculate width and height from coordinates
                width = abs(x2) + abs(x1) + corner_radius
                height = abs(y2)+ abs(y1) + corner_radius

                # Just treat it as a rectangle.
                self.apertures[aperture_num] = {'type': 'rectangle', 'width': width, 'height': height}

        # Units
        if 'MOMM*%' in line:
            self.unit_mult = 1
        elif 'MOIN*%' in line:
            self.unit_mult = 25.4

    def _process_command(self, line:str):
        # Process regular Gerber commands
        global x_min,x_max,y_min,y_max

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
            # Parse coordinates.  From KiCad, 1 million units per mm!
            x = float(coord_match.group(1)) * 0.000001 * self.unit_mult
            y = float(coord_match.group(2)) * 0.000001 * self.unit_mult
            operation = int(coord_match.group(3))

            # Compute extents.  More margin around pads as those can be bigger.
            m = 1.5 if operation == 3 else 0.6
            if x-m < x_min: x_min = x-m
            if x+m > x_max: x_max = x+m
            if y-m < y_min: y_min = y-m
            if y+m > y_max: y_max = y+m

            # Process operation
            if operation == 1:  # Linear interpolation with exposure on
                if self.current_aperture and self.current_aperture in self.apertures:
                    aperture = self.apertures[self.current_aperture]
                    width = aperture.get('diameter', aperture.get('width', 0.1))
                    self.traces.append([(self.current_x, self.current_y),(x, y),width])
            elif operation == 2:  # Move without exposure
                pass

            elif operation == 3:  # Flash (create pad)
                if self.current_aperture and self.current_aperture in self.apertures:
                    aperture = self.apertures[self.current_aperture]
                    self.pads.append([(x, y),aperture])

            # Update current position
            self.current_x, self.current_y = x, y

    def _parse_gerber_file(self, filename:str):
        # Parse a Gerber file and extract traces and aperture information
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()

        # Split into lines and process
        lines = content.split('\n')

        for line in lines:
            line = line.strip()
            if not line or line.startswith('%'):
                self._process_extended_command(line)
            else:
                self._process_command(line)

    def shift(self, x_shift:float, y_shift:float):
        # Shift all the coordinates so that X and Y go from zero up
        for trace in self.traces:
            for startstop in range(2):
                x,y = trace[startstop]
                trace[startstop] = [x-x_shift,y-y_shift]

        for pad in self.pads:
            x, y = pad[0]
            pad[0] = [x-x_shift, y-y_shift]
            aperture = pad[0]

#=========================================================================================
class Gerber_EdgeCuts_Parser:
    def __init__(self, filename:str):
        self.outline: list[tuple[float, float]] = []
        self.unit_mult:float = 1.0
        global x_min,x_max,y_min,y_max

        # Parse the edge cuts gerber file.
        try:
            f = open(filename, 'r', encoding='utf-8')
        except:
            print("No edge cuts defined, thats OK")
            return

        for line in f:
            line = line.strip()

            # Units
            if 'MOMM*%' in line:
                self.unit_mult = 1.0
            elif 'MOIN*%' in line:
                self.unit_mode = 25.4

            # Aperture selection -- just ignore for now.
            #aperture_match = re.match(r'D(\d+)', line)

            # Coordinate and operation commands
            coord_match = re.match(r'X(-?[0-9.]+)Y(-?[0-9.]+)D0([0123])?', line)
            if coord_match:
                # Parse coordinates.  From KiCad, 1 million units per mm!
                x = float(coord_match.group(1)) * 0.000001 * self.unit_mult
                y = float(coord_match.group(2)) * 0.000001 * self.unit_mult
                operation = coord_match.group(3)

                m = 0.2 # 0.2 mm margin around edge cuts so we can see them on the screen
                if x-m < x_min: x_min = x-m
                if x+m > x_max: x_max = x+m
                if y-m < y_min: y_min = y-m
                if y+m > y_max: y_max = y+m

                if self.outline and operation != '1':
                    # If you draw the PCB outline as individual line segments one at
                    # a time in KiCad, you will hit this error.  Non rectangular outlines
                    # may work as long as they are drawn as ONE polyline.
                    print("Outline must be drawn as ONE rectangle")
                self.outline.append((x,y))

        if self.outline and self.outline[0] != self.outline[-1]:
            print("Error: non closed outline")

    def shift(self, x_base, y_base):
        # Offset same way as traces were offset.
        for i in range(0, len(self.outline)):
            point = self.outline[i]
            self.outline[i] = (point[0]-x_base, point[1]-y_base)

#=========================================================================================
class Drillfile_Parser:
    def __init__(self,filename:str):
        self.tool_diameters:dict = {}
        self.holes: list[tuple[float, float, float]] = []

        current_tool = None
        self.units_mult = 1.0
        global x_min,x_max,y_min,y_max

        # Parse the edge cuts gerber file.
        try:
            f = open(filename, 'r', encoding='utf-8')
        except:
            print("No drill file, thats OK")
            # Maybe your board is all SMT and doesn't need holes?
            return

        for line in f:
            line = line.strip()
            if not line or line.startswith(";"):  continue # comment

            # Detect units
            if "METRIC" in line.upper(): self.units_mult = 1.0
            elif "INCH" in line.upper(): self.units_mult = 25.4

            # Tool definition: e.g., T01C0.800
            match_tool = re.match(r"^T(\d+)C([\d\.]+)", line)
            if match_tool:
                tool_num = match_tool.group(1)
                diameter = float(match_tool.group(2))*self.units_mult
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
                x = float(match_coord.group(1))*self.units_mult
                y = float(match_coord.group(3))*self.units_mult

                dia = self.tool_diameters[current_tool]
                #print("dia=",dia)
                self.holes.append((x, y, dia))
                print("Hole (%5.1f,%5.1f),%5.2f"%(x, y, dia))

                if x < x_min: x_min = x
                if x > x_max: x_max = x
                if y < y_min: y_min = y
                if y > y_max: y_max = y

    def shift(self, x_shift:float, y_shift:float):
        # Offset same way as traces were offset.
        for i in range(0, len(self.holes)):
            hole = self.holes[i]
            self.holes[i] = (hole[0]-x_shift, hole[1]-y_shift, hole[2])


#=========================================================================================

class Shapely_bases:
    def __init__(self, parser):
        # Init and Add the geometries
        traces = []
        pads = []

        # Add traces
        for trace in parser.traces:
            start_x, start_y = trace[0]
            end_x, end_y = trace[1]
            width_mm = trace[2]
            #print("line mm %5.1f,%5.1f -> %5.1f,%5.1f  w=%d"%(start_x,start_y,end_x,end_y, width_mm))
            traces.append(LineString([trace[0], trace[1]]).buffer(width_mm/2))

        # Add pads
        for pad in parser.pads:
            x_mm, y_mm = pad[0]
            aperture = pad[1]

            if aperture['type'] == 'circle':
                print("circle pad at: (%7.2f,%7.2f)"%(x_mm,y_mm))
                radius = (aperture['diameter'] / 2)
                pads.append(Point(x_mm, y_mm).buffer(radius))
            elif aperture['type'] == 'rectangle':
                width = aperture['width']/2
                height = aperture['height']/2
                pads.append(box(x_mm-width, y_mm-height, x_mm+width, y_mm+height))
            else:
                print("Pad type :",aperture['type'],"ignored")

        # Combine shapes into one geometry for shapely library
        self.combined_geometry = unary_union(traces+pads)

    def compute_trace_toolpaths(self, offset_distance:float, num_passes:int, path_spacing:float):
        all_passes = []

        for passnum in range (0, num_passes):
            offset = offset_distance + path_spacing * passnum

            # This call computes the offset toolpath.
            thispath = self.combined_geometry.buffer(offset).simplify(0.03).boundary

            if thispath == "LineString": thispath = MultiLineString(thispath)
            all_passes += list(thispath.geoms)

        return MultiLineString(all_passes)

#=========================================================================================
class Output_visualizer:
    # Show the PCB and tool paths on the screen for review (I caught a lot of errors this way)
    def __init__(self):
        self.offset_geometry = False # None yet
        self.holes = []
        self.scale = 25  # pixels per mm

    def load_trace_geometries(self, traces):
        self.traces = traces

    def load_holes(self, p_holes:list):
        self.holes = p_holes

    def load_trace_mill_geometry(self, offsets):
        self.trace_mill_geometry = offsets

    def load_edge_cut_geometry(self, edgecuts):
        self.edgecuts = edgecuts

    def create_tkinter_visualization(self):
        # Create tkinter canvas visualization of the traces
        import tkinter as tk # Import locally as its only used here.
        root = tk.Tk()
        self.canvas = None
        global base_name
        root.title(base_name+':   Edge cut paths in white.  Close this window to continue')

        # Calculate canvas size based on content
        global x_min,x_max,y_min,y_max
        width_mm =  x_max - x_min
        height_mm = y_max - y_min

        max_window_width = root.winfo_screenwidth() * 0.9
        if self.scale*width_mm > max_window_width: self.scale = max_window_width/width_mm

        canvas_width = int(width_mm * self.scale)
        canvas_height = int(height_mm * self.scale)

        self.canvas = tk.Canvas(root, width=canvas_width, height=canvas_height,
                    bg='#202020' if self.edgecuts else '#005000')

        self.canvas.pack(padx=5, pady=5)

        # Draw the edge cut toolpath and PCB color.  Edge cuts assumed to be one rectangle.
        if self.edgecuts:
            coords = []
            for x, y in self.edgecuts:
                screen_x = x * self.scale
                screen_y = canvas_height - y * self.scale
                coords.extend([screen_x, screen_y])

            self.canvas.create_polygon(coords[:-2], fill='#005000', outline="white", width=2)

        # Draw traces
        for trace in self.traces.traces:
            start_x, start_y = trace[0]
            end_x, end_y = trace[1]
            width_mm = trace[2]

            # Convert to canvas coordinates (flip Y axis for display)
            x1 = start_x * self.scale
            y1 = canvas_height - start_y * self.scale
            x2 = end_x * self.scale
            y2 = canvas_height - end_y * self.scale

            # Line width in pixels
            line_width = max(1, int(width_mm * self.scale))

            self.canvas.create_line(x1, y1, x2, y2, fill='red', width=line_width, capstyle=tk.ROUND)

        for pad in self.traces.pads:
            x_mm, y_mm = pad[0]
            aperture = pad[1]

            # Convert to canvas coordinates
            x = x_mm * self.scale
            y = canvas_height - y_mm * self.scale

            if aperture['type'] == 'circle':
                radius = (aperture['diameter'] / 2) * self.scale
                self.canvas.create_oval(x - radius, y - radius,
                                      x + radius, y + radius,
                                      fill='blue', outline='lightblue')

            elif aperture['type'] == 'rectangle':
                width = aperture['width'] * self.scale / 2
                height = aperture['height'] * self.scale / 2
                self.canvas.create_rectangle(x - width, y - height,
                                           x + width, y + height,
                                           fill='blue', outline='lightblue')

        for shape in self.trace_mill_geometry.geoms:
            coords = []
            for x, y in shape.coords:
                screen_x = x * self.scale
                screen_y = canvas_height - y * self.scale
                coords.extend([screen_x, screen_y])

            self.canvas.create_line(coords, fill="white", width=2)

        # Draw the holes.
        for hole in self.holes:
            x,y,diameter = hole
            screen_x = x * self.scale
            screen_y = canvas_height - y * self.scale
            radius = diameter/2 * self.scale

            self.canvas.create_oval(screen_x - radius, screen_y - radius,
                                      screen_x + radius, screen_y + radius,
                                      fill='black', outline='white', width=1)

        root.mainloop() # Show window until its clicked away.

#=========================================================================================
class Gcode_Generator:
    def __init__(self):
        # Milling parameters
        self.spindle_speed = 12000  # RPM
        self.cut_depth = -0.1       # mm (for outlinding traces)
        self.edge_cut_depth = -0.2  # mm For PCB outline
        self.safe_height = 3.0      # mm above workpiece
        self.plunge_feed_rate = 200 # mm/min
        self.feed_rate = 450        # mm/min

        self.hole_start = 0.1       # Depth to go to before slow drilling
        self.hole_depth = -1.8      # Final depth to make it through the PCB


    def OutputGcode (self, filename:str, edgecuts:list, trace_mill_geometry, holes:list):
        f = open(filename, "w")
        f.write("%\n")
        f.write("G21  ; Set units to mm\n")
        f.write("G90  ; Absolute positioning\n")
        f.write(f"G0 Z{self.safe_height}  ; Move to safe height\n")
        f.write(f"(load 0.2 mm engraving tool)\nT1 M06\n")
        f.write(f"S{self.spindle_speed} M3  ; Start spindle clockwise\n")

        # Generate G-code for trace edge geometries
        for path in trace_mill_geometry.geoms:
            started = False
            for x, y in path.coords:
                if not started:
                    # Rapid move to start of cut
                    f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
                    f.write(f"G0 Z0.1\n")
                    # Plunge to cut depth
                    f.write(f"G1 Z{self.cut_depth:.3f} F{self.plunge_feed_rate:d}\n")
                    f.write(f"G1 F{self.feed_rate:d}\n")
                    started = True
                else:
                    # Continue movement
                    f.write(f"G1 X{x:.2f} Y{y:.2f}\n")

            # Retract to safe height
            f.write(f"G0 Z{self.safe_height}\n")

        # With the same tool, mill around the edges, but a little bit deeper
        f.write("(mill edgecut mark)\n")
        started = False

        # Mark the circuit board edges eith the engraving tool.
        # not actually cutting it out, faster to cut it out with a hacksaw afterwards
        for x, y in edgecuts:
            if not started:
                # Rapid move to start of cut
                f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
                f.write(f"G0 Z0.1\n")
                # Plunge to cut depth
                f.write(f"G1 Z{self.edge_cut_depth:.3f} F{self.plunge_feed_rate:d}\n")
                f.write(f"G1 F{self.feed_rate:.1f}\n")
                started = True
            else:
                # Continue movement
                f.write(f"G1 X{x:.2f} Y{y:.2f}\n")

        # Retract to safe height
        f.write(f"G0 Z{self.safe_height}\n")
        f.write("M5  ; Stop spindle\n")

        # Next drill the holes.
        for do_large_holes in range (0,2):
            # We round all the holes to two sizes: small and large
            # If you need something like mounting holes, just use the small hole as a pilot
            # hole to drill the larger hole manually afterwards.
            drill_loaded = False

            for hole in holes:
                x,y,diameter = hole
                is_big_hole = diameter > 0.85

                if is_big_hole == do_large_holes:
                    if not drill_loaded:
                        # Change to this size drill only if we have holes for it.
                        f.write("(load %s drill)\n"%("large" if is_big_hole else "small"))
                        f.write("T%d M06\n"%(do_large_holes+2))
                        f.write(f"S{self.spindle_speed} M3  ; Start spindle clockwise\n")
                        drill_loaded = True

                    # Rapid move to start of hole
                    f.write(f"G0 X{x:.2f} Y{y:.2f}\n")
                    f.write(f"G0 Z{self.hole_start:.2f}\n")

                    # Slow drill the hole
                    f.write(f"G1 Z{self.hole_depth:.2f} F{self.plunge_feed_rate:d}\n")

                    # Lift back up.
                    f.write(f"G0 Z{self.safe_height}\n")

        # End G-code
        f.write("M5  ; Stop spindle\n")

        # Move the spindle out of the way.
        f.write(f"G0 X0 Y{y_max-y_min:.1f} Z50  ; Return home, raise spindle out of the way\n")
        f.write("M30 ; End of program\n")
        f.write("%\n")

        f.close();

        print("\nG-code generated in '%s'"%(filename))
#=========================================================================================

if len(sys.argv) < 2:
    print("Python script to make G-code form simple one side PCB layouts")
    print("Usage:")
    print("gerber2nc.py project [outname]")
    print("    Where project is where your KiCad project, gerber and drill files are")
    print("    For example, if your project file is in c:\\myproject\\myboard.kicad_pro")
    print("    You would speicfy 'c:\\myproject\\myboard'")
    print("")
    print("    [outname] is optional output file name, otherwise based on prject name");
    sys.exit();

base_name = sys.argv[1].replace("\\","/")
outname = base_name.split("/")[-1]+".gcode"

# For calculating extents of the board
x_min:float = 1000000.0
x_max:float = -1000000.0
y_min:float = 1000000.0
y_max:float = -1000000.0

# Use KiCad's naming convention to  get the copper front layer, ege cuts, and drill files.
gerber_traces = Gerber_Traces_Parser(base_name+"-F_Cu.gbr")
gerber_edgecuts = Gerber_EdgeCuts_Parser(base_name+"-Edge_cuts.gbr")
drilldata = Drillfile_Parser(base_name+"-PTH.drl")

# Offset all the coordinates so that the origin is on the bottom left.
# Set the CNC origin to the botom left corner of where your PCB should be milled.
gerber_traces.shift(x_min, y_min)
gerber_edgecuts.shift(x_min, y_min)
drilldata.shift(x_min, y_min)

# Now compute the outlines to use for tool paths.
# Parameters for outlne milling the traces
offset_distance = 0.22 # Offset of initial path from trace or pad edge
num_passes = 3         # Number of passes to take around traces
path_spacing = 0.2     # Additional offset per pass
sh_base = Shapely_bases(gerber_traces)
trace_mill_geometry = sh_base.compute_trace_toolpaths(offset_distance, num_passes, path_spacing)

# Now visualize it, then wait for window to be closed before proceeding.
visualizer = Output_visualizer()
visualizer.load_trace_geometries(gerber_traces)
visualizer.load_trace_mill_geometry(trace_mill_geometry)
visualizer.load_edge_cut_geometry(gerber_edgecuts.outline)
visualizer.load_holes(drilldata.holes)
visualizer.create_tkinter_visualization();

# After window is closed, generate the G-code
if len(sys.argv) > 2:  outname = sys.argv[2]+".gcode"
gcode = Gcode_Generator()
gcode.OutputGcode(outname, gerber_edgecuts.outline, trace_mill_geometry, drilldata.holes)
