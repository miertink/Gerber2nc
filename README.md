# Gerber2CNC - KiCad to G-code Converter

## Overview
Converts single-sided KiCad PCB Gerber files to CNC milling G-code for trace isolation and hole drilling. Includes interactive visualization to verify toolpaths before milling.

**Requirements:**
- Python 3.6+
- `shapely` library: `pip install shapely`
- KiCad project with generated Gerber and drill files

## Usage
```bash
python gerber2nc.py <project_path> [output_name]
```

**Example:**
```bash
# For project at /home/user/myboard/
python gerber2nc.py /home/user/myboard

# Custom output filename
python gerber2nc.py /home/user/myboard myboard_mill
```

**Input Files (KiCad naming convention):**
- `<project>-F_Cu.gbr` - Front copper layer (traces + pads)
- `<project>-Edge_cuts.gbr` - PCB outline
- `<project>-PTH.drl` - Through-hole drill file

**Output:**
- `<project>.nc` or `<output_name>.nc` - G-code file
- Interactive visualization window

## Workflow
1. **Generate Gerbers in KiCad:**
   - Plot → Layers: F.Cu, Edge.cuts
   - Drill files → Excellon format

2. **Run the script:**
   ```bash
   python gerber2nc.py /path/to/your/project
   ```

3. **Review Visualization:**
   - **Green/Black background**: PCB outline
   - **Red lines**: Original traces (varying thickness)
   - **Blue circles/rectangles**: Pads
   - **White lines**: Calculated milling toolpaths
   - **Black circles**: Drill holes
   - Close window when satisfied

4. **Load G-code into your CNC:**
   - T1: 0.2mm engraving bit (trace milling)
   - T2: Small drill (~0.8mm)
   - T3: Large drill (>0.85mm)

## Limitations
- **Single-sided PCBs only** (front copper layer)
- **No rotated pads** or complex apertures
- **KiCad-specific** file formats and naming
- **Outline must be one continuous polyline** in Edge.cuts
- **No copper pours** or ground planes

## Parameters (edit in code)
Search for "parameters" to modify:
- `offset_distance = 0.22` - Initial tool offset from traces
- `num_passes = 3` - Number of milling passes
- `path_spacing = 0.2` - Spacing between passes
- `cut_depth = -0.1` - Trace milling depth
- `spindle_speed = 12000` - RPM
- `feed_rate = 450` - mm/min

## Troubleshooting
- **"No edge cuts defined"**: Draw PCB outline in KiCad Edge.cuts layer
- **Empty visualization**: Check Gerber file paths and KiCad plot settings
- **Milling paths look wrong**: Adjust offset parameters or verify trace widths
- **Drill holes missing**: Ensure PTH drill file is generated

**Note:** The script marks (but doesn't cut) the PCB outline. Use a bandsaw for final separation.