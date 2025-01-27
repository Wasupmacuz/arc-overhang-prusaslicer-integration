"""
This script generates overhangs by stringing together arcs, enabling successful FDM 3D printing of large 90° overhangs!

The core idea originated with Steven McCulloch, who developed a demonstration and implemented the basic mechanics:
https://github.com/stmcculloch/arc-overhang. After Steven's contributions, the script was further fleshed out by Nic
(https://github.com/nicolai-wachenschwan) before being brought to its current state.

HOW TO USE:
    Option A: Open your system console and type `python` followed by the path to this script and the path of the Gcode file.
              This will overwrite the file unless "Path2Output" is specified.

    Option B: Open your slicer software (e.g., PrusaSlicer, or OrcaSlicer) and locate the post-processing script option.
              For example:
                - In PrusaSlicer: Go to the "Print Settings" tab -> "Output Options" section. 
                - In OrcaSlicer: Go to the "Process" menu -> enable the "Advanced" options toggle -> "Others".
              In the Post-processing Scripts window, enter:
                  the full path to your Python executable, followed by a space, and the full path to this script.
              If either path contains spaces, enclose the path in quotation marks. Refer to your slicer's documentation for details on handling paths:
                  - https://manual.slic3r.org/advanced/post-processing

    Note: Your slicer will execute the script after exporting the Gcode. Therefore, the view in the window won't change.
          Open the finished Gcode file to see the results.

To change generation settings, scroll to the 'Parameter' section in the script. Parameters from your slicer will be extracted
automatically from the Gcode.

REQUIREMENTS:
    - Python 3.10+
    - Libraries: shapely 2.0.6+, numpy 2.2.0+, numpy-hilbert-curve 1.0.1, matplotlib 3.8.4(for debugging)
    - Tested with PrusaSlicer 2.5, 2.8, 2.9; OrcaSlicer 2.2.0; and Python 3.10, 3.12. Other versions might require adjustments to keywords.

NOTES:
    - This code is somewhat messy. Normally, it would be split into multiple files, but that would compromise ease of use.
      To maintain usability, the code is divided into sections marked with `###`.
    - Feel free to refactor and add more functionalities!

    Coding Style:
        - Variable names: smallStartEveryWordCapitalized
        - Abbreviations: 'to' replaced by '2', 'for' replaced by '4'
        - Parameters: BigStartEveryWordCapitalized

KNOWN ISSUES:
    - When `UseLeastAmountOfCenterPoints: False`, `ArcPointsPerMillimeter <= 8` has caused failures.
    - `MinDistanceFromPerimeter >= 2 * perimeterWidth` may yield unexpected results.
    - Avoid applying the script multiple times to the same Gcode, as the bridge infill is deleted when arcs are generated.
"""

#!/usr/bin/python
import sys
import argparse
import re
from typing import Any, List
from math import (
    log2,
    ceil,
    sin,
    cos,
    atan2,
    pi,
    hypot,
    degrees,
    radians
)
from shapely import (
    Geometry,
    GeometryCollection,
    LinearRing,
    Point,
    Polygon,
    LineString,
    MultiLineString,
    difference,
    overlaps,
    points,
    prepare,
    destroy_prepared,
    contains_xy,
    intersects,
    distance,
    buffer,
    intersection,
    segmentize
)
from shapely.geometry.base import GeometrySequence
from shapely.ops import linemerge, unary_union
from shapely.strtree import STRtree
import matplotlib.pyplot as plt
import numpy as np
from ast import literal_eval
import warnings
from random import shuffle, randint, choice
import platform
from hilbert import decode

# Global variables for increased speed
round=round
abs=abs
max=max
min=min
len=len

slicer: str = None
# Map from this slicer's settings to PrusaSlicer. If you add a new slicer, please submit a pull request!
_SLICER_SETTINGS_MAP = {
    'PrusaSlicer': {
        "avoid_crossing_perimeters": "avoid_crossing_perimeters",
        "bridge_speed": "bridge_speed",
        "external_perimeters_first": "external_perimeters_first",
        'extrusion_width': 'extrusion_width',
        "filament_diameter": "filament_diameter",
        "infill_extrusion_width": "infill_extrusion_width",
        "infill_first": "infill_first",
        "layer_height": "layer_height",
        'nozzle_diameter': 'nozzle_diameter',
        'overhangs': 'overhangs',
        'perimeter_extrusion_width': 'perimeter_extrusion_width',
        "retract_length": "retract_length",
        'retract_speed': 'retract_speed',
        "solid_infill_extrusion_width": "solid_infill_extrusion_width",
        'travel_speed': 'travel_speed',
        'use_relative_e_distances': 'use_relative_e_distances',
    },
    'OrcaSlicer': {
        "reduce_crossing_wall": "avoid_crossing_perimeters",
        "bridge_speed": "bridge_speed",
        #"wall_sequence": "external_perimeters_first", SETTING HANDLED DIFFERENTLY, STORE AS DEFAULT NAME:
        "wall_sequence": "wall_sequence",
        'line_width': 'extrusion_width',
        "filament_diameter": "filament_diameter",
        "sparse_infill_line_width": "infill_extrusion_width",
        "is_infill_first": "infill_first",
        "layer_height": "layer_height",
        'nozzle_diameter': 'nozzle_diameter',
        'detect_overhang_wall': 'overhangs',
        'inner_wall_line_width': 'perimeter_extrusion_width',
        "retraction_length": "retract_length",
        'retraction_speed': 'retract_speed',
        "internal_solid_infill_line_width": "solid_infill_extrusion_width",
        'travel_speed': 'travel_speed',
        'use_relative_e_distances': 'use_relative_e_distances',
    },
    # Add mappings for other slicers
}

_EQUIVALENT_NAMES = {
    "PrusaSlicer": {
        ";TYPE:Bridge infill": ";TYPE:Bridge infill",
        ";TYPE:External perimeter": ";TYPE:External perimeter",
        ";TYPE:Overhang perimeter": ";TYPE:Overhang perimeter",
        ";TYPE:Solid infill": ";TYPE:Solid infill",
        ";WIPE_START": ";WIPE_START",
        ";WIPE_END": ";WIPE_END",
    },
    "OrcaSlicer": {
        ";TYPE:Bridge infill": ";TYPE:Bridge",
        ";TYPE:External perimeter": ";TYPE:Outer wall",
        ";TYPE:Overhang perimeter": ";TYPE:Overhang wall",
        ";TYPE:Solid infill": ";TYPE:Internal solid infill",
        ";WIPE_START": ";WIPE_START",
        ";WIPE_END": ";WIPE_END",
    },
    # Add mappings for other slicers
}
def getSlicerSpecificName(name: str):
    if slicer == "PrusaSlicer":  # No need to map in this case, but the mapping is left to help contributors translate their own slicer.
        return name
    return _EQUIVALENT_NAMES.get(slicer).get(name, name)

########## Parameters  - adjust values here as needed ##########
def makeFullSettingDict(gCodeSettingDict: dict) -> dict:
    """Merge two dictionaries and set some keys/values explicitly."""
    # The slicer settings will be imported from GCode. However, some are Arc-specific and need to be adapted by you.
    AddManualSettingsDict: dict[str, Any] = {
        # Adapt these settings as needed for your specific geometry/printer:
        "AllowedArcRetries": 2,  # Tries at slightly different points if arc generation fails.
        "CheckForAllowedSpace": False,  # Use the following x&y filter or not
        "AllowedSpaceForArcs": Polygon([[0, 0], [500, 0], [500, 500], [0, 500]]),  # Control in which areas Arcs shall be generated
        "ArcCenterOffset": 1.5 * gCodeSettingDict.get("nozzle_diameter"),  # Unit: mm, prevents very small Arcs by hiding the center in not printed section. Make 0 to get into tricky spots with smaller arcs.
        "ArcExtrusionMultiplier": 1.35, # Multiplies how much filament will be extruded while printing arcs.
        "ArcFanSpeed": 255,  # Cooling to full blast = 255
        "ArcMinPrintSpeed": 0.5 * 60,  # Unit: mm/min
        "ArcPrintSpeed": 1.5 * 60,  # Unit: mm/min
        "ArcSlowDownBelowThisDuration": 3,  # Arc Time below this Duration => slow down, Unit: sec
        "ArcPointsPerMillimeter": 10,  # Higher will slow down the code but give better support for following arcs. Recommended values: >=10 when "UseLeastAmountOfCenterPoints": False; else, value can be as low as 1.
        "ArcTravelFeedRate": 30 * 60,  # Slower travel speed, Unit: mm/min
        "ArcWidth": gCodeSettingDict.get("nozzle_diameter") * 0.95,  # Change the spacing between the arcs, should be nozzle_diameter
        "CornerImportanceMultiplier": 0.2,  # Startpoint for Arc generation is chosen close to the middle of the StartLineString and at a corner. Higher => Corner selection more important.
        "DistanceBetweenPointsOnStartLine": 0.1,  # Used for redistribution, if start fails.
        "ExtendArcDist": gCodeSettingDict.get("nozzle_diameter"),  # Extend Arcs perpendicularly for better bonding between them. Unit: mm
        "ExtendArcsIntoPerimeter": 0.5 * gCodeSettingDict.get("extrusion_width"),  # Min = 0.5 extrusion width!, extends the Area for arc generation, put higher to go through small passages. Unit: mm
        "ExtendHilbertIntoPerimeter": 1 * gCodeSettingDict.get("extrusion_width"),  # Extends the Area for Hilbert curve generation, put higher to go through small passages. Unit: mm
        "GCodeArcPtMinDist": 0.1,  # Min distance between points on the Arcs to form separate GCode Command. Unit: mm
        "HilbertFillingPercentage": 100,  # Infill percentage of the massive layers with special cooling.
        "HilbertInfillExtrusionMultiplier": 1.05, # Multiplies how much filament will be extruded while printing Hilbert curves.
        "HilbertTravelEveryNSeconds": 6,  # When N seconds are driven, it will continue printing somewhere else (very rough approx).
        "MinArea": 0,  # Minimum overhang area to generate arcs. Unit: mm²
        "MinBridgeLength": 0,  # Minimum bridge length to generate arcs. Unit: mm
        "MinDistanceFromPerimeter": 1 * gCodeSettingDict.get("extrusion_width"),  # Control how much bumpiness you allow between arcs and perimeter. Lower will follow perimeter better, but create a lot of very small arcs. Should be more than 1 Arc width! Unit: mm
        "MinStartArcs": 2,  # How many arcs shall be generated in the first step
        "Path2Output": r"",  # Leave empty to overwrite the file or write to a new file. Full path required.
        "RMax": 30,  # The max radius of the arcs.
        "ReplaceInternalBridging": True, # If true, will replace bridging that goes over external perimeters but does not have overhang perimeters nearby.
        "SafetyBreak_MaxArcNumber": 2000,  # Max Number of Arc Start Points. Prevents While loop from running forever.
        "TimeLapseEveryNArcs": 0,  # Deactivate with 0, inserts M240 after N ArcLines, 5 is a good value to start.
        "UseLeastAmountOfCenterPoints": False,  # Always generates arcs until rMax is reached, divide the arcs into pieces if needed. Reduces the amount of center points.
        "WarnBelowThisFillingPercentage": 90,  # Fill the overhang at least XX%, else don't replace overhang. Easier detection of errors in small/delicate areas. Unit: Percent

        # Special cooling to prevent warping:
        "aboveArcsFanSpeed": 25,  # 0 -> 255, 255 = 100%
        "aboveArcsInfillPrintSpeed": 10 * 60,  # Unit: mm/min
        "aboveArcsPerimeterFanSpeed": 25,  # 0 -> 255, 255 = 100%
        "aboveArcsPerimeterPrintSpeed": 3 * 60,  # Unit: mm/min
        "applyAboveFanSpeedToWholeLayer": True,
        "CoolingSettingDetectionDistance": 3,  # If the GCode line is closer than this distance to an infill polygon, the cooling settings will be applied. Unit: mm
        "doSpecialCooling": True,  # Use to enable/disable Hilbert curves and slower movement above arc overhangs. Should be `True` to prevent warping
        "specialCoolingZdist": 3,  # Use the special cooling XX mm above the arcs.

        # Settings for easier debugging:
        "plotArcsEachStep": False,  # Plot arcs for every filled polygon. Use for debugging.
        "plotArcsFinal": False,  # Plot arcs for every filled polygon, when completely filled. Use for debugging.
        "plotDetectedInfillPoly": False,  # Plot each detected overhang polygon. Use for debugging.
        "plotDetectedSolidInfillPoly": False,  # Plot each solid infill polygon. Use for debugging.
        "plotEachHilbert": False,  # Plot each generated Hilbert curve. Use for debugging.
        "plotStart": False,  # Plot the detected geometry in the previous layer and the StartLine for Arc-Generation. Use for debugging.
        "PrintDebugVerification": False  # Used for console logging of the process.
    }

    gCodeSettingDict.update(AddManualSettingsDict)
    return gCodeSettingDict

################################# MAIN FUNCTION #################################
#################################################################################
#at the top, for better reading
def main(gCodeFileStream, path2GCode) -> None:
    """Process G-code to generate and inject arc infill for overhangs."""
    gCodeLines = gCodeFileStream.readlines()
    gCodeSettingDict = readSettingsFromGCode2dict(gcodeLines=gCodeLines, fallbackValuesDict={"Fallback_nozzle_diameter": 0.4, "Fallback_filament_diameter": 1.75})  # ADD FALLBACK VALUES HERE
    parameters = makeFullSettingDict(gCodeSettingDict=gCodeSettingDict)
    
    if not checkforNecesarrySettings(gCodeSettingDict=gCodeSettingDict):
        warnings.warn(message=f"Incompatible {slicer}-Settings used!")
        input("Can not run script, gcode unmodified. Press enter to close.")
        raise ValueError("Incompatible Settings used!")
    
    layerobjs = []
    gcodeWasModified = False
    numOverhangs = 0
    
    layers = splitGCodeIntoLayers(gcode=gCodeLines)
    gCodeFileStream.close()
    print("layers:", len(layers))
    lastfansetting = 0  # Initialize variable
    
    for idl, layerlines in enumerate(layers):
        layer = Layer(layerlines, parameters, idl)
        layer.addZ()
        layer.addHeight()
        lastfansetting = layer.spotFanSetting(lastfansetting)
        layerobjs.append(layer)
    
    for idl, layer in enumerate(layerobjs):
        modify = False
        if idl < 2:
            continue  # No overhangs in the first layer and don't mess with the setup
        else:
            prevLayer = layerobjs[idl - 1]
            layer.extract_features()
            layer.spotBridgeInfill()
            layer.makePolysFromBridgeInfill(extend=parameters.get("ExtendArcsIntoPerimeter", 1))
            layer.polys = layer.mergePolys()
            layer.verifyinfillpolys(prevLayer=prevLayer, maxDistForValidation=2 * parameters.get("perimeter_extrusion_width"))

            # ARC GENERATION
            if layer.validpolys:
                numOverhangs += 1
                print(f"overhang found layer {idl}:", len(layer.polys), f"Z: {layer.z:.2f}")
                layer.indexValidPolys()
                
                # Set special cooling settings for the follow-up layers
                maxZ = layer.z + parameters.get("specialCoolingZdist")
                idoffset = 1
                currZ = layer.z
                while currZ < maxZ and idl + idoffset <= len(layerobjs) - 1:
                    currZ = layerobjs[idl + idoffset].z
                    layerobjs[idl + idoffset].oldpolys.extend(layer.validpolys)
                    layerobjs[idl + idoffset].indexOldPolys()
                    idoffset += 1

                arcOverhangGCode = []
                
                for poly in layer.validpolys:
                    # Make parameters more readable
                    MinDistanceFromPerimeter = parameters.get("MinDistanceFromPerimeter")  # How much 'bumpiness' you accept in the outline. Lower will generate more small arcs to follow the perimeter better (corners!). Good practice: 2 perimeters + threshold of 2 width = minimal exact touching (if rMin satisfied)
                    rMax = parameters.get("RMax", 15)
                    arcWidth = parameters.get("ArcWidth")
                    rMin = parameters.get("ArcCenterOffset") + arcWidth / 1.5
                    rMinStart = parameters.get("nozzle_diameter")
                    
                    # Initialize
                    finalarcs = []
                    arcs4gcode = []
                    
                    # Find StartPoint and StartLineString
                    startLineString, boundaryWithOutStartLine = prevLayer.makeStartLineString(poly, parameters)
                    if startLineString is None:
                        warnings.warn("Skipping Polygon because no StartLine Found")
                        layer.failedArcGenPolys.append(poly)
                        continue
                    prepare(startLineString)
                    prepare(boundaryWithOutStartLine)
                    startpt = getStartPtOnLS(startLineString, parameters)

                    # First step in Arc Generation
                    concentricArcs = generateMultipleConcentricArcs(startpt, rMinStart, rMax, boundaryWithOutStartLine, poly, parameters)
                    # print(f"number of concentric arcs generated:", len(concentricArcs))
                    if len(concentricArcs) < parameters.get("MinStartArcs"):
                        # Possibly bad chosen startpt, error handling:
                        startpt = getStartPtOnLS(segmentize(startLineString, 0.1), parameters)
                        concentricArcs = generateMultipleConcentricArcs(startpt, rMinStart, rMax, boundaryWithOutStartLine, poly, parameters)
                        if len(concentricArcs) < parameters.get("MinStartArcs"):  # Still insufficient start: try random
                            print(f"Layer {idl}: Using random Startpoint")
                            for idr in range(10):
                                startpt = getStartPtOnLS(startLineString, parameters, choseRandom=True)
                                concentricArcs = generateMultipleConcentricArcs(startpt, rMinStart, rMax, boundaryWithOutStartLine, poly, parameters)
                                if len(concentricArcs) >= parameters.get("MinStartArcs"):
                                    break
                            if len(concentricArcs) < parameters.get("MinStartArcs"):
                                for idr in range(10):
                                    startpt = getStartPtOnLS(segmentize(startLineString, 0.1), parameters, choseRandom=True)
                                    concentricArcs = generateMultipleConcentricArcs(startpt, rMinStart, rMax, boundaryWithOutStartLine, poly, parameters)
                                    if len(concentricArcs) >= parameters.get("MinStartArcs"):
                                        break
                            if len(concentricArcs) < parameters.get("MinStartArcs"):
                                warnings.warn("Initialization Error: no concentric Arc could be generated at startpoints, moving on")
                                layer.failedArcGenPolys.append(poly)
                                continue
                    destroy_prepared(startLineString)
                    destroy_prepared(boundaryWithOutStartLine)
                    arcBoundaries = getArcBoundaries(concentricArcs)
                    finalarcs.append(concentricArcs[-1])
                    filledSpace = Polygon(finalarcs[0].circle).intersection(poly)

                    # Start BFS (breadth first search algorithm) to fill the remaining space
                    remainingArcs, finalFilledSpace = fill_remaining_space(concentricArcs[-1], rMin, rMax, MinDistanceFromPerimeter, filledSpace, poly, parameters)
                    arcBoundaries.extend(getArcBoundaries(remainingArcs))
                    if parameters.get("plotArcsFinal"):
                        plt.title(f"Total No Arcs: {len(arcBoundaries)}")
                        plot_geometry([arc for arc in arcBoundaries], changecolor=True)
                        plot_geometry(poly, 'r')
                        plt.axis('square')
                        plt.show()
                    arcs4gcode.extend(arcBoundaries)

                    # Poly finished
                    remain2FillPercent = (1 - finalFilledSpace.area / poly.area) * 100
                    if remain2FillPercent > 100 - parameters.get("WarnBelowThisFillingPercentage"):
                        # layer.failedArcGenPolys.append(poly) # Bugged percentage detection, not reliable TODO
                        warnings.warn(f"layer {idl}: The Overhang Area is only {100 - remain2FillPercent:.0f}% filled with Arcs. Please try again with adapted Parameters: set 'ExtendArcsIntoPerimeter' higher to enlarge small areas. Lower the MaxDistanceFromPerimeter to follow the curvature more precise. Set 'ArcCenterOffset' to 0 to reach delicate areas.")
                        # plot_geometry(poly)
                        # plot_geometry(finalFilledSpace, color='b', kwargs={"filled"})
                        # plt.axis('square')
                        # plt.show()

                    # Generate G-code for arc and insert at the beginning of the layer
                    eSteps = calcESteps(parameters)
                    for ida, arc in enumerate(arcs4gcode):
                        if not arc.is_empty:
                            arcGCode = arc2GCode(arcline=arc, eSteps=eSteps, arcidx=ida, kwargs=parameters)
                            arcOverhangGCode.append(arcGCode)
                            if parameters.get("TimeLapseEveryNArcs") > 0:
                                if ida % parameters.get("TimeLapseEveryNArcs"):
                                    arcOverhangGCode.append("M240\n")

                    modify = True
                    gcodeWasModified = True

            # Apply special cooling settings:
            if parameters.get("doSpecialCooling") and len(layer.oldpolys) > 0 and gcodeWasModified:
                modify = True
                print("oldpolys found in layer:", idl)
                layer.spotSolidInfill()
                layer.makePolysFromSolidInfill(extend=parameters.get("ExtendHilbertIntoPerimeter"))
                layer.solidPolys = layer.mergePolys(layer.solidPolys)
                allhilbertpts = []
                for poly in layer.solidPolys:
                    prepare(poly)
                    hilbertpts = layer.createHilbertCurveInPoly(poly)
                    allhilbertpts.extend(hilbertpts)
                    if parameters.get("plotEachHilbert"):
                        plot_geometry(hilbertpts, changecolor=True)
                        plot_geometry(layer.solidPolys)
                        plt.title("Debug")
                        plt.axis('square')
                        plt.show()
                    destroy_prepared(poly)

            if modify:
                modifiedlayer = Layer([], parameters, idl)
                isInjected = False
                hilbertIsInjected = False
                curPrintSpeed = "G1 F600"
                messedWithSpeed = False
                messedWithFan = False
                if gcodeWasModified:
                    layer.prepareDeletion(featurename=getSlicerSpecificName(";TYPE:Bridge infill"), polys=layer.validpolys)
                    if len(layer.oldpolys) > 0 and parameters.get("doSpecialCooling"):
                        layer.prepareDeletion(featurename=getSlicerSpecificName(";TYPE:Solid infill"), polys=layer.oldpolys)
                # print("FEATURES:", [(f[0], f[2]) for f in layer.features])
                injectionStart = None
                print("modifying GCode")
                for idline, line in enumerate(layer.lines):
                    if layer.validpolys:
                        if ";TYPE" in line and not isInjected:  # Inject arcs at the very start
                            injectionStart = idline
                            modifiedlayer.lines.append(";TYPE:Arc infill\n")
                            modifiedlayer.lines.append(f"M106 S{parameters.get('ArcFanSpeed')}\n")
                            for overhangline in arcOverhangGCode:
                                for arcline in overhangline:
                                    for cmdline in arcline:
                                        modifiedlayer.lines.append(cmdline)
                            isInjected = True
                            # Travel to restored pre-injected tool position
                            for id in reversed(range(injectionStart)):
                                if "G1 X" in layer.lines[id]:  # TODO: should find changes to Z instead of ignoring them
                                    modifiedlayer.lines.append(retractGCode(retract=True, kwargs=parameters))  # Retract
                                    modifiedlayer.lines.append(line2TravelMove(layer.lines[id], parameters, ignoreZ=True))  # Travel
                                    modifiedlayer.lines.append(retractGCode(retract=False, kwargs=parameters))  # Extrude
                                    break
                    if layer.oldpolys and parameters.get("doSpecialCooling"):
                        if getSlicerSpecificName(";TYPE:Solid infill") in line and not hilbertIsInjected:  # Startpoint of solid infill: print all hilberts from here.
                            hilbertIsInjected = True
                            injectionStart = idline
                            modifiedlayer.lines.append(getSlicerSpecificName(";TYPE:Solid infill") + "\n")
                            modifiedlayer.lines.append(f"M106 S{parameters.get('aboveArcsFanSpeed')}\n")
                            hilbertGCode = hilbert2GCode(allhilbertpts, parameters, layer.height)
                            modifiedlayer.lines.extend(hilbertGCode)
                            # Add restored pre-injected tool position
                            for id in reversed(range(injectionStart)):
                                if "G1 X" in layer.lines[id]:  # TODO: should find changes to Z instead of ignoring them
                                    modifiedlayer.lines.append(retractGCode(retract=True, kwargs=parameters))  # Retract
                                    modifiedlayer.lines.append(line2TravelMove(layer.lines[id], parameters, ignoreZ=True))  # Travel
                                    modifiedlayer.lines.append(retractGCode(retract=False, kwargs=parameters))  # Extrude
                                    break
                    if "G1 F" in line.split(";", 1)[0]:  # Special block-speed-command
                        curPrintSpeed = line
                    if layer.exportThisLine(idline - 1):  # Subtract 1 because there's a disconnect between line IDs here and line IDs when calculating which lines to delete. Should fix TODO
                        if layer.isClose2Bridging(line, parameters.get("CoolingSettingDetectionDistance")):
                            if not messedWithFan:
                                modifiedlayer.lines.append(f"M106 S{parameters.get('aboveArcsFanSpeed')}\n")
                                messedWithFan = True
                            modline = line.strip("\n") + f" F{parameters.get('aboveArcsPerimeterPrintSpeed')}\n"
                            modifiedlayer.lines.append(modline)
                            messedWithSpeed = True
                        else:
                            if messedWithFan and not parameters.get("applyAboveFanSpeedToWholeLayer"):
                                modifiedlayer.lines.append(f"M106 S{layer.fansetting:.0f}\n")
                                messedWithFan = False
                            if messedWithSpeed:
                                modifiedlayer.lines.append(curPrintSpeed + "\n")
                                messedWithSpeed = False
                            modifiedlayer.lines.append(line)
                if messedWithFan:
                    modifiedlayer.lines.append(f"M106 S{layer.fansetting:.0f}\n")
                    messedWithFan = False
                modifiedlayer.extract_features()
                layerobjs[idl] = modifiedlayer  # Overwrite the infos
    
    if gcodeWasModified:
        overwrite = True
        if parameters.get("Path2Output"):
            path2GCode = parameters.get("Path2Output")
            overwrite = False
        f = open(path2GCode, "w", encoding="UTF-8")
        if overwrite:
            print("overwriting file")
        else:
            print("write to", path2GCode)
        for layer in layerobjs:
            f.writelines(layer.lines)
        f.close()
    else:
        if numOverhangs > 0:
            print(f"Found {numOverhangs} overhangs, but no arcs could be generated due to unusual geometry.")
        else:
            print(f"Analysed {len(layerobjs)} Layers, but no matching overhangs found -> no arcs generated. If unexpected: look if restricting settings like 'minArea' or 'MinBridgeLength' are correct.")
    # os.startfile(path2GCode, 'open')
    print("Script execution complete.")

################################# HELPER FUNCTIONS GCode->Polygon #################################
###################################################################################################

def getFileStreamAndPath(path, read=True):
    """Open a file stream and return it along with the file path."""
    try:
        if read:
            f = open(path, "r", encoding="UTF-8")  # Open file for reading
        else:
            f = open(path, "w", encoding="UTF-8")  # Open file for writing
        return f, path
    except IOError:
        input("File not found. Press enter.")
        sys.exit(1)  # Exit if file cannot be opened

def splitGCodeIntoLayers(gcode: list) -> list:
    """Split G-code into layers based on layer change comments."""
    gcode_list = []
    buff = []
    for linenumber, line in enumerate(gcode):
        if ";LAYER_CHANGE" in line:
            gcode_list.append(buff)  # Save the current layer
            buff = []
            buff.append(line)  # Start a new layer
        else:
            buff.append(line)  # Add the line to the current layer
    gcode_list.append(buff)  # Catch the last layer
    print("last read linenumber:", linenumber)
    return gcode_list

def getPtfromCmd(line: str, prevPoint = None, **kwargs: dict) -> Point | LineString:
    """Extract a Point from a G-code line by parsing X and Y coordinates."""
    x = None
    y = None
    i = None
    j = None
    cmdType = None
    line = line.split(";", 1)[0]  # Remove comments
    cmds = line.split(" ")
    if re.search(r"G[0-3]", cmds[0]):
        cmdType = cmds.pop(0)
        for c in cmds:
            if x is None and "X" in c:
                try:
                    x = float(c[1:])  # Extract X coordinate
                except ValueError:
                    break
            elif "Y" in c:
                try:
                    y = float(c[1:])  # Extract Y coordinate
                except ValueError:
                    break
                if prevPoint is None or re.search("G[0,1]", cmdType): # If this is a linear movement
                    break
            elif "I" in c:
                try:
                    i = float(c[1:])  # Extract X offset
                except ValueError:
                    break
            elif "J" in c:
                try:
                    j = float(c[1:])  # Extract Y offset
                except ValueError:
                    break
                break

    
    if x is not None and y is not None:
        if i is None or j is None:
            return Point(x, y)  # Return the Point if valid coordinates are found
        else:
            clockwise = (cmdType == "G2")
            radius = hypot((i, j))
            center = Point(prevPoint.x + i, prevPoint.y + j)
            startAngle = atan2(j, i)
            endAngle = atan2(y - center.y, x - center.x)
            arcLine = create_circle_between_angles(center, radius, startAngle, endAngle, kwargs.get("ArcPointsPerMillimeter"), clockwise)
            return arcLine
    else:
        return None  # Return None if no valid coordinates are found

def makePolygonFromGCode(lines: list) -> Polygon | None:
    """Create a polygon from G-code lines by extracting points."""
    pts = []
    wiping = False
    for line in lines:
        if isTravelMove(line):
            break  # Stop if a travel move is encountered

        if getSlicerSpecificName(";WIPE_END") in line:
            wiping = False  # End wiping mode
        elif wiping:
            continue  # Skip lines during wiping
        elif getSlicerSpecificName(";WIPE_START") in line:
            wiping = True  # Start wiping mode

        if "G1 X" in line:
            p = getPtfromCmd(line)
            if p:
                pts.append(p)  # Collect valid points
    
    if len(pts) > 2:
        return Polygon(pts)  # Return the polygon if enough points are collected
    else:
        # print("invalid poly: not enough pts")
        return None  # Return None if not enough points
    
def isTravelMove(line: str) -> bool:
    """Check if a G-code line represents a travel move."""
    if "G1 E" in line or ("G1 X" in line and not "E" in line):
        return True  # Return True if it's a travel move
    return False  # Return False otherwise

################################# CLASSES #################################
###########################################################################

class Layer():
    def __init__(self,lines:list=[],kwargs:dict={},layernumber:int=-1)->None:
        self.allowedSpacePolygon=kwargs.get("AllowedSpaceForArcs")
        prepare(self.allowedSpacePolygon)
        self.lines=lines
        self.layernumber=layernumber
        self.z=kwargs.get("z",None)
        self.polys=[]
        self.validpolys=[]
        self.indexedValidPolys=STRtree([])
        self.extPerimeterPolys=[]
        self.failedArcGenPolys=[]
        self.failedSolidInfillLocations=[]
        self.binfills=[]
        self.features=[]
        self.oldpolys=[]
        self.indexedOldPolys=STRtree([])
        self.solidPolys=[]
        self.dontPerformPerimeterCheck=kwargs.get('notPerformPerimeterCheck',False)
        self.deleteTheseInfills=[]
        self.deletelines=set()
        self.associatedIDs=[]
        self.sinfills=[]
        self.parameters=kwargs
        self.lastP=None

    def indexValidPolys(self):
        """Index valid polygons for efficient spatial queries."""
        for poly in self.validpolys:
            prepare(poly)  # Prepare polygons for faster operations
        self.indexedValidPolys = STRtree(self.validpolys)  # Create an STR-tree index

    def indexOldPolys(self) -> None:
        """Index old polygons for efficient spatial queries."""
        for poly in self.oldpolys:
            prepare(poly)  # Prepare polygons for faster operations
        self.indexedOldPolys = STRtree(self.oldpolys)  # Create an STR-tree index

    def extract_features(self) -> None:
        """Extract features (e.g., perimeter, infill) from G-code lines."""
        buff = []
        currenttype = ""
        start = 0
        for idl, line in enumerate(self.lines):
            if ";TYPE:" in line:
                if currenttype:
                    self.features.append([currenttype, buff, start])  # Save the previous feature
                    buff = []
                    start = idl  # Update the start index for the new feature
                currenttype = line  # Update the current feature type
            else:
                buff.append(line)  # Collect lines for the current feature
        self.features.append([currenttype, buff, start])  # Save the last feature

    def addZ(self, z: float = 0.0) -> None:
        """Set the Z-coordinate either directly or by extracting it from G-code."""
        if z != 0.0:
            self.z = z  # Set Z-coordinate directly if provided
        else:
            for l in self.lines:
                cmd = l.split(";", 1)[0]  # Extract the command part
                if "G1" in cmd and "Z" in cmd:
                    cmds = cmd.split(" ")
                    for c in cmds:
                        if "Z" in c:
                            self.z = float(c[1:])  # Extract and set the Z-coordinate
                            return

    def addHeight(self):
        """Extract and set the layer height from G-code comments."""
        for l in self.lines:
            if ";HEIGHT" in l:
                h = l.split(":", 1)
                self.height = float(h[-1])  # Extract and set the height
                return
        warnings.warn(f"Layer {self.layernumber}: no height found, using layerheight default!")
        self.height = self.parameters.get("layer_height")  # Use default height if not found

    def getRealFeatureStartPoint(self, idf: int) -> Point | None:
        """Retrieve the real start point of a feature by looking at the previous feature's last move."""
        if idf < 1:
            return None  # No previous feature to reference
        
        lines = self.features[idf - 1][1]  # Get lines from the previous feature
        for line in reversed(lines):
            if "G1 X" in line:
                return getPtfromCmd(line)  # Return the point from the last G1 move

    def makeExternalPerimeter2Polys(self) -> None:
        """Create polygons from external perimeter and overhang features in G-code."""
        extPerimeterIsStarted = False
        for idf, fe in enumerate(self.features):
            ftype = fe[0]
            lines = fe[1]

            if getSlicerSpecificName(";TYPE:External perimeter") in ftype or (getSlicerSpecificName(";TYPE:Overhang perimeter") in ftype and extPerimeterIsStarted) or (getSlicerSpecificName(";TYPE:Overhang perimeter") in ftype and self.dontPerformPerimeterCheck):
                # Start collecting lines for external perimeter or overhang perimeter
                if not extPerimeterIsStarted:
                    linesWithStart = []
                    if idf > 1:
                        pt = self.getRealFeatureStartPoint(idf)  # Fetch the real start point
                        if type(pt) == type(Point):
                            linesWithStart.append(p2GCode(pt))
                        else:
                            warnings.warn(f"Layer {self.layernumber}: Could not fetch real StartPoint.")
                    extPerimeterIsStarted = True
                linesWithStart = linesWithStart + lines  # Append current feature lines
            
            if extPerimeterIsStarted and (idf == len(self.features) - 1 or not (getSlicerSpecificName(";TYPE:External perimeter") in ftype or getSlicerSpecificName(";TYPE:Overhang perimeter") in ftype)):
                # Finish the polygon if end of feature list or different feature
                poly = makePolygonFromGCode(linesWithStart)  # Create polygon from collected lines
                if poly:
                    self.extPerimeterPolys.append(poly)  # Add polygon to the list
                extPerimeterIsStarted = False

    def makeStartLineString(self, poly: Polygon, kwargs: dict = {}):
        """Create a starting LineString for arc generation by intersecting with previous layer's external perimeter."""
        if not self.extPerimeterPolys:
            self.makeExternalPerimeter2Polys()  # Generate external perimeter polygons if not available
        
        if len(self.extPerimeterPolys) < 1:
            warnings.warn(f"Layer {self.layernumber}: No ExternalPerimeterPolys found in prev Layer")
            return None, None
        
        for ep in self.extPerimeterPolys:
            ep = buffer(ep, 1e-2)  # Avoid self-intersection errors
            if intersects(ep, poly):
                startArea = ep.intersection(poly)  # Find the intersection area
                startLineString = startArea.boundary.intersection(buffer(poly.boundary, 1e-2))  # Get the boundary intersection
                
                if startLineString.is_empty:
                    if poly.contains(startArea):  # If inside, no boundaries can overlap
                        startLineString = startArea.boundary
                        boundaryLineString = poly.boundary
                        
                        if startLineString.is_empty:  # Still empty? Unlikely to happen
                            # plt.title("StartLineString is None")
                            # plot_geometry(poly, 'b')
                            # plot_geometry(startArea, filled=True)
                            # plot_geometry([ep for ep in self.extPerimeterPolys])
                            # plt.legend(["currentLayerPoly", "StartArea", "prevLayerPoly"])
                            # plt.axis('square')
                            # plt.show()
                            warnings.warn(f"Layer {self.layernumber}: No Intersection in Boundary, Poly + ExternalPoly")
                            return None, None
                
                else:
                    boundaryLineString = poly.boundary.difference(buffer(startArea.boundary, 1e-2))  # Get the remaining boundary
                
                if kwargs.get("plotStart"):
                    print("Geom-Type:", poly.geom_type)
                    plot_geometry(poly, color="b")
                    plot_geometry(ep, 'g')
                    plot_geometry(startLineString, color="m")
                    plt.title("Start-Geometry")
                    plt.legend(["Poly4ArcOverhang", "External Perimeter prev Layer", "StartLine for Arc Generation"])
                    plt.axis('square')
                    plt.show()
                
                return startLineString, boundaryLineString
        
        # End of for loop, and no intersection found
        # plt.title("no intersection with prev Layer Boundary")
        # plot_geometry(poly, 'b')
        # plot_geometry([ep for ep in self.extPerimeterPolys])
        # plt.legend(["currentLayerPoly", "prevLayerPoly"])
        # plt.axis('square')
        # plt.show()
        warnings.warn(f"Layer {self.layernumber}: No intersection with prevLayer External Perimeter detected")
        return None, None

    def mergePolys(self, thesepolys: list = None) -> list:
        """Merge polygons into a single geometry and split into individual polygons."""
        if not thesepolys:
            thesepolys = self.polys  # Use default polygons if none provided
        mergedPolys = unary_union(thesepolys)  # Merge all polygons into a single geometry
        
        if mergedPolys.geom_type == "Polygon":
            thesepolys = [mergedPolys]  # If single polygon, wrap in a list
        elif mergedPolys.geom_type == "MultiPolygon" or mergedPolys.geom_type == "GeometryCollection":
            thesepolys = [poly for poly in mergedPolys.geoms]  # Split MultiPolygon into individual polygons
        
        return thesepolys

    def spotFeaturePoints(self, featureName: str, splitAtWipe=False, includeRealStartPt=False, splitAtTravel=False) -> list:
        """Extract feature points from G-code based on the specified feature name."""
        parts = []
        partLocations = []
        for idf, fe in enumerate(self.features):
            ftype = fe[0]
            lines = fe[1]
            start = fe[2]
            begin = 0
            travelBegin = -1
            end = -1
            pts = []
            travelPoints = []
            isWipeMove = False
            isTravelling = False
            
            if featureName not in ftype:
                continue

            if includeRealStartPt and idf > 0:
                sp = self.getRealFeatureStartPoint(idf)  # Include the real start point if requested
                if sp:
                    pts.append(sp)

            for idl, line in enumerate(lines):
                if not isWipeMove and re.search(r"G\d", line):
                    if isTravelling:
                        if isTravelMove(line) or not "X" in line:
                            travelP = getPtfromCmd(line)
                            if travelP:
                                travelPoints.append(travelP)
                                travelBegin = idl + start
                        else:
                            isTravelling = False
                            if len(travelPoints) > 0:
                                pts.append(travelPoints[-1])
                                begin = travelBegin
                                end = travelBegin + 2
                                travelPoints = []
                            p = getPtfromCmd(line)
                            if p:
                                if not pts:
                                    begin = idl + start
                                pts.append(p)
                                end = idl + start + 2
                    elif splitAtTravel and isTravelMove(line):
                        if len(pts) >= 2:  # Split at travel moves if requested
                            parts.append(pts)
                            partLocations.append((begin, end))
                        if len(parts) > 0:
                            pts = []
                        travelP = getPtfromCmd(line)
                        if travelP:
                            travelPoints.append(travelP)
                            travelBegin = idl + start
                        isTravelling = True
                    elif "E" in line:  # Include points with extrusion
                        if "G1" in line:
                            p = getPtfromCmd(line)
                            if p:
                                if not pts:
                                    begin = idl + start
                                pts.append(p)
                                end = idl + start + 2
                        else: # Deal with arcing movements.
                            arc = getPtfromCmd(line)
                            if arc:
                                if not pts:
                                    begin = idl + start
                                pts.extend(Point(coord) for coord in arc.coords)
                                end = idl + start + 2

                if getSlicerSpecificName(';WIPE_START') in line:
                    isWipeMove = True
                    if splitAtWipe:  # Split at wipe moves if requested
                        parts.append(pts)
                        partLocations.append((begin, end))
                        pts = []
                
                if getSlicerSpecificName(';WIPE_END') in line:
                    isWipeMove = False
            
            if len(pts) >= 2:  # Append the last set of points
                parts.append(pts)
                partLocations.append((begin, end))

        return parts, partLocations

    def spotSolidInfill(self) -> None:
        """Identify and store solid infill features from G-code."""
        for part, location in zip(*self.spotFeaturePoints(getSlicerSpecificName(";TYPE:Solid infill"), splitAtTravel=True, includeRealStartPt=True)):
            if self.verifySolidInfillPts(part):  # Verify the infill points
                self.sinfills.append(LineString(part))  # Create and store LineString objects
            else:
                self.failedSolidInfillLocations.append(location)  # Store the locations of the infill we don't want to delete


    def makePolysFromSolidInfill(self, extend: float = 1) -> None:
        """Create polygons from solid infill LineStrings by buffering them."""
        for sInfill in self.sinfills:
            infillPoly = buffer(sInfill, extend + 5e-2)  # Buffer the LineString to create a polygon
            self.solidPolys.append(infillPoly)  # Add the polygon to the list

            if self.parameters.get("plotDetectedSolidInfillPoly"):
                plot_geometry(self.solidPolys)  # Plot the polygon
                plot_geometry(self.sinfills, "g")  # Plot the LineString in green
                plt.axis('square')
                plt.show()

    def verifySolidInfillPts(self, infillpts: list) -> bool:
        """Verify solid infill points by checking if any are within the desired polygon locations."""
        possible_pairs = self.indexedOldPolys.query(infillpts, predicate="within").T.tolist()  # Find pairs of points and polygons that may intersect
        for pair in possible_pairs:
            p = infillpts[pair[0]]  # Get the point
            poly = self.indexedOldPolys.geometries.take(pair[1])  # Get the corresponding polygon
            if contains_xy(poly, p.x, p.y):
                return True  # Return True if the point is inside the polygon
        return False  # Return False if no points are inside the polygons

    def spotBridgeInfill(self) -> None:
        """Identify and store bridge infill features from G-code."""
        parts = self.spotFeaturePoints(getSlicerSpecificName(";TYPE:Bridge infill"), splitAtTravel=True, includeRealStartPt=True)[0]  # Find bridge infill points
        for infillpts in parts:
            self.binfills.append(BridgeInfill(infillpts))  # Create and store BridgeInfill objects

    def makePolysFromBridgeInfill(self, extend: float = 1) -> None:
        """Create polygons from bridge infill LineStrings by buffering them."""
        for bInfill in self.binfills:
            infillPts = bInfill.pts  # Get infill points
            infillLS = LineString(infillPts)  # Create LineString from points
            infillPoly = buffer(infillLS, extend + 5e-2)  # Buffer the LineString to create a polygon
            self.polys.append(infillPoly)  # Add the polygon to the list
            self.associatedIDs.append(bInfill.id)  # Store the associated ID

            if self.parameters.get("plotDetectedInfillPoly"):
                plot_geometry(infillPoly)  # Plot the polygon
                plot_geometry(infillLS, "g")  # Plot the LineString in green
                plt.axis('square')
                plt.show()

    def getOverhangPerimeterLineStrings(self):
        """Extract and return overhang perimeter LineStrings from G-code features."""
        parts = self.spotFeaturePoints(getSlicerSpecificName(";TYPE:Overhang perimeter"), includeRealStartPt=True)[0]  # Find overhang perimeter points
        if parts:
            return [LineString(pts) for pts in parts]  # Convert points to LineStrings
        else:
            return []  # Return empty list if no overhang perimeters found

    def verifyinfillpolys(self, prevLayer, maxDistForValidation: float = 0.5) -> None:
        """Verify infill polygons by checking their proximity to overhangs and other criteria."""
        overhangs = self.getOverhangPerimeterLineStrings()  # Get overhang perimeters
        if len(overhangs) > 0 or self.parameters.get("ReplaceInternalBridging"):
            if self.parameters.get("PrintDebugVerification"):
                print(f"Layer {self.layernumber}: {len(overhangs)} Overhangs found")
            
            if not self.allowedSpacePolygon:
                input(f"Layer {self.layernumber}: no allowed space Polygon provided to layer obj, unable to run script. Press Enter.")
                raise ValueError(f"Layer {self.layernumber}: no allowed space Polygon provided to layer obj")
            
            if self.parameters.get("PrintDebugVerification"):
                print("No of Polys:", len(self.polys))
            
            for idp, poly in enumerate(self.polys):
                if not poly.is_valid:
                    if self.parameters.get("PrintDebugVerification"):
                        print(f"Layer {self.layernumber}: Poly{idp} is (shapely-)invalid")
                    continue  # Skip invalid polygons
                
                if (not self.allowedSpacePolygon.contains(poly)) and self.parameters.get("CheckForAllowedSpace"):
                    if self.parameters.get("PrintDebugVerification"):
                        print(f"Layer {self.layernumber}: Poly{idp} is not in allowedSpacePolygon")
                    continue  # Skip polygons outside the allowed space
                
                if poly.area < self.parameters.get("MinArea"):
                    if self.parameters.get("PrintDebugVerification"):
                        print(f"Layer {self.layernumber}: Poly{idp} has too little area: {poly.area:.2f}")
                    continue  # Skip polygons with insufficient area
                
                verified = False
                indexedOverhangPerimeters = STRtree(overhangs)
                prevLayer.makeExternalPerimeter2Polys()
                indexedExtPerimeters = STRtree(prevLayer.extPerimeterPolys)
                dists = indexedOverhangPerimeters.query_nearest(poly, maxDistForValidation, return_distance=True)[1]
                overlappers = indexedExtPerimeters.query(poly, predicate="overlaps")
                for dist in dists:
                    if dist < maxDistForValidation:  # Check if this poly is close to an overhang
                        verified = True
                        break
                if not verified and self.parameters.get("ReplaceInternalBridging"):
                    for overlap in overlappers:
                        if overlaps(poly, prevLayer.extPerimeterPolys[overlap]):  # Check if this poly hangs over an edge
                            verified = True
                            break
                if verified:
                    self.validpolys.append(poly)  # Mark polygon as valid
                    self.deleteTheseInfills.append(idp)  # Mark for deletion
                
                if self.parameters.get("PrintDebugVerification"):
                    print(f"Layer {self.layernumber}: Poly{idp} is not close enough to overhang perimeters")

    def prepareDeletion(self, featurename: str = getSlicerSpecificName(";TYPE:Bridge infill"), polys: list = None) -> None:
        """Prepare deletion ranges for G-code lines based on feature and polygon overlaps."""
        if not polys:
            polys = self.validpolys  # Use default polygons if none provided
        
        if polys is self.validpolys:
            idx = self.indexedValidPolys  # Use indexed valid polygons
        elif polys is self.oldpolys:
            idx = self.indexedOldPolys  # Use indexed old polygons
        else:
            idx = STRtree(polys)  # Create an index for custom polygons
        
        for idf, fe in enumerate(self.features):
            if not featurename in fe[0]:
                continue  # Skip features that don't match the feature name
            
            lines = fe[1]
            start = fe[2]
            deleteThis = False
            
            for line in lines:
                p = getPtfromCmd(line)
                if p is None:
                    continue  # Skip lines without coordinates
                
                possible_polys = idx.query(p, predicate='within').tolist()  # Find polygons that may contain the point
                if len(possible_polys) > 0:
                    for i in possible_polys:
                        if polys[i] in self.failedArcGenPolys:
                            continue  # Skip polygons that failed arc generation
                        if contains_xy(polys[i], p.x, p.y):
                            deleteThis = True  # Mark for deletion if point is within a valid polygon
                            break
                if deleteThis:
                    break  # No need to check further lines
            
            if deleteThis:
                if idf < len(self.features) - 1:
                    end = self.features[idf + 1][2] - 1  # Set end to the start of the next feature
                    
                    while isTravelMove(self.lines[end - 1]) or isTravelMove(self.lines[end]):
                        end -= 1  # Exclude the last travel move from the deletion range
                else:
                    end = len(self.lines)  # Set end to the end of the G-code lines
                
                # Create a set of all line numbers between start and end (inclusive)
                finalDeletionLines = set(range(start, end))  

                if self.failedSolidInfillLocations:  # If there are solid infill locations we want to preserve
                    remainingGroups = []  # List to store groups to keep

                    for saveGroup in self.failedSolidInfillLocations:
                        if not finalDeletionLines.isdisjoint(saveGroup):  # Check if the saveGroup overlaps with lines to be deleted
                            finalDeletionLines -= set(range(*saveGroup))  # Remove overlapping lines from the deletion set
                        else:
                            remainingGroups.append(saveGroup)  # Keep this group for future checks as it doesn't affect deletion lines
                    
                    self.failedSolidInfillLocations = remainingGroups  # Update the failedSolidInfillLocations with remaining groups

                self.deletelines.update(finalDeletionLines)  # Add the deletion set

    def exportThisLine(self, linenumber: int) -> bool:
        """Determine if a G-code line should be exported based on deletion ranges."""
        export = True
        if len(self.deletelines) > 0:
            if linenumber in self.deletelines:  # Check if line is in the set of lines we want deleted
                export = False
        return export

    def createHilbertCurveInPoly(self, poly: Polygon):
        """Generate a Hilbert curve within a polygon using specified parameters."""
        print("making hilbert surface")
        dimensions = 2
        w = self.parameters.get("solid_infill_extrusion_width")
        a = self.parameters.get("HilbertFillingPercentage") / 100
        mmBetweenTravels = (self.parameters.get("aboveArcsInfillPrintSpeed") / 60) * self.parameters.get("HilbertTravelEveryNSeconds")
        minX, minY, maxX, maxY = poly.bounds
        lx = maxX - minX
        ly = maxY - minY
        l = max(lx, ly)
        
        segments_needed = a * l / w
        iterationCount = ceil(log2(segments_needed + 1))  # Calculate iterations for Hilbert curve
        
        scale = w / a
        maxidx = 2 ** (dimensions * iterationCount) - 1
        
        locs = decode(np.arange(maxidx), dimensions, iterationCount)  # Generate Hilbert curve points
        
        movX = self.layernumber % 2 * w / a  # Adjust movement based on layer number
        movY = self.layernumber % 2 * w / a
        
        x = locs[:, 0] * scale + minX - movX  # Scale and shift x coordinates
        y = locs[:, 1] * scale + minY - movY  # Scale and shift y coordinates
        
        points = np.column_stack((x, y))  # Combine x and y into points array
        
        contains_mask = contains_xy(poly, x, y)  # Check which points are inside the polygon

        diff = np.diff(contains_mask.astype(int))
        run_starts = np.where(diff == 1)[0] + 1  # Find start indices of runs inside the polygon
        run_ends = np.where(diff == -1)[0] + 1  # Find end indices of runs inside the polygon
        if contains_mask[0]:
            run_starts = np.insert(run_starts, 0, 0)  # Include the first point if inside
        if contains_mask[-1]:
            run_ends = np.append(run_ends, len(points))  # Include the last point if inside
        
        noEl = ceil(mmBetweenTravels / scale)  # Calculate chunk size for splitting runs
        
        compositeList = []
        for start, end in zip(run_starts, run_ends):
            run_points = points[start:end]  # Extract points for each run
            if len(run_points) > 1:
                chunks = [run_points[i:i + noEl] for i in range(0, len(run_points), noEl)]  # Split into chunks
                for chunk in chunks:
                    if len(chunk) > 1:
                        compositeList.append([Point(p) for p in chunk])  # Convert to Points and add to list
        
        shuffle(compositeList)  # Shuffle to prevent localized overheating
        
        return compositeList

    def isClose2Bridging(self, line: str, maxDetectionDistance: float = 3) -> bool:
        """Check if a G-code line is close to a bridging area."""
        if not "G1" in line:
            return False  # Skip non-G1 lines
        p = getPtfromCmd(line)
        if not p:
            return False  # Skip if no point is extracted
        distances = self.indexedOldPolys.query_nearest(p, max_distance=maxDetectionDistance, return_distance=True)[1] # Return all distances to polygons that may be in the detection distance
        return any(dist <= maxDetectionDistance for dist in distances) # Return True if any distance is within the threshold

    def spotFanSetting(self, lastfansetting: float) -> float:
        """Find and return the fan setting (M106) from the G-code lines."""
        for line in self.lines:
            if "M106" in line.split(";", 1)[0]:  # Check for M106 command
                svalue = line.strip("\n").split(";", 1)[0].split(" ")[1]  # Extract the S value
                self.fansetting = float(svalue[1:])  # Convert S value to float
                return self.fansetting  # Return the fan setting
        self.fansetting = lastfansetting  # Use the last fan setting if no M106 found
        return lastfansetting




class Arc():
    def __init__(self,center:Point,r:float,kwargs:dict={}) -> None:
        self.center=center
        self.r=r
        self.pointsPerMillimeter=kwargs.get("ArcPointsPerMillimeter", 0.1)
        self.parameters=kwargs

    def extractArcBoundary(self):
        """Extract and merge the boundary of the arc as a LineString or MultiLineString."""
        trueArc = self.arcline
        if isinstance(trueArc, MultiLineString):
            merged = linemerge(trueArc)  # Merge multiple LineStrings into one
            return merged if isinstance(merged, LineString) else merged.geoms  # Return merged LineString or its parts
        elif isinstance(trueArc, LineString):
            return trueArc  # Return the LineString as is
        elif isinstance(trueArc, GeometryCollection):
            lines = 0
            geoms = []
            for geom in trueArc.geoms:
                if isinstance(geom, LineString):
                    lines += 1
                    geoms.append(geom) # Put LineString into return array
                elif isinstance(geom, MultiLineString):
                    lines += len(geom.geoms)
                    geoms.extend(geom.geoms)  # Put component LineStrings into return array
            if lines == 1:
                return geoms[0]  # Return the single LineString
            elif lines > 1:
                return linemerge(MultiLineString(geoms))  # Merge multiple LineStrings
            else:
                print(trueArc)
                input(f"ArcBoundary merging Error. Arc is of geometry type {trueArc.__class__.__name__}. Unable to run script. Press Enter.")
                raise ValueError("ArcBoundary merging Error")
        else:
            print(trueArc)
            input(f"ArcBoundary merging Error. Arc is of geometry type {trueArc.__class__.__name__}. Unable to run script. Press Enter.")
            raise ValueError("ArcBoundary merging Error")

    def generateConcentricArc(self, startpt: Point, remainingSpace: Polygon) -> Polygon:
        """Generate a concentric arc by intersecting a circle with the remaining space."""
        self.circle = create_circle(startpt, self.r, self.pointsPerMillimeter)
        self.arcline = intersection(self.circle, remainingSpace)  # Intersect the circle with the remaining space
        if isinstance(self.arcline, MultiLineString):
            self.arcline = linemerge(self.arcline)

        # sector_coords = []
        # for geom in self.arcline.geoms if isinstance(self.arcline, MultiLineString) else [self.arcline]:
        #     if geom.is_empty:
        #         continue
        #     coords = list(geom.coords)
        #     coords.append(self.center.coords[0])
        #     sector_coords.append(coords)
        # sectorPolys = []
        # for sector in sector_coords:
        #     sectorPolys = Polygon(sector)
        # self.sector = GeometryCollection(sectorPolys)

        return self.arcline  # Return the resulting arc

class BridgeInfill():
    def __init__(self,pts=[],id=randint(1,int(1e10))) -> None:
        self.pts=pts
        self.deleteLater=False
        self.id=id

################################# HELPER FUNCITONS Polygon->Arc #################################
#################################################################################################

def fill_remaining_space(last_arc: Arc, r_min: float, r_max: float, min_distance_from_perimeter: float, filled_space: Polygon, poly: Polygon, parameters: dict):
    """Fill the remaining space with concentric arcs until the minimum distance is reached."""
    arcs = []
    allowedRetries = parameters.get("AllowedArcRetries")
    failureCount = 0

    text = "Recursion not needed to fill space."
    for id in range(parameters.get("SafetyBreak_MaxArcNumber")):
        remaining_space = difference(poly, buffer(filled_space, parameters.get("ArcWidth") / 2))  # Calculate remaining space
        farthest_points, longest_distances = get_farthest_points(filled_space.boundary, poly, allowedRetries + 1)  # Find the farthest point

        if farthest_points.size == 0 or longest_distances[failureCount] < min_distance_from_perimeter:
            break  # Stop if no valid point or distance is too small

        start_pt = move_toward_point(farthest_points[failureCount], last_arc.center, parameters.get("ArcCenterOffset", 2))  # Adjust start point
        concentric_arcs = generateMultipleConcentricArcs(start_pt, r_min, r_max, poly.boundary, remaining_space, parameters)  # Generate arcs

        if len(concentric_arcs) == 0:
            failureCount += 1
            if failureCount >= allowedRetries:
                break  # Stop if no arcs are generated
            continue
        
        failureCount = 0
        last_arc = concentric_arcs[-1]  # Update the last arc
        filled_space = intersection(poly, unary_union((filled_space, Polygon(last_arc.circle))))  # Merge filled space with new arcs
        arcs.extend(concentric_arcs)  # Add new arcs to the list
        
        text = f"Filling remaining space. Iterations: {id}. Arcs this iteration: {len(concentric_arcs)}."
        print(text, end='\r', flush=True)

        if parameters.get("plotArcsEachStep"):
            plt.title(f"Total No Arcs: {len(arcs)}")
            plot_geometry([arc.arcline for arc in arcs], changecolor=True)
            plot_geometry(filled_space, 'g', filled=True)
            plot_geometry(poly, 'r')
            plt.axis('square')
            plt.show()

    print(text)

    return arcs, buffer(filled_space, parameters.get("ArcWidth") / 2)

def midpoint(p1: Point, p2: Point) -> Point:
    """Calculate the midpoint between two points."""
    return Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)  # Return the midpoint as a Point

def getStartPtOnLS(ls: LineString, kwargs: dict = {}, choseRandom: bool = False) -> Point:
    """Select a starting point on a LineString, preferring corners or random points."""
    if ls.geom_type == "MultiLineString" or ls.geom_type == "GeometryCollection":
        lengths = []
        for lss in ls.geoms:
            if lss.geom_type == "LineString":
                lengths.append(lss.length)
            else:
                print("Startline Item bizzare Type of geometry:", lss.geom_type)
                lengths.append(0)
        lsidx = np.argmax(lengths)
        ls = ls.geoms[lsidx]  # Use the longest LineString

    if len(ls.coords) < 2:
        warnings.warn("Start LineString with <2 Points invalid")
        input("Can not run script, gcode unmodified. Press Enter")
        raise ValueError("Start LineString with <2 Points invalid")

    if len(ls.coords) == 2:
        return midpoint(Point(ls.coords[0]), Point(ls.coords[1]))  # Return midpoint for 2-point LineString

    scores = []
    curLength = 0
    pts = [Point(p) for p in ls.coords]  # Convert coordinates to Points

    if choseRandom:
        return choice(pts)  # Return a random point if chosen

    coords = [np.array(p) for p in ls.coords]  # Convert coordinates to numpy arrays

    for idp, p in enumerate(pts):
        if idp == 0 or idp == len(pts) - 1:
            scores.append(0)  # Ignore start and end points
            continue

        curLength += distance(p, pts[idp - 1])  # Accumulate length
        relLength = curLength / ls.length  # Relative length
        lengthscore = 1 - abs(relLength - 0.5)  # Hat-function: score=1 at middle, 0 at start/end

        v1 = coords[idp] - coords[idp - 1]  # Vector to previous point
        v2 = coords[idp + 1] - coords[idp]  # Vector to next point

        if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:  # Non-zero vectors
            v1 = v1 / np.linalg.norm(v1)
            v2 = v2 / np.linalg.norm(v2)
            anglescore = np.abs(np.sin(np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))))  # Score for corner angles
            anglescore *= kwargs.get("CornerImportanceMultiplier", 1)  # Adjust with multiplier
            scores.append(lengthscore + anglescore)  # Combine length and angle scores
        else:
            scores.append(lengthscore)  # Use length score only

    maxIndex = scores.index(max(scores))  # Find index of highest score
    return pts[maxIndex]  # Return the point with the highest score

def create_circle(center: Point, radius: float, points_per_mm: float) -> LinearRing:
    """Create a circular ring around a given point with a specified radius and resolution."""
    x, y = center.x, center.y  # Extract the center point coordinates
    n = ceil(2 * pi * radius * points_per_mm)  # Calculate the number of points based on resolution
    theta = np.linspace(0, 2 * pi - 2 * pi / n, n)  # Generate evenly spaced angles
    points = np.column_stack((radius * np.sin(theta) + x, radius * np.cos(theta) + y))  # Compute circle points
    return LinearRing(points)  # Return the circle as a LinearRing

def create_circle_between_angles(center:Point, radius:float, startAngle:float, endAngle:float, points_per_mm: float, clockwise: bool = False)->List[float]:
    x, y = center.x, center.y
    n = ceil(abs(endAngle - startAngle + ((2 * pi) if endAngle < startAngle and not clockwise else 0)) * radius * points_per_mm)  # Calculate the number of points based on resolution
    theta = np.linspace(startAngle, endAngle, n)
    if clockwise:
        theta = np.flip(theta)
    points = np.column_stack((radius * np.sin(theta) + x, radius * np.cos(theta) + y))  # Compute circle points
    return LineString(points)

def get_farthest_points(from_geom: Geometry, to_poly: Polygon, number_of_points: int = 1) -> tuple:
    """
    Find the point on a given geometry that is farthest away from the boundary of a polygon.
    
    Parameters
    ----------
    from_geom : Geometry
        The geometry (e.g., LineString, MultiLineString) from which to find the farthest point.
    to_poly : Polygon
        The polygon representing the boundary.
    
    Returns
    -------
    farthest_point : Point or None
        The point on the geometry that is farthest from the polygon's boundary.
    longest_distance : float or None
        The distance from the farthest point to the polygon's boundary.
    """
    if from_geom.is_empty:
        return None, None  # Return None if the input geometry is empty
    
    prepare(to_poly.boundary)  # Prepare the polygon boundary for faster distance calculations

    try:
        coords = points(from_geom.coords)  # Extract points from the geometry's coordinates
    except NotImplementedError:
        # Handle MultiLineString by extracting coordinates from each part
        if isinstance(from_geom, MultiLineString):
            coords = []
            for geom in from_geom.geoms:
                coords.extend(geom.coords)
            coords = points(coords)

    distances = distance(to_poly.boundary, coords)  # Calculate distances from each point to the polygon's boundary
    farthest_points = []
    longest_distances = []
    # Get indices sorted by descending distances
    sorted_indices = np.argsort(distances)[::-1]

    # Select top 'number_of_points' indices
    top_indices = sorted_indices[:number_of_points]

    # Retrieve the longest distances and farthest points
    longest_distances = distances[top_indices]
    farthest_points = coords[top_indices]
    
    return farthest_points, longest_distances

def move_toward_point(start_point: Point, target_point: Point, distance: float, angle_correction: float = 0.0) -> Point:
    """Move a point by a set distance toward another point and adjust the angle direction"""
    
    # Calculate the difference in coordinates
    dx = target_point.x - start_point.x
    dy = target_point.y - start_point.y
    
    # Calculate the magnitude of the difference vector
    magnitude = hypot(dx, dy)
    
    # Handle the case where start and target points are the same
    if magnitude == 0:
        # If no movement, return the start point as is
        return Point(start_point.x, start_point.y)
    
    # Normalize the direction vector
    dx /= magnitude
    dy /= magnitude
    
    # Calculate the angle in degrees
    angle = degrees(atan2(dy, dx))
    
    # Apply angle correction
    new_angle = angle + angle_correction
    
    # Calculate the movement in x and y directions
    move_x = cos(radians(new_angle)) * distance
    move_y = sin(radians(new_angle)) * distance
    
    # Move the start point in the new direction
    new_x = start_point.x + move_x
    new_y = start_point.y + move_y
    
    # Return the new point
    return Point(new_x, new_y)

# def redistribute_vertices(geom: LineString, dist: float) -> LineString:  # TODO: consider shapely's segmentize(geometry, max_segment_length, ...)
#     """Redistribute vertices of a LineString or MultiLineString at a specified distance."""
#     if geom.geom_type == 'LineString':
#         num_vert = ceil(geom.length / dist)  # Calculate number of vertices
#         if num_vert == 0:
#             num_vert = 1  # Ensure at least one vertex
#         return LineString(
#             [geom.interpolate(float(n) / num_vert, normalized=True)  # Interpolate vertices
#              for n in range(num_vert + 1)])
#     elif geom.geom_type == 'MultiLineString':
#         parts = [redistribute_vertices(part, dist) for part in geom.geoms]  # Recursively process each part
#         return type(geom)([p for p in parts if not p.is_empty])  # Filter out empty parts
#     else:
#         warnings.warn('unhandled geometry %s', (geom.geom_type,))  # Warn for unsupported geometry types
#         return geom

def generateMultipleConcentricArcs(startpt: Point, rMin: float, rMax: float, basePoly: Polygon, remainingSpace: Polygon, kwargs={}) -> list:
    """Generate concentric arcs within a given range of the radius and boundary."""
    r_values = np.arange(rMin, rMax + kwargs.get("ArcWidth"), kwargs.get("ArcWidth"))  # Generate radii for arcs
    arcs = []  # Initialize list to store arcs

    for r in r_values:
        arcObj = Arc(startpt, r, kwargs=kwargs)  # Create an Arc object
        arc = arcObj.generateConcentricArc(startpt, remainingSpace)  # Generate the concentric arc
        if arc.is_empty:
            break  # Stop if the arc lies completely outside the polygon or it intersects the boundary and the least amount of center points is not used
        arcs.append(arcObj)  # Add the arc to the list
        if intersects(basePoly, arc) and not kwargs.get("UseLeastAmountOfCenterPoints", False):
            break

    return arcs

################################# HELPER FUNCTIONS Arc Validation #################################
###################################################################################################

def getValueBasedColor(val: float, max_val=10) -> tuple:
    """Generate a color based on a normalized value."""
    normalizedVal = val / max_val  # Normalize the value
    rgb = [0, 0, 0]  # Initialize RGB color
    rgb[0] = min(normalizedVal, 1)  # Set red channel
    rgb[2] = 1 - rgb[0]  # Set blue channel inversely
    return tuple(rgb)  # Return RGB color as a tuple

def plot_geometry(geometry, color='black', linewidth=1, **kwargs):
    """
    Plot various geometry types using matplotlib.

    Args:
        geometry: A geometry object or list of geometry objects to plot.
        color (str): The color of the plotted geometry (default: 'black').
        linewidth (int): The width of the plotted lines (default: 1).\n
        **kwargs: Optional keyword arguments:
        - changecolor (bool): Dynamically change color for each geometry in a list (default: False).
        - filled (bool): Fill polygons with the specified color (default: False).
        - filled_holes (bool): Fill polygon interiors (holes) with the specified color (default: False).

    Returns:
        None: Plots the geometry using matplotlib.
    """
    if type(geometry) == type([]):
        # Handle list of geometries
        for idx, geo in enumerate(geometry):
            if kwargs.get("changecolor"):
                color = getValueBasedColor(idx, len(geometry))  # Change color dynamically
            plot_geometry(geo, color=color, linewidth=linewidth, kwargs=kwargs)
    elif geometry.geom_type == 'Point':
        # Plot a single point
        x, y = geometry.x, geometry.y
        plt.scatter(x, y, color=color, linewidth=linewidth)
    elif geometry.geom_type == 'LineString' or geometry.geom_type == "LinearRing":
        # Plot a line string
        x, y = geometry.xy
        plt.plot(x, y, color=color, linewidth=linewidth)
    elif geometry.geom_type == 'Polygon':
        # Plot the exterior of a polygon
        x, y = geometry.exterior.xy
        plt.plot(x, y, color=color, linewidth=linewidth)
        if kwargs.get("filled"):
            plt.fill(x, y, color=color, alpha=0.8)  # Fill the polygon
        for interior in geometry.interiors:
            # Plot polygon interiors (holes)
            x, y = interior.xy
            plt.plot(x, y, color=color, linewidth=linewidth)
            if kwargs.get("filled_holes"):
                plt.fill(x, y, color=color, alpha=0.5)  # Fill holes
    elif geometry.geom_type == 'MultiLineString':
        # Plot multiple line strings
        for line in geometry.geoms:
            x, y = line.xy
            plt.plot(x, y, color=color, linewidth=linewidth)
    elif geometry.geom_type == 'MultiPolygon' or geometry.geom_type == "GeometryCollection":
        # Recursively plot polygons or geometry collections
        for polygon in geometry.geoms:
            plot_geometry(polygon, color=color, linewidth=linewidth, kwargs=kwargs)
    else:
        print('Unhandled geometry type: ' + geometry.geom_type)  # Notify of unsupported geometry type

################################# HELPER FUNCTIONS Arc->GCode #################################
###############################################################################################

def getArcBoundaries(concentricArcs: list) -> list:
    """Extract boundary lines from concentric arcs."""
    boundaries = []  # Initialize list to store boundary lines
    for arc in concentricArcs:
        arcLines = arc.extractArcBoundary()  # Extract boundary lines from the arc
        if isinstance(arcLines, GeometrySequence):
            # If arcLines is a sequence, append all
            boundaries.extend(arcLines)
        else:
            # Otherwise, add the single boundary line
            boundaries.append(arcLines)
    return boundaries

def readSettingsFromGCode2dict(gcodeLines: list, fallbackValuesDict: dict) -> dict:
    """Extract slicer settings from G-code lines into a dictionary."""
    gCodeSettingDict = fallbackValuesDict  # Start with fallback values
    isSetting = False
    global slicer

    for line in gcodeLines:
        if "; prusaslicer_config = begin" in line:
            slicer = "PrusaSlicer"
            isSetting = True  # Start reading settings
            continue
        elif "; CONFIG_BLOCK_START" in line:
            slicer = "OrcaSlicer"
            isSetting = True  # Start reading settings
            continue
        #elif... Add detections for other slicers
        if isSetting:
            line = line.strip(';').strip()
            if '=' in line:
                key, value = line.split('=', 1)
                key = key.strip()
                value = value.strip()
                internal_key = _SLICER_SETTINGS_MAP.get(slicer).get(key)
                if internal_key:
                    try:
                        gCodeSettingDict[internal_key] = literal_eval(value)
                    except:
                        gCodeSettingDict[internal_key] = value
    
    if slicer is None:
        warnings.warn("No slicer detected. Please make sure your slicer can be detected by the script.")
        sys.exit(0)

    isWarned = False
    for key, val in gCodeSettingDict.items():
        if isinstance(val, tuple):
            # Use fallback value if available, otherwise use first tuple element
            if gCodeSettingDict.get("Fallback_" + key):
                gCodeSettingDict[key] = gCodeSettingDict.get("Fallback_" + key)
            else:
                gCodeSettingDict[key] = val[0]
                if not isWarned:
                    warnings.warn(message=f"{key} was specified as tuple/list, this is normal for using multiple extruders. For all list values First values will be used. If unhappy: Add manual fallback value by searching for ADD FALLBACK in the code. And add 'Fallback_<key>:<yourValue>' into the dictionary.")
                    isWarned = True

    # Handle percentage-based perimeter extrusion width (credit: 5axes via GitHub)
    if "%" in str(gCodeSettingDict.get("perimeter_extrusion_width")):
        gCodeSettingDict["perimeter_extrusion_width"] = gCodeSettingDict.get("nozzle_diameter", 0.4) * (float(gCodeSettingDict.get("perimeter_extrusion_width").strip("%")) / 100)

    return gCodeSettingDict

def checkforNecesarrySettings(gCodeSettingDict: dict) -> bool:
    """Check if necessary slicer settings are enabled for the script to work."""
    if not gCodeSettingDict.get("use_relative_e_distances"):
        warnings.warn(f"Script only works with relative e-distances enabled in {slicer}. Change accordingly.")
        return False
    if gCodeSettingDict.get("extrusion_width") < 0.001 or gCodeSettingDict.get("perimeter_extrusion_width") < 0.001 or gCodeSettingDict.get("solid_infill_extrusion_width") < 0.001:
        warnings.warn(f"Script only works with {getSlicerSpecificName("extrusion_width")}, {getSlicerSpecificName("perimeter_extrusion_width")}, and {getSlicerSpecificName("solid_infill_extrusion_width")} > 0. Change in {slicer} accordingly.")
        return False
    if not gCodeSettingDict.get("overhangs"):
        warnings.warn(f"Overhang detection disabled in {slicer}. Activate for script success!")
        return False
    if gCodeSettingDict.get("bridge_speed") > 5:
        warnings.warn(f"Your Bridging Speed is set to {gCodeSettingDict.get('bridge_speed'):.0f} mm/s in {slicer}. This can cause problems with warping. <=5mm/s is recommended.")
    if gCodeSettingDict.get("infill_first"):
        warnings.warn(f"Infill set in {slicer} to be printed before perimeter. This can cause problems with the script.")
    if gCodeSettingDict.get("external_perimeters_first") or gCodeSettingDict.get("wall_sequence") == "outer wall/inner wall":
        warnings.warn(f"{slicer}-Setting: External perimeter is printed before inner perimeters. Change for better overhang performance.")
    if not gCodeSettingDict.get("avoid_crossing_perimeters"):
        warnings.warn(f"{slicer}-Setting: Travel Moves may cross the outline and therefore cause artifacts in arc generation.")
    return True

def calcESteps(settingsdict: dict, layerheight: float = None) -> float:
    """Calculate extrusion steps based on layer height or bridging settings."""
    if layerheight:  # Case: printing on surface
        w = settingsdict.get("infill_extrusion_width")
        h = layerheight
        eSurfaceArea = (w - h) * h + pi * (h / 2)**2 * settingsdict.get("HilbertInfillExtrusionMultiplier", 1)
    else:  # Case: bridging (used for arcs)
        eSurfaceArea = (settingsdict.get("nozzle_diameter") / 2)**2 * pi * settingsdict.get("ArcExtrusionMultiplier", 1)
        # Source: https://manual.slic3r.org/advanced/flow-math

    if settingsdict.get("use_volumetric_e"):
        return eSurfaceArea  # Return surface area directly if using volumetric E
    else:
        eSteps = eSurfaceArea / ((settingsdict.get("filament_diameter") / 2)**2 * pi)
        return eSteps  # Calculate and return extrusion steps

def p2GCode(p: Point, E=0, **kwargs) -> str:
    """Generate G-code for moving to a point with specified extrusion and feed rate."""
    line = f"G1 X{p.x:.6} Y{p.y:.6} "  # Start G-code line with X and Y coordinates
    line += "E0" if E == 0 else f"E{E:.7f}"  # Add extrusion value (E) if specified
    if kwargs.get('F'):
        line += f" F{kwargs.get('F'):0d}"  # Add feed rate (F) if provided
    line += '\n'  # End line with a newline character
    return line

def retractGCode(retract: bool = True, kwargs: dict = {}) -> str:
    """Generate G-code for filament retraction or unretraction."""
    retractDist = kwargs.get("retract_length", 1)  # Get retraction distance
    E = -retractDist if retract else retractDist  # retract or extrude
    return f"G1 E{E} F{kwargs.get('retract_speed', 35) * 60}\n"  # Return G-code for retraction/unretraction

def setFeedRateGCode(F: int) -> str:
    """Generate G-code to set the feed rate (F) for the printer."""
    return f"G1 F{F}\n"

def arc2GCode(arcline: LineString, eSteps: float, arcidx=None, kwargs={}) -> list:
    """Generate G-code for an arc segment based on its LineString and extrusion steps."""
    GCodeLines = []  # Initialize G-code list
    p1 = None
    pts = points(arcline.coords)  # Convert coordinates to Point objects
    if len(pts) < 2:
        return []  # Return empty list if not enough points

    extDist = kwargs.get("ExtendArcDist", 0.5)  # Get tangential extension distance
    pExtendBegin = move_toward_point(pts[0], pts[1], extDist, -90)  # Extend the arc tangentially
    pExtendEnd = move_toward_point(pts[-1], pts[-2], extDist, 90)  # Extend the arc tangentially
    arcPrintSpeed = np.clip(arcline.length / (kwargs.get("ArcSlowDownBelowThisDuration", 3)) * 60,
                            kwargs.get("ArcMinPrintSpeed", 1 * 60), kwargs.get('ArcPrintSpeed', 2 * 60))  # Calculate print speed

    for idp, p in enumerate(pts):
        if idp == 0:
            # Retract filament and move to the first point
            GCodeLines.append(retractGCode(retract=True, kwargs=kwargs))
            dist=distance(pExtendBegin, p)
            p1 = p
            GCodeLines.append(f";Arc {arcidx if arcidx else ' '} Length:{arcline.length}\n")
            GCodeLines.append(p2GCode(pExtendBegin, F=kwargs.get('ArcTravelFeedRate', 100 * 60)))
            GCodeLines.append(p2GCode(p, E=dist * eSteps))
            GCodeLines.append(retractGCode(retract=False, kwargs=kwargs))
            GCodeLines.append(setFeedRateGCode(arcPrintSpeed))
        else:
            dist = distance(p, p1)
            if dist > kwargs.get("GCodeArcPtMinDist", 0.1):
                # Extrude while moving to the next point
                GCodeLines.append(p2GCode(p, E=dist * eSteps))
                p1 = p
        if idp == len(pts) - 1:
            # Extend the arc tangentially for better bonding
            GCodeLines.append(p2GCode(pExtendEnd, E=extDist * eSteps))

    return GCodeLines

def hilbert2GCode(allhilbertpts: list, parameters: dict, layerheight: float):
    """Generates G-code for a 3D printer based on a list of Hilbert curve points."""
    hilbertGCode = []  # Initialize an empty list to store the generated G-code
    eSteps = calcESteps(parameters, layerheight)  # Calculate extrusion steps based on parameters and layer height

    for idc, curvepts in enumerate(allhilbertpts):
        for idp, p in enumerate(curvepts):
            if idp == 0:
                # Move to the first point of the curve without extruding
                hilbertGCode.append(p2GCode(p, F=parameters.get("ArcTravelFeedRate")))
                if idc == 0:
                    # Extrude filament before starting the first curve
                    hilbertGCode.append(retractGCode(False, parameters))
            elif idp == 1:
                # Extrude filament while moving to the second point of the curve
                hilbertGCode.append(p2GCode(p, E=eSteps * distance(p, lastP), F=parameters.get("aboveArcsInfillPrintSpeed")))
            else:
                # Extrude filament while moving to subsequent points of the curve
                hilbertGCode.append(p2GCode(p, E=eSteps * distance(p, lastP)))
            lastP = p  # Update the last point to the current point

        # End of the current curve segment

    # Retract filament at the end of the entire Hilbert curve
    hilbertGCode.append(retractGCode(True, parameters))
    return hilbertGCode

def line2TravelMove(line: str, parameters: dict, ignoreZ: bool = False) -> str:
    """Convert a G-code line to a travel move by modifying extrusion and feed rate."""
    if "E0 " in line or "E0\n" in line:
        return line  # This is already a travel move
    
    if ignoreZ:
        regex = r" Z\d*\.?\d*" # Regex to match any change to Z (e.g., Z.24, Z5, Z2.12) https://regex101.com/.
        line = re.sub(regex, "", line)

    travelstr = f"F{parameters.get('travel_speed') * 60}"  # Create travel feed rate string

    if not "E" in line:
        line=line.replace("\n", " E0 " + travelstr + "\n") # there was no extrusion code, lets add one just to keep tidy
        return line 
    
    regex=r"E-?\d*\.?\d*" # Regex to match any extrusion code (e.g., E-1.24, E0, E5) https://regex101.com/.
    line=re.sub(regex, "E0 " + travelstr, line) # replaces any extrusion code with E0, followed by travelstr
    return line

def _warning(message,category = UserWarning, filename = '', lineno = -1,*args, **kwargs):
    print(f"{filename}:{lineno}: {message}")
warnings.showwarning = _warning

################################# MAIN EXECUTION #################################
##################################################################################
def parse_args():
    parser = argparse.ArgumentParser(description="Process overhangs within G-code files into circular arcs.")
    parser.add_argument('path', type=str, help='Path to the G-code file')
    parser.add_argument('--skip-input', action='store_true', help='Skip any user input prompts (Windows only)')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    # Get file stream and path based on the provided path
    gCodeFileStream, path2GCode = getFileStreamAndPath(args.path)

    # Determine whether to skip input based on the platform and command line argument
    skipInput = args.skip_input or platform.system() != "Windows"

    # Call the main function with the arguments
    main(gCodeFileStream, path2GCode)
    if not skipInput:
        input("Press enter to exit.")
