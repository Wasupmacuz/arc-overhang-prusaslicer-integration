# *Please Check Out Discussions!*
# Arc Overhang
<p align="center">
<img src="https://github.com/nicolai-wachenschwan/arc-overhang-prusaslicer-integration/blob/main/examples/ExampleCatchImage.png" width=600>
  </p>
A 3D printer toolpath generation algorithm that lets you print up to 90° overhangs without support material, the original idea was by Steven McCulloch: https://github.com/stmcculloch/arc-overhang

**Now it is easy and convenient to use by integrating the functionality into PrusaSlicer as a post-processing script.**

Steven, Nic, and I hope that someday this feature gets integrated into slicing software. But until then you can use this script to get the added functionality today!

![possible usecases](examples/UseCasesGallery.png)
## 0. Videos

- Arc Overhang Initial Video: https://youtu.be/fjGeBYOPmHA
- CNC kitchen's video: https://youtu.be/B0yo-o47688
- Steven's Instagram: https://www.instagram.com/layershift3d/

This is a basic visualization of how the algorithm works: 

![arc-overhang visualization](examples/gcode_vis3.gif)

(this visualization uses depth-first generation, the current version uses a breadth-first search algorithm to fill the remaining space in the overhang.
## 1. Brief Explanation (from Steven McCulloch):

1. You can print 90° overhangs by wrapping filament around itself in concentric **arcs**. You may have seen the [fullcontrol.xyz overhang challenge](https://fullcontrol.xyz/#/models/b70938). This uses the exact same principle.
Here's what this effect looks like while printing:  

![printing demo](examples/printing_demo.gif)

2. You can start an **arc** on an **arc** to get ridiculously large overhangs.
To get perfect results you need to tune the process to fit your machine. Also it is painfully slow, but still faster than support + removal :P

For more details visit: https://github.com/stmcculloch/arc-overhang
## 3. Setup-Process
1. download and install Python 3, at least Version 3.10, and check the "add to PATH" box during the installation.
2. install the libraries [shapely](https://shapely.readthedocs.io/en/stable/), [numpy](https://numpy.org/), [matplotlib](https://matplotlib.org/), **and new [numpy-hilbert-curve](https://pypi.org/project/numpy-hilbert-curve/)** via "python -m pip install "+library-name in your console (type cmd in windows start-menu search) `python -m pip install  shapely numpy matplotlib numpy-hilbert-curve`.
3. Ready to go! Tested only with PrusaSlicer 2.5 & Python 3.10 :)


## 4. How to use it:
#### Option A) via Console
Simply open your system console and type 'python ' 
followed by the path to this script 
and the path of the gcode file. Will overwrite the file.
#### Option B) Use it as an automatic post-processing script in PrusaSlicer
1. open PrusaSlicer, go to print-settings-tab->output-options. Locate the window for post-processing-script. 
2. In that window enter: `C:\full\path\to\your\python.exe C:\full\path\to\this\script\including\prusa_slicer_post_processing_script.py`  (with blank space between the two paths!). For Unix-like systems (Linux, macOS, etc.) use the `/` instead of `\`, obtaining something like this: `full/path/to/your/python full/path/to/this/script/including/prusa_slicer_post_processing_script.py`
3. PrusaSlicer will execute the script after the export of the Gcode, therefore the view in PrusaSlicer won't change. 
4. Open the finished gcode file to see the results.

Notes to nail it first try:
If any path contains empty spaces, mask them as described here (using "\ " on Unix-like systems and "! " on Windows): 
https://manual.slic3r.org/advanced/post-processing
https://help.prusa3d.com/article/post-processing-scripts_283913

currently only using quotation marks " at the beginning and end of the path with empty spaces seems to work.


If you want to change generation settings: Open the Script in an editor and scroll to the 'Parameter' section. Settings from PrusaSlicer will be extracted automatically from the gcode.

## 5. Current Limitations
1. Some settings need to be tailored to your specific geometry, just like you adapt the settings in your slicer. Details below.
2. Code is slow on more complicated models.
3. The Arcs are extruded very thick, so the layer will be 0.1-0.5mm thicker (depending on nozzle dia) than expected
=> If precision is needed make test prints to counter this effect.
4. Some warping, when printing the follow-up layers. Print the follow-up layers as slowly as possible with the least amount of cooling possible. You don't need to change any PrusaSlicer Settings, as this is handled automatically by the script via the special cooling parameters. When you print to low cooling it will result in sagging/heat softening. Balance the cooling of these layers with gravity/heat softening: More Cooling=>More Warping. There is no generalized solution for all printers, but the default values should be a good start. 
5. no wiping or z-hop during travel moves
6. The remaining print time shown during printing is wrong. The real printtime can be seen when opening the finished file in GcodeViewer.
7. Will take the first island of the previous perimeter as a starting point. If you don't like that point, turn the models along the z-axis.
9. settings testet for PLA only. You can of course try other materials and share your knowledge here!
10. Physics: The arcs need to be able to support their own weight without much deformation. Therefore narrow and long bridges will be generated, but will not print successfully. Use some support to stabilize critical areas.

## 5.1 Updates:
Reduced warping significantly! The warping is caused by the follow-up layers that contract during the cooldown. The Script now automatically applies warping-reducing print settings within a certain z-distance over the arcs. There are parameters defined in the script that can be used to adapt the settings to your printer. The following settings are applied: Lower speed and lower fan speed. Conversion from solid infill to Hilbert curve. 

A Hilbert curve pattern is associated with the smallest residual thermal stresses [Research on SLS-3d-printing](https://www.researchgate.net/publication/313685481_Fractal_Scan_Strategies_for_Selective_Laser_Melting_of_'Unweldable'_Nickel_Superalloys). This shows great success, but some more time is needed to find the limits (e.g printing speed) and effects of this printing strategy. 
Additionally, the script splits up the Hilbert curve to avoid heat accumulation and therefore softening of the material below.

Unfortunately, we can't convert the perimeters, so print them as slowly as possible and with the least cooling possible to reduce the warping.
On the other hand: printing without cooling causes the overhang to bend downwards. Possibly caused by the added weight + heat softening, so a balance of the thermal contraction forces and gravity might be a solution. For me cooling at 10% worked quite nicely. Research is in Progress, Steven, Nic, and I would be happy if you share your knowledge and experiences from your prints!

Example images of minimized warping, tested on an extreme overhang with a 100mm diameter:
<p align="center">
<img src="https://github.com/nicolai-wachenschwan/arc-overhang-prusaslicer-integration/blob/main/examples/solving_warping.png" width=800>
  </p>
The bulging downwards happens due to the geometry of the object: The long arcs contract when cooled down, causing some stress in the printed part. The next one adds more contraction stress, which is released in the bulging. This happens due to physics and geometry. The bulging issue can be solved by a) setting rMax to 40mm, and b) changing the geometry so the arclines are split into multiple parts and extremely long arcs are avoided.

The warping of < 2mm is in my opinion acceptable, as we talk about a very large surface and it is only occurring at the very edge.

## 6. Suggested Print Settings
Some PrusaSlicer PrintSettings will be checked and warned if "wrong".

### Important Settings in the Script are:

a) **"ArcCenterOffset":** The surface quality is improved by Offsetting the arc center because the smallest r is larger->more time to cool. Set to 0 to get into delicate areas.

b) **"ExtendArcsIntoPerimeter":** Expand the area where arcs are generated. Increase to thicken small/delicate passages. The minimum value for the algorithm to work is 0.5extrusionwidth! 

c) **"MaxDistanceFromPerimeter":** Controls how bumpy you tolerate your edge. big: less tiny arcs, better surface. small: follow the curvature closer.

d) Thresholds for area and bridging length, adjust as needed, but Arc shine at large surfaces :)

e) **"UseLeastAmountOfCenterPoints":** experimental: use only one arc-center until rMax is reached (then iterate as usual), improves surface finish but can lead to failed prints on complex geometries.

f) Adjust the special cooling settings to your printer. This affects the follow-up layers to reduce warping. (More cooling->more warping)

### General Print Settings for the arcs: 
The overhang print quality is greatly improved when the material solidifies as quickly as possible. Therefore:
  
1. **Print as cold as possible.** Nic used 190 degrees for PLA. You can probably go even lower. If you require a higher temp for the rest of the print, you could insert some temp-change gcode before and after the arcs are printed. Might waste a lot of time though.
   
2. **Maximize cooling.** Set your fans to full blast. This technique will probably not work too well with ABS and materials that can't use cooling fans, but it hasn't been tested.
3. **Print slowly.** I use around 2 mm/s. Even that is too fast sometimes for the really tiny arcs since they have almost no time to cool before the next layer begins.

## 6.1 Examples:
<p align="center">
<img src="https://github.com/nicolai-wachenschwan/arc-overhang-prusaslicer-integration/blob/main/examples/example_different_surface_finish.png" width=800>
  </p>

## 7. Room for Improvement
We would be happy if you contributed!
The surface finish seems to be better with using as few as possible start points for the arcs. But where are the Limits? Finding an algorithm deciding when to start a new arc, and working reliably on a wide set of geometries is the next ongoing development. If you have any questions/ideas I would be happy if you share them!

Printing long overhangs is tricky due to gravity. A meet-in-the-middle concept could help, but how do we teach that to the computer?
Feel free to incorporate any ideas in the post-processing script and try them!

We can further optimize the settings or add features like z-hop and quality of life features like correct displaying of remaining print time.



## 8. Printer Compatibility

By default, the output gcode should print fine on most standard desktop FDM printers you can use with PrusaSlicer. PrusaSlicer is mandatory as the script listens to specific keywords...

## 9. Easy Way to try it out

If you want to try the prints without installing, Steven and Nic added some test print gcode files in the root directory that you can directly download. They should print fine on most printers although you may need to manually adjust the gcode so that it works with your printer.



## 10. Print it! 
If you get a successful print using this algorithm, I'd (and I am sure Steven too) love to hear about it.

## 11. How the Post-Processing Script works
The script analyses the given gcode file and splits it into layers. For every layer, the information is saved as objects (Class: Layer).
Then, it searches for the "Bridge Infill" tag in the gcode, kindly provided by PrusaSlicer.

The process will be shown with this simple example geometry(left). It has Bridge Infill at 2 areas in one layer, the right one is the overhang we want to replace (right).
![Explain Geometry](examples/Algorithm_explained/Algorithm_Geometry.png)

The real work is done by the shapely library. The Algorithm extracts the gcode and converts it into a shapely line (plot: green). By thickening the line we get one continuous Polygon(plot: black):
![infill to poly](examples/Algorithm_explained/Infill2Polygon.png)

The Polygon is verified by several steps, including touching some overhang perimeter.
To find a start point the external perimeter of the previous layer is extracted and the intersection area with our Polygon is calculated.
The common boundary of the intersection area and the Polygon will be the startline (magenta) for the arc generation.
![Start Geometry](examples/Algorithm_explained/Start_Geometry_Plot.png)

For each step, concentric arcs are generated, until they hit a boundary or rMax. The Farthest Point on the previous arc will be the next start point.
![Algorithm Animation](examples/Algorithm_explained/Algorithm_Step.gif)

The process is repeated until all points on the arcs are close enough to the boundary or overprinted by other arcs.
Finally, the gcode file will be rewritten. All infos are copied and the Arcs are injected at the beginning of the layer. the replaced bridge infill will be excluded:
![Result](examples/Algorithm_explained/Algorithm_3_Result.png)
