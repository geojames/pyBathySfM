## Welcome to pyBathySfM<a name="top"></a>
If you use this tutorial and the accompanying software, please cite the following paper:
>Dietrich JT. 2017. Bathymetric Structure-from-Motion: extracting shallow stream bathymetry from multi-view stereo photogrammetry. Earth Surface Processes and Landforms 42 : 355–364. DOI: [10.1002/esp.4060](http://onlinelibrary.wiley.com/doi/10.1002/esp.4060/abstract)

The above article was focused on UAV-based imagery collection, but ground-based imagery should be usable as long as the images do not include the horizon (i.e. the camera is pointing mainly down)

The instructions here are very similar to the original software [py_sfm_dpeth](https://geojames.github.io/py_sfm_depth/)

Because this is a long tutorial, I've added some table of contents links throughout for easy navigation between the sections.

>[Top](https://geojames.github.io/pyBathySfM/#top) | [SfM Collection and Processing](https://geojames.github.io/pyBathySfM/#step-1) | [Point Cloud Processing](https://geojames.github.io/pyBathySfM/#step-2) | [Data Prep](https://geojames.github.io/pyBathySfM/#step-3) | [Running the Script](https://geojames.github.io/pyBathySfM/#step-4)

### Quick Start
If your already familiar with the processing workflow you can run the code (load the GUI) at the command line. Change the directory to the location of the downloaded code and run:
``python py_BathySfM.py``
## Prerequisites
In order to use this this tutorial, you will need:

1.  Optimal site conditions:
    -   Clear water is first and foremost. High levels of suspended sediment (cloudy water) or tannic conditions (dark brown water) will inhibit the use of SfM to measure depths. The rule is if you can’t see the bottom of the stream, neither can the camera…and that’s going to cause problems.
    -   Minimal surface waves. Both wind-driven waves and hydraulic waves (riffles and standing waves) will increase the “noise” in the SFM point cloud. This will lead to inaccuracies/errors in the final outputs.
    -   Cloudy, hazy, or smoky weather conditions create a lot of surface reflections on the water with will inhibit accurate measurements.
2.  A Structure from Motion (or other photogrammetry) software package that can:
    -   Export a georeferenced (or scaled) point cloud dataset
    -   Export the calculated camera positions (x,y,z) and orientations (pitch, roll, yaw)
3.  A point cloud/3D data processing software:
    -   In this tutorial, I will be using my favorite (which is free and offers cross-platform support) – CloudCompare ([cloudcompare.org](http://www.cloudcompare.org/))
4.  An installation of Python ([python.org](http://www.python.org/)) that has a number of libraries installed
    -   I use a scientific python distribution, Anaconda ([https://www.continuum.io/downloads](https://www.continuum.io/downloads)), that has most of the needed libraries.
    - The minimum libraries are: **PyQT5, numpy, pandas, sympy, matplotlib**
