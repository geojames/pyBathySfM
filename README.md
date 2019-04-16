## Welcome to pyBathySfM<a name="top"></a>
If you plan to use this tutorial and the accompanying software, I would suggest you read (and cite) :
>Dietrich JT. 2017. Bathymetric Structure-from-Motion: extracting shallow stream bathymetry from multi-view stereo photogrammetry. Earth Surface Processes and Landforms 42 : 355–364. DOI: [10.1002/esp.4060](http://onlinelibrary.wiley.com/doi/10.1002/esp.4060/abstract)
>Also available on [ResearchGate](https://www.researchgate.net/publication/308765034_Bathymetric_Structure_from_Motion_Extracting_shallow_stream_bathymetry_from_multi-view_stereo_photogrammetry)

The above article was focused on UAV-based imagery collection, but ground-based imagery should be usable as long as the images do not include the horizon (i.e. the camera is pointing mainly down)

The instructions here are very similar to the original software [py_sfm_dpeth](https://geojames.github.io/py_sfm_depth/)

Because this is a long tutorial, I've added some table of contents links throughout for easy navigation between the sections.

****
[Top](https://geojames.github.io/pyBathySfM/#top) | [SfM Collection and Processing](https://geojames.github.io/pyBathySfM/#step-1) | [Point Cloud Processing](https://geojames.github.io/pyBathySfM/#step-2) | [Data Prep](https://geojames.github.io/pyBathySfM/#step-3) | [Running the Script](https://geojames.github.io/pyBathySfM/#step-4)
****
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
5.  As with any SfM collection technique, practice is your best friend.  **Do not**  expect perfect results from the first attempt to collect and correct SfM bathymetry.
___
[Top](https://geojames.github.io/pyBathySfM/#top) | [SfM Collection and Processing](https://geojames.github.io/pyBathySfM/#step-1) | [Point Cloud Processing](https://geojames.github.io/pyBathySfM/#step-2) | [Data Prep](https://geojames.github.io/pyBathySfM/#step-3) | [Running the Script](https://geojames.github.io/pyBathySfM/#step-4)
___

## SfM Collection and Processing <a name="step-1"></a>

#### Collect imagery for your site
I’m not going to go into the details here, I’m assuming you have knowledge/experience with imagery collection for SfM.

#### Some tips for bathymetric data collection:
-   I’ll reiterate, clear water is one of the most important factors and calmer wind conditions are ideal ( i.e. less surface waves).
-   Imagery should be collected at low-oblique angles (~20° off nadir) in convergent, overlapping patterns. Collecting at two heights/altitudes is also advantageous.
-   Using a polarizing filter, adjusted to reduce glare, will help the camera see through the water column to the bottom.
-   As much as possible, keep the sun behind the sensor.
-   Collect images at a time of day that minimizes shadows from the banks/riparian vegetation. Noon sun is not required and it is better to optimize for the site.
-   Accurate ground control points are critical for any SfM collection. RTK-GPS or a Total Station are the recommended survey methods.

#### Surveying
Accurate ground control points are critical for any SfM collection. RTK-GPS or a Total Station are the recommended survey methods.

-   Additional surveying requirements:
    -   In-water validation points:
        -   You should collect a number of validation points (100 is a fairly easy number to reach) at a variety of the depths present in the study area. These are for error checking/error statistics.
    -   Water’s edge points:
        -   You will need to collect a number of survey points on/at the water’s edge (for establishing the water surface elevations).
        -   You can do an extensive survey of the water’s edge (one point every 3-5 meters, plus extras at elevation transitions) or take a samples (one point every 20-30 meters, plus extras at elevation transitions).
            -   Either way you will need a water’s edge points for every part of the stream that where you want to map the bathymetry.
        -   With sample surveys, you will need to fill in by digitizing points on the water’s edge (more on that later).
        -   For streams with steep banks, this will be a challenge. However, surveying is the best option because digitizing the water level on step banks is notoriously difficult.

#### Processing

Go through your software’s processing chain to the point where you have a georeferenced, dense point cloud (any additional processing, orthophotos or DEMs, is optional).

1.  Export your point cloud (LAS files are good, but it’s up to you)
2.  Export the calculated/estimated camera positions and orientations
    -   The key here is that you need:
        -   The x,y,z coordinates of the cameras in the same reference system as our points
        -   The orientation angles of the cameras (pitch, roll, and yaw)
        -   Pitch should be 0° = Nadir (straight down), you may need to convert pitch angles if your software uses a different 0° reference.
        -   Roll in most software should be 0° = horizontal.
        -   Yaw should be a compass angle, 0° = North

___
[Top](https://geojames.github.io/pyBathySfM/#top) | [SfM Collection and Processing](https://geojames.github.io/pyBathySfM/#step-1) | [Point Cloud Processing](https://geojames.github.io/pyBathySfM/#step-2) | [Data Prep](https://geojames.github.io/pyBathySfM/#step-3) | [Running the Script](https://geojames.github.io/pyBathySfM/#step-4)
___

## Point Cloud Processing

The instructions in this section use the tools in CloudCompare (CC), so assume that the tool names are for CloudCompare. If you are using a different software, you’ll need to read through and translate the instructions/tools to your specific software. Info on how to use the specific tools in CC can be found on the CloudCompare Wiki ([http://www.cloudcompare.org/doc/wiki/index.php?title=Main_Page](http://www.cloudcompare.org/doc/wiki/index.php?title=Main_Page))

### Open your point cloud in CloudCompare

If you are prompted to apply coordinate offsets, you should accept the default X and Y, HOWEVER…set the Z offset to zero (it makes the calculations easier)

### Water Surface Processing

> **Water surface elevations are one of the primary sources of error!**
> Accurate water surfaces are critical to the success of this method

- There are several ways to get a spatially variable water surface:
	1. Digitizing from orthophotos
	2. Digitizing directly from the point cloud
	3. Surveying in the field
	- Regardless of the method, I would suggest a couple things to help:
		- The edge points should be extended out beyond the waters edge, by about 5 meters. I usually do this along cross-sections that are orthogonal to the channel. This will allow for a cleaner "cut" when extracting the below water points.
![Water's Edge Extensions](https://i.imgur.com/aexEJg4l.png)
		-	Use a statistical smoothing filter (upcoming paper) to smooth the edge data in the downstream direction.
		-   For study areas with “infinite” water surfaces (large lakes/ocean), I would suggest creating “synthetic” water’s edge points. These are just additional points with arbitrary X and Y coordinates and Z coordinates that set to the water surface elevation. These points should extend slightly beyond the study area. You can make the points in a text editor or Excel and import them.

### Create a Delaunay Mesh using your edge points
Import your surveyed or digitized water’s edge points (with the same offset as the point cloud)

Select the water surface points in the DB Tree add the Z values as a new scalar field (SF) attribute
- Tools…Projection…Export coordinates(s) to SF(s)…check ‘Z’…OK

Select Edit…Mesh…Delaunay 2.5D (XY plane)

-   Max edge length = your point spacing (2-5m, this limits wild interpolation)

The mesh should appear with colors representing elevation.
-   Check the mesh to make sure that there are not anomalous, abrupt changes in elevation along the edges (e.g. color changes).
-   If there are errors, you can edit the points and recreate the mesh.

#### Subsampling the point cloud

In order to limit the influence of noise in the point cloud, we need to subsample the point cloud to a uniform point spacing using a minimum elevation filter. The point spacing will need to be determined by you and the requirements/limitations of the site and your specific research question(s).

#### Delete extra scalar fields (*Optional*)
- Any extra scalar/attribute fields will be passed through the software, but if you have extra scalar fields in the point cloud that are unnecessary (e.g. Point Source ID or Scan Direction) you can delete them.
-   Select your SfM point cloud in the DB Tree
-   In the ‘Properties’ window, scroll down to the ‘Scalar Field’ section
       -   Use the drop-down to select them and press the ‘Delete Current Scalar Field’ button on the toolbar. Repeat for all the fields you do not need.

#### With the SfM point cloud selected, launch the ‘Rasterize tool’:

1.  Tools…Projection…Rasterize…
2.  In the Grid step box, enter the distance between points you want
3.  Active layer = Height grid values
4.  Projection, direction = Z
5.  Cell height = minimum height
6.  No checks in ‘interpolate SF’ or ‘resample input cloud’
7.  Empty Cells, Fill with = leave empty
8.  Click the ‘Update Grid’ button at the top

-   At the bottom, select the ‘Export tab’
    -   In the Export per-cell statistics section, I would recommend you check all the boxes. This will export the statistics for each point and may be useful in doing a final error analysis (e.g correction error vs. roughness (std. dev.))
-   Click the ‘Cloud button’ to export the resampled points as a new point cloud

The new cloud will appear in the DB Tree with the suffix *- Cloud.raster(#), where # is the spacing you specified.

### Transferring attributes to the subsampled point cloud

Select your new subsampled point cloud in the DB Tree...

#### Calculate the point-to-mesh distances (points to water surface = apparent depth)

Select both the subsampled point cloud and the water surface mesh

-   On the toolbar click ‘Calculate point/mesh distance’ OR Tools…Distances…Point/Mesh Dist
    -   Leave the options on the default
-   This will add a new scalar field to the point cloud called ‘C2M Signed Distance’ which represents the “depth” of the points below the water surface

#### Filtering the “underwater” points

Select the subsampled point cloud

Click the ‘Filter points by value’ tool on the toolbar

-   Change the Min value to the max depth for your area (round up to a whole number)
-   Change the Max value to zero
-   Click OK

The underwater points will be extracted to a new cloud with a suffix *- Cloud.extract

-   The edges outside the water surface mesh will likely be included in the extract
    -   Use the editing tools to delete these edges, cropped to just inside the water surface mesh.

#### Convert depth to water surface elevation

With the extracted point cloud selected, click the Calculator icon on the toolbar (“Add, subtract, multiply, or divide two scalar fields”). Here we will add the calculated depth back to the elevations to get the water surface elevation.

-   Set SF 1 = Coord. Z
-   Set operation = minus
-   Set Sf 2 = C2M Signed Distances
-   Uncheck ‘Update SF1 directly’
-   …OK

This subtracts the negative depth value from (adding it to) the SfM elevation value to give the water surface elevation for each point.

#### Rename scalar fields

In the properties window…change the active SF to ‘C2M Signed Distances’ and delete this field

-   Switch to Height grid value, and rename it to  `sfm_z`
    -   Edit…Scalar fields…Rename…[name]…OK
-   Switch to (SF# - SF#), and rename it to  `w_surf`

#### Export the point cloud

Select the edited point cloud

-   File…Save...
    -   Choose your file path
    -   File name = [your_file_name].csv
    -   Save as type = ‘ASCII cloud’

#### Optional/Recommended…Save all of the processing data

Click in the DB Tree and select all (Ctrl+A)

-   File…Save...
    -   Choose your file path
    -   File name = [name]
    -   Save as type = ‘CloudCompare entities (*.bin)’

___
[Top](https://geojames.github.io/pyBathySfM/#top) | [SfM Collection and Processing](https://geojames.github.io/pyBathySfM/#step-1) | [Point Cloud Processing](https://geojames.github.io/pyBathySfM/#step-2) | [Data Prep](https://geojames.github.io/pyBathySfM/#step-3) | [Running the Script](https://geojames.github.io/pyBathySfM/#step-4)
___

## Data Prep<a name="step-2"></a>

The inputs for the Python script are the edited point cloud, the camera location/orientations, and a file with camera sensor parameters. These all need to be comma-delimited (*.csv) files. You will need open all of the files in a text editor or Excel to double-check the header names (they are case-sensitive). There are examples included in the "Sample Data" folder in the Git repository.

#### Point cloud

-   The required columns/headers are  `x,y,sfm_z,w_surf`
-   Any extra columns like color [r,g,b] will be copied to the output point cloud without any modification.
-   If you followed the instructions above, you will have an extra column named ‘z’ in the point cloud, you can delete it if you like. It is the same as the ‘sfm_z’ column.

#### Camera positions

-   The required columns/headers are  `x,y,z,pitch,roll,yaw`
-   Camera/photo names can be included, but are not required
- This file should be 
	- From Agisoft Photoscan/Metashape: Export the Estimated Positions/Orientations from your project

#### Sensor properties

-   The required columns/headers are  `focal,sensor_x,sensor_y`
-   Focal is the focal length of the camera in millimeters
-   sensor_x & sensor_y are the physical sensor dimensions in millimeters
> You may have to dig around the internet to find these values if they are not listed with the technical specifications of the camera.

___
[Top](https://geojames.github.io/pyBathySfM/#top) | [SfM Collection and Processing](https://geojames.github.io/pyBathySfM/#step-1) | [Point Cloud Processing](https://geojames.github.io/pyBathySfM/#step-2) | [Data Prep](https://geojames.github.io/pyBathySfM/#step-3) | [Running the Script](https://geojames.github.io/pyBathySfM/#step-4)
___

## Running the script <a name='step-4'></a>

> A quick note, there is very little error checking/handling in this program. Most of the errors that are likely to occur will be because of incorrect header names in the CSV files, double check them against the sample data files. If the output is empty or looks weird, double check that the camera orientation values are in the correct directions and that the point cloud has correct water surface elevations.

Download the Git repository

-   Unzip and files into a convenient spot (near your exported data is always a good choice)

Launch the script

-   The preferred method is from the command line:
    -   Open your command line/terminal program of choice
    -   Change your directory to the place you unzipped the files
    -   Execute the script with:  `python py_BathySfM.py`
		- Do not run `py_BathySfM_gui.py` - this is the source code, but will not launch the GUI
![enter image description here](https://i.imgur.com/qCnhKBe.png)

1. **Input Point Cloud**: Your point cloud (i.e. the one you exported from CloudCompare).
	- Navigate to the directory and select the CSV file
2.  **Camera Position/Orientation**: CSV file with camera x,y,z,pitch,roll,yaw
3. **Camera Options**:
	- *Export Camera File*: This will output a Pickle (*.pkl) file in the Output Directory with the calculated camera fields of view (useful for reprocessing to avoid waiting during subsequent processing)
	- *Use Precalc'd Camera*: If, in a previous run, you exported the camera file above you can input the Pickle file here to bypass the camera FOV processing.
4. **Sensor File**: The sensor CSV file.
5. **Output File**: Forth is the output file.
	- I use something like  `siteName_CORRECTED.csv`
6. **Extra Output Options**:
	- *Small Angle Approx*: This will add an extra column to the output using the Small Angle Approximation for refraction correction `1.34 x Apparent_Depth` (Woodget et al., 2015)
	- *Extra Point Stat*: This will add an extra columns with additional statistics about the corrected depth (h).
		- Standard Dev., Median, Min/Max, a range of percentile ranks, and interquartile ranges
	- *Extra Camera Outputs*: Useful for error analysis (see below), this outputs a number of different files relating to the master camera visibility calculations, including raw outputs for corrected depths and elevations. This is meant as a debugging output, so only for the first batch of 10,000 points is output.
	- *Filtered depth values*: **IMPORTANT**
		- Add additional columns: `h_filt` and `corElev_avg_filt`
		- Recent trials/research (by me) have demonstrated that filtering some of the extreme values (high camera angles, points far from the camera) significantly improves accuracy. I'd suggest trying the defaults 35° off nadir and 100 meters, as a first cut, but feel free to play... 
7. The 'OK' button will start the…
	- Processing the camera's instantaneous fields of view first (This takes a while...)
	- Then processing points in batches.
	- The progress bars are for show, they don't generally work...in your command line there is feedback on the progress

When the script is finished, it will report how many points were processed and approximately how long it took. You can exit the window or hit the 'Cancel' button.

#### Examining the results

Open the output file in CloudCompare

-   The ‘Open ASCII File’ import window will open
-   The header names will be in the first line, note the column number of ‘corElev_avg’
-   At the bottom, separator is comma(,) and skip line = 1 and check ‘extract scalar fields from first line’
    -   CC will try to make the third column ‘Z’. With the drop downs, reassign ‘Z’ to the ‘corElev_avg’ column
-   If you have extra columns…
    -   You can set RGB columns, if they are not automatically interpreted
    -   Make sure none of the columns are interpreted as  `Nx,Ny,Nz`. These are the “normal” directions (the surface orientation of the point). Change these columns to “Scalar”. (That is, unless you do have normals stored as part of the point cloud, but you will most likely not have these fields).

The cloud should be shown in the viewer. You can change the scalar field to ‘h,avg’ to display depth, or change the colors to RGB (if you have them in your data).

#### And that’s your data corrected for refraction. I would suggest doing some error analysis before going too far, a discussion of error is a topic for another tutorial, however…

## Basic Error Analysis
You will need independently collected elevation data for the bed of your stream/waterbody
1. Load your surveyed validation points into CloudCompare
2. Following a similar workflow as the [Point Cloud Processing](https://geojames.github.io/pyBathySfM/#step-2) above.
	- Add a Point to Mesh distance to the the water surface (name the scalar `w_surf`)
3. Add a scalar field for the uncorrected elevation from the SfM dataset to your validation points
	- Can be done in the raster world through a GIS or some other nearest neighbor attribute transfer
	- Make sure you name the attribute `sfm_z` 
4. You can then run the enhanced validation points through the bathymetric correction like you would a full point cloud.
5. Tada! you have directly comparable error stats.  

##### References
- Dietrich JT. 2017. Bathymetric Structure-from-Motion: extracting shallow stream bathymetry from multi-view stereo photogrammetry. Earth Surface Processes and Landforms **42** : 355–364. DOI: [10.1002/esp.4060](https://doi.org/10.1002/esp.4060)
- Woodget AS, Carbonneau PE, Visser F, Maddock IP. 2015. Quantifying submerged fluvial topography using hyperspatial resolution UAS imagery and structure from motion photogrammetry. Earth Surface Processes and Landforms **40** : 47–64. DOI: [10.1002/esp.3613](https://doi.org/10.1002/esp.3613)

___
[Top](https://geojames.github.io/pyBathySfM/#top) | [SfM Collection and Processing](https://geojames.github.io/pyBathySfM/#step-1) | [Point Cloud Processing](https://geojames.github.io/pyBathySfM/#step-2) | [Data Prep](https://geojames.github.io/pyBathySfM/#step-3) | [Running the Script](https://geojames.github.io/pyBathySfM/#step-4)
___
