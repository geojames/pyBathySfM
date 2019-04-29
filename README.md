# pyBathySfM
Updates to the original [py_sfm_depth](https://github.com/geojames/py_sfm_depth) with a new fancy name and a shiny new GUI interface

## Tutorial - [https://geojames.github.io/pyBathySfM/](https://geojames.github.io/pyBathySfM/)

### Updates from the original `py_sfm_depth`
1. Shiny new GUI interface (QT)
2. More options
   + Angle and distance filters
   + Debugging outputs (raw processing outputs)
   + added small angle refraction calculation option
   
### This script is SfM software indifferent (Agisoft, Pix4D, etc)
- you just need to be able to export a CSV point cloud and get camera parameters (x,y,z,pitch,roll,yaw)
- see the tutorial for more info.

### Running the script
Download and unzip the Github Repository to a 
In your command line program of choice (i.e. Windows command prompt, Anaconda prompt)
- Change the direcotry `cd c:\...` to the unzipped repository
- Run the script with `python pyBathySfM.py`
   - do not run `pyBathySfM_gui.py` this is the source code, but it will not display the GUI

#### Requirements
- Python v3.x (written in 3.6 & 3.7)
- Required Libraries: PyQT5, numpy, pandas, sympy, matplotlib

** I use [Anaconda Python](https://www.anaconda.com/distribution/) that has all the required libraries

Licence: MIT

© 2019, James T. Dietrich, Ph.D.
