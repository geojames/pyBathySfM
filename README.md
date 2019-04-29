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
In your command line program of choice (i.e. Windows command prompt, Anaconda prompt

#### Requirements
- Python v3.x (written in 3.6 & 3.7)
- Required Libraries: PyQT5, numpy, pandas, sympy, matplotlib

** I use [Anaconda Python](https://www.anaconda.com/distribution/) that has all the required libraries

Licence: MIT

© 2019, James T. Dietrich, Ph.D.
