#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------------------------------------
__author__ = 'James T. Dietrich'
__contact__ = 'james.dietrich@uni.edu'
__copyright__ = '(c) James Dietrich 2019'
__license__ = 'MIT'
__date__ = '26 JUNE 2019'
__version__ = '4.0'
__status__ = "initial release"
__url__ = "https://github.com/geojames/pyBathySfM"

"""
Name:           py_BathySfM_gui.py
Compatibility:  Python 3.7
Description:    This program performs a per-camera refration correction on a 
                Structure-from-Motion point cloud. Additional documnetation,
                sample data, and a tutorial are availible from the GitHub
                address below.

URL:            https://github.com/geojames/pyBathySfM

Requires:       PyQT5, numpy, pandas, sympy, matplotlib

Dev ToDo:       1) speed up camera geometry calculations

AUTHOR:         James T. Dietrich
ORGANIZATION:   University of Northern Iowa
Contact:        james.dietrich@uni.edu
Copyright:      (c) James Dietrich 2019

Licence:        MIT
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to 
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
of the Software, and to permit persons to whom the Software is furnished to do 
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.
"""
# TO RUN -  *Do not run this code*...from the command line (or editor) run  
#           py_bathySfM.py to load the GUI
# Main Code at bottom in - def main_prog()
#------------------------------------------------------------------------------

#pyqt import
from PyQt5 import QtCore, QtGui, QtWidgets

# other imports
import os
import sys
import numpy as np
import pandas as pd
import sympy.geometry as spg
import matplotlib.path as mplPath
from datetime import datetime

# MAIN PROGRAM HELPER FUNCTIONS (run on OK button)

def footprints(cam, sensor, base_elev, gui):
    """
    This function calculates the instantaneous field of view (IFOV) for 
    the camera(s) that are passed.\n
    Vars:\n
    \t cam = pandas dataframe (n x ~6, fields: x,y,z,yaw,pitch,roll)\n
    \t sensor = pandas dataframe (1 x 3, fields: focal, sensor_x, sensor_y):
    \t focal length (mm), sensor x dim (mm), sensor y dim (mm)\n
    \t base_elev = average elevation of your site (meters, or in the same
    \t measure as your coordinates)\n
    Creates approx. coordinates for sensor
    corners (north-oriented and zero pitch) at the camera's x,y,z. Rotates
    the sensor coords in 3D space to the camera's pitch and yaw angles (roll
    angles are ignored for now) and projects corner rays through the camera 
    x,y,z to a approx ground plane. The intersection of the rays with the
    ground are the corners of the photo footprint.\n
    *** Photos that have picth angles that cause the horizon to be visable will
    cause the UL and UR path coordniates to wrong. These cameras are 
    disreguarded and the footprint will be set to NaN in the output.***\n 
    RETURNS: footprints = Pandas dataframe (n x 1) of Matplotlib Path objects()
    """
    
    #qt progress bar
    gui.top_progBar.setValue(0)
    gui.top_progBar.setMaximum(cam.shape[0])
    
    # Setup DF to house camera footprint polygons
    footprints = pd.DataFrame(np.zeros((cam.shape[0],1)), columns=['fov'])
    
    # debug - blank 3d array for inter_points
#        itp_f = '//thor.ad.uni.edu/users/jdietric/Documents/Python Scripts/py_sfm_depth/WhiteR_2016/itp.npy'
#        itp = np.zeros((cam.shape[0],4,2))
    
    # convert sensor dimensions to meters, divide x/y for corner coord calc
    f = sensor.focal[0] * 0.001
    sx = sensor.sensor_x[0] / 2 * 0.001
    sy = sensor.sensor_y[0] / 2 * 0.001

    # calculate the critical pitch (in degrees) where the horizon will be 
    #   visible with the horizon viable, the ray projections go backward 
    #   and produce erroneous IFOV polygons (90 - 0.5*vert_fov)
    crit_pitch = 90 - np.rad2deg(np.arctan(sy / f))
    
    # User Feedback
    print("Proccesing Camera IFOVs (%i total)..." %(cam.shape[0]))
    sys.stdout.flush()
     
    # for each camera...
    for idx, row in cam.iterrows():
        
        # check is the camera pitch is over the critical value
        if row.pitch < crit_pitch:
            
            # sensor corners (UR,LR,LL,UL), north-oriented and zero pitch
            corners = np.array([[row.x+sx,row.y-f,row.z+sy],
                               [row.x+sx,row.y-f,row.z-sy],
                               [row.x-sx,row.y-f,row.z-sy],
                               [row.x-sx,row.y-f,row.z+sy]])
            
            # offset corner points by cam x,y,z for rotation
            cam_pt = np.atleast_2d(np.array([row.x, row.y, row.z]))
            corner_p = corners - cam_pt
    
            # get pitch and yaw from the camera, convert to radians
            pitch = np.deg2rad(90.0-row.pitch)
            roll = np.deg2rad(row.roll)
            yaw = np.deg2rad(row.yaw)
            
            # setup picth rotation matrix (r_x) and yaw rotation matrix (r_z)
            r_x = np.matrix([[1.0,0.0,0.0],
                             [0.0,np.cos(pitch),-1*np.sin(pitch)],
                             [0.0,np.sin(pitch),np.cos(pitch)]])
                             
            r_y = np.matrix([[np.cos(roll),0.0,np.sin(roll)],
                             [0.0,1.0,0.0],
                             [-1*np.sin(roll),0.0,np.cos(roll)]])
            
            r_z =  np.matrix([[np.cos(yaw),-1*np.sin(yaw),0],
                              [np.sin(yaw),np.cos(yaw),0],
                              [0,0,1]])
            
            # rotate corner_p by r_x, then r_z, add back cam x,y,z offsets
            # produces corner coords rotated for pitch and yaw
            p_pr = np.matmul(np.matmul(corner_p, r_x),r_y)            
            p_out = np.matmul(p_pr, r_z) + cam_pt
            
            # GEOMETRY
            # Set Sympy 3D point for the camera and a 3D plane for intersection
            cam_sp = spg.Point3D(row.x, row.y, row.z)
            plane = spg.Plane(spg.Point3D(row.x, row.y, base_elev),
                                      normal_vector=(0,0,1))
            
            # blank array for footprint intersection coords
            inter_points = np.zeros((corners.shape[0],2))
            
            # for each sensor corner point
            idx_b = 0
            for pt in np.asarray(p_out):
                
                # create a Sympy 3D point and create a Sympy 3D ray from 
                #   corner point through camera point
                pt_sp = spg.Point3D(pt[0],pt[1],pt[2])
                ray = spg.Ray3D(pt_sp,cam_sp)
                
                # calculate the intersection of the ray with the plane                
                inter_pt = plane.intersection(ray)
                
                # Extract out the X,Y coords of the intersection point
                #   ground intersect points will be in this order (LL,UL,UR,LR)
                inter_points[idx_b,0] = inter_pt[0].x.evalf()
                inter_points[idx_b,1] = inter_pt[0].y.evalf()
                
                idx_b += 1
        
        # if crit_pitch is exceeded set inter_points to NaN
        else:
            inter_points = np.full((4,2),np.nan)
        
        # append inter_points to footprints as a matplotlib path object
        footprints.fov[idx] = mplPath.Path(inter_points)
        
        #debug - save inter_points
#            itp[idx,:,:] = inter_points
        
        # User feedback and progress bar
        if (idx+1) % 10 == 0:
            print("%i cameras processed..." %(idx+1))
            gui.top_progBar.setValue(idx)
            gui.topProg_Lbl.setText("Calculating Camera Footprints - " + str(idx+1))
            #sys.stdout.flush()
            
    #debug - save inter_points
    #np.save(itp_f,itp)
    
    return footprints
# END - footprints
    
def visibility(cam, footprints, targets):
    """    
    This function tests is the target points (x,y only) are "visible" (i.e.
    within the photo footprints) and calculates the "r" angle for the refraction 
    correction\n
    Vars:\n
    \t cam = Pandas dataframe (n x ~6, fields: x,y,z,yaw,pitch,roll)\n
    \t footprints = Pandas dataframe (n x 1) of Matplotlib Path objects\n
    \t targets = Pandas dataframe (n x ~3, fields: x,y,sfm_z...)\n
    
    RETURNS: r_filt = numpy array (n_points x n_cams) of filtered "r" angles.\n
    Points that are not visible to a camera will have a NaN "r" angle. 
    """
    
    # Setup boolean array for visibility
    vis = np.zeros((targets.shape[0],cam.shape[0])) 
    
    # for each path object in footprints, check is the points in targets are
    #   within the path polygon. path.contains_points returns boolean.
    #   the results are accumulated in the vis array.
    for idx in range(footprints.shape[0]):
        path = footprints.fov[idx]
        vis[:,idx] = path.contains_points(np.array([targets.x.values, targets.y.values]).T)
    
    # calculate the coord. deltas between the cameras and the target
    dx = np.atleast_2d(cam.x.values) - np.atleast_2d(targets.x.values).T
    dy = np.atleast_2d(cam.y.values) - np.atleast_2d(targets.y.values).T
    dz = np.atleast_2d(cam.z.values) - np.atleast_2d(targets.sfm_z).T
    
    # calc xy distance (d)
    d = np.sqrt((dx)**2+(dy)**2)
    
    # calc inclination angle (r) from targets to cams
    r = np.rad2deg(np.arctan(d/dz))
    
    r_filt = r * vis
    r_filt[r_filt == 0] = np.nan
    
    # If camera quality stats exist in the camera table, broadcast the camera 
    #   quality values into into a point x cams table and filter based on the 
    #   visability matrix calc'd above
    if 'quality' in cam.columns:
        qual = cam.quality.values
        qual_mat = np.repeat(np.atleast_2d(qual),targets.shape[0],axis=0)
        qual_vis = qual_mat * vis
        qual_vis[qual_vis==0] = np.nan
    else:      
        qual_vis = np.zeros((targets.shape[0],cam.shape[0]))
        qual_vis[:] = np.nan
        
    return r_filt, d, qual_vis

def correction(r, target, r_index,extras):
    """Performs the per camera refraction correction on a target point.
    Refer to the documentation for the specifics.
    extras = [smAng, stats, camStats, weight]"""
    
    # convert r array to radians for trig calculations
    ang_r = np.radians(r)

    # calculate the refraction angle i 
    ang_i = np.arcsin(1.0/r_index * np.sin(ang_r))
    
    # calculate the apparent depth from the water surface elev. and the 
    #   target SfM elevation
    target['h_a'] = target.w_surf - target.sfm_z

    # calculate the distance from the point to the air/water interface
    x_dist = np.array([target.h_a.values]).T * np.tan(ang_r)
    
    # calculate the corrected (actual) depth
    h =  x_dist / np.tan(ang_i)
   
    # subtract the corrected depth from the water surface elevation to get the
    #   corrected elevation
    cor_elev = np.array([target.w_surf]).T - h
    
    # append the mean values for the actual depth and corrected elevation to
    #   the target data frame   
    target['h_avg'] = np.nanmean(h, axis = 1)
    target['corElev_avg'] = np.nanmean(cor_elev, axis = 1)
    
    if extras[0]:
    # calc small angle approximation
        target['smAng_h'] = target['h_a'] * 1.34
        target['smAng_elev'] = target.w_surf - target['smAng_h']       
       
    if extras[1]:
    # some extra statistics for playing around (option from GUI)
        target['h_std'] = np.nanstd(h, axis = 1)
        target['h_med'] = np.nanmedian(h, axis = 1)
        target['h_min'] = np.nanmin(h, axis = 1)
        target['h_max'] = np.nanmax(h, axis = 1)
        target['h_per15'] = np.nanpercentile(h, 15,axis = 1)
        target['h_per25'] = np.nanpercentile(h, 25,axis = 1)
        target['h_per55'] = np.nanpercentile(h, 55,axis = 1)
        target['h_per60'] = np.nanpercentile(h, 60,axis = 1)
        target['h_per75'] = np.nanpercentile(h, 75,axis = 1)
        target['h_per80'] = np.nanpercentile(h, 80,axis = 1)
        target['h_per90'] = np.nanpercentile(h, 90,axis = 1)
        target['h_per95'] = np.nanpercentile(h, 95,axis = 1)
        target['h_iqr'] = target['h_per75'] - target['h_per25']
        target['h_lif'] = target['h_per25'] - (1.5 * target['h_iqr'])
        target['h_uif'] = target['h_per75'] + (1.5 * target['h_iqr'])
        target['h_lof'] = target['h_per25'] - (3 * target['h_iqr'])
        target['h_uof'] = target['h_per75'] + (3 * target['h_iqr'])
#        l_whisk = np.repeat(np.atleast_2d(target['h_lif']).T,188,axis=1)
#        u_whisk = np.repeat(np.atleast_2d(target['h_uif']).T,188,axis=1)
#        target['u_whisk'] = np.nanmax(np.ma.masked_less_equal(h,u_whisk).data,axis=1)
#        target['l_whisk'] = np.nanmin(np.ma.masked_greater_equal(h,l_whisk).data,axis=1)
#        mild_out = np.zeros_like(h)
#        ext_out = np.zeros_like(h)
#        mild_out[h < np.atleast_2d(target['h_lif']).T] = 1
#        mild_out[h > np.atleast_2d(target['h_uif']).T] = 1   
#        ext_out[h < np.atleast_2d(target['h_lof']).T] = 1
#        ext_out[h > np.atleast_2d(target['h_uof']).T] = 1   
#        target['h_mildout'] = np.nansum(mild_out, axis = 1)
#        target['h_extout'] = np.nansum(ext_out, axis = 1)
    
    # return the target dataframe
    return target, ang_r, x_dist, h, cor_elev
# END def correction
    
def pointFilter(tar,h,r,d,extras):
    # tar = target out table from F(correction)
    # h = full corrected depth table from F(correction)
    # r = camera angles from F(visibility)
    # d = camera distances from F(visibility)
    
    max_ang = extras[4]
    max_dist = extras[5]
    
    h_filt = h
    h_filt[r > max_ang] = np.nan
    h_filt[d > max_dist] = np.nan
    
    tar['h_filt'] = np.nanmean(h_filt, axis = 1)
    
    cor_elev_filt = np.array([tar.w_surf]).T - h_filt
    tar['corElev_avg_filt'] = np.nanmean(cor_elev_filt, axis = 1)
    
    if extras[1]:
    # some extra statistics for playing around (option from GUI)
        tar['h_filt_std'] = np.nanstd(h_filt, axis = 1)
        tar['h_filt_med'] = np.nanmedian(h_filt, axis = 1)
        tar['h_filt_min'] = np.nanmin(h_filt, axis = 1)
        tar['h_filt_max'] = np.nanmax(h_filt, axis = 1)
        tar['h_filt_per15'] = np.nanpercentile(h_filt, 15,axis = 1)
        tar['h_filt_per25'] = np.nanpercentile(h_filt, 25,axis = 1)
        tar['h_filt_per55'] = np.nanpercentile(h_filt, 55,axis = 1)
        tar['h_filt_per60'] = np.nanpercentile(h_filt, 60,axis = 1)
        tar['h_filt_per75'] = np.nanpercentile(h_filt, 75,axis = 1)
        tar['h_filt_per80'] = np.nanpercentile(h_filt, 80,axis = 1)
        tar['h_filt_per90'] = np.nanpercentile(h_filt, 90,axis = 1)
        tar['h_filt_per95'] = np.nanpercentile(h_filt, 95,axis = 1)
        tar['h_filt_iqr'] = tar['h_filt_per75'] - tar['h_filt_per25']
        tar['h_filt_lif'] = tar['h_filt_per25'] - (1.5 * tar['h_filt_iqr'])
        tar['h_filt_uif'] = tar['h_filt_per75'] + (1.5 * tar['h_filt_iqr'])
        tar['h_filt_lof'] = tar['h_filt_per25'] - (3 * tar['h_filt_iqr'])
        tar['h_filt_uof'] = tar['h_filt_per75'] + (3 * tar['h_filt_iqr'])
    
    return tar

def timer(length,start_t):
    """timer function to calculate the running time"""
    
    num_proc = sum(length)    
    
    # time since processing started
    t_step = datetime.now() - start_t

    if t_step.total_seconds() <= 60:
        print("-> Finished %i points in %0.2f secs" %(num_proc,t_step.total_seconds()))
    else:
        ts = t_step.total_seconds() / 60    
        print("-> Finished %i points in %0.2f mins" %(num_proc,ts))
  
#END def timer
        
# END - MAIN PROGRAM HELPER FUNCTIONS
        
# --------------------------------------------
# PyQT GUI Builder Code
# pyqt base class
#   Mostly autogenerated from QtDesigner
class Ui_bathySfM_gui(object):
    def setupUi(self, bathySfM_gui):
        bathySfM_gui.setObjectName("BathySfM_gui")
        bathySfM_gui.resize(517, 629)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("icon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        bathySfM_gui.setWindowIcon(icon)
        self.buttonBox = QtWidgets.QDialogButtonBox(bathySfM_gui)
        self.buttonBox.setGeometry(QtCore.QRect(20, 443, 481, 32))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.ptCloud_btn = QtWidgets.QToolButton(bathySfM_gui)
        self.ptCloud_btn.setGeometry(QtCore.QRect(460, 31, 31, 22))
        self.ptCloud_btn.setObjectName("ptCloud_btn")
        self.ptCloud_txt = QtWidgets.QLineEdit(bathySfM_gui)
        self.ptCloud_txt.setGeometry(QtCore.QRect(20, 31, 431, 22))
        self.ptCloud_txt.setObjectName("ptCloud_txt")
        self.ptCloud_lbl = QtWidgets.QLabel(bathySfM_gui)
        self.ptCloud_lbl.setGeometry(QtCore.QRect(20, 10, 97, 22))
        self.ptCloud_lbl.setObjectName("ptCloud_lbl")
        self.cam_btn = QtWidgets.QToolButton(bathySfM_gui)
        self.cam_btn.setGeometry(QtCore.QRect(460, 80, 31, 22))
        self.cam_btn.setObjectName("cam_btn")
        self.cam_txt = QtWidgets.QLineEdit(bathySfM_gui)
        self.cam_txt.setGeometry(QtCore.QRect(20, 80, 431, 22))
        self.cam_txt.setObjectName("cam_txt")
        self.cam_lbl = QtWidgets.QLabel(bathySfM_gui)
        self.cam_lbl.setGeometry(QtCore.QRect(20, 60, 161, 16))
        self.cam_lbl.setObjectName("cam_lbl")
        self.sensor_lbl = QtWidgets.QLabel(bathySfM_gui)
        self.sensor_lbl.setGeometry(QtCore.QRect(20, 210, 71, 16))
        self.sensor_lbl.setObjectName("sensor_lbl")
        self.sensor_txt = QtWidgets.QLineEdit(bathySfM_gui)
        self.sensor_txt.setGeometry(QtCore.QRect(20, 230, 431, 22))
        self.sensor_txt.setObjectName("sensor_txt")
        self.sensor_btn = QtWidgets.QToolButton(bathySfM_gui)
        self.sensor_btn.setGeometry(QtCore.QRect(460, 230, 31, 22))
        self.sensor_btn.setObjectName("sensor_btn")
        self.out_btn = QtWidgets.QToolButton(bathySfM_gui)
        self.out_btn.setGeometry(QtCore.QRect(460, 280, 31, 22))
        self.out_btn.setObjectName("out_btn")
        self.out_txt = QtWidgets.QLineEdit(bathySfM_gui)
        self.out_txt.setGeometry(QtCore.QRect(20, 280, 431, 22))
        self.out_txt.setObjectName("out_txt")
        self.out_lbl = QtWidgets.QLabel(bathySfM_gui)
        self.out_lbl.setGeometry(QtCore.QRect(20, 260, 71, 16))
        self.out_lbl.setObjectName("out_lbl")
        self.top_progBar = QtWidgets.QProgressBar(bathySfM_gui)
        self.top_progBar.setGeometry(QtCore.QRect(20, 490, 481, 23))
        self.top_progBar.setProperty("value", 0)
        self.top_progBar.setObjectName("top_progBar")
        self.topProg_Lbl = QtWidgets.QLabel(bathySfM_gui)
        self.topProg_Lbl.setGeometry(QtCore.QRect(20, 520, 451, 16))
        self.topProg_Lbl.setObjectName("topProg_Lbl")
        self.bot_progBar = QtWidgets.QProgressBar(bathySfM_gui)
        self.bot_progBar.setGeometry(QtCore.QRect(20, 550, 481, 23))
        self.bot_progBar.setProperty("value", 0)
        self.bot_progBar.setObjectName("bot_progBar")
        self.botProg_lbl = QtWidgets.QLabel(bathySfM_gui)
        self.botProg_lbl.setGeometry(QtCore.QRect(20, 580, 451, 16))
        self.botProg_lbl.setObjectName("botProg_lbl")
        self.groupBox = QtWidgets.QGroupBox(bathySfM_gui)
        self.groupBox.setGeometry(QtCore.QRect(30, 110, 481, 91))
        self.groupBox.setObjectName("groupBox")
        self.precalcCam_box = QtWidgets.QCheckBox(self.groupBox)
        self.precalcCam_box.setGeometry(QtCore.QRect(20, 60, 161, 21))
        self.precalcCam_box.setObjectName("precalcCam_box")
        self.exportCam_box = QtWidgets.QCheckBox(self.groupBox)
        self.exportCam_box.setGeometry(QtCore.QRect(20, 30, 441, 21))
        self.exportCam_box.setObjectName("exportCam_box")
        self.camPickle = QtWidgets.QLineEdit(self.groupBox)
        self.camPickle.setEnabled(False)
        self.camPickle.setGeometry(QtCore.QRect(190, 60, 231, 22))
        self.camPickle.setObjectName("camPickle")
        self.pickle_btn = QtWidgets.QToolButton(self.groupBox)
        self.pickle_btn.setEnabled(False)
        self.pickle_btn.setGeometry(QtCore.QRect(430, 60, 31, 22))
        self.pickle_btn.setObjectName("pickle_btn")
        self.groupBox_2 = QtWidgets.QGroupBox(bathySfM_gui)
        self.groupBox_2.setGeometry(QtCore.QRect(20, 310, 481, 121))
        self.groupBox_2.setObjectName("groupBox_2")
        self.check_smAng = QtWidgets.QCheckBox(self.groupBox_2)
        self.check_smAng.setGeometry(QtCore.QRect(130, 50, 141, 17))
        self.check_smAng.setObjectName("check_smAng")
        self.check_stats = QtWidgets.QCheckBox(self.groupBox_2)
        self.check_stats.setGeometry(QtCore.QRect(130, 20, 121, 17))
        self.check_stats.setObjectName("check_stats")
        self.check_camStats = QtWidgets.QCheckBox(self.groupBox_2)
        self.check_camStats.setGeometry(QtCore.QRect(130, 80, 151, 17))
        self.check_camStats.setObjectName("check_camStats")
        self.check_filter = QtWidgets.QCheckBox(self.groupBox_2)
        self.check_filter.setGeometry(QtCore.QRect(314, 30, 151, 16))
        self.check_filter.setObjectName("check_filter")
        self.spinBox_angle = QtWidgets.QSpinBox(self.groupBox_2)
        self.spinBox_angle.setEnabled(False)
        self.spinBox_angle.setGeometry(QtCore.QRect(320, 56, 42, 20))
        self.spinBox_angle.setMaximum(90)
        self.spinBox_angle.setProperty("value", 35)
        self.spinBox_angle.setObjectName("spinBox_angle")
        self.spinBox_dist = QtWidgets.QSpinBox(self.groupBox_2)
        self.spinBox_dist.setEnabled(False)
        self.spinBox_dist.setGeometry(QtCore.QRect(410, 56, 42, 20))
        self.spinBox_dist.setMaximum(2000)
        self.spinBox_dist.setProperty("value", 100)
        self.spinBox_dist.setObjectName("spinBox_dist")
        self.label = QtWidgets.QLabel(self.groupBox_2)
        self.label.setGeometry(QtCore.QRect(290, 76, 101, 21))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.groupBox_2)
        self.label_2.setGeometry(QtCore.QRect(400, 76, 81, 21))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.groupBox_2)
        self.label_3.setGeometry(QtCore.QRect(150, 95, 81, 21))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.spinBox_rf = QtWidgets.QDoubleSpinBox(self.groupBox_2)
        self.spinBox_rf.setGeometry(QtCore.QRect(30, 70, 71, 22))
        self.spinBox_rf.setDecimals(3)
        self.spinBox_rf.setMaximum(3.0)
        self.spinBox_rf.setSingleStep(0.001)
        self.spinBox_rf.setProperty("value", 1.337)
        self.spinBox_rf.setObjectName("spinBox_rf")
        self.label_4 = QtWidgets.QLabel(self.groupBox_2)
        self.label_4.setGeometry(QtCore.QRect(34, 30, 61, 20))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.groupBox_2)
        self.label_5.setGeometry(QtCore.QRect(47, 50, 31, 16))
        self.label_5.setObjectName("label_5")
        self.txt_copyright = QtWidgets.QLabel(bathySfM_gui)
        self.txt_copyright.setGeometry(QtCore.QRect(310, 600, 191, 16))
        self.txt_copyright.setObjectName("txt_copyright")
        self.txt_licence = QtWidgets.QLabel(bathySfM_gui)
        self.txt_licence.setGeometry(QtCore.QRect(20, 600, 71, 16))
        self.txt_licence.setObjectName("txt_licence")
        self.groupBox.raise_()
        self.groupBox_2.raise_()
        self.buttonBox.raise_()
        self.ptCloud_btn.raise_()
        self.ptCloud_txt.raise_()
        self.ptCloud_lbl.raise_()
        self.cam_btn.raise_()
        self.cam_txt.raise_()
        self.cam_lbl.raise_()
        self.sensor_lbl.raise_()
        self.sensor_txt.raise_()
        self.sensor_btn.raise_()
        self.out_btn.raise_()
        self.out_txt.raise_()
        self.out_lbl.raise_()
        self.top_progBar.raise_()
        self.topProg_Lbl.raise_()
        self.bot_progBar.raise_()
        self.botProg_lbl.raise_()
        self.txt_copyright.raise_()
        self.txt_licence.raise_()
        
        # connections (textbox, check box actions)

        self.ptCloud_btn.clicked.connect(self.ptCloud_picker)
        self.cam_btn.clicked.connect(self.cam_picker)
        self.pickle_btn.clicked.connect(self.pickle_picker)
        self.sensor_btn.clicked.connect(self.sensor_picker)
        self.out_btn.clicked.connect(self.outfile_picker)
        self.precalcCam_box.stateChanged.connect(self.precalc_change)
        self.check_filter.stateChanged.connect(self.filter_change)
        
        # blank global working directory variable
        global working_dir
        working_dir = '.'

        self.retranslateUi(bathySfM_gui)
        self.buttonBox.accepted.connect(self.main_prog)
        self.buttonBox.rejected.connect(bathySfM_gui.reject)
        QtCore.QMetaObject.connectSlotsByName(bathySfM_gui)
        bathySfM_gui.setTabOrder(self.ptCloud_txt, self.ptCloud_btn)
        bathySfM_gui.setTabOrder(self.ptCloud_btn, self.cam_txt)
        bathySfM_gui.setTabOrder(self.cam_txt, self.cam_btn)
        bathySfM_gui.setTabOrder(self.cam_btn, self.exportCam_box)
        bathySfM_gui.setTabOrder(self.exportCam_box, self.precalcCam_box)
        bathySfM_gui.setTabOrder(self.precalcCam_box, self.camPickle)
        bathySfM_gui.setTabOrder(self.camPickle, self.pickle_btn)
        bathySfM_gui.setTabOrder(self.pickle_btn, self.sensor_txt)
        bathySfM_gui.setTabOrder(self.sensor_txt, self.sensor_btn)
        bathySfM_gui.setTabOrder(self.sensor_btn, self.out_txt)
        bathySfM_gui.setTabOrder(self.out_txt, self.out_btn)
        bathySfM_gui.setTabOrder(self.out_btn, self.check_smAng)
        bathySfM_gui.setTabOrder(self.check_smAng, self.check_stats)
        bathySfM_gui.setTabOrder(self.check_stats, self.check_camStats)
        bathySfM_gui.setTabOrder(self.check_camStats, self.check_filter)

    def retranslateUi(self, bathySfM_gui):
        _translate = QtCore.QCoreApplication.translate
        bathySfM_gui.setWindowTitle(_translate("bathySfM_gui", "py_BathySfM"))
        self.ptCloud_btn.setText(_translate("bathySfM_gui", "..."))
        self.ptCloud_lbl.setText(_translate("bathySfM_gui", "Input Point Cloud"))
        self.cam_btn.setText(_translate("bathySfM_gui", "..."))
        self.cam_lbl.setText(_translate("bathySfM_gui", "Camera Position/Orientation File"))
        self.sensor_lbl.setText(_translate("bathySfM_gui", "Sensor File"))
        self.sensor_btn.setText(_translate("bathySfM_gui", "..."))
        self.out_btn.setText(_translate("bathySfM_gui", "..."))
        self.out_lbl.setText(_translate("bathySfM_gui", "Output File"))
        self.topProg_Lbl.setText(_translate("bathySfM_gui", "Choose Files..."))
        self.botProg_lbl.setText(_translate("bathySfM_gui", "Overall Progress"))
        self.groupBox.setTitle(_translate("bathySfM_gui", "Camera Options (Advanced Use)"))
        self.precalcCam_box.setText(_translate("bathySfM_gui", "Use Precalc\'d Cameras"))
        self.exportCam_box.setText(_translate("bathySfM_gui", "Export Camera File (output will be in the Output File folder)"))
        self.camPickle.setText(_translate("bathySfM_gui", "Camera Pickle File (*.pkl)"))
        self.pickle_btn.setText(_translate("bathySfM_gui", "..."))
        self.groupBox_2.setTitle(_translate("bathySfM_gui", "Extra Options"))
        self.check_smAng.setText(_translate("bathySfM_gui", "Small Angle Approx."))
        self.check_stats.setText(_translate("bathySfM_gui", "Extra Point Stats"))
        self.check_camStats.setText(_translate("bathySfM_gui", "Extra Camera Outputs "))
        self.check_filter.setText(_translate("bathySfM_gui", "Filtered depth values"))
        self.label.setText(_translate("bathySfM_gui", "Max Angle (Deg)"))
        self.label_2.setText(_translate("bathySfM_gui", "Max Dist (m)"))
        self.label_3.setText(_translate("bathySfM_gui", "First 10000 pts"))
        self.label_4.setText(_translate("bathySfM_gui", "Refractive"))
        self.label_5.setText(_translate("bathySfM_gui", "Index"))
        self.txt_copyright.setText(_translate("bathySfM_gui", "© James T. Dietrich,Ph.D. 2019"))
        self.txt_licence.setText(_translate("bathySfM_gui", "MIT Licence"))

# file picker connection functions
    
    # point cloud file picker    
    def ptCloud_picker(self):
        global working_dir
        [filename,filt] = QtWidgets.QFileDialog.getOpenFileName(None,'Open Point Cloud', working_dir, 'Comma Separated File (*.csv)')
        if filename:
            working_dir = os.path.dirname(filename)
            self.ptCloud_txt.setText(filename)
    
    # camera file picker  
    def cam_picker(self):
        global working_dir
        [filename,filt] = QtWidgets.QFileDialog.getOpenFileName(None, 'Open Camera File', working_dir, 'Comma Separated File (*.csv)')
        if filename:
            working_dir = os.path.dirname(filename)
            self.cam_txt.setText(filename)
            
    # pre calc'd camera file picker  
    def pickle_picker (self):
        global working_dir
        [filename,filt] = QtWidgets.QFileDialog.getOpenFileName(None, 'Open Camera Pickle File', working_dir, 'Pickle (*.pkl)')
        if filename:
            working_dir = os.path.dirname(filename)
            self.camPickle.setText(filename)
    
    # sensor file picker  
    def sensor_picker(self):
        global working_dir
        [filename,filt] = QtWidgets.QFileDialog.getOpenFileName(None, 'Open Sensor File', working_dir, 'Comma Separated File (*.csv)')
        if filename:
            working_dir = os.path.dirname(filename)
            self.sensor_txt.setText(filename)
    
    # output file picker          
    def outfile_picker(self):
        global working_dir
        [filename,filt] = QtWidgets.QFileDialog.getSaveFileName(None, 'Output File Name', working_dir, 'Comma Separated File (*.csv)')
        self.out_txt.setText(filename)
    
    # precalc checkbox - enable options     
    def precalc_change(self):
        if self.precalcCam_box.isChecked():
            self.camPickle.setEnabled(True)
            self.pickle_btn.setEnabled(True)
            self.exportCam_box.setChecked(0)
            self.exportCam_box.setEnabled(False)
        else:
            self.camPickle.setEnabled(False)
            self.pickle_btn.setEnabled(False)
            self.exportCam_box.setEnabled(True)
    
    # filter checkbox - enable options         
    def filter_change(self):
        if self.check_filter.isChecked():
            self.spinBox_angle.setEnabled(True)
            self.spinBox_dist.setEnabled(True)
        else:
            self.spinBox_angle.setEnabled(False)
            self.spinBox_dist.setEnabled(False)

# END PyQt Setup 
#-------------------------------
        
#-----------
# MAIN
        
    def main_prog(self):
        
        # get file names from the GUI
        target_file = self.ptCloud_txt.text()
        cam_file = self.cam_txt.text()
        pickle_file = self.camPickle.text()
        sensor_file = self.sensor_txt.text()
        outfile = self.out_txt.text()
        ref_index = self.spinBox_rf.value()
        
        # array of extra options
        #   True/False for check boxes, numbers from spin boxes
        # [0:small angle calc, 1:extra point statistics, 2:extra camera outputs,
        #  3:filter depth values, 4:max angle(deg), 5:max distance (m)]
        extraOpt = np.array([self.check_smAng.isChecked(),self.check_stats.isChecked(),
                             self.check_camStats.isChecked(),self.check_filter.isChecked(),
                             self.spinBox_angle.value(),self.spinBox_dist.value()])

        # Debug messages 
        if self.exportCam_box.isChecked():
            print("The Export Cam box is checked")
        
        if self.precalcCam_box.isChecked():
            print("The Pre Calc'd Cam box is checked")
            
        print("The extra options are: ")
        print(extraOpt)
        print("Refractive Index = ", ref_index)
              
                
        # qt progress bar
        self.topProg_Lbl.setText("Data Loading...")
        
        #READ INPUT FILES
        
        # target points - as CSV point cloud (x,y,z,w_surf,r,g,b) from CloudCompare
        #   will be read in 10000 point chunks for memory management purposes   
        target_file = self.ptCloud_txt.text()
        targets = pd.read_csv(target_file, chunksize = 10000)
        self.top_progBar.setValue(25)
        
        # camera file - from Photoscan (Name, Position, Orientation...)
        # check for precalc checkbox, if so read directly to cam_r variable
        cam_file = self.cam_txt.text()
        if self.precalcCam_box.isChecked():
            foot_prints = pd.read_pickle(pickle_file)
            cams = pd.read_csv(cam_file)
            self.top_progBar.setValue(50)
        else:
            cams = pd.read_csv(cam_file)
            self.top_progBar.setValue(50)
        
        # camera sensor parameters - user generated
        sensor_file = self.sensor_txt.text()
        sensor = pd.read_csv(sensor_file)
        self.top_progBar.setValue(75)
        
        # OUTPUT - CSV file for saving outputs
        outfile = self.out_txt.text()
        self.top_progBar.setValue(100)
        
        # user feedback
        self.topProg_Lbl.setText("Data Loaded...")
        print("Data Loaded...")
        
        # record the start time of the actual processing
        start_time = datetime.now()
        
        # array for count of total points
        count = []
        
        # bottom progress bar
        self.bot_progBar.setValue(0)
        for idx, tar in enumerate(targets):
            chunk_num = idx
        self.bot_progBar.setMaximum(chunk_num)
        targets = pd.read_csv(target_file, chunksize = 7500)
        
        # Main Processing Loop, for each chunk of points from the reader
        for idx, tar in enumerate(targets):
            
            # Error check for mislabeled columns, from CloudCompare the column 
            #    header starts '//X,Y,Z
            if tar.columns.values[0] == '//X':
                tar.columns.values[0] = 'x'
                tar.columns.values[1] = 'y'
            
            if tar.columns.values[0] == 'X':
                tar.columns.values[0] = 'x'
                tar.columns.values[1] = 'y'
            
            count.append(tar.shape[0])    
            
            # if the index(idx) equals 0, use the mean elevation of the first chunk
            #   use the mean elevation of the first chunk of points to calculate the camera footprints
            if idx == 0 and not self.precalcCam_box.isChecked():
                # establish mean elevation for footprint mapping from the mean 
                #   elevation of the target points
                base_elev = np.mean(tar.sfm_z)      
                
                # build camera footprints
                foot_prints = footprints(cams,sensor,base_elev,self)
                self.topProg_Lbl.setText("Calculating Camera Footprints...")
                
                if self.exportCam_box.isChecked():
                    print("Saving footprints")
                    foot_file = os.path.dirname(outfile) + '/' + os.path.basename(cam_file)[:-4]  + '_cam_foot.pkl'
                    foot_prints.to_pickle(foot_file)
                
                # timer
                cam_end_time = datetime.now()
                mins_c = (cam_end_time - start_time).total_seconds() / 60
                print("Processed %i cameras in %0.2f minutes" %(np.count_nonzero(cams.x),mins_c))
            
            # use feedback and timer start
            self.topProg_Lbl.setText("Performing Bathymetric Correction...")
            self.top_progBar.setValue(0)
            if idx == 0 and self.precalcCam_box.isChecked():
                refract_start_time = datetime.now()
            
            # test the visibility of target point based on the camera footprints
            cam_r,cam_dist,cam_qual = visibility(cams,foot_prints,tar)        
            
            # perform the refraction correction
            tar_out, ang_r, x_dist, h, cor_elev = correction(cam_r, tar, ref_index, extraOpt)
            
            #if filter checkbox is ticked, run the pitonFilter function
            if extraOpt[3]:
                tar_out = pointFilter(tar_out,h,cam_r,cam_dist,extraOpt)
                
            # DEBUG export - Extra Camera Outputs
            #   will output extra debugging tables for the first batch (10000)
            #   of points. OUtputs will be in the output folder
            #   Ideal for working with small test datasets or with validation datasets
            if idx == 0 and extraOpt[2]:
                
                debug_path = os.path.dirname(outfile) + '/' + os.path.basename(target_file)[:-4]
                file =  debug_path + '_tar_out.pkl'
                tar_out.to_pickle(file)
                
                file = debug_path + '_cam_r.csv'
                np.savetxt(file,cam_r,delimiter=",")
                
                file = debug_path + '_cam_dist.csv'
                np.savetxt(file,cam_dist,delimiter=",")
                
                file = debug_path + '_ang_r.csv'
                np.savetxt(file,ang_r,delimiter=",")
                
                file = debug_path + '_x_dist.csv'
                np.savetxt(file,x_dist,delimiter=",")
                
                file = debug_path + '_h_all.csv'
                np.savetxt(file,h,delimiter=",")
                
                file = debug_path + '_cor_elev.csv'
                np.savetxt(file,cor_elev,delimiter=",")
            
            # Camera Quality Statistics
            if 'quality' in cams.columns:
                tar_out['cam_qual_mean'] = np.nanmean(cam_qual, axis = 1)
                tar_out['cam_qual_median'] = np.nanmean(cam_qual, axis = 1)
                tar_out['cam_qual_min'] = np.nanmin(cam_qual, axis = 1)
                tar_out['cam_qual_max'] = np.nanmax(cam_qual, axis = 1)
            
            # output - for the first chunk write header row, else append subsequent
            #   chunks without headers
            if idx == 0:
                with open(outfile, 'a') as f:
                    tar_out.to_csv(f, header=True, index=False)
            else:
                with open(outfile, 'a') as f:
                     tar_out.to_csv(f, header=False, index=False)
    
            # user feedback, def timer and bottom progress bar
            self.bot_progBar.setValue(idx)
            if self.precalcCam_box.isChecked():
                timer(count, refract_start_time)
            else:
                timer(count, cam_end_time)
                
        # User feedback on the total processing time
        tot_count = sum(count)
        tot_time = (datetime.now() - start_time).total_seconds() / 60
        print("%i points processed, Total Running Time = %0.2f minutes" %(tot_count,tot_time))
        self.botProg_lbl.setText('Processing Complete')
