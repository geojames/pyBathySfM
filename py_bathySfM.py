#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------------------------------------
__author__ = 'James T. Dietrich'
__contact__ = 'james.dietrich@uni.edu'
__copyright__ = '(c) James Dietrich 2018'
__license__ = 'MIT'
__date__ = '23 Oct 2018'
__version__ = '2.1'
__status__ = "initial release"
__url__ = "https://github.com/geojames/py_sfm_depth"

"""
Name:           py_bathySfM.py
Compatibility:  Python 3.6
Description:    This program performs a per-camera refration correction on a 
                Structure-from-Motion point cloud. Additional documnetation,
                sample data, and a tutorial are availible from the GitHub
                address below.

URL:            https://github.com/geojames/py_sfm_depth

Requires:       tkinter, numpy, pandas, sympy, matplotlib

Dev ToDo:       1) speed up camera geometry calculations

AUTHOR:         James T. Dietrich
ORGANIZATION:   University of Northern Iowa
Contact:        james.dietrich@uni.edu
Copyright:      (c) James Dietrich 2018

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
#------------------------------------------------------------------------------
# Imports
 
import sys
from PyQt5.QtWidgets import QDialog, QApplication
from py_bathySfM_gui import Ui_bathySfM_gui

class AppWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_bathySfM_gui()
        self.ui.setupUi(self)
        self.show()

app = QApplication(sys.argv)
w = AppWindow()
w.show()
sys.exit(app.exec_())
