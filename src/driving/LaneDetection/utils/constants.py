# -*- coding: utf-8 -*-
"""
 
 Title: Constants-File
 Author: Anton ELmiger
 Created: 2020-06-23

 Information: all parameters of the whole image processing are stored in this module
              and can be administrated and modified here

"""

######################### MODE CONSTS #########################
MODE = "camera"                      # set mode: "camera" = live camera input, "video" = video input, "simulink" = simulink input
VISUALIZE = True                    #  set to True or False to visualize lane detection
WRITE_VIDEO = True                 # Write Video File after Execution
PRINT_TIME_PER_FRAME = False         # Print ms per loop in console
######################### PATH CONSTS #########################
PATH = ""  # path of the current directory ("workspace") 
VIDEO_FILE = PATH + "output.avi"    # filename of video file

######################### IMG  CONSTS #########################
IMAGE_WIDTH = 960
IMAGE_HEIGHT = 540
EDGE_SIZE = 3
X_START = 0  # početna koordinata za X os
X_STOP = 540   # završna koordinata za X os
Y_START = 0   # početna koordinata za Y os
Y_STOP = 960   # završna koordinata za Y os


######################## LANE DETECTION #######################
LANE_WIDTH = 2                        # specify lane width. no specific unit
BOUNDARY_THRESH = 1.8                   # smaller value means a broader search area for boundary points
HORIZON = -40.1                         # horizon in image pixel. NEEDS TO HAVE A DECIMAL PLACE TO AVOID DIVISION BY 0 (80 => 80.1)
FILTER_STRENGTH = 1                    # number of frames where lane is smoothed with moving average
