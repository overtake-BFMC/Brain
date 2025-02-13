from __future__ import print_function
import numpy as np
import argparse
import cv2

def adjust_gamma(image, gamma=0.5):

    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                     for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def apply_gamma_on_frame(frame, gamma=0.5):
  
    adjusted_frame = adjust_gamma(frame, gamma)
    
    return adjusted_frame