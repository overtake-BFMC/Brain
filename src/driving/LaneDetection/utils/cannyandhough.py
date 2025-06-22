import cv2
import numpy as np

import math
from matplotlib import pyplot as plt, cm, colors

class LaneFollowing:

    def __init__( self ):
        
        # self.left_lines = None
        # self.right_lines = None

        # self.arr_left = [] #ili prazno ili da mi prvi element bude pretpostavka pocetka linije
        # self.arr_right = []

        # self.prev_lane_center = None
        self.lane_center = None
        self.masked_img = None
        self.frame_width = 960
        # self.frame_counter_right = 0
        # self.frame_counter_left = 0
        # self.max_frame_counter = 5

        # self.left_slope = None     
        # self.right_slope = None     

        # self.lane_len = 430 #
        
        self.histogram = None
        self.leftxBase = None
        self.rightxBase = None

        self.last_left_line = 480 - 150
        self.last_right_line = 480 + 150 

        self.region = np.array([[(0, 540), (0, 400), (960, 400), (960, 540)]])  

        self.crosswalk_active = False
        self.priority_active = False

        self.angle = 0

    def ROI( self, frame, vertices ):
        
        mask = np.zeros_like( frame )
        cv2.fillPoly( mask, vertices, 255 ) 

        self.masked_img = cv2.bitwise_and( frame, mask )  

        return self
    
    def draw_ROI( self, frame, region ):

        overlay = frame.copy()  
        cv2.polylines(overlay, [region], isClosed=True, color=(0, 255, 0), thickness=2)

        # cv2.fillPoly(overlay, [self.region], (0, 255, 0))  
        
        alpha = 0.3  
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)  

        return frame 
    
    def CannyEdge( self, frame ):
        
        #white pixels
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        lower_white = np.array([0, 160, 10])
        # lower_white = np.array([0, 160, 10])
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(frame, lower_white, upper_white)
        hls_result = cv2.bitwise_and(frame, frame, mask = mask)

        gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

        blur = cv2.GaussianBlur( thresh, (3,3), 0)

        edges = cv2.Canny( blur, 30, 120 )

        self.ROI(edges, self.region)
        edges = self.masked_img

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold = 50, minLineLength=30, maxLineGap=30)

        # self.draw_all_lines( frame, lines )
        # frame = self.draw_ROI( frame )

        self.binary_warped = thresh

        return frame, lines, edges, hls_result, thresh
        

    def sliding_window_search( self, thresh_frame, original_frame ):

        mask = np.zeros_like( thresh_frame )
        #test
        outer_vertices = np.array([[(0, 540), (150, 350), (810, 350), (960, 540)]])    
        # outer_vertices = np.array([[(0, 540), (150, 400), (810, 400), (960, 540)]])    trnutly
        # outer_vertices = np.array([[(0, 540), (0, 400), (960, 400), (960, 540)]])  
        cv2.fillPoly(mask, outer_vertices, 255)

        #sasija roi
        # inner_vertices = np.array([[(350, 540), (350, 430), (610, 430), (610, 540)]], dtype=np.int32)
        inner_vertices = np.array([[(200, 540), (350, 430), (610, 430), (660, 540)]], dtype=np.int32)
        cv2.fillPoly(mask, inner_vertices, 0)

        if self.crosswalk_active:

            inner_vertices = np.array([[(200, 540), (340, 400), (600, 400), (760, 540)]], dtype=np.int32)
            cv2.fillPoly(mask, inner_vertices, 0)        

        thresh_frame = cv2.bitwise_and( thresh_frame, mask )

        histogram = np.sum( thresh_frame[thresh_frame.shape[0]//2:, :], axis=0 )
        midpoint = int( histogram.shape[0] // 2 )
        leftx_base = np.argmax( histogram[:midpoint] )
        rightx_base = np.argmax( histogram[midpoint:] ) + midpoint

        window_num = 15
        margin = 50
        minpix = 100

        window_height = int( thresh_frame.shape[0] // window_num )
        nonzero = thresh_frame.nonzero()
        WHITE_Y = np.array( nonzero[0] )
        WHITE_X = np.array( nonzero[1] )

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        out_img = np.dstack(( thresh_frame, thresh_frame, thresh_frame ))


        for window in range(window_num):
            
            win_y_low = thresh_frame.shape[0] - (window + 1) * window_height
            win_y_high = thresh_frame.shape[0] - window * window_height

            left_x_L = leftx_current - margin
            left_x_H =  leftx_current + margin

            right_x_L = rightx_current - margin
            right_x_H = rightx_current + margin

            #BW
            # cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
            # (0,255,0), 2)
            # cv2.rectangle(out_img, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
            # (0,255,0), 2)

            # out_img = cv2.rectangle(original_frame, (left_x_L, win_y_low), (left_x_H, win_y_high),
            # (0,255,0), 2)
            # out_img = cv2.rectangle(original_frame, (right_x_L,win_y_low), (right_x_H,win_y_high),
            # (0,255,0), 2)

            left_inds = (
                ( WHITE_Y >= win_y_low ) & ( WHITE_Y < win_y_high ) &
                ( WHITE_X >= left_x_L ) & ( WHITE_X < left_x_H )
            ).nonzero()[0]

            right_inds = (
                ( WHITE_Y >= win_y_low ) & ( WHITE_Y < win_y_high ) &
                ( WHITE_X >= right_x_L ) & ( WHITE_X < right_x_H )
            ).nonzero()[0]

            left_lane_inds.append( left_inds )
            right_lane_inds.append( right_inds )

            if len( left_inds ) > minpix:
                leftx_current = int( np.mean(WHITE_X[left_inds]) )
            if len( right_inds ) > minpix:
                rightx_current = int( np.mean(WHITE_X[right_inds]) )

        left_lane_inds = np.concatenate( left_lane_inds )
        right_lane_inds = np.concatenate( right_lane_inds )

        leftx  = WHITE_X[ left_lane_inds ]
        lefty  = WHITE_Y[ left_lane_inds ]
        rightx = WHITE_X[ right_lane_inds]
        righty = WHITE_Y[ right_lane_inds]  

        left_points = np.vstack(( leftx, lefty )).T.reshape((-1, 1, 2))
        right_points = np.vstack(( rightx, righty )).T.reshape((-1, 1, 2))

        out_img = cv2.polylines( original_frame, [left_points], 0, (100,255,0), 2)
        out_img = cv2.polylines( original_frame, [right_points], 0, (100,255,0), 2)

        mean_left = 0
        mean_right = 960

        if len(leftx) > 100:
            m_left = np.mean(leftx)
            if not math.isnan(m_left):
                mean_left = m_left
        else:
            mean_left = self.last_left_line


        if len(rightx) > 50:
            m_right = np.mean(rightx)
            if not math.isnan(m_right):
                mean_right = m_right
        else:
            mean_right = self.last_right_line

        #near center values, center: 480, 60+-
        if mean_left > 400:
            mean_left -= 50
        # print(f"left: {mean_left}")

        if mean_right < 560:
            mean_right += 50
        # print(f"left: {mean_right}")

        # cv2.line( out_img, ( 400, 0), ( 400, 540), (0,0,255), 5 )
        # cv2.line( out_img, ( 560, 0), ( 560, 540), (0,0,255), 5 )

        self.lane_center = ( mean_left + mean_right ) // 2

        self.last_left_line = mean_left 
        self.last_right_line = mean_right 


        if np.isnan(self.lane_center):
            self.lane_center = 960 // 2  # Postavi u sredinu slike ako je NaN

        out_img = self.draw_ROI( out_img, outer_vertices )
        out_img = self.draw_ROI( out_img, inner_vertices )

        # cv2.line( out_img, (int(self.lane_center), 540), (int(self.lane_center), 340), (0,255,0), 10)
        cv2.circle( out_img, (int(self.lane_center), 450), radius=5, color=(0, 255, 0), thickness=10 )

        return out_img, original_frame
