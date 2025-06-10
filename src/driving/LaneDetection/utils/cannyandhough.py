import cv2
import numpy as np

import math
from matplotlib import pyplot as plt, cm, colors

class LaneFollowing:

    def __init__( self ):
        
        self.left_lines = None
        self.right_lines = None

        self.arr_left = [] #ili prazno ili da mi prvi element bude pretpostavka pocetka linije
        self.arr_right = []

        self.prev_lane_center = None
        self.lane_center = None
        self.masked_img = None
        self.frame_width = 960
        self.frame_counter_right = 0
        self.frame_counter_left = 0
        self.max_frame_counter = 5

        self.left_slope = None     
        self.right_slope = None     

        self.lane_len = 430 #
        
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
    
    def draw_ROI( self, frame ):

        overlay = frame.copy()  
        cv2.fillPoly(overlay, [self.region], (0, 255, 0))  
        
        alpha = 0.3  
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)  

        return frame 
    
    def CannyEdge( self, frame ):
        
        #white pixels
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        lower_white = np.array([0, 160, 10])
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
        

    def sliding_window_search( self, binary_warped, original_frame ):

        mask = np.zeros_like(binary_warped)
        
        #test
        # outer_vertices = np.array([[(80, 540), (80, 400), (880, 400), (880, 540)]])  

        outer_vertices = np.array([[(0, 540), (0, 400), (960, 400), (960, 540)]])  
        cv2.fillPoly(mask, outer_vertices, 255)

        if self.crosswalk_active:

            inner_vertices = np.array([[(200, 540), (340, 400), (600, 400), (760, 540)]], dtype=np.int32)
            cv2.fillPoly(mask, inner_vertices, 0)        

        binary_warped = cv2.bitwise_and( self.binary_warped, mask )

        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 15
        window_height = int(binary_warped.shape[0] // nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        margin = 50
        minpix = 100

        left_lane_inds = []
        right_lane_inds = []

        out_img = np.dstack((binary_warped, binary_warped, binary_warped))


        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height

            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            #BW
            # cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
            # (0,255,0), 2)
            # cv2.rectangle(out_img, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
            # (0,255,0), 2)

            out_img = cv2.rectangle(original_frame, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
            (0,0,255), 2)
            out_img = cv2.rectangle(original_frame, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
            (0,0,255), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]

            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]  


        left_points = np.vstack((leftx, lefty)).T.reshape((-1, 1, 2))
        right_points = np.vstack((rightx, righty)).T.reshape((-1, 1, 2))

        out_img = cv2.polylines( out_img, [left_points], 0, (100,255,0), 2)
        out_img = cv2.polylines( out_img, [right_points], 0, (100,255,0), 2)

        mean_left = 0
        mean_right = 960

        if len(leftx) > 0:
            m_left = np.mean(leftx)
            if not math.isnan(m_left):
                mean_left = m_left
        else:
            mean_left = self.last_left_line


        if len(rightx) > 0:
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

        #raskrsnica pravac
        # if self.priority_active:

        #     left = mean_left
        #     right = 960 - mean_right

        #     if left > right and right - left > 10: 
        #         # print("mora desno")
        #         self.angle = -1

        #     else:
        #         # print("mora levo")


        cv2.line( out_img, ( 400, 0), ( 400, 540), (0,0,255), 5 )
        cv2.line( out_img, ( 560, 0), ( 560, 540), (0,0,255), 5 )

        # print(f"MEAN LEFT {mean_left}")
        # print(f"MEAN RIGHT {mean_right}")
        # print("* * * *  *")

        self.lane_center = ( mean_left + mean_right ) // 2
        # self.lane_center = 500

        # print("left line", mean_left)
        # print("***")
        # print( self.lane_center )

        self.last_left_line = mean_left
        self.last_right_line = mean_right

        if np.isnan(self.lane_center):
            self.lane_center = 960 // 2  # Postavi u sredinu slike ako je NaN

        cv2.line( out_img, (int(self.lane_center), 540), (int(self.lane_center), 340), (0,255,0), 10)

        # print( left_lane_inds )
        # print( right_lane_inds )

        return out_img, original_frame
