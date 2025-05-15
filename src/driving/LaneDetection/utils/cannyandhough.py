import cv2
import numpy as np

import math
from matplotlib import pyplot as plt, cm, colors

class LaneFollowing:

    def __init__( self, region ):
        
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

        self.region = region

        self.lane_len = 430 #
        
        self.histogram = None
        self.leftxBase = None
        self.rightxBase = None
    

    def perspectiveWarp( self, frame ):

        img_size = (frame.shape[1], frame.shape[0])

        #FRAME TRANSFORMATION
        src = np.float32([
            [0, 540], 
            [50, 350], 
            [690, 350], 
            [800, 540] 
        ])
        
        dst = np.float32([
            [200, 540],
            [200, 0],
            [700, 0],
            [600, 540]
        ])

        matrix = cv2.getPerspectiveTransform( src, dst ) #transforms original to bov
        #used to map lane detections back to the original view
        minv = cv2.getPerspectiveTransform( dst, src )

        birdseye = cv2.warpPerspective( frame, matrix, img_size )

         # Get the birdseye window dimensions
        height, width = birdseye.shape[:2]


        # Divide the birdseye view into 2 halves to separate left & right lanes
        birdseyeLeft  = birdseye[0:height, 0:width // 2]
        birdseyeRight = birdseye[0:height, width // 2:width]

        return birdseye, birdseyeLeft, birdseyeRight, minv

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
        
        # self.ROI( frame, self.region )
        # frame = self.masked_img

        #white pixels
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        lower_white = np.array([0, 160, 10])
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(frame, lower_white, upper_white)
        hls_result = cv2.bitwise_and(frame, frame, mask = mask)

        gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
        blur = cv2.GaussianBlur( thresh, (3,3), 0)

        edges = cv2.Canny( blur, 30, 120 )

        self.ROI(edges, self.region)
        edges = self.masked_img

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold = 50, minLineLength=30, maxLineGap=30)

        self.draw_all_lines( frame, lines )
        # frame = self.draw_ROI( frame )

        return frame, lines, edges, hls_result, thresh
        
    def draw_all_lines( self, frame, lines ):

        left_lines_x1, left_lines_x2, right_lines_x1, right_lines_x2 = [], [], [], []
        left_lines_y1, left_lines_y2, right_lines_y1, right_lines_y2 = [], [], [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                if abs( y2 - y1 ) < 100:
                    continue

                if x1 - x2 == 0:
                    continue

                if x1 < self.frame_width // 2 and x2 < self.frame_width // 2:
                    
                    #ogranicenje za pesacki
                    # if x2 > 350:
                    #     continue

                    left_lines_x1.append( int(x1) )
                    left_lines_x2.append( int(x2) )
                    left_lines_y1.append( int(y1) )
                    left_lines_y2.append( int(y2) )

                    angle_rad = math.atan2(y2 - y1, x2 - x1)
                    self.left_slope = angle_rad

                elif x1 > self.frame_width // 2 and x2 > self.frame_width // 2:
                    
                    #ogranicenje za pesacki
                    # if x1 < 610:
                    #     continue;

                    right_lines_x1.append( int(x1) )
                    right_lines_x2.append( int(x2) )
                    right_lines_y1.append( int(y1) )
                    right_lines_y2.append( int(y2) )

                    angle_rad = math.atan2(y2 - y1, x2 - x1)
                    self.right_slope = angle_rad
                

        output_left_lines = []
        output_right_lines = []


        #######CHANGES########

        # for x1, y1, x2, y2 in zip(left_lines_x1, left_lines_y1, left_lines_x2, left_lines_y2):
        #     output_left_lines.append((x1, y1, x2, y2))

        # for x1, y1, x2, y2 in zip(right_lines_x1, right_lines_y1, right_lines_x2, right_lines_y2):
        #     output_right_lines.append((x1, y1, x2, y2))

        # left = np.array(output_left_lines)
        # right = np.array(output_right_lines)

        # if left.shape[0] > 0:
        #     avg_left = np.mean(left, axis=0).astype(int)  # (x1, y1, x2, y2)
        #     cv2.line(frame, (avg_left[0], avg_left[1]), (avg_left[2], avg_left[3]), (0, 0, 255), 10)

        # if right.shape[0] > 0:
        #     avg_right = np.mean(right, axis=0).astype(int)
        #     cv2.line(frame, (avg_right[0], avg_right[1]), (avg_right[2], avg_right[3]), (0, 0, 255), 10)

            
        #     #changes
        if left_lines_x1 and left_lines_x2:
            # min_index_L = left_lines_x1.index(min(left_lines_x1))  
            min_index_L = left_lines_x1.index(max(left_lines_x1))  

            left_line_x1 = left_lines_x1[min_index_L]  
            left_line_y1 = left_lines_y1[min_index_L]  

            max_index_L = left_lines_x2.index( max( left_lines_x2 ) )
            left_line_x2 = left_lines_x2[max_index_L]
            left_line_y2 = left_lines_y2[max_index_L]

            cv2.line(frame, (left_line_x1, left_line_y1), (left_line_x2, left_line_y2), (0, 0, 255), 10)
            output_left_lines.append( (left_line_x1, left_line_x2, left_line_y1, left_line_y2) )

            self.arr_left.append( output_left_lines )


        if right_lines_x1 and right_lines_x2:
            
            max_index_R = right_lines_x2.index( min( right_lines_x2 ) )
            right_line_x2 = right_lines_x2[max_index_R]
            right_line_y2 = right_lines_y2[max_index_R]

            #changes
            min_index_L = right_lines_x1.index( min( right_lines_x1 ) )
            # min_index_L = right_lines_x1.index( min( right_lines_x1 ) )
            right_line_x1 = right_lines_x1[min_index_L]
            right_line_y1 = right_lines_y1[min_index_L]

            cv2.line(frame, (right_line_x2, right_line_y2), (right_line_x1, right_line_y1), (0, 0, 255), 10)
            output_right_lines.append( (right_line_x1, right_line_x2, right_line_y1, right_line_y2) )

            self.arr_right.append( output_right_lines )




        self.left_lines = output_left_lines
        self.right_lines = output_right_lines

        return output_left_lines, output_right_lines

    def draw_all_lines___( self, frame, lines ):

        left_lines_x1, left_lines_x2, right_lines_x1, right_lines_x2 = [], [], [], []
        left_lines_y1, left_lines_y2, right_lines_y1, right_lines_y2 = [], [], [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                if abs( y2 - y1 ) < 100:
                    continue

                if x1 - x2 == 0:
                    continue

                if x1 < self.frame_width // 2 and x2 < self.frame_width // 2:
                    
                    #ogranicenje za pesacki
                    if x2 > 350:
                        continue

                    left_lines_x1.append( int(x1) )
                    left_lines_x2.append( int(x2) )
                    left_lines_y1.append( int(y1) )
                    left_lines_y2.append( int(y2) )

                    angle_rad = math.atan2(y2 - y1, x2 - x1)
                    self.left_slope = angle_rad

                elif x1 > self.frame_width // 2 and x2 > self.frame_width // 2:
                    
                    #ogranicenje za pesacki
                    if x1 < 610:
                        continue;

                    right_lines_x1.append( int(x1) )
                    right_lines_x2.append( int(x2) )
                    right_lines_y1.append( int(y1) )
                    right_lines_y2.append( int(y2) )

                    angle_rad = math.atan2(y2 - y1, x2 - x1)
                    self.right_slope = angle_rad
                

        output_left_lines = []
        output_right_lines = []

        if left_lines_x1 and left_lines_x2:
            
            #changes
            # min_index_L = left_lines_x1.index(min(left_lines_x1))  
            min_index_L = left_lines_x1.index(max(left_lines_x1))  

            left_line_x1 = left_lines_x1[min_index_L]  
            left_line_y1 = left_lines_y1[min_index_L]  

            max_index_L = left_lines_x2.index( max( left_lines_x2 ) )
            left_line_x2 = left_lines_x2[max_index_L]
            left_line_y2 = left_lines_y2[max_index_L]

            cv2.line(frame, (left_line_x1, left_line_y1), (left_line_x2, left_line_y2), (0, 0, 255), 10)
            output_left_lines.append( (left_line_x1, left_line_x2, left_line_y1, left_line_y2) )

            self.arr_left.append( output_left_lines )


        if right_lines_x1 and right_lines_x2:
            
            max_index_R = right_lines_x2.index( min( right_lines_x2 ) )
            right_line_x2 = right_lines_x2[max_index_R]
            right_line_y2 = right_lines_y2[max_index_R]

            #changes
            min_index_L = right_lines_x1.index( min( right_lines_x1 ) )
            # min_index_L = right_lines_x1.index( min( right_lines_x1 ) )
            right_line_x1 = right_lines_x1[min_index_L]
            right_line_y1 = right_lines_y1[min_index_L]

            cv2.line(frame, (right_line_x2, right_line_y2), (right_line_x1, right_line_y1), (0, 0, 255), 10)
            output_right_lines.append( (right_line_x1, right_line_x2, right_line_y1, right_line_y2) )

            self.arr_right.append( output_right_lines )




        self.left_lines = output_left_lines
        self.right_lines = output_right_lines

        return output_left_lines, output_right_lines


    def twoLanes( self, left_lines, right_lines, prev_lane_center ):

        if left_lines and right_lines:
            left_x = int( np.mean([x for x1, x2, _, _ in left_lines for x in (x1, x2)]) )
            right_x = int( np.mean([x for x1, x2, _, _ in right_lines for x in (x1, x2)]) )

            return ( left_x + right_x ) // 2
        else:
            return prev_lane_center   
        
    def oneLine( self, last_line, slope, frame, known_lines ):

        x1, x2, y1, y2 = last_line[0] 

        x1_2 = int( x1 + 200 * math.cos( slope ) ) #last detected point + 200 px * last line slope
        y1_2 = int( y1 + 200 * math.sin( slope ) )

        cv2.line( frame, (x1, y1), (x1_2, y1_2), (0, 0, 255), 10)

        unknown_x = ( x1 + x1_2 ) // 2
        known_x = int( np.mean([x for x1, x2, _, _ in known_lines for x in (x1, x2)]) )

        lane_center = ( unknown_x + known_x ) // 2

        return lane_center
     

    def get_lane_center( self, lines, frame, prev_lane_center ):
        
        left_lines, right_lines = self.draw_all_lines( frame, lines )

        if not left_lines:
            self.frame_counter_left += 1
        else:
            self.frame_counter_left = 0
        if not right_lines:
            self.frame_counter_right += 1
        else:
            self.frame_counter_right = 0

        left_ind = self.frame_counter_left >= self.max_frame_counter
        right_ind = self.frame_counter_right >= self.max_frame_counter
   
        if left_lines and right_lines:
            lane_center = self.twoLanes( left_lines, right_lines, prev_lane_center )
       
        elif left_lines and not right_lines and right_ind:
            
            if len( self.arr_right ) != 0:
                last_line = self.arr_right.pop()
        
                lane_center = self.oneLine( last_line, self.right_slope, frame, left_lines )
            else:
                left_x = int( np.mean([x for x1, x2, _, _ in left_lines for x in (x1, x2)]) )
                right_x = left_x + self.lane_len

                lane_center = ( left_x + right_x ) // 2 

        elif not left_lines and right_lines and left_ind:

            if len( self.arr_left ) != 0:
                last_line = self.arr_left.pop()

                lane_center = self.oneLine( last_line, self.left_slope, frame, right_lines )
            else:
                right_x = int( np.mean([x for x1, x2, _, _ in right_lines for x in (x1, x2)]) )
                left_x = right_x - self.lane_len

                lane_center = ( left_x + right_x ) // 2
            
        else:
            lane_center = prev_lane_center

        lane_center = int(0.6 * lane_center + (1 - 0.6) * prev_lane_center)
        cv2.line( frame, (lane_center, frame.shape[0] // 2), (lane_center, frame.shape[0] ), (0, 255, 0 ), 5 ) 

        self.lane_center = lane_center

        return lane_center
    

    def sliding_window_search( self, binary_warped, original_frame ):

        mask = np.zeros_like(binary_warped)
        
        vertices = np.array([[(0, 540), (100, 400), (860, 400), (960, 540)]])

        cv2.fillPoly(mask, vertices, 255)
        binary_warped = cv2.bitwise_and(binary_warped, mask)

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
        minpix = 120

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

            cv2.rectangle(original_frame, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
            (0,255,0), 2)
            cv2.rectangle(original_frame, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
            (0,255,0), 2)

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

        # Slično za desnu traku:
        right_points = np.vstack((rightx, righty)).T.reshape((-1, 1, 2))


        out_img = cv2.polylines( original_frame, [left_points], 0, (100,255,0), 2)
        out_img = cv2.polylines( original_frame, [right_points], 0, (100,255,0), 2)

        mean_left = np.mean( leftx )
        mean_right = np.mean( rightx )

        #non values
        if math.isnan( mean_left) :
            mean_left = 0
        if math.isnan( mean_right ) :
            mean_right = 960
            
        #near center values
        if mean_left > 430:
            mean_left = 430

        if mean_right < 530:
            mean_right = 530

        cv2.line( original_frame, ( 430, 0), ( 430, 540), (0,0,255), 5 )
        cv2.line( original_frame, ( 530, 0), ( 530, 540), (0,0,255), 5 )

        # print(f"MEAN LEFT {mean_left}")
        # print(f"MEAN RIGHT {mean_right}")
        # print("* * * *  *")

        self.lane_center = ( mean_left + mean_right ) // 2

        # if np.isnan(self.lane_center):
        #     self.lane_center = 960 // 2  # Postavi u sredinu slike ako je NaN

        # cv2.line( out_img, (int(self.lane_center), 540), (int(self.lane_center), 340), (0,255,0), 10)

        # print( left_lane_inds )
        # print( right_lane_inds )

        return out_img, original_frame







