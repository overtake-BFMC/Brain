import cv2
import numpy as np

class LaneFollowing:

    def __init__( self ):
        
        self.left_lines = None
        self.right_lines = None
        self.prev_lane_center = None
        self.lane_center = None
        self.masked_img = None
        self.frame_width = 960
        self.frame_counter_right = 0
        self.frame_counter_left = 0
        self.max_frame_counter = 3
        self.distance_from_right = 225 #otprilike 
        self.distance_from_left = 225

        # self.kontejner = []

    def ROI( self, frame, vertices ):
        
        mask = np.zeros_like( frame )
        cv2.fillPoly( mask, vertices, 255 )  
        self.masked_img = cv2.bitwise_and( frame, mask )  

        return self
    
    def draw_ROI( self, frame, region ):

        overlay = frame.copy()  
        cv2.fillPoly(overlay, [region], (0, 255, 0))  
        
        alpha = 0.3  
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)  

        return frame 
    
    def CannyEdge( self, frame ):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (7, 7), 0)
        low_sigma = cv2.GaussianBlur(gray,(3,3),0)
        high_sigma = cv2.GaussianBlur(gray,(5,5),0)
        
        # Calculate the DoG by subtracting
        gray = low_sigma - high_sigma

        _, gray = cv2.threshold(frame, 180, 255, cv2.THRESH_BINARY)



        edges = cv2.Canny( gray, 30, 120 )

        # region = np.array( [[ ( 80, 540 ), ( 330, 170 ), ( 620, 170 ), (840, 540) ]]) 
        region = np.array( [[ ( 80, 540, ), ( 330, 170 ), ( 680, 170 ), ( 900, 540 ) ]] ) #za snimak u raskrsnici
        # region = np.array( [[ ( 0, 540, ), ( 100, 300 ), ( 860, 300 ), ( 960, 540 ) ]] ) #za snimak u raskrsnici


        self.ROI(edges, region)
        edges = self.masked_img

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold = 50, minLineLength=50, maxLineGap=50)

        self.draw_all_lines( frame, lines )
        frame = self.draw_ROI( frame, region )
        # self.draw_ROI( frame, region )

        return frame, lines
    
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

                    left_lines_x1.append( int(x1) )
                    left_lines_x2.append( int(x2) )
                    left_lines_y1.append( int(y1) )
                    left_lines_y2.append( int(y2) )

                elif x1 > self.frame_width // 2 and x2 > self.frame_width // 2:

                    right_lines_x1.append( int(x1) )
                    right_lines_x2.append( int(x2) )
                    right_lines_y1.append( int(y1) )
                    right_lines_y2.append( int(y2) )

        output_left_lines = []
        output_right_lines = []

        if left_lines_x1 and left_lines_x2:

            min_index_L = left_lines_x1.index(min(left_lines_x1))  
            left_line_x1 = left_lines_x1[min_index_L]  
            left_line_y1 = left_lines_y1[min_index_L]  

            max_index_L = left_lines_x2.index( max( left_lines_x2 ) )
            left_line_x2 = left_lines_x2[max_index_L]
            left_line_y2 = left_lines_y2[max_index_L]

            cv2.line(frame, (left_line_x1, left_line_y1), (left_line_x2, left_line_y2), (0, 0, 255), 10)
            output_left_lines.append( (left_line_x1, left_line_x2, left_line_y1, left_line_y2) )

        if right_lines_x1 and right_lines_x2:
            
            max_index_R = right_lines_x2.index( max( right_lines_x2 ) )
            right_line_x2 = right_lines_x2[max_index_R]
            right_line_y2 = right_lines_y2[max_index_R]

            min_index_L = right_lines_x1.index( min( right_lines_x1 ) )
            right_line_x1 = right_lines_x1[min_index_L]
            right_line_y1 = right_lines_y1[min_index_L]

            cv2.line(frame, (right_line_x2, right_line_y2), (right_line_x1, right_line_y1), (0, 0, 255), 10)
            output_right_lines.append( (right_line_x1, right_line_x2, right_line_y1, right_line_y2) )

        self.left_lines = output_left_lines
        self.right_lines = output_right_lines

        return output_left_lines, output_right_lines

    def twoLanes( self, left_lines, right_lines, frame, prev_lane_center ):

        if left_lines and right_lines:
            left_x = int( np.mean([x for x1, x2, _, _ in left_lines for x in (x1, x2)]) )
            right_x = int( np.mean([x for x1, x2, _, _ in right_lines for x in (x1, x2)]) )

            # print( "levo", left_x )
            # print( "desno", right_x )

            # print( right_x - left_x )
            # self.kontejner.append( right_x - left_x )

            # print( "avg", sum(self.kontejner) / len(self.kontejner) )

            return ( left_x + right_x ) // 2
        # elif not left_lines and right_lines:
        #     return self.frame_width // 2 - 70
        # elif left_lines and not right_lines:
        #     return self.frame_width // 2 + 70
        else:
            return prev_lane_center   

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
            lane_center = self.twoLanes( left_lines, right_lines, frame, prev_lane_center )
       
        elif left_lines and not right_lines and right_ind:
            left_x = int( np.mean([x for x1, x2, _, _ in left_lines for x in (x1, x2)]) )
            # print( left_x )
            lane_center = left_x + self.distance_from_left
       
        elif not left_lines and right_lines and left_ind:
            right_x = int( np.mean([x for x1, x2, _, _ in right_lines for x in (x1, x2)]) )
            lane_center = right_x - self.distance_from_right
       
        else:
            lane_center = prev_lane_center + 15


        lane_center = int(0.6 * lane_center + (1 - 0.6) * prev_lane_center)
        cv2.line( frame, (lane_center, frame.shape[0] // 2), (lane_center, frame.shape[0] ), (0, 255, 0 ), 4 ) 

        self.lane_center = lane_center

        return lane_center
    