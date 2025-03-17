import cv2
import numpy as np
import scipy.interpolate as si

from scipy.interpolate import UnivariateSpline

def region_of_interest( img, vertices ):

    mask = np.zeros_like( img )  
   
    cv2.fillPoly( mask, vertices, 255 )  
   
    masked_img = cv2.bitwise_and( img, mask )  
   
    return masked_img


def CannyEdge( frame ):

    # hsv = cv2.cvtColor( frame, cv2.COLOR_BGR2HSV )

    gray = cv2.GaussianBlur(frame, (5, 5), 0)

    edges = cv2.Canny( gray, 50, 200 )

    height = edges.shape[0] #1484
    width = edges.shape[1] #3280

    region = np.array( [ [ ( 80, 540 ), ( 360, 200 ), ( 650 , 200 ), ( 820  ,540)]])

    edges = region_of_interest( edges, region )

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold = 30, minLineLength=10, maxLineGap=100)

    draw_all_lines( frame, lines )
    frame = draw_ROI( frame, region )

    return edges, frame, lines      


def draw_all_lines(frame, lines):
    left_lines_x1, left_lines_x2, right_lines_x1, right_lines_x2 = [], [], [], []
    left_lines_y1, left_lines_y2, right_lines_y1, right_lines_y2 = [], [], [], []

    l_points, r_points = [], []

    LEVO = False
    DESNO = False

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Izbegavamo skoro horizontalne linije
            if abs(y2 - y1) < 20:
                continue
            
            if x2 - x1 == 0:
                continue

            slope = ( y2 - y1 ) / ( x2 - x1 )


            if slope > 0.8 or slope < -0.8:

                if x1 < 960 // 2 and x2 < 960 // 2:  
                    left_lines_x1.append(int(x1))
                    left_lines_x2.append(int(x2))
                    left_lines_y1.append(int(y1))
                    left_lines_y2.append(int(y2))

                    # print( "LEVA", slope)

                elif x1 > 960 // 2 and x2 > 960 // 2:
                    right_lines_x1.append(int(x1))
                    right_lines_x2.append(int(x2))
                    right_lines_y1.append(int(y1))
                    right_lines_y2.append(int(y2))

    output_left_lines = []
    output_right_lines = []

    Left = False
    Right = False

    if left_lines_x1 and left_lines_x2:
        Left = True
        min_index_L = left_lines_x1.index(min(left_lines_x1))  
        left_line_x1 = left_lines_x1[min_index_L]  
        left_line_y1 = left_lines_y1[min_index_L]  

        max_index_L = left_lines_x2.index(max(left_lines_x2))
        left_line_x2 = left_lines_x2[max_index_L]
        left_line_y2 = left_lines_y2[max_index_L]

        # left_line_x2 = 960 // 2
        # left_line_y2 = 180

        cv2.line(frame, (left_line_x1, left_line_y1), (left_line_x2, left_line_y2), (0, 0, 255), 10)
        output_left_lines.append((left_line_x1, left_line_x2, left_line_y1, left_line_y2))

    if right_lines_x1 and right_lines_x2:
        Right = True
        max_index_R = right_lines_x2.index(max(right_lines_x2))
        right_line_x2 = right_lines_x2[max_index_R]
        right_line_y2 = right_lines_y2[max_index_R]

        min_index_L = right_lines_x1.index(min(right_lines_x1))
        right_line_x1 = right_lines_x1[min_index_L]
        right_line_y1 = right_lines_y1[min_index_L]

        # right_line_x1 = 960 // 2
        # right_line_y1 = 180

        cv2.line(frame, (right_line_x2, right_line_y2), (right_line_x1, right_line_y1), (0, 0, 255), 10)
        output_right_lines.append((right_line_x1, right_line_x2, right_line_y1, right_line_y2))
        

    return output_left_lines, output_right_lines, DESNO, LEVO       

def draw_ROI( frame, region ):

    overlay = frame.copy()  
    cv2.fillPoly(overlay, [region], (0, 255, 0))  
    
    alpha = 0.3  
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)  

    return frame

def get_lane_center( lines, frame, prev_lane_center ):

    left_lines, right_lines, DESNO, LEVO = draw_all_lines( frame, lines )

    
    frame_width = frame.shape[1]

    left_ind = False
    right_ind = False

    if left_lines:
        left_x = int( np.mean([x for x1, x2, _, _ in left_lines for x in (x1, x2)]) )
        left_ind = True 

    if right_lines:
        right_x = int( np.mean([x for x1, x2, _, _ in right_lines for x in (x1, x2)]) )
        right_ind = True
   
    if DESNO:
        lane_center = frame_width // 2 - 120
    elif left_ind and right_ind:
        lane_center = ( left_x + right_x ) // 2
    elif not left_ind and right_ind:
        lane_center = frame_width // 2 - 70
    elif left_ind and not right_ind:
        lane_center = frame_width // 2 + 70
    else:
        lane_center = prev_lane_center

    lane_center = int(0.7 * lane_center + (1 - 0.7) * prev_lane_center)

    cv2.line( frame, (lane_center, frame.shape[0] // 2), (lane_center, frame.shape[0] ), (0, 255, 0 ), 4 ) 

    

    return left_lines, right_lines, lane_center
