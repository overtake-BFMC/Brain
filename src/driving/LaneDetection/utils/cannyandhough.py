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

    hsv = cv2.cvtColor( frame, cv2.COLOR_BGR2HSV )

    #opseg bele boje u HSV prostoru
    lower_white = np.array( [0, 0, 180], dtype=np.uint8 )  
    upper_white = np.array( [255, 50, 255], dtype=np.uint8 ) 

    mask = cv2.inRange( hsv, lower_white, upper_white ) #maska za bele piksele
    masked_image = cv2.bitwise_and( frame, frame, mask=mask )

    gray = cv2.cvtColor( masked_image, cv2.COLOR_BGR2GRAY )
    gray = cv2.GaussianBlur(frame, (5, 5), 0)

    edges = cv2.Canny( gray, 50, 200 )

    height = edges.shape[0] #1484
    width = edges.shape[1] #3280

    #region = np.array([[(0, 540), (300, 270), (800, 270), (960, 540)]], dtype=np.int32)    
    # region = np.array( [[ (0 , 540), ( 370, 100 ), ( 700 ,100), ( 960, 540)]])
    # region = np.array( [ [ ( width // 8, height ), ( 2 * width // 5, height // 3 ), ( 4 * width // 7 ,height // 3), (7 * width // 8  ,height)]])
    region = np.array( [ [ ( 80, 540 ), ( 360, 180 ), ( 600 , 180 ), ( 820  ,540)]])

    edges = region_of_interest( edges, region )

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold = 50, minLineLength=30, maxLineGap=90)

    draw_all_lines( frame, lines )
    frame = draw_ROI( frame, region )

    return edges, frame, lines


def draw_parabola( frame, points ):

       points = np.array( points )
       x_values, y_values = points[:, 0], points[:, 1]

       coeffs = np.polyfit( x_values, y_values, 2 )
       a, b, c = coeffs

       x_curve = np.linspace( min(x_values), max(x_values), 100 )
       y_curve = a * x_curve**2 + b * x_curve + c

       for i in range(len(x_curve) - 1 ):
           cv2.line( frame, 
               ( int( x_curve[i] ), int( y_curve[i] ) ), 
               ( int( x_curve[i+1] ), int( y_curve[i+1] ) ), 
               (0, 0, 255), 10)
           

def draw_all_lines( frame, lines ):

    left_lines_x1, left_lines_x2, right_lines_x1, right_lines_x2 = [], [], [], []
    left_lines_y1, left_lines_y2, right_lines_y1, right_lines_y2 = [], [], [], []

    points = []
    output_left_lines = []
    output_right_lines = []


    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                continue
            
            slope = abs( (y2 - y1 ) / ( x2 - x1) )

            if slope < 1:
            
                points.append( ( int(x1), int(y1), int( x2), int(y2) ) )
                
            #prava npr
            if slope >= 1:
            
                if x1 < 960 // 2:  

                    left_lines_x1.append( int(x1) )
                    left_lines_x2.append( int(x2) )
                    left_lines_y1.append( int( y1) )
                    left_lines_y2.append( int( y2 ) )
                
                else:
                    
                    right_lines_x1.append( int(x1) )
                    right_lines_x2.append( int(x2) )
                    right_lines_y1.append( int( y1) )
                    right_lines_y2.append( int( y2))


       
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

    return output_left_lines, output_right_lines
       
def draw_ROI( frame, region ):

    overlay = frame.copy()  
    cv2.fillPoly(overlay, [region], (0, 255, 0))  
    
    alpha = 0.3  
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)  

    return frame


def get_lane_center( lines, frame ):

    left_lines, right_lines = draw_all_lines( frame, lines )
    
    frame_width = frame.shape[1]

    if left_lines:
        left_x = int( np.mean([x for x1, x2, _, _ in left_lines for x in (x1, x2)]) )
    else:
        left_x = 0  

    if right_lines:
        right_x = int( np.mean([x for x1, x2, _, _ in right_lines for x in (x1, x2)]) )
    else:
        right_x = frame_width  

    lane_center = ( left_x + right_x ) // 2

    cv2.line( frame, (lane_center, frame.shape[0] // 2), (lane_center, frame.shape[0] ), (0, 255, 0 ), 4 ) 

    return left_lines, right_lines, lane_center

