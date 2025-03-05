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

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold = 30, minLineLength=10, maxLineGap=100)

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


def draw_all_lines__( frame, lines ):

    left_lines_x1, left_lines_x2, right_lines_x1, right_lines_x2 = [], [], [], []
    left_lines_y1, left_lines_y2, right_lines_y1, right_lines_y2 = [], [], [], []

    left_curve_x1, left_curve_x2, left_curve_y1, left_curve_y2 = [], [], [], []
    right_curve_x1output_left_lines, right_curve_x2, right_curve_y1, right_curve_y2 = [], [], [], []

    output_left_lines, output_right_lines = [], []

    points = []
    l_points, r_points = [], []

    r_lines, l_lines = [], []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                continue
            
            slope = abs( (y2 - y1 ) / ( x2 - x1) )

            if slope < 0:

                if x1 > 960 // 2:
                    r_lines.append(x1, y1)
                    r_lines.append(x2, y2)

                    cv2.line( frame, ( x1, y1), ( x2, y2 ), ( 0,255,0),  5)
            else:
                if x1 < 960 // 2:
                    l_lines.append((x1, y1))
                    l_lines.append((x2, y2))

                    cv2.line( frame, ( x1, y1), ( x2, y2 ), ( 0,255,0),  5)
        


        # if len(r_lines) > 0:

        #     print( r_lines )

        #     r_lines = np.array(r_lines)  # Konverzija u NumPy niz
        #     max_right_x, max_right_y = r_lines.max(axis=0)
        #     min_right_x, min_right_y = r_lines.min(axis=0)
        
        # if len(l_lines) > 0:
        #     l_lines = np.array(l_lines)
        #     max_left_x, max_left_y = l_lines.max(axis=0)
        #     min_left_x, min_left_y = l_lines.min(axis=0)
        
        # # Provera pre korišćenja np.polyfit
        # if len(r_lines) > 0 and r_lines.shape[0] > 2:
        #     right_curve = np.poly1d(np.polyfit(r_lines[:,1], r_lines[:,0], 2))
        #     max_right_x = int(right_curve(frame.shape[0]))
        #     min_right_x = int(right_curve(min_right_y))

        # if len(l_lines) > 0 and l_lines.shape[0] > 2:
        #     left_curve = np.poly1d(np.polyfit(l_lines[:,1], l_lines[:,0], 2))
        #     min_left_x = int(left_curve(frame.shape[0]))

        # min_y = min(min_left_y, min_right_y)

        # # Crtanje linija
        # if len(r_lines) > 0:
        #     r1 = (min_right_x, min_y)
        #     r2 = (max_right_x, img.shape[0])
        #     print('Right points r1 and r2,', r1, r2)
        #     cv2.line(frame, r1, r2, (0,255,0), 5)

        # if len(l_lines) > 0:
        #     l1 = (max_left_x, min_y)
        #     l2 = (min_left_x, img.shape[0])
        #     print('Left points l1 and l2,', l1, l2)
        #     cv2.line(frame, l1, l2, (0,255,0), 5)

        
    return r_lines, l_lines




def draw_all_lines( frame, lines ):

    left_lines_x1, left_lines_x2, right_lines_x1, right_lines_x2 = [], [], [], []
    left_lines_y1, left_lines_y2, right_lines_y1, right_lines_y2 = [], [], [], []

    left_curve_x1, left_curve_x2, left_curve_y1, left_curve_y2 = [], [], [], []
    right_curve_x1output_left_lines, right_curve_x2, right_curve_y1, right_curve_y2 = [], [], [], []

    output_left_lines, output_right_lines = [], []

    points = []
    l_points, r_points = [], []


    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                continue
            
            slope = abs( (y2 - y1 ) / ( x2 - x1) )

            if slope < 1:

                if x1 > 960 // 2:

                    r_points.append([x1, y1])
                    r_points.append([x2, y2])
                else:
                    l_points.append([x1, y1])
                    l_points.append([x1,y1])
                
            #prava npr
            if slope >= 1:
            
                if x1 < 960 // 2 and x2 < 960 // 2:  

                    left_lines_x1.append( int(x1)  )
                    left_lines_x2.append( int(x2)  )
                    left_lines_y1.append( int( y1) )
                    left_lines_y2.append( int( y2 ))
                
                elif x1 > 960 // 2 and x2 > 960 // 2: 
                    
                    right_lines_x1.append( int(x1)  ) 
                    right_lines_x2.append( int(x2)  )
                    right_lines_y1.append( int( y1) )
                    right_lines_y2.append( int( y2) )   
            # else:                
            #         cv2.line( frame, ( x1, y1 ), ( x2, y2 ), ( 0, 0, 255 ), 5 )

       
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


def get_lane_center( lines, frame, prev_lane_center ):

    left_lines, right_lines = draw_all_lines( frame, lines )
    
    frame_width = frame.shape[1]

    left_ind = False
    right_ind = False

    if left_lines:
        left_x = int( np.mean([x for x1, x2, _, _ in left_lines for x in (x1, x2)]) )
        left_ind = True
    # else:
    #     left_x = 0    

    if right_lines:
        right_x = int( np.mean([x for x1, x2, _, _ in right_lines for x in (x1, x2)]) )
        right_ind = True
    # else:
    #     right_x = frame_width  

    if left_ind and right_ind:
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
