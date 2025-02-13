import cv2
import numpy as np


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

    edges = cv2.Canny( gray, 50, 200 )

    height = edges.shape[0] #540
    width = edges.shape[1] #960

    #region = np.array([[(0, height), (width // 2, height // 2), (width, height)]], dtype=np.int32)
    #region = np.array([[(0, 540), (100, 270), (860, 270), (540, 960)]], dtype=np.int32)    
    #region = np.array([[(0, 540), (300, 270), (560, 270), (540,960)]], dtype=np.int32)        

    region = np.array([[(0, 540), (300, 270), (800, 270), (960, 540)]], dtype=np.int32)    
    #region = np.array([[(0, 540), (430, 200), (650, 200), (960, 540)]], dtype=np.int32)    

    edges = region_of_interest( edges, region )

    #lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold = 68, minLineLength=15, maxLineGap=250)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold = 75, minLineLength=50, maxLineGap=250)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                continue

            slope = abs( (y2 - y1 ) / ( x2 - x1) )

            if slope > 0.5:
                cv2.line( frame, (x1, y1), (x2, y2), (130, 15, 250), 5 )

    #crtanje trapeza
    overlay = frame.copy()
    cv2.fillPoly( overlay, [region], (0, 255, 0 ) )

    frame = cv2.addWeighted( overlay, 0.3, frame, 0.7, 0)

    return edges, frame, lines


def get_lane_center( lines, frame ):

    frame_width = frame.shape[1]
    left_lines = []
    right_lines = []

    if lines is not None:
        for line in lines:
            
            x1, y1, x2, y2 = line[0]

            if x2 - x1 != 0:
                slope = ( y2 - y1 ) / ( x2 - x1 ) 
            else:
                slope = 0

            if slope < 0:
                left_lines.append( ( x1, y1, x2, y2) )
            else:
                right_lines.append( (x1, y1, x2, y2 ) )


    if left_lines:
        left_x = int(np.mean([x for x1, _, x2, _ in left_lines for x in (x1, x2)]))
    else:
        left_x = 0  

    if right_lines:
        right_x = int(np.mean([x for x1, _, x2, _ in right_lines for x in (x1, x2)]))
    else:
        right_x = frame_width  

    lane_center = ( left_x + right_x ) // 2
   # print("lane center ", lane_center)
  #  print( "left x", left_x )
  #  print( "right_x", right_x )

    bias = 50 #ovde mogu dodati bias ako uvek bezi u jednu stranu
    #lane_center += bias

    cv2.line( frame, (lane_center, frame.shape[0] // 2), (lane_center, frame.shape[0] ), (0, 255, 0 ), 4 )  # Zelena linija

    return lane_center
