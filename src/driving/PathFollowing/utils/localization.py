import math

mappedStopLines = [
    (373.0, 260.0),
    (416.0, 295.0),
    (577.0, 115.0),
    (377.0, 80.0),
    (217.0, 115.0),
    (174.0, 80.0),
]

def isMappedStopLine( detected, treshold = 1): 
    for mapped in mappedStopLines:
        dx = mapped[0] - detected[0]
        dy = mapped[1] - detected[1]

        if math.hypot(dx, dy) <= treshold:
            return mapped
        
def detectedStopLines( mappedStopLines):
    pass
