
class vehicleState:

    wheelbase = 26
    centerOfMass = 16 #from the rear wheels
    steeringAngle = 0
    speed = 0
    
    x = 0
    y = 0

    theta = 0

    def __init__(self, speed = 0, x = 0, y = 0, theta = 0):
        self.speed = speed
        
        self.x = x
        self.y = y
        self.theta = theta
    
    def updateVehicleState(self, speed, x, y, theta):
        self.speed = speed
        
        self.x = x
        self.y = y
        self.theta = theta
    
    def setSpeed(self, speed):
        self.speed = speed

    def setPosition(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
