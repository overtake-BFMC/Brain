
class vehicleState:

    wheelbase = 26
    #centerOfMass = 16 #from the rear wheels
    steeringAngle = 0
    speed = 0
    
    x = 0
    y = 0

    yaw = 0

    def __init__(self, speed = 0, x = 0, y = 0, yaw = 0):
        self.speed = speed
        
        self.x = x
        self.y = y
        self.yaw = yaw
    
    def updateVehicleState(self, speed, x, y, yaw):
        self.speed = speed
        
        self.x = x
        self.y = y
        self.yaw = yaw
    
    def setSpeed(self, speed):
        self.speed = speed

    def getSpeed(self):
        return self.speed

    def setSteeringAngle(self, steer):
        self.steeringAngle = steer

    def setPosition(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def getPosition(self):
        return self.x, self.y, self.yaw

    def getSpeedSteerWheelbase(self):
        return self.speed, self.steeringAngle, self.wheelbase
