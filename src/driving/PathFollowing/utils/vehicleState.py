from enum import Enum

class stateSignalType(Enum):
        APROACHING_INTERSECTION = 0
        IN_INTERSECION = 1
        INTERSECTION_LEFT = 2
        INTERSECTION_RIGHT = 3
        APROACHING_ROADBLOCK = 4
        ROADBLOCK_MANEUVER = 5

class vehicleState:

    wheelbase = 26
    #centerOfMass = 16 #from the rear wheels
    steeringAngle = 0
    speed = 0
    
    x = 0
    y = 0

    yaw = 0

    #turnSignal = False
    #aproachingIntersection = False
    vehicleStateSignals = [False] * 10

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
    
    def getWheelbase(self):
        return self.wheelbase
    
    def getSteeringAngle(self):
        return self.steeringAngle

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
    
    # def setTurnSignal(self, signalState):
    #     self.turnSignal = signalState

    # def getTurnSignal(self):
    #     return self.turnSignal

    def setStateSignal(self, signalType: stateSignalType, signalState):
        self.vehicleStateSignals[signalType.value] = signalState

    def getStateSignal(self, signalType: stateSignalType):
        return self.vehicleStateSignals[signalType.value]
    
    def resetStateFlags(self):
        self.vehicleStateSignals = [False] * 5