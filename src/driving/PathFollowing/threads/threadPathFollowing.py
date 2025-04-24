from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    startRun,
    SteerMotor,
    SpeedMotor,
    ImuData,
    ShMemConfig,
    ShMemResponse,
    SelectTrackNo,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.driving.PathFollowing.utils.nodes import nodes as nodesData
from src.driving.PathFollowing.utils.vehicleState import vehicleState

import numpy as np
import matplotlib.pyplot as plt
import time
import ast
from src.utils.logger.setupLogger import LoggerConfigs, configLogger

class threadPathFollowing(ThreadWithStop):
    """This thread handles pathFollowing.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, loggingQueue, lookAheadDistance = 1, dt = 0.05, debugging = False):
        self.queuesList = queueList
        self.loggingQueue = loggingQueue
        self.debugging = debugging
        self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)

        self.nodes = nodesData
        # speed = 20
        # self.yaw = 0

        # self.waypoints = []
        # self.waypoints.extend(range(1, 9))

        # # self.path = []
        # self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
        # self.path = self.createAdditionalNodesInThePath(times = 2)
        # self.path = self.pathSmoother(weightData=0.85)

        self.waypoints = []
        self.path = []

        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.ShMemConfigSender = messageHandlerSender(self.queuesList, ShMemConfig)

        self.lookAheadDistance = lookAheadDistance #*speed
        self.timeStep = dt
        
        self.isDriving = False
        self.chosenTrack = -1

        self.last_sent_time = time.time()

        self.subscribe()
        time.sleep(1)

        self.shMemMsgOwner = "threadPathFollowing"
        self.init_vehicleState()

        # self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(self.yaw))


        super(threadPathFollowing, self).__init__()

    def init_vehicleState(self):
        self.ShMemConfigSender.send({"action": "getVehicleState","owner": self.shMemMsgOwner})

        isVehicleStateConfigured = False
        while not isVehicleStateConfigured:
            getVehicleStateResp = self.shMemResponseSubscriber.receive()
            if getVehicleStateResp is not None:
                if getVehicleStateResp["status"] == 3 and getVehicleStateResp["dest"] == self.shMemMsgOwner:
                    self.vehicle = getVehicleStateResp["vehicleState"]
                    isVehicleStateConfigured = True
                    #print("PathFollowing VehicleState Initialized!")
                    self.logger.info("PathFollowing VehicleState Initialized!")

    def run(self):
        self.chosenTrack = -1

        speed = 20
        yaw = 0

        while self._running:
            trackChoice = self.selectTrackSubscriber.receive()
            #trackChoice = 0
            
            if trackChoice is not None:
                self.chosenTrack = trackChoice
                
                if self.chosenTrack == 0:
                    #ROUNDABOUT RUN

                    speed = 20
                    yaw = 0

                    if self.waypoints:
                        self.waypoints.clear() 

                    self.waypoints.extend(range(1, 7))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.path = self.createAdditionalNodesInThePath(times = 2)
                    #self.path = self.pathSmoother(weightData=0.85)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))


                elif self.chosenTrack == 1:
                    #HIGHWAY RUN

                    speed = 20
                    yaw = 90

                    if self.waypoints:
                        self.waypoints.clear() 

                    self.waypoints.extend(range(10, 26))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.path = self.createAdditionalNodesInThePath(times = 2)
                    #self.path = self.pathSmoother(weightData=0.85)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))

                elif self.chosenTrack == 2:
                    #INTERSECTION RUN
                    
                    speed = 20
                    yaw = 180

                    if self.waypoints:
                        self.waypoints.clear() 

                    self.waypoints.extend(range(30, 51))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.path = self.createAdditionalNodesInThePath(times = 2)
                    #self.path = self.pathSmoother(weightData=0.85)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))

            startRun = self.startRunSubscriber.receive()
            if startRun is not None:
                if startRun == 'false':
                    self.isDriving = False
                else:
                    self.isDriving = True
                print("Start Test Run: ", startRun)
                if self.isDriving and self.chosenTrack != -1:
                    #Repeated testing, delete for prod
                    self.vehicle.setPosition(self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))
                    self.purePursuit()

    def createAdditionalNodesInThePath(self, times = 1):
        
        oldPath = self.path
        for _ in range(0, times):
            newPath = []

            for i in range(0, len(oldPath) - 1):
                id1 = i
                id2 = i + 1

                newX = (oldPath[id1][0] + oldPath[id2][0]) / 2
                newY = (oldPath[id1][1] + oldPath[id2][1]) / 2

                newPath.append(oldPath[id1])
                newPath.append((newX, newY))

            newPath.append(oldPath[id2])
            oldPath = newPath

        return newPath

    def calculateDistanceToTarget(self, targetX, targetY):

        vehicleX, vehicleY, _ = self.vehicle.getPosition()
        return np.sqrt((targetX - vehicleX)**2 + (targetY - vehicleY)**2)

    def calculateAngleToTarget(self, targetX, targetY):

        vehicleX, vehicleY, _ = self.vehicle.getPosition()
        return np.arctan2(targetY - vehicleY, targetX - vehicleX)
    
    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def calculateAlphaSteer(self, angleToTarget):

        _, _, vehicleYaw = self.vehicle.getPosition()
        return self.normalize_angle(angleToTarget - vehicleYaw)
    
    def pathSmoother(self, weightData):
        
        weightSmooth = 1 - weightData
        tolerance = 0.001

        newPath = np.copy(self.path)
        change = tolerance

        while change >= tolerance:
            change = 0.0
            
            for i in range(1, len(self.path) - 1):
                
                aux = newPath[i]
                
                new_x = newPath[i][0] + weightData * (self.path[i][0] - newPath[i][0]) + weightSmooth * (newPath[i - 1][0] + newPath[i + 1][0] - (2.0 * newPath[i][0]))
                new_y = newPath[i][1] + weightData * (self.path[i][1] - newPath[i][1]) + weightSmooth * (newPath[i - 1][1] + newPath[i + 1][1] - (2.0 * newPath[i][1]))
                newPath[i] = (new_x, new_y)
                
                change += abs(aux[0] - new_x) + abs(aux[1] - new_y)

        return newPath

    def kinematicBicycleModel(self, elapsedTime):
        
        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
        vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase = self.vehicle.getSpeedSteerWheelbase()

        # https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
        rotationRate = vehicleSpeed * np.tan(vehicleSteeringAngle) / vehicleWheelbase
        vehicleYaw = vehicleYaw + rotationRate * elapsedTime
        
        vehicleX = vehicleX + vehicleSpeed * np.cos(vehicleYaw) * elapsedTime
        vehicleY = vehicleY + vehicleSpeed * np.sin(vehicleYaw) * elapsedTime

        self.vehicle.setPosition(vehicleX, vehicleY, vehicleYaw)
    
    def findLookAheadPoint(self, lastPathPointId):
        
        closestPoint = self.path[-1]
        closestDistance = float('inf')

        for i in range(lastPathPointId, len(self.path)):
            distance = self.calculateDistanceToTarget(self.path[i][0], self.path[i][1])

            if distance >= self.lookAheadDistance and distance < closestDistance:
                closestPoint = self.path[i]
                closestDistance = distance
        
        if lastPathPointId == len(self.path) - 1:
            return (None, None)
            
        return closestPoint

    def purePursuit(self):

        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
        vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase = self.vehicle.getSpeedSteerWheelbase()

        startTime = time.time()

        path_x, path_y = [], []
        lastPathPointId = 1

        self.speedMotorSender.send(str(np.clip(vehicleSpeed*10, -500, 500)))
        targetX, targetY = self.findLookAheadPoint(lastPathPointId)

        startX = vehicleX
        starty = vehicleY
        velocity = 0

        while targetX and targetY:
            print(f"{targetX}  {targetY}")

            vehicleSpeed = self.vehicle.getSpeed()
            
            self.speedMotorSender.send(str(np.clip(vehicleSpeed*10, -500, 500)))
            # while(not velocity):
            #     IMUData = self.IMUDataSubscriber.receive()
            #     if IMUData is not None:
            #         self.speedMotorSender.send(str(np.clip(vehicleSpeed*10, -500, 500)))

            #         FormattedIMUData = ast.literal_eval(IMUData)
            #         velocityX = abs(float(FormattedIMUData["accelx"]))
            #         velocityY = abs(float(FormattedIMUData["accely"]))
            #         velocity = int(np.sqrt(velocityX**2 + velocityY**2) * 100)
                    

            if self.calculateDistanceToTarget(self.path[lastPathPointId][0], self.path[lastPathPointId][1]) < vehicleWheelbase:
                lastPathPointId += 1

            distanceToTarget = self.calculateDistanceToTarget(targetX, targetY)
            angleToTarget = self.calculateAngleToTarget(targetX, targetY)

            alpha = self.calculateAlphaSteer(angleToTarget)
            alpha = np.clip(alpha, -0.4363, 0.4363)

            vehicleSteeringAngle = np.arctan2(2 * vehicleWheelbase * np.sin(alpha), distanceToTarget)
            vehicleSteeringAngle = np.clip(vehicleSteeringAngle, -0.4363, 0.4363)
            self.vehicle.setSteeringAngle(vehicleSteeringAngle)

            elapsedTime = time.time() - startTime
            self.kinematicBicycleModel(elapsedTime)
            startTime = time.time()
 
            self.steerMotorSender.send(str(round(np.rad2deg(np.clip(-vehicleSteeringAngle, -0.4363, 0.4363)))*10))
            
            targetX, targetY = self.findLookAheadPoint(lastPathPointId)
                            
            vehicleX, vehicleY, _ = self.vehicle.getPosition()
            path_x.append(vehicleX)
            path_y.append(vehicleY)

            # time.sleep(self.timeStep)

            start_time = time.perf_counter()
            while time.perf_counter() - start_time < self.timeStep:  # Waits for 2 seconds
                pass
                
            self.last_sent_time = time.time()

        self.steerMotorSender.send(str(0))
        time.sleep(0.3)
        self.speedMotorSender.send(str(0))
        
        # plt.plot(path_x, path_y, label="Robot Path")
        # plt.scatter(*zip(*self.nodes.values()), color="red", label="Waypoints")  # Mark the nodes/waypoints
        # plt.scatter(*zip(*self.path), color="yellow", label="Path")  # Mark the nodes/waypoints
        # plt.title("Path Following with Pure Pursuit")
        # plt.xlabel("X (cm)")
        # plt.ylabel("Y (cm)")
        # plt.gca().invert_yaxis()

        # plt.legend()
        # plt.grid(True)
        # #plt.show()
        # plt.savefig("path_following.pdf")  

        return path_x, path_y, self.path

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.startRunSubscriber = messageHandlerSubscriber(self.queuesList, startRun, "lastOnly", True)
        self.IMUDataSubscriber = messageHandlerSubscriber(self.queuesList, ImuData, "lastOnly", True)
        self.shMemResponseSubscriber = messageHandlerSubscriber(self.queuesList, ShMemResponse, "lastOnly", True)
        self.selectTrackSubscriber = messageHandlerSubscriber(self.queuesList, SelectTrackNo, "lastOnly", True)
