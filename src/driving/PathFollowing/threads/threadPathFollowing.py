from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    startRun,
    SteerMotor,
    SpeedMotor,
    ImuData
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.driving.PathFollowing.utils.nodes import nodes as nodesData
from src.driving.PathFollowing.utils.vehicleState import vehicleState

import numpy as np
import matplotlib.pyplot as plt
import time
import ast

class threadPathFollowing(ThreadWithStop):
    """This thread handles pathFollowing.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, waypoints, yaw = 0, speed = 0, lookAheadDistance = 1, dt = 0.05, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.nodes = nodesData
        self.waypoints = waypoints
        self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]

        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)

        self.vehicle = vehicleState(speed, self.nodes[waypoints[0]][0], self.nodes[waypoints[0]][1], np.deg2rad(yaw))

        self.lookAheadDistance = lookAheadDistance #*speed
        self.timeStep = dt
        
        self.speed = speed

        self.isDriving = False

        self.last_sent_time = time.time()

        self.subscribe()
        super(threadPathFollowing, self).__init__()

    def run(self):
        while self._running:
            startRun = self.startRunSubscriber.receive()
            if startRun is not None:
                if startRun == 'false':
                    self.isDriving = False
                else:
                    self.isDriving = True
                print("Start Test Run: ", startRun)
                if self.isDriving:
                    #Repeated testing, delete for prod
                    self.vehicle.setPosition(self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(0))
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
        return np.sqrt((targetX - self.vehicle.x)**2 + (targetY - self.vehicle.y)**2)

    def calculateAngleToTarget(self, targetX, targetY):
        return np.arctan2(targetY - self.vehicle.y, targetX - self.vehicle.x)
    
    def calculateAlphaSteer(self, angleToTarget):
        return angleToTarget - self.vehicle.yaw
    
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
        
        # https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
        beta = np.arctan2(self.vehicle.centerOfMass * np.tan(self.vehicle.steeringAngle), self.vehicle.wheelbase)

        self.vehicle.x = self.vehicle.x + self.vehicle.speed * np.cos(self.vehicle.yaw + beta) * elapsedTime
        self.vehicle.y = self.vehicle.y + self.vehicle.speed * np.sin(self.vehicle.yaw + beta) * elapsedTime
        if self.vehicle.steeringAngle != 0:
            rearWheelsDistanceFromICR = self.vehicle.wheelbase / np.tan(self.vehicle.steeringAngle)
            
            CenterOfMassDistanceFromICR = rearWheelsDistanceFromICR / np.cos(beta)
            self.vehicle.yaw = self.vehicle.yaw + self.vehicle.speed / CenterOfMassDistanceFromICR * elapsedTime

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

        startTime = time.time()

        self.path = self.createAdditionalNodesInThePath(times = 2)
        self.path = self.pathSmoother(weightData=0.95)

        path_x, path_y = [], []
        lastPathPointId = 1

        self.speedMotorSender.send(str(np.clip(self.vehicle.speed*10, -500, 500)))
        targetX, targetY = self.findLookAheadPoint(lastPathPointId)

        startX = self.vehicle.x
        starty = self.vehicle.y
        velocity = 0

        while targetX and targetY:
            self.speedMotorSender.send(str(np.clip(self.vehicle.speed*10, -500, 500)))
            while(not velocity):
                IMUData = self.IMUDataSubscriber.receive()
                if IMUData is not None:
                    self.speedMotorSender.send(str(np.clip(self.vehicle.speed*10, -500, 500)))

                    FormattedIMUData = ast.literal_eval(IMUData)
                    velocityX = abs(float(FormattedIMUData["accelx"]))
                    velocityY = abs(float(FormattedIMUData["accely"]))
                    velocity = int(np.sqrt(velocityX**2 + velocityY**2) * 100)
                    
                
            # self.vehicle.setSpeed(self.speed + (self.vehicle.speed - velocity))

            # IMUData = self.IMUDataSubscriber.receive()
            # if IMUData is not None:
            #     self.speedMotorSender.send(str(np.clip(self.vehicle.speed*10, -500, 500)))

            #     FormattedIMUData = ast.literal_eval(IMUData)
            #     velocityX = abs(float(FormattedIMUData["accelx"]))
            #     velocityY = abs(float(FormattedIMUData["accely"]))
            #     velocity = int(np.sqrt(velocityX**2 + velocityY**2) * 100)
                #print(f"{velocityX} , {velocityY}")

            #print(f"Vel: {velocityX} , {velocityY}")
            #print(f"{self.vehicle.speed} , {velocity}")
            # print(f"{targetX} {targetY} {np.rad2deg(self.vehicle.steeringAngle)}")

            if self.calculateDistanceToTarget(startX, starty) >= 230:
                self.vehicle.setSpeed(0)
                self.speedMotorSender.send(str(np.clip(self.vehicle.speed*10, -500, 500)))
                print(f"time: {time.time()} stop")

                break

            if self.calculateDistanceToTarget(self.path[lastPathPointId][0], self.path[lastPathPointId][1]) < 10:
                lastPathPointId += 1

            distanceToTarget = self.calculateDistanceToTarget(targetX, targetY)
            angleToTarget = self.calculateAngleToTarget(targetX, targetY)

            alpha = self.calculateAlphaSteer(angleToTarget)
            alpha = np.clip(alpha, -0.4363, 0.4363)

            # self.vehicle.steeringAngle = np.arctan2(2 * self.vehicle.wheelbase * np.sin(alpha), distanceToTarget)

            self.vehicle.steeringAngle = alpha
            self.vehicle.steeringAngle = np.clip(self.vehicle.steeringAngle, -0.4363, 0.4363)

            elapsedTime = time.time() - startTime
            self.kinematicBicycleModel(elapsedTime)
            startTime = time.time()

            self.steerMotorSender.send(str(round(np.rad2deg(np.clip(self.vehicle.steeringAngle, -0.4363, 0.4363)))*10))
            
            targetX, targetY = self.findLookAheadPoint(lastPathPointId)
                            
            path_x.append(self.vehicle.x)
            path_y.append(self.vehicle.y)

            # time.sleep(self.timeStep)

            start_time = time.perf_counter()
            while time.perf_counter() - start_time < self.timeStep:  # Waits for 2 seconds
                pass

            self.last_sent_time = time.time()

        self.steerMotorSender.send(str(0))
        time.sleep(0.3)
        self.speedMotorSender.send(str(0))
        
        plt.plot(path_x, path_y, label="Robot Path")
        plt.scatter(*zip(*self.nodes.values()), color="red", label="Waypoints")  # Mark the nodes/waypoints
        plt.scatter(*zip(*self.path), color="yellow", label="Path")  # Mark the nodes/waypoints
        plt.title("Path Following with Pure Pursuit")
        plt.xlabel("X (cm)")
        plt.ylabel("Y (cm)")
        plt.gca().invert_yaxis()

        plt.legend()
        plt.grid(True)
        #plt.show()
        plt.savefig("path_following.pdf")  

        return path_x, path_y, self.path

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.startRunSubscriber = messageHandlerSubscriber(self.queuesList, startRun, "lastOnly", True)
        self.IMUDataSubscriber = messageHandlerSubscriber(self.queuesList, ImuData, "lastOnly", True)
