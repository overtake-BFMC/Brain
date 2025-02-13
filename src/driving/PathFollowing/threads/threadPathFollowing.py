from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    startRun,
    SteerMotor,
    SpeedMotor
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.driving.PathFollowing.utils.nodes import nodes as nodesData
from src.driving.PathFollowing.utils.vehicleState import vehicleState

import numpy as np
import time

class threadPathFollowing(ThreadWithStop):
    """This thread handles pathFollowing.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, wayPoints, theta = 0, speed = 0, lookAheadDistance = 1, dt = 0.05, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.nodes = nodesData
        self.wayPoints = wayPoints

        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)

        self.vehicle = vehicleState(speed, self.nodes[wayPoints[0]][0], self.nodes[wayPoints[0]][1], np.deg2rad(theta))

        self.lookAheadDistance = lookAheadDistance
        self.timeStep = dt
        
        self.speed = speed

        self.last_sent_time = time.time()

        self.subscribe()
        super(threadPathFollowing, self).__init__()

    def run(self):
        while self._running:
            startRun = self.startRunSubscriber.receive()
            if startRun is not None:
                self.purePursuit()

    def calculateDistanceToTarget(self, targetX, targetY):
        return np.sqrt((targetX - self.vehicle.x)**2 + (targetY - self.vehicle.y)**2)

    def calculateAngleToTarget(self, targetX, targetY):
        return np.arctan2(targetY - self.vehicle.y, targetX - self.vehicle.x)
    
    def calculateSteeringAngle(self, angleToTarget):
        return angleToTarget - self.vehicle.theta
    
    def kinematicBicycleModel(self):
        
        # https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
        rearWheelsDistanceFromICR = self.vehicle.wheelbase / np.tan(self.vehicle.steeringAngle)
        beta = np.arctan2(self.vehicle.centerOfMass * np.tan(self.vehicle.steeringAngle), self.vehicle.wheelbase)

        CenterOfMassDistanceFromICR = rearWheelsDistanceFromICR / np.cos(beta)

        self.vehicle.x = self.vehicle.x + self.vehicle.speed * np.cos(self.vehicle.theta + beta) * self.timeStep
        self.vehicle.y = self.vehicle.y + self.vehicle.speed * np.sin(self.vehicle.theta + beta) * self.timeStep
        self.vehicle.theta = self.vehicle.theta + self.vehicle.speed / CenterOfMassDistanceFromICR * self.timeStep

    def purePursuit(self):
        path_x, path_y = [], []
        #self.speedMotorSender.send(str(300))
        for i in range(1, len(self.wayPoints)):
            targetX, targetY = self.nodes[self.wayPoints[i]]
            self.speedMotorSender.send(str(300))
            distanceToTarget = self.calculateDistanceToTarget(targetX, targetY)
            while distanceToTarget > 10:
                
                print(distanceToTarget)

                distanceToTarget = self.calculateDistanceToTarget(targetX, targetY)
                angleToTarget = self.calculateAngleToTarget(targetX, targetY)
                
                #self.vehicle.steeringAngle = np.arctan2(2 * self.lookAheadDistance * np.sin(angleToTarget - self.vehicle.theta), distanceToTarget)
                self.vehicle.steeringAngle = self.calculateSteeringAngle(angleToTarget) 
                # PRETVORI U DEGREES KAD SALJES NUCLEU np.rad2deg(self.vehicle.steeringAngle)
                self.steerMotorSender.send(str(-round(np.rad2deg(self.vehicle.steeringAngle))*10))
                self.kinematicBicycleModel()
                
                path_x.append(self.vehicle.x)
                path_y.append(self.vehicle.y)

                #time.sleep(self.timeStep)
                #aktiviraj ovo kad runujes na nucleo
                time_to_wait = self.timeStep - (time.time() - self.last_sent_time)
                if time_to_wait > 0:
                    time.sleep(time_to_wait)

                self.last_sent_time = time.time()

        self.steerMotorSender.send(str(0))
        time.sleep(time_to_wait)
        self.speedMotorSender.send(str(0))                
                
        return path_x, path_y

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.startRunSubscriber = messageHandlerSubscriber(self.queuesList, startRun, "lastOnly", True)
