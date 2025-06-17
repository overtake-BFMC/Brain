from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    startRun,
    SteerMotor,
    SpeedMotor,
    ImuData,
    ShMemConfig,
    ShMemResponse,
    SelectTrackNo,
    CurrentPos,
    WhiteLine,
    SetCalibrationSpeed,
    SetCalibrationTime,
    startLaneDetection,
    ManualPWMSpeedMotor,
    SetManualPWMSpeed,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.driving.PathFollowing.utils.nodes import nodes as nodesData

import numpy as np
import matplotlib.pyplot as plt

import time
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger
from src.driving.PathFollowing.utils.kalmanFilterLocalization import kalmanFilterLocalization
from src.driving.PathFollowing.utils.localization import detectedStopLines
from src.driving.PathFollowing.utils.vehicleState import stateSignalType

class threadPathFollowing(ThreadWithStop):
    """This thread handles pathFollowing.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logger, lookAheadDistance = 1, dt = 0.05, debugging = False):

        self.queuesList = queueList
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)

        self.nodes = nodesData

        self.waypoints = []
        self.path = []

        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.ShMemConfigSender = messageHandlerSender(self.queuesList, ShMemConfig)
        self.currentPosSender = messageHandlerSender(self.queuesList, CurrentPos)
        self.startLaneDetectionSender = messageHandlerSender(self.queuesList, startLaneDetection)
        self.manualPWMSpeedSender = messageHandlerSender(self.queuesList, ManualPWMSpeedMotor)

        self.lookAheadDistance = lookAheadDistance #*speed
        self.timeStep = dt
        
        self.isDriving = False
        self.chosenTrack = -1

        self.subscribe()
        time.sleep(1)

        self.shMemMsgOwner = "threadPathFollowing"
        self.init_vehicleState()


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
        self.calibrationTime = -1
        self.calibrationSpeed = -1
        self.manualPWMSpeed = 1500

        speed = 20
        yaw = 0

        self.KFLocalization = kalmanFilterLocalization(dt = self.timeStep)

        while self._running:

            trackChoice = self.selectTrackSubscriber.receive()    
            if trackChoice is not None:
                self.chosenTrack = trackChoice
                
                if self.chosenTrack == 0:
                    #ROUNDABOUT RUN

                    speed = 20
                    yaw = 0

                    if self.waypoints:
                        self.waypoints.clear() 

                    #old map
                    self.waypoints.extend(range(198, 231))


                    # self.waypoints.extend(range(3,11))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))


                elif self.chosenTrack == 1:
                    #HIGHWAY RUN

                    speed = 25
                    # yaw = 90

                    #old map
                    yaw = 0

                    if self.waypoints:
                        self.waypoints.clear() 

                    # self.waypoints.extend(range(10, 26))

                    #old map
                    self.waypoints.extend(range(296, 302))
                    self.waypoints.extend(range(303, 307))
                    self.waypoints.extend(range(231, 233))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))

                elif self.chosenTrack == 2:
                    #INTERSECTION RUN
                    
                    speed = 25
                    # yaw = 180

                    #old map right
                    yaw = 0

                    #old map left
                    yaw = 270

                    if self.waypoints:
                        self.waypoints.clear() 

                    # self.waypoints.extend(range(30, 51))

                    #old map right turn
                    # self.waypoints.extend(range(17, 18))
                    # self.waypoints.extend(range(146, 147))
                    # self.waypoints.extend(range(25, 27))
                    # self.waypoints.extend(range(119, 120))

                    #old map left turn
                    self.waypoints.extend(range(118, 119))
                    self.waypoints.extend(range(27, 28))
                    self.waypoints.extend(range(24, 25))

                    

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))
                elif self.chosenTrack == 3:
                    ##CURVE RUN

                    speed = 25
                    yaw = 0

                    if self.waypoints:
                        self.waypoints.clear()

                    #self.waypoints.extend(range(198, 231))
                    #self.waypoints.extend(range(267, 274))

                    #RIGHT TURN
                    self.waypoints.extend(range(51, 72))
                    #LEFT TURN
                    #yaw = 90
                    #self.waypoints.extend(range(62, 67))

                    # self.waypoints.extend(range(83, 90))
                    # self.waypoints.extend(range(90, 110))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))

                self.currentPosSender.send(str(self.vehicle.getPosition()))

            calibrationSpeed = self.calibrationSpeedSubscriber.receive()
            if calibrationSpeed is not None:
                self.calibrationSpeed = calibrationSpeed
            
            calibrationTime = self.calibrationTimeSubscriber.receive()
            if calibrationTime is not None:
                self.calibrationTime = calibrationTime

            manualPWMSpeed = self.manualPWMSpeedSubscriber.receive()
            if manualPWMSpeed is not None:
                self.manualPWMSpeed = manualPWMSpeed

            startRun = self.startRunSubscriber.receive()
            if startRun is not None:
                if startRun == 'false':
                    self.isDriving = False
                else:
                    self.isDriving = True
                print("Start Test Run: ", startRun)
                if self.isDriving and self.chosenTrack in range(10):
                    #Repeated testing, delete for prod
                    self.vehicle.setPosition(self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))
                    self.KFLocalization.resetFilter()
                    self.purePursuit()
                elif self.isDriving and self.chosenTrack == 99:
                    if self.calibrationSpeed != -1 and self.calibrationTime != -1:

                        self.speedMotorSender.send(str(self.calibrationSpeed*10))
                        calibrationStartTime = time.perf_counter()

                        while(time.perf_counter() - calibrationStartTime) < self.calibrationTime:
                            pass

                        self.speedMotorSender.send(str(0))
                        calibrationEndTime = time.perf_counter()

                        elapsed = calibrationEndTime - calibrationStartTime
                        print(f"Calibration Elapsed Time: {elapsed}")
                        print(f"Calibration Speed: {self.calibrationSpeed}")
                    else:
                        print("Missing calibration speed or calibration time")
                elif self.isDriving and self.chosenTrack == 98:
                    if self.manualPWMSpeed != 1500 and self.calibrationTime != -1:

                        print(f"HERE {self.manualPWMSpeed}")

                        self.manualPWMSpeedSender.send(self.manualPWMSpeed)
                        calibrationStartTime = time.perf_counter()

                        while(time.perf_counter() - calibrationStartTime) < self.calibrationTime:
                            pass

                        self.manualPWMSpeedSender.send(1500)
                        calibrationEndTime = time.perf_counter()

                        elapsed = calibrationEndTime - calibrationStartTime
                        print(f"Calibration Elapsed Time: {elapsed}")
                        print(f"Calibration PWM: {self.manualPWMSpeed}")
                    else:
                        print("Manual PWM speed at neutral or calibration time")

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


    def integrate_arc(self, x, y, theta, v, delta, dt, L):
    
        max_steering_rad = np.deg2rad(25)
        delta = np.clip(delta, -max_steering_rad, max_steering_rad)
        if abs(delta) < 1e-6:
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
        else:
            R = L / np.tan(delta)
            dtheta = v / R * dt
            theta += dtheta
            x += v * np.cos(theta - dtheta) * dt
            y += v * np.sin(theta - dtheta) * dt

        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        return x, y, theta

    def kinematicBicycleModel(self, elapsedTime):
        
        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
        vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase = self.vehicle.getSpeedSteerWheelbase()

        vehicleX, vehicleY, vehicleYaw = self.integrate_arc(
            vehicleX,
            vehicleY,
            vehicleYaw,
            vehicleSpeed,
            vehicleSteeringAngle,
            elapsedTime,
            vehicleWheelbase
        )

        self.vehicle.setPosition(vehicleX, vehicleY, vehicleYaw)

    
    def findLookAheadPoint(self, lastPathPointId):
        if lastPathPointId == len(self.path) - 1:
            return (None, None)
            
        return self.path[lastPathPointId]


    def hasPassedWaypoint(self, waypoint_x, waypoint_y):
        
        x, y, yaw = self.vehicle.getPosition()
        dx = waypoint_x - x
        dy = waypoint_y - y
        
        forward_x = np.cos(yaw)
        forward_y = np.sin(yaw)
        
        dot_product = dx * forward_x + dy * forward_y
        
        distance = np.hypot(dx, dy)
        return distance < 10 or dot_product < 0
    
    def roadblockManeuver(self):
        startTime = time.perf_counter()

        self.startLaneDetectionSender.send("false")
        self.steerMotorSender.send(str(-250))
        while time.perf_counter() - startTime < 5.0:
                pass
        self.startLaneDetectionSender.send("true")
        while time.perf_counter() - startTime < 8.0:
                pass
        self.startLaneDetectionSender.send("false")
        self.steerMotorSender.send(str(250))
        while time.perf_counter() - startTime < 13.0:
                pass
        self.startLaneDetectionSender.send("true")
        return time.perf_counter() - startTime
    
    def purePursuit(self):

        refSpeed = 25.0
        minDt = 0.03
        maxDt = 0.08


        lastPointSignal = False
        self.whiteLineSubscriber.subscribe()

        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
        vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase = self.vehicle.getSpeedSteerWheelbase()
        self.vehicle.resetStateFlags()

        self.startLaneDetectionSender.send("true")

        lastTime = time.time()
        
        path_x, path_y = [], []
        lastPathPointId = 1

        self.speedMotorSender.send(str(np.clip(vehicleSpeed*10, -500, 500)))
        targetX, targetY = self.findLookAheadPoint(lastPathPointId)

        isTurning = False

        while targetX and targetY: 

            currSpeed, _, _ = self.vehicle.getSpeedSteerWheelbase()
            self.timeStep = np.clip((refSpeed / (currSpeed + 1e-6)) / 10, minDt, maxDt)

            nowTime = time.time()
            dt = nowTime - lastTime

            lastTime = nowTime

            vehicleSpeed = self.vehicle.getSpeed()

            while lastPathPointId < len(self.path) and self.hasPassedWaypoint(self.path[lastPathPointId][0], self.path[lastPathPointId][1]):
                lastPathPointId += 1
            

            self.speedMotorSender.send(str(vehicleSpeed * 10))
            
            g1, g2, g3 = self.vehicle.getPosition()
            print(f"{g1} {g2} {np.rad2deg(g3)}  {np.rad2deg(vehicleSteeringAngle)}")

            distanceToTarget = self.calculateDistanceToTarget(targetX, targetY)
            angleToTarget = self.calculateAngleToTarget(targetX, targetY)

            alpha = self.calculateAlphaSteer(angleToTarget)

            vehicleSteeringAngle = np.arctan2(2 * vehicleWheelbase * np.sin(alpha), distanceToTarget)
            vehicleSteeringAngle = np.clip(vehicleSteeringAngle, -0.4363, 0.4363)
            self.vehicle.setSteeringAngle(vehicleSteeringAngle)

            if np.abs(np.rad2deg(vehicleSteeringAngle)) >= 5:
                self.timeStep = 0.03

            self.kinematicBicycleModel(dt)

            startTime = time.time()

            vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
            self.KFLocalization.prediction([vehicleX, vehicleY, vehicleYaw], vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase )

            whiteLine = self.whiteLineSubscriber.receive()
            if whiteLine is not None and whiteLine and self.vehicle.getStateSignal(stateSignalType.APROACHING_INTERSECTION):
                #RIGHT TURN
                # self.vehicle.setPosition(520.0, 260.0, vehicleYaw)
                #LEFT TURN
                #self.vehicle.setPosition(662.0, 217.0, vehicleYaw)
                #INTERSECTION
                #self.vehicle.setPosition(342.5, 80.0, vehicleYaw) #-35 x for sensor
                #Track 4
                #self.vehicle.setPosition(576.5, 115.0, vehicleYaw) #+35

                self.vehicle.setStateSignal(stateSignalType.IN_INTERSECION, True)
                self.vehicle.setStateSignal(stateSignalType.APROACHING_INTERSECTION, False)

            steeringAngleSend = round(np.rad2deg(vehicleSteeringAngle)) * 10
            self.steerMotorSender.send(str(steeringAngleSend))

            # if self.vehicle.getStateSignal(stateSignalType.IN_INTERSECION):
            #     self.startLaneDetectionSender.send("false")
            #     print(f"Send steering angle: {steeringAngleSend}")
            #     if steeringAngleSend > 100:
            #         isTurning = True
            #         steeringAngleSend = 250
            #         self.steerMotorSender.send(str(steeringAngleSend))
            #     elif steeringAngleSend < -100:
            #         isTurning = True
            #         steeringAngleSend = -180
            #         self.steerMotorSender.send(str(steeringAngleSend))
            #     elif isTurning and steeringAngleSend < 100 and steeringAngleSend > -100:
            #         self.vehicle.setStateSignal(stateSignalType.IN_INTERSECION, False)
            #         isTurning = False
            #         self.startLaneDetectionSender.send("true")

            # if self.vehicle.getStateSignal(stateSignalType.ROADBLOCK_MANEUVER):
            #     manueverTime = self.roadblockManeuver()
            #     startTime += manueverTime
            #     print(f"StartTime: {startTime}")

            # print(f"Steering: {str(round(np.rad2deg(np.clip(-vehicleSteeringAngle, -0.4363, 0.4363)))*10)}")
            #self.steerMotorSender.send(str(round(np.rad2deg(np.clip(-vehicleSteeringAngle, -0.4363, 0.4363)))*10))

            targetX, targetY = self.findLookAheadPoint(lastPathPointId)
                            
            #vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
            path_x.append(vehicleX)
            path_y.append(vehicleY)
            self.currentPosSender.send(str([vehicleX, vehicleY, vehicleYaw]))

            start_time = time.perf_counter()
            while time.perf_counter() - start_time < self.timeStep:
                pass
            
            startRun = self.startRunSubscriber.receive()
            if startRun is not None:
                if startRun == 'false':
                    self.isDriving = False
                    break
            

        self.steerMotorSender.send(str(0))
        time.sleep(0.3)
        self.speedMotorSender.send(str(0))

        self.whiteLineSubscriber.unsubscribe()

        self.startLaneDetectionSender.send("false")

        plt.plot(path_x, path_y, label="Robot Path", color="orange")
        plt.scatter(*zip(*self.nodes.values()), color="red", label="Waypoints")  # Mark the nodes/waypoints
        plt.scatter(*zip(*self.path), color="green", label="Additional Nodes")
        
        plt.title("Path Following with Pure Pursuit")
        plt.xlabel("X (cm)")
        plt.ylabel("Y (cm)")
        plt.gca().invert_yaxis()

        plt.legend()
        plt.grid(True)
        plt.show()
        plt.savefig("path_following.svg", format = 'svg')

        return path_x, path_y, self.path

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.startRunSubscriber = messageHandlerSubscriber(self.queuesList, startRun, "lastOnly", True)
        #self.IMUDataSubscriber = messageHandlerSubscriber(self.queuesList, ImuData, "lastOnly", True)
        self.shMemResponseSubscriber = messageHandlerSubscriber(self.queuesList, ShMemResponse, "lastOnly", True)
        self.selectTrackSubscriber = messageHandlerSubscriber(self.queuesList, SelectTrackNo, "lastOnly", True)
        self.whiteLineSubscriber = messageHandlerSubscriber(self.queuesList, WhiteLine, "lastOnly", False)
        self.calibrationSpeedSubscriber = messageHandlerSubscriber(self.queuesList, SetCalibrationSpeed, "lastOnly", True)
        self.calibrationTimeSubscriber = messageHandlerSubscriber(self.queuesList, SetCalibrationTime, "lastOnly", True)
        self.manualPWMSpeedSubscriber = messageHandlerSubscriber(self.queuesList, SetManualPWMSpeed, "lastOnly", True)
