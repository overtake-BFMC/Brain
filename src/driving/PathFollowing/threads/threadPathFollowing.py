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
    SetCalibrationSteer,
    SetManualPWMSteer,
    ManualPWMSteerMotor,
    TrafficComInternal
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

    def __init__(self, queueList, logger, lookAheadValue = 1, dt = 0.05, debugging = False):

        self.queuesList = queueList
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)

        self.nodes = nodesData

        self.waypoints = []
        self.path = []
        self.pathLength = 0

        self.maxSteeringRadians = np.deg2rad(27)

        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.ShMemConfigSender = messageHandlerSender(self.queuesList, ShMemConfig)
        self.currentPosSender = messageHandlerSender(self.queuesList, CurrentPos)
        self.startLaneDetectionSender = messageHandlerSender(self.queuesList, startLaneDetection)
        self.manualPWMSpeedSender = messageHandlerSender(self.queuesList, ManualPWMSpeedMotor)
        self.manualPWMSteerSender = messageHandlerSender(self.queuesList, ManualPWMSteerMotor)
        self.trafficCommInternalSender = messageHandlerSender(self.queuesList, TrafficComInternal)

        self.lookAheadValue = lookAheadValue
        self.lookAheadDistance =  15
        self.timeStep = dt
        
        self.isDriving = False
        self.chosenTrack = -1

        self.subscribe()
        time.sleep(1)

        self.shMemMsgOwner = "threadPathFollowing"
        self.init_vehicleState()

        self.max_intersection_angle = False


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
        self.calibrationSteer = 9999
        self.manualPWMSpeed = 1500
        self.manualPWMSteer = 1524

        speed = 20
        yaw = 0

        self.KFLocalization = kalmanFilterLocalization(dt = self.timeStep)


        #code by MS
        #white lines to be added to final run
        # self.WhitelinePosition.append(( 1706.9, 205, np.deg2rad(90) )) #2
        # self.WhitelinePosition.append(( 1646.8, 401.1, np.deg2rad(180) )) #3
        # self.WhitelinePosition.append(( 1622.1, 932.7, np.deg2rad( 90) )) #4
        # self.WhitelinePosition.append(( 1587.5, 1190.2, np.deg2rad( 270) )) #5
        # self.WhitelinePosition.append(( 34.5, 1080.6, np.deg2rad( 270) )) #6
        # self.WhitelinePosition.append(( 93.2, 951.1, np.deg2rad( 0) )) #7
        # self.WhitelinePosition.append(( 475.6, 816.3, np.deg2rad( 2700) )) #8
        # self.WhitelinePosition.append(( 339.4, 688.6, np.deg2rad( 180) )) #9
        # self.WhitelinePosition.append(( 123.9, 688.6, np.deg2rad( 180) )) #10
        # self.WhitelinePosition.append(( 71.9, 909.9, np.deg2rad( 90) )) #11
        # self.WhitelinePosition.append(( 1499.6, 1039.4, np.deg2rad( 0) )) #12
        # self.WhitelinePosition.append(( 1527.3, 451.7, np.deg2rad( 270) )) #13
        # self.WhitelinePosition.append(( 1527.3, 294.2, np.deg2rad( 270) )) #14
        # self.WhitelinePosition.append(( 1527.3, 143.5, np.deg2rad( 270) )) #15 NAPOMENA cilj je pre poslednje stop linije



        while self._running:

            trackChoice = self.selectTrackSubscriber.receive()    
            if trackChoice is not None:
                self.chosenTrack = trackChoice
                
                if self.chosenTrack == 0:
                    #ROUNDABOUT RUN desno skretanje

                    self.WhitelinePosition = (1646.8, 401.1)

                    speed = 25
                    # self.lookAheadDistance = speed * self.lookAheadValue
                    yaw = 180

                    if self.waypoints:
                        self.waypoints.clear() 

                    self.waypoints.extend(range(9000, 9002))
                    self.waypoints.extend(range(54, 55))
                    self.waypoints.extend(range(2001, 2002))
                    self.waypoints.extend(range(55, 56))
                    self.waypoints.extend(range(305, 310))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.pathLength = len(self.path)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))


                elif self.chosenTrack == 1:
                    #HIGHWAY RUN

                    self.WhitelinePosition = []
                    # self.WhitelinePosition.append((475.6, 756.3, np.deg2rad(270)))
                    #TUNNEL EXIT
                    # self.WhitelinePosition.append((309.4, 688.6, np.deg2rad(180)))
                    # self.WhitelinePosition.append((93.9, 688.6, np.deg2rad(180)))
                    #TUNNEL ENTRY
                    self.WhitelinePosition.append((34.5, 1080.6, np.deg2rad(270)))
                    self.WhitelinePosition.append((93.2, 921.1, 9999))
                    #TUNNEL PASS
                    #self.WhitelinePosition.append((71.9, 909.9, np.deg2rad(90)))
                    speed = 25
                    # self.lookAheadDistance = speed * self.lookAheadValue
                    #TUNNEL EXIT
                    #yaw = 275
                    #TUNNEL ENTRY
                    yaw = 270
                    #TUNEL PASS
                    #yaw = 90
                    if self.waypoints:
                        self.waypoints.clear() 

                    #TUNNEL EXIT
                    # self.waypoints.extend(range(175, 178))
                    # self.waypoints.extend(range(8, 9))
                    # self.waypoints.extend(range(2002, 2006))
                    # self.waypoints.extend(range(1, 2))
                    # self.waypoints.extend(range(150, 151))
                    # self.waypoints.extend(range(18, 19))
                    # self.waypoints.extend(range(13, 14))
                    # self.waypoints.extend(range(148, 149))
                    # self.waypoints.extend(range(91, 93))
                    # self.waypoints.extend(range(178, 179))
                    #TUNNEL ENTRY
                    self.waypoints.extend(range(442, 444))
                    self.waypoints.extend(range(84, 85))
                    # self.waypoints.extend(range(10001, 10002))
                    # self.waypoints.extend(range(81, 82))
                    self.waypoints.extend(range(165, 169))
                    #TUNNEL PASS
                    # self.waypoints.extend(range(178, 181))
                    # self.waypoints.extend(range(80, 81))
                    # self.waypoints.extend(range(83, 84))
                    # self.waypoints.extend(range(406, 408))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.pathLength = len(self.path)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))

                elif self.chosenTrack == 2:
                    #right turn

                    self.waypoints = []

                    self.WhitelinePosition = []
                    self.WhitelinePosition.append((288.0, 599.6, 9999.0))
                    
                    speed = 25
                    # self.lookAheadDistance = speed * self.lookAheadValue

                    yaw = 90

                    if self.waypoints:
                        self.waypoints.clear() 

                    self.waypoints.extend(range(135, 138))
                    self.waypoints.extend(range(16, 17))
                    self.waypoints.extend(range(2000, 2001))
                    self.waypoints.extend(range(17, 18))
                    self.waypoints.extend(range(149, 150))
                    self.waypoints.extend(range(2, 3))


                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.pathLength = len(self.path)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))
                elif self.chosenTrack == 3:
                    ##left turn
                    self.waypoints = []

                    self.WhitelinePosition = []
                    self.WhitelinePosition.append((288.0, 599.6, 9999.0))

                    speed = 25
                    # self.lookAheadDistance = speed * self.lookAheadValue

                    yaw = 90

                    if self.waypoints:
                        self.waypoints.clear()

                    self.waypoints.extend(range(135, 138))
                    self.waypoints.extend(range(16, 7))
                    self.waypoints.extend(range(13, 14))
                    self.waypoints.extend(range(148, 149))
                    self.waypoints.extend(range(91, 92))


                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.pathLength = len(self.path)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))

                self.currentPosSender.send(str(self.vehicle.getPosition()))

            calibrationSpeed = self.calibrationSpeedSubscriber.receive()
            if calibrationSpeed is not None:
                self.calibrationSpeed = calibrationSpeed

            calibrationSteer = self.calibrationSteerSubscriber.receive()
            if calibrationSteer is not None:
                self.calibrationSteer = calibrationSteer
            
            calibrationTime = self.calibrationTimeSubscriber.receive()
            if calibrationTime is not None:
                self.calibrationTime = calibrationTime

            manualPWMSpeed = self.manualPWMSpeedSubscriber.receive()
            if manualPWMSpeed is not None:
                self.manualPWMSpeed = manualPWMSpeed

            manualPWMSteer = self.manualPWMSteerSubscriber.receive()
            if manualPWMSteer is not None:
                self.manualPWMSteer = manualPWMSteer

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
                elif self.isDriving and self.chosenTrack == 97:
                    if self.calibrationSteer != 9999 and self.calibrationSpeed != -1 and self.calibrationTime != -1:

                        self.steerMotorSender.send(str(self.calibrationSteer * 10))
                        self.speedMotorSender.send(str(self.calibrationSpeed * 10))

                        calibrationStartTime = time.perf_counter()

                        while(time.perf_counter() - calibrationStartTime) < self.calibrationTime:
                            pass

                        self.speedMotorSender.send(str(0))
                        self.steerMotorSender.send(str(0))
                        calibrationEndTime = time.perf_counter()

                        elapsed = calibrationEndTime - calibrationStartTime
                        print(f"Calibration Elapsed Time: {elapsed}")
                        print(f"Calibration Steer: {self.calibrationSteer}")
                        print(f"Calibration Speed: {self.calibrationSpeed}")
                    else:
                        print("Missing calibration steer, speed or calibration time")

                elif self.isDriving and self.chosenTrack == 96:
                    if self.manualPWMSteer != 1524:

                        print(f"HERE {self.manualPWMSteer}")

                        self.manualPWMSteerSender.send(self.manualPWMSteer)
                        if self.calibrationSpeed != -1 and self.calibrationTime != -1:
                            self.speedMotorSender.send(str(self.calibrationSpeed * 10))
                        if self.calibrationTime != -1:
                            calibrationStartTime = time.perf_counter()

                            while(time.perf_counter() - calibrationStartTime) < self.calibrationTime:
                                pass

                            self.manualPWMSteerSender.send(1524)
                        if self.calibrationSpeed != -1 and self.calibrationTime != -1:
                            self.speedMotorSender.send(str(0))
                        if self.calibrationTime != -1:
                            calibrationEndTime = time.perf_counter()

                            elapsed = calibrationEndTime - calibrationStartTime
                            print(f"Calibration Elapsed Time: {elapsed}")
                            print(f"Calibration Speed: {self.calibrationSpeed}")
                        print(f"Calibration PWM Steer: {self.manualPWMSteer}")
                    else:
                        print("Manual PWM steer at neutral, speed missing or calibration time")

    def setBaseLookAheadDistance(self):
        self.lookAheadDistance = 15
    
    def resetLookAheadDistance(self):
        self.lookAheadDistance = self.lookAheadValue * self.vehicle.getSpeed()

    def setTimeStep(self, dt):
        self.timeStep = dt

    def calculateDistanceToTarget(self, targetX, targetY):

        vehicleX, vehicleY, _ = self.vehicle.getPosition()

        return np.sqrt((targetX - vehicleX)**2 + (targetY - vehicleY)**2)

    def calculateAngleToTarget(self, targetX, targetY):

        vehicleX, vehicleY, _ = self.vehicle.getPosition()
        return np.arctan2(targetY - vehicleY, targetX - vehicleX)
    
    def normalizeAngle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def calculateAlphaSteer(self, targetX, targetY):

        angleToTarget = self.calculateAngleToTarget(targetX, targetY)
        _, _, vehicleYaw = self.vehicle.getPosition()

        return self.normalizeAngle(angleToTarget - vehicleYaw)


    def integrateArc(self, x, y, theta, v, delta, elapsedTime, L):
    
        
        delta = np.clip(delta, -self.maxSteeringRadians, self.maxSteeringRadians)
        if abs(delta) < 1e-6:
            x += v * np.cos(theta) * elapsedTime
            y += v * np.sin(theta) * elapsedTime
        else:
            R = L / np.tan(delta)
            dtheta = v / R * elapsedTime
            theta += dtheta
            x += v * np.cos(theta - dtheta) * elapsedTime
            y += v * np.sin(theta - dtheta) * elapsedTime

        theta = self.normalizeAngle(theta)
        return x, y, theta

    def kinematicBicycleModel(self, elapsedTime):
        
        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
        vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase = self.vehicle.getSpeedSteerWheelbase()

        vehicleX, vehicleY, vehicleYaw = self.integrateArc(
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

        pathLength = len(self.path)

        if lastPathPointId == pathLength:
            return (None, None)

        window_size = 10
        px, py, yaw = self.vehicle.getPosition()
        
        start_idx = lastPathPointId + 1
        end_idx = min(start_idx + window_size, pathLength)

        for i in range(start_idx, end_idx):
            x, y = self.path[i]

            distance = np.hypot(x - px, y - py)
            if distance >= self.lookAheadDistance:
                return x, y

        return self.path[pathLength - 1]


    def hasPassedWaypoint(self, waypoint_x, waypoint_y):
        
        x, y, yaw = self.vehicle.getPosition()
        dx = waypoint_x - x
        dy = waypoint_y - y
        
        forward_x = np.cos(yaw)
        forward_y = np.sin(yaw)
        
        dot_product = dx * forward_x + dy * forward_y
        
        return dot_product < 0
    
    def computeSteer(self, target_x, target_y):

        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
        _, _, wheelbase = self.vehicle.getSpeedSteerWheelbase()

        dx = target_x - vehicleX
        dy = target_y - vehicleY
        distance = np.hypot(dx, dy)

        heading_vec = (np.cos(vehicleYaw), np.sin(vehicleYaw))
        target_vec = (dx, dy)

        dot = heading_vec[0] * target_vec[0] + heading_vec[1] * target_vec[1]
        det = heading_vec[0] * target_vec[1] - heading_vec[1] * target_vec[0]
        theta = np.arctan2(det, dot)

        if abs(np.sin(theta)) < 1e-6:
            return 0.0

        R = distance / (2 * np.sin(theta))
        delta = np.arctan(wheelbase / R)
        
        return np.clip(delta, -self.maxSteeringRadians, self.maxSteeringRadians)
    
    
    def purePursuit(self):

        self.whiteLineSubscriber.subscribe()
        self.vehicle.resetStateFlags()
        self.startLaneDetectionSender.send("true")

        refSpeed = 25.0
        minDt = 0.03
        maxDt = 0.08

        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
        vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase = self.vehicle.getSpeedSteerWheelbase()

        path_x, path_y = [], []

        lastTime = time.time()        
        lastPathPointId = 1
        whitelineCounter = 0

        DataSenderCounter = 0

        targetX, targetY = self.findLookAheadPoint(lastPathPointId)
        while targetX and targetY: 

            DataSenderCounter += 1

            if DataSenderCounter >= 12:
                DataSenderCounter = 0

                #code by MS
                vehicleYawSendValue = ( vehicleYaw + 2 * np.pi ) % ( 2 * np.pi )  #realno pi = 6.283185307
                ###DATA SENDIING
                self.trafficCommInternalSender.send({"dataType": "devicePos", "vals": [vehicleX/1000, vehicleY/1000]})
                self.trafficCommInternalSender.send({"dataType": "deviceRot", "vals": [np.rad2deg(vehicleYawSendValue)]})
                self.trafficCommInternalSender.send({"dataType": "deviceSpeed", "vals": [vehicleSpeed]})


            self.resetLookAheadDistance()

            vehicleSpeed = self.vehicle.getSpeed()
            self.speedMotorSender.send(str(vehicleSpeed * 10))

            dt = np.clip((refSpeed / (vehicleSpeed + 1e-6)) / 10, minDt, maxDt)
            self.setTimeStep(dt)

            nowTime = time.time()
            elapsedTime = nowTime - lastTime

            lastTime = nowTime
           

            while lastPathPointId < self.pathLength and self.hasPassedWaypoint(self.path[lastPathPointId][0], self.path[lastPathPointId][1]):
                lastPathPointId += 1

            vehicleSteeringAngle = self.computeSteer(targetX, targetY)
            self.vehicle.setSteeringAngle(vehicleSteeringAngle)

            tmpSteer = np.abs(round(np.rad2deg(vehicleSteeringAngle)) * 10)

            if np.abs(np.rad2deg(vehicleSteeringAngle)) >= 5:
                self.setTimeStep(0.03)


            if self.vehicle.getStateSignal(stateSignalType.APROACHING_INTERSECTION) and tmpSteer < 90:
                 vehicleSteeringAngle = 0

            steeringAngleSend = round(np.rad2deg(vehicleSteeringAngle)) * 10
            print(f"{steeringAngleSend}")

            self.kinematicBicycleModel(elapsedTime)
            vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()

            whiteLine = self.whiteLineSubscriber.receive()
            if whiteLine is not None and whiteLine and self.vehicle.getStateSignal(stateSignalType.APROACHING_INTERSECTION):

                tmpSteer = 120
                self.vehicle.setStateSignal(stateSignalType.IN_INTERSECTION, True)
                self.vehicle.setStateSignal(stateSignalType.APROACHING_INTERSECTION, False)
                self.startLaneDetectionSender.send("false")

                
                x, y, yaw = self.WhitelinePosition[whitelineCounter]
                if yaw == 9999:
                    yaw = vehicleYaw
                self.vehicle.setPosition(x, y, yaw)
                # g1, g2, g3 = self.vehicle.getPosition()
                # print(f"{np.rad2deg(g3)}")

                whitelineCounter += 1
            

            if self.vehicle.getStateSignal(stateSignalType.IN_INTERSECTION):
                self.steerMotorSender.send(str(-steeringAngleSend))
                
                if tmpSteer < 110:
                    self.vehicle.setStateSignal(stateSignalType.IN_INTERSECTION, False)
                    self.startLaneDetectionSender.send("true")

                    
                    

            targetX, targetY = self.findLookAheadPoint(lastPathPointId)
                            
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
            

        g1, g2, g3 = self.vehicle.getPosition()
        print(f"{g1} {g2} {np.rad2deg(g3)}  {np.rad2deg(vehicleSteeringAngle)}")
        
        self.steerMotorSender.send(str(0))
        time.sleep(0.3)
        self.speedMotorSender.send(str(0))

        self.whiteLineSubscriber.unsubscribe()

        self.startLaneDetectionSender.send("false")

        plt.plot(path_x, path_y, label="Robot Path", color="orange")
        plt.scatter(*zip(*self.nodes.values()), color="red", label="Waypoints")
        plt.scatter(*zip(*self.path), color="green", label="Additional Nodes")
        
        plt.title("Path Following with Pure Pursuit")
        plt.xlabel("X (cm)")
        plt.ylabel("Y (cm)")
        # plt.gca().invert_yaxis()

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
        self.calibrationSteerSubscriber = messageHandlerSubscriber(self.queuesList, SetCalibrationSteer, "lastOnly", True)
        self.manualPWMSteerSubscriber = messageHandlerSubscriber(self.queuesList, SetManualPWMSteer, "lastOnly", True)
