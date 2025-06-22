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

        while self._running:

            trackChoice = self.selectTrackSubscriber.receive()    
            if trackChoice is not None:
                self.chosenTrack = trackChoice
                
                if self.chosenTrack == 0:
                    #ROUNDABOUT RUN desno skretanje

                    self.WhitelinePosition = (1622.1, 952.7)

                    speed = 15
                    self.lookAheadDistance = speed * self.lookAheadValue
                    yaw = 90

                    if self.waypoints:
                        self.waypoints.clear() 

                    self.waypoints.extend(range(315, 318))
                    self.waypoints.extend(range(332, 333))
                    self.waypoints.extend(range(368, 370))

                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.pathLength = len(self.path)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))


                elif self.chosenTrack == 1:
                    #HIGHWAY RUN

                    speed = 25
                    self.lookAheadDistance = speed * self.lookAheadValue
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
                    self.pathLength = len(self.path)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))

                elif self.chosenTrack == 2:
                    #right turn

                    self.WhitelinePosition = (288.0, 599.6)
                    
                    speed = 15
                    self.lookAheadDistance = speed * self.lookAheadValue

                    yaw = 90

                    if self.waypoints:
                        self.waypoints.clear() 

                    self.waypoints.extend(range(135, 138))
                    self.waypoints.extend(range(16, 17))
                    self.waypoints.extend(range(2000, 2001))
                    self.waypoints.extend(range(17, 18))
                    self.waypoints.extend(range(149, 150))


                    if len(self.path):
                        self.path = np.array([])

                    self.path = [(self.nodes[pointId][0], self.nodes[pointId][1]) for pointId in self.waypoints]
                    self.pathLength = len(self.path)

                    self.vehicle.updateVehicleState(speed, self.nodes[self.waypoints[0]][0], self.nodes[self.waypoints[0]][1], np.deg2rad(yaw))
                elif self.chosenTrack == 3:
                    ##left turn

                    self.WhitelinePosition = (288.0, 599.6)

                    speed = 15
                    self.lookAheadDistance = speed * self.lookAheadValue

                    yaw = 90

                    if self.waypoints:
                        self.waypoints.clear()

                    self.waypoints.extend(range(135, 138))
                    self.waypoints.extend(range(16, 7))
                    self.waypoints.extend(range(13, 14))
                    self.waypoints.extend(range(148, 149))


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

    
    # def findLookAheadPoint(self, lastPathPointId):
    #     if lastPathPointId >= self.pathLength:
    #         return (None, None)
            
    #     return self.path[lastPathPointId]
    
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
        
        distance = np.hypot(dx, dy)
        return dot_product < 0
    
    def isBehind(self, currX, currY,  waypoint_x, waypoint_y):
        dx = waypoint_x - currX
        dy = waypoint_y - currY
        
        forward_x = np.cos(self.vehicle.getYaw())
        forward_y = np.sin(self.vehicle.getYaw())
        
        dot_product = dx * forward_x + dy * forward_y
        distance = np.hypot(dx, dy)

        return distance < 30 or dot_product < 0
    
    def roadblockManeuver(self, nextNodeId, distanceFromRoadblock, distanceToTarget, distanceToNode = 25.0):
        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()

        nextNodeX, nextNodeY = self.path[nextNodeId]

        heading = vehicleYaw

        left_dx = np.sin(heading)
        left_dy = -np.cos(heading)

        node1_x = nextNodeX + 20.0 * left_dx
        node1_y = nextNodeY + 20.0 * left_dy

        forward_dx = np.cos(heading)
        forward_dy = np.sin(heading)

        node1_x += distanceFromRoadblock * forward_dx
        node1_y += distanceFromRoadblock * forward_dy

        node2_x = node1_x + distanceFromRoadblock * forward_dx
        node2_y = node1_y + distanceFromRoadblock * forward_dy


        while len(self.path) <= nextNodeId + 1:
            self.path.append((0.0, 0.0))

        self.path[nextNodeId] = (node1_x, node1_y)
        self.path[nextNodeId + 1] = (node2_x, node2_y)

        new_path = self.path[:nextNodeId + 2]
        remaining_path = self.path[nextNodeId + 2:]

        index = 0
        for (x, y) in remaining_path:
            if self.isBehind(node2_x, node2_y,  x, y):
                index += 1
            else:
                break

        self.path = new_path + remaining_path[index:]
        self.pathLength = len(self.path)

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
        # self.startLaneDetectionSender.send("true")

        isTurning = False

        refSpeed = 25.0
        minDt = 0.03
        maxDt = 0.08

        manueverNodeId = -1

        vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()
        vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase = self.vehicle.getSpeedSteerWheelbase()

        path_x, path_y = [], []

        lastTime = time.time()        
        lastPathPointId = 1

        self.vehicle.setStateSignal(stateSignalType.APROACHING_INTERSECTION, True)

        # self.speedMotorSender.send(str(np.clip(vehicleSpeed*10, -500, 500)))


        flagZeroAngle = False
        targetX, targetY = self.findLookAheadPoint(lastPathPointId)
        while targetX and targetY: 

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

            # g1, g2, g3 = self.vehicle.getPosition()

            # {g1} {g2} {np.rad2deg(g3)}  
            print(f"{np.rad2deg(vehicleSteeringAngle)}")

            distanceToTarget = self.calculateDistanceToTarget(targetX, targetY)
            vehicleSteeringAngle = self.computeSteer(targetX, targetY)
            self.vehicle.setSteeringAngle(vehicleSteeringAngle)

            if np.abs(np.rad2deg(vehicleSteeringAngle)) >= 5:
                self.setTimeStep(0.03)


            if self.vehicle.getStateSignal(stateSignalType.APROACHING_INTERSECTION):
                 vehicleSteeringAngle = 0

            steeringAngleSend = round(np.rad2deg(vehicleSteeringAngle)) * 10


            self.kinematicBicycleModel(elapsedTime)
            vehicleX, vehicleY, vehicleYaw = self.vehicle.getPosition()

            # self.KFLocalization.prediction([vehicleX, vehicleY, vehicleYaw], vehicleSpeed, vehicleSteeringAngle, vehicleWheelbase )
            ###DATA SENDIING
            self.trafficCommInternalSender.send({"dataType": "devicePos", "vals": [vehicleX, vehicleY]})
            self.trafficCommInternalSender.send({"dataType": "deviceRot", "vals": [vehicleYaw]})

            whiteLine = self.whiteLineSubscriber.receive()
            if whiteLine is not None and whiteLine and self.vehicle.getStateSignal(stateSignalType.APROACHING_INTERSECTION):
                self.vehicle.setStateSignal(stateSignalType.IN_INTERSECION, True)
                self.vehicle.setStateSignal(stateSignalType.APROACHING_INTERSECTION, False)
                flagZeroAngle = False
            

            if self.vehicle.getStateSignal(stateSignalType.IN_INTERSECION):
                self.startLaneDetectionSender.send("false")
                self.steerMotorSender.send(str(-steeringAngleSend))

                if np.abs(steeringAngleSend) < 70 and not flagZeroAngle:
                    self.startLaneDetectionSender.send("true")
                    self.vehicle.setStateSignal(stateSignalType.IN_INTERSECION, False)
                    flagZeroAngle = True
    
            


                # x, y = self.WhitelinePosition
                # self.vehicle.setPosition(x, y, vehicleYaw)


            # if self.vehicle.getStateSignal(stateSignalType.ROADBLOCK_MANEUVER):


            #     manueverNodeId = lastPathPointId + 1
            #     distanceFromRoadBlock = self.vehicle.getFrontDistanceSensorValue() 

            #     # self.setBaseLookAheadDistance()
            #     self.roadblockManeuver(manueverNodeId, distanceFromRoadBlock, distanceToTarget, 100)
            #     self.vehicle.setStateSignal(stateSignalType.ROADBLOCK_MANEUVER, False)

            #     self.vehicle.setSpeed(20)
            #     self.speedMotorSender.send(str(200))
                    

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
