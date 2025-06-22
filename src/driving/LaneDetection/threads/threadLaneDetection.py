import time
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    startLaneDetection,
    SteerMotor,
    SpeedMotor,
    ImuData,
    ShMemConfig,
    ShMemResponse,
    DistanceFront,
    WarningSignal,
    Semaphores,
    DistanceRight,
    TrafficComInternal,
    SetLaneKP,
    SetLaneKD,
    SetLaneKI
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.driving.LaneDetection.utils import preprocessing as pre
from src.driving.LaneDetection.utils import PID
from src.driving.LaneDetection.utils.gamma import apply_gamma_on_frame
from src.driving.LaneDetection.utils.kalman_filter import KalmanFilter_LANE
from multiprocessing.shared_memory import SharedMemory
import numpy as np
import os
import cv2
#from ultralytics import YOLO
import ast
from src.driving.LaneDetection.utils.signDetection import signDetection
from src.driving.LaneDetection.utils.signDetection import CLASSES

from src.driving.LaneDetection.utils.cannyandhough import LaneFollowing
from src.driving.PathFollowing.utils.vehicleState import stateSignalType
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger

CLASSES_ROI = [
    [0, 0, 960, 540], #car 0
    #ovde treba odvojiti car roi za parking i za prepreku

    # [0, 0, 960, 540], #closed-road-stand 1
    [300, 0, 650, 370], #closed-road-stand NEW


    [650, 50, 900, 220], #crosswalk-sign 
    # [700, 90, 850, 220], #crosswalk-sign NEW

    [700, 90, 900, 220], #highway-entry-sign 3
    [700, 90, 900, 220], #highway-exit-sign 4
    [720, 0, 960, 540 ], #nov
    # [700, 90, 900, 220], #no-entry-road-sign 5 #this is traffic-light for now
    [700, 90, 900, 220], #one-way-road-sign 6
    [0, 130, 960, 410], #parking-sign 7
    [0, 0, 960, 540], #parking-spot 8

    # [[150, 150, 650, 400],[50, 150, 900, 400]], #pedestrian 9
    [[250, 170, 710, 350], [250, 150, 710, 350]], #pedestrian NEW 9
    
    [700, 90, 900, 220], #priority-sign 10
    [700, 90, 900, 220], #round-about-sign 11
    [[300, 400, 660, 540], [300, 335, 660, 540]],#[400, 400, 550, 540], #stop-line 12
    [720, 0, 960, 540], #stop-sign 13 

    # [720, 0, 960, 500 ], #traffic-light 14 #this does not work
    [720, 100, 900, 540 ], #traffic-light 14 #this does not work NEW

]

class threadLaneDetection(ThreadWithStop):
    """This thread handles laneDetection.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logger, debugging = False):

        self.queuesList = queueList
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)

        #self.laneVideoSender = messageHandlerSender(self.queuesList, LaneVideo)

        self.subscribe()

        self.isLaneKeeping = False
        #self.vehicle = vehicleState( 15, 0, 0, 0 )
        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.warningSignalSender = messageHandlerSender(self.queuesList, WarningSignal)
        self.trafficCommInternalSender = messageHandlerSender(self.queuesList, TrafficComInternal) ## SEND THROUGH HERE
        #IN FORMAT
        #["devicePos" , "val1", "val2", "val3"]

        self.kf = KalmanFilter_LANE()

        # kalibrisani podaci
        dir_path = os.path.dirname(os.path.realpath(__file__))

        self.ppData, self.maps = pre.loadPPData(dir_path + "/../utils/data")  

        # self.pid = PID.PIDController( Kp = 0.15, Ki = 0.05, Kd = 0.09 ) #40cms

        self.Kp = 0.20 # trenutly
        self.Ki = 0.03
        self.Kd = 0.09

        self.initialSpeed = 20
        self.pid = PID.PIDController( self.Kp, self.Ki, self.Kd )

        self.detector = signDetection(conf_thresh=0.5, iou_thresh=0.45)
        self.noOfDetections = [0] * 15
        self.frame = np.empty((540, 960, 3), dtype=np.uint8)

        self.ShMemConfigSender = messageHandlerSender(self.queuesList, ShMemConfig)

        self.shMemNameMainVideo = "mainVideoFrames"
        self.shMemNameLaneVideo = "laneVideoFrames"
        self.shMemMsgOwner = "threadLaneDetection"

        self.isOnHighway = False
        self.isStop = False
        self.semaphoresState = {}
        self.redLightFlag = False
        self.semaphoreID = None

        self.isInRoundabout = False
        self.parkingSignFlag = False

        self.distanceF = 99
        self.distanceR = 99

        self.init_shMem()
        self.init_vehicleState()

        super(threadLaneDetection, self).__init__()

    def init_shMem(self):
        self.ShMemConfigSender.send({"action": "getShMem","name": self.shMemNameMainVideo, "owner": self.shMemMsgOwner})

        isMainVideoMemoryConfigured = False
        isLaneVideoMemoryConfigured = False
        while not isMainVideoMemoryConfigured or not isLaneVideoMemoryConfigured:
            shMemResp = self.shMemResponseSubscriber.receive()
            if shMemResp is not None:
                if shMemResp["status"] == 1 and shMemResp["dest"] == self.shMemMsgOwner:
                    self.shMemNameMainVideo = shMemResp["name"]
                    self.MainVideolock = shMemResp["lock"]
                    self.shmMainVideo = SharedMemory(name=self.shMemNameMainVideo)
                    self.frameBuffer = np.ndarray((540, 960, 3), dtype=np.uint8, buffer=self.shmMainVideo.buf)
                    isMainVideoMemoryConfigured = True
                    #print("LaneDetection MainVideoShMem Init Success!")
                    self.logger.info("LaneDetection MainVideoShMem Init Success!")
                    self.ShMemConfigSender.send({"action": "createShMem", "name": self.shMemNameLaneVideo, "shape": (540, 960, 3), "dtype": "uint8", "owner": self.shMemMsgOwner})

            if shMemResp is not None:
                if shMemResp["status"] == 0 and shMemResp["dest"] == self.shMemMsgOwner:
                    self.shMemNameLaneVideo = shMemResp["name"]
                    self.LaneVideolock = shMemResp["lock"]
                    self.shmLaneVideo = SharedMemory(self.shMemNameLaneVideo)
                    self.frameLaneBuffer = np.ndarray((540, 960, 3), dtype=np.uint8, buffer=self.shmLaneVideo.buf)
                    isLaneVideoMemoryConfigured = True
                    #print("LaneDetection LaneVideoShMem Init Success!")
                    self.logger.info("LaneDetection LaneVideoShMem Init Success!")

    def init_vehicleState(self):
        self.ShMemConfigSender.send({"action": "getVehicleState","owner": self.shMemMsgOwner})

        isVehicleStateConfigured = False
        while not isVehicleStateConfigured:
            getVehicleStateResp = self.shMemResponseSubscriber.receive()
            if getVehicleStateResp is not None:
                if getVehicleStateResp["status"] == 3 and getVehicleStateResp["dest"] == self.shMemMsgOwner:
                    self.vehicleState = getVehicleStateResp["vehicleState"]
                    isVehicleStateConfigured = True
                    #print("LaneDetection VehicleState Initialized!")
                    self.logger.info("LaneDetection VehicleState Initialized!")

    def isInsideROI(self, x, y, ROIBox):
    #     if (ROIBox[0] <= x <= ROIBox[2] and ROIBox[1] <= y <= ROIBox[3]):
    #         print(f"In roi x,y: {x}, {y}")
    #         return True
    #     return False
        return ROIBox[0] <= x <= ROIBox[2] and ROIBox[1] <= y <= ROIBox[3]

    def makeDecision(self, detections, bufferDetections, timer ):
        
        isDetected = False
        desiredSpeed = self.vehicleState.getSpeed()
        boolDetections = [False] * 15
        boolDetections[9] = [False] * 2
        boolDetections[12] = [False] * 2
        #print(f"Disntace FR: {distanceF}")
        
        for box, class_id, class_score in detections:
            x, y, w, h = box

            #boolDetections[class_id] = self.isInsideROI(x, y, CLASSES_ROI[class_id])
            if isinstance(boolDetections[class_id], list):
                for i, _ in enumerate(boolDetections[class_id]):
                    boolDetections[class_id][i] = self.isInsideROI(x, y, CLASSES_ROI[class_id][i])
            else:
                boolDetections[class_id] = self.isInsideROI(x, y, CLASSES_ROI[class_id])

        for i in range(len(boolDetections)):
            if isinstance(boolDetections[i], list):
                for j in range(len(boolDetections[i])):
                    if not boolDetections[i][j]:
                        bufferDetections[i][j] -= 1
                    else:
                        bufferDetections[i][j] = 3
            else:
                if not boolDetections[i]:
                    bufferDetections[i] -= 1
                else:
                    bufferDetections[i] = 3

            if isinstance(bufferDetections[i], list):
                for j in range(len(bufferDetections[i])):
                    if bufferDetections[i][j] <= 0:
                        boolDetections[i][j] = False
                    else:
                        boolDetections[i][j] = True
            else:
                if bufferDetections[i] <= 0:
                    boolDetections[i] = False
                else:
                    boolDetections[i] = True


    
        if boolDetections[1] and not self.vehicleState.getStateSignal(stateSignalType.APROACHING_ROADBLOCK):
            # self.vehicleState.setStateSignal(stateSignalType.APROACHING_ROADBLOCK, True)

            if self.distanceF < 99:
                boolDetections[1] = False
                self.vehicleState.setStateSignal(stateSignalType.APROACHING_ROADBLOCK, True)
                self.vehicleState.setStateSignal(stateSignalType.ROADBLOCK_MANEUVER, True)
                desiredSpeed = 0

                # self.vehicleState.setStateSignal(stateSignalType.APROACHING_ROADBLOCK, False)

        if boolDetections[2]: #crosswalk-sign
            #trafficComunicationID = 4 
            print("crosswalk detected")
            isDetected = True
            #self.vehicleState.setSpeed(10)
            desiredSpeed = 10
            self.LANEFOL.crosswalk_active = True
            if not timer[2][0]:
                timer[2][0] = 1
                timer[2][1] = time.perf_counter()
                timer[2][2] = 10.0
            self.warningSignalSender.send({"WarningName":"Crosswalk Ahead", "WarningID": 4})
        if boolDetections[12][0] and boolDetections[9][1]: #stop-line and pedestrian
            #THIS NEEDS TO BE CHANGED
            print("pedestrian on crosswalk")
            #trafficComunicationID = 11 
            #self.vehicleState.setSpeed(0)
            desiredSpeed = 0
            isDetected = True
            self.warningSignalSender.send({"WarningName":"Pedestrian on Crosswalk", "WarningID": 11})
        if boolDetections[3]: #highwayEntry
            #trafficComunicationID = 5 
            desiredSpeed = 40

            self.isOnHighway = True
            self.warningSignalSender.send({"WarningName":"Highway Ahead", "WarningID": 6})
        if boolDetections[4]:  #highwayExit
            #trafficComunicationID = 6 
            self.isOnHighway = False
            self.warningSignalSender.send({"WarningName":"Highway Exit", "WarningID": 7})
        if boolDetections[9][0]: #pedestrian on road

            print( "pedestrian on road" )
            #trafficComunicationID = 12
            desiredSpeed = 0
            isDetected = True

            self.warningSignalSender.send({"WarningName":"Pedestrian On Road", "WarningID": 12})

        if boolDetections[13]:
            self.vehicleState.setStateSignal(stateSignalType.APROACHING_INTERSECTION, True)
            
        if boolDetections[12][1] and boolDetections[13]: #stop-sign and stop line
            #trafficComunicationID = 1 
            
            if not timer[0][0] and not timer[1][0]:
                timer[0][0] = 1
                timer[0][1] = time.perf_counter()
                timer[0][2] = 3.0
                desiredSpeed = 0
                isDetected = True
            elif not timer[1][0]:
                elapsed = time.perf_counter() - timer[0][1]
                if elapsed >= timer[0][2]:
                    desiredSpeed = 20
                    timer[0][0] = 0
                    timer[1][0] = 1
                    timer[1][1] = time.perf_counter()
                    timer[1][2] = 2.0
            self.warningSignalSender.send({"WarningName":"Stop Sign", "WarningID": 20})
            isDetected = True 

        # if boolDetections[6]: #one way road sign
            #trafficComunicationID = 8
            # print( "one way road" ) 

        if boolDetections[10]: #priority
            #trafficComunicationID = 4 
            self.warningSignalSender.send({"WarningName":"Priority Ahead", "WarningID": 13})

            #self.LANEFOL.priority_active = True
            self.vehicleState.setStateSignal(stateSignalType.APROACHING_INTERSECTION, True)
            desiredSpeed = 25
            # timer[2][0] = 1
            # timer[2][1] = time.perf_counter()
            # timer[2][2] = 6.0
        if boolDetections[11]: #roundabout
            #trafficComunicationID = 7
            self.vehicleState.setStateSignal(stateSignalType.APROACHING_INTERSECTION, True)

            self.isInRoundabout = True
            self.warningSignalSender.send({"WarningName":"Roundabout Ahead", "WarningID": 15})

        if boolDetections[7]: #Parking Sign
            #trafficComunicationID = 3 
            self.parkingSignFlag = True

            self.warningSignalSender.send({"WarningName":"Parking Ahead", "WarningID": 10})
        if boolDetections[8]: #Parking Spot
            self.warningSignalSender.send({"WarningName":"Parking Spot Ahead", "WarningID": 3})
        #print("Timer0 : ", timer[0])
        #print("Timer1 : ", timer[1])
        #print("Timer2 : ", timer[2])

        if boolDetections[5]: #real no entry sign implementation
            #trafficComunicationID = 9

            self.warningSignalSender.send({"WarningName":"No Entry Sign", "WarningID": 12})
            desiredSpeed = 0

        if boolDetections[0]:
            if self.parkingSignFlag:
                print("static car on parking spot")

            # print("car")
        
        ######### THIS IS TRAFFIC-LIGHT FOR TESTING ########
        if boolDetections[14] or self.redLightFlag: #THIS IS TRAFIC LIGHT
            #trafficComunicationID = 14 

            print("semaphore detected")


            # self.vehicleState.getPosition()
            if len(self.semaphoresState) > 0:
                if self.semaphoreID is not None:
                    min_distance_state = self.semaphoresState[self.semaphoreID]["state"]
                else:

                    x, y, _ = self.vehicleState.getPosition()

                    min_distance = 1e+5
                    min_distance_state = None

                    for id_, semaphoreState in self.semaphoresState.items():
                        #print(f"Len: {len(self.semaphoresState)}, Semaphores: {str(semaphoreState)}")
                        x_s = semaphoreState["x"]
                        y_s = semaphoreState["y"]

                        distance = np.sqrt((x - x_s)**2 + (y - y_s)**2)

                        if distance < min_distance:
                            min_distance = distance
                            min_distance_state = semaphoreState["state"]
                            self.semaphoreID = id_

                if min_distance_state in("red", "yellow"):
                    print("SHOULD STOP ON TRAFFIC!")
                    print( self.semaphoreID )
                    #self.vehicleState.setSpeed(0)
                    desiredSpeed = 0
                    print(f"desired: {self.vehicleState.getSpeed()}")
                    # isDetected = True
                    self.redLightFlag = True
                         
                    #return 
                else:
                    print("GREEN LIGHT")
                    print(self.semaphoreID)
                    self.redLightFlag = False
                    self.semaphoreID = None

                self.warningSignalSender.send({"WarningName":"Traffic Light Detected", "WarningID": 21})
                    
        if timer[1][0]:
            elapsed = time.perf_counter() - timer[1][1]
            if elapsed >= timer[1][2]:
                timer[1][0] = 0
                timer[1][1] = 0
                timer[1][2] = 0
        if timer[0][0] and not boolDetections[13]:
            elapsed = time.perf_counter() - timer[0][1]
            if elapsed >= timer[0][2]:
                timer[0][0] = 0
                timer[0][1] = 0
                timer[0][2] = 0
        # if timer[0][0]:
        #     isDetected = True
        if timer[2][0]:
            elapsed = time.perf_counter() - timer[2][1]
            if elapsed >= timer[2][2]:
                timer[2][0] = 0
                timer[2][1] = 0
                timer[2][2] = 0
                self.LANEFOL.crosswalk_active = False

        if not isDetected and \
            not self.isOnHighway and \
            not self.redLightFlag and \
            not self.vehicleState.getStateSignal(stateSignalType.IN_INTERSECION) and \
            not self.vehicleState.getStateSignal(stateSignalType.APROACHING_ROADBLOCK) and \
            not self.vehicleState.getStateSignal(stateSignalType.ROADBLOCK_MANEUVER):
                self.vehicleState.setSpeed(25)
            #desiredSpeed = 20
        else:
            self.vehicleState.setSpeed(desiredSpeed)
            

    def run(self):
        PIDStepValue = 0.01

        counter = 0
        counter_max = 5
        prev_lane_center = 960 // 2

        filtered_boxes = np.array([])
        class_ids = np.array([])
        class_scores = np.array([])

        bufferDetections = [0] * 15
        bufferDetections[9] = [0] * 2
        bufferDetections[12] = [0] * 2
        #[isTimerOn, startTime, targetTime]
        timer = [0] * 3
        timer[0] = [0.0] * 3
        timer[1] = [0.0] * 3
        timer[2] = [0.0] * 3

        #distanceF = 99

        # region_curve = np.array( [[ ( 80, 540, ), ( 200, 320), ( 760, 320 ), ( 900, 540 ) ]] ) 
        # region_straight = np.array( [[ ( 90, 540, ), ( 330, 170 ), ( 620, 170 ), ( 890, 540 ) ]] ) 
        # region_straight = np.array( [[ ( 90, 540, ), ( 350, 230 ), ( 620, 230 ), ( 890, 540 ) ]] ) 
        # region_straight = np.array( [[ ( 0, 540, ), ( 0, 300 ), ( 960, 300 ), ( 960, 540 ) ]] ) 
        # region_straight = np.array([[(170, 540), (170, 300), (650, 300), (650, 540)]])
        # region_straight = np.array([[(50, 540), (170, 370), (780, 370), (910, 540)]])

        #create object for curves
        # LANEFOL = LaneFollowing( region_curve )
        #create object for straight road
        self.LANEFOL = LaneFollowing( )

        while self._running:
            #frame = self.MainVideoSubscriber.receive()
            with self.MainVideolock:
                self.frame = self.frameBuffer.copy()
                #np.copyto(self.frame, self.frameBuffer)
            #frame = None
            startLaneDet = self.LaneDetectionStartSubscriber.receive()
            if startLaneDet is not None:
                if startLaneDet == 'false':
                    self.isLaneKeeping = False
                else:
                    self.isLaneKeeping = True
                print("Start Lane Keeping: ", startLaneDet)

            distanceFRecv = self.distanceFrontSubscriber.receive()
            if distanceFRecv is not None:
                self.distanceF = distanceFRecv
                self.vehicleState.setFrontDistanceSensorValue(self.distanceF)

            #DISTANCE_RIGHT
            distanceRRecv = self.distanceRightSubscriber.receive()
            if distanceRRecv is not None:
                self.distanceR = distanceRRecv    

            ##########################
            #semaphoreState = None

            semaphoreState = self.semaphoresSubscriber.receive()
            if semaphoreState is not None:
                #print(f"Semaphore: {semaphoreState}")
                self.semaphoresState[semaphoreState["id"]] = semaphoreState


            if self.frame is not None:
                frame_width = self.frame.shape[1] 

                counter += 1
                detection_frame = self.frame

                if counter >= counter_max:
                    filtered_boxes, class_ids, class_scores = self.detector.detect(detection_frame, showOutput= False)
                    self.makeDecision(zip(filtered_boxes, class_ids, class_scores), bufferDetections, timer)

                    #print(self.vehicleState.getPosition())

                frame_lines, _, _, _, thresh = self.LANEFOL.CannyEdge( self.frame )
    

                frame_lines = apply_gamma_on_frame(frame_lines, gamma=0.5)

                # LANEFOL.perspectiveWarp( frame_lines )

                frame_lines, original_frame = self.LANEFOL.sliding_window_search( thresh, frame_lines )

                lane_center = self.LANEFOL.lane_center

                if abs( prev_lane_center - lane_center ) < 20:
                    lane_center = prev_lane_center

                prev_lane_center = lane_center
                    
                error = lane_center - frame_width // 2
                if error < 25 and error > -25:
                    error = 0

                speedDiff = np.abs(self.vehicleState.getSpeed() - self.initialSpeed)
                # if 5 < speedDiff <= 15:
                #     self.Kp = 0.19
                #     # self.Kp = 0.x
                # elif  15 < speedDiff <= 25:
                #     self.Kp = 0.18
                #     # self.Kp = 0.15
                # else:
                #     self.Kp = 0.20
                    # self.Kp = 0.15

                # print(f"speed;: {self.vehicleState.getSpeed()}  KP: {self.Kp}")

                setKDrecv = self.setKDSubscriber.receive()
                if setKDrecv is not None:
                    self.Kd = int(setKDrecv) / 100
                    print(f"New KD: {self.Kd}")

                setKPRecv = self.setKPSubscriber.receive()
                if setKPRecv is not None:
                    self.Kp = int(setKPRecv) / 100
                    print(f"New KP: {self.Kp}")

                setKIRecv = self.setKISubscriber.receive()
                if setKIRecv is not None:
                    self.Ki = int(setKIRecv) / 100
                    print(f"New Ki: {self.Ki}")
                        
                self.pid.setParameters( self.Kp, self.Ki, self.Kd )
                compute_error = self.pid.pid_formula( error )

                compute_error = max( -25, min( compute_error, 25 ))
                self.kf.correct(compute_error)
                filtriran = self.kf.get_filtered_error()

        

                if counter >= counter_max:
                    if self.isLaneKeeping:
                        self.steerMotorSender.send(str(round(filtriran)*10))
                    #filtered_boxes, class_ids, class_scores = self.detector.detect(detection_frame)
                    counter = 0

                if class_scores.size != 0:
                        frame_lines = self.detector.draw_detections(frame_lines, filtered_boxes, class_ids, class_scores)

                cv2.line(frame_lines, (frame_width // 2, 400), (frame_width // 2, 540), (0, 0, 255), 5)

                with self.LaneVideolock:
                    np.copyto(self.frameLaneBuffer, frame_lines)
                    #np.copyto(self.frameLaneBuffer, detection_frame)


    def stop(self):
        super(threadLaneDetection, self).stop()
        #self.shmLaneVideo.close()
        #self.shmMainVideo.close()
        #Does not work because resource tracker deletes it by itself when process is deleted
        #Fixed in python version 3.13 by adding track = False in shm constructor.. example: self.shm = SharedMemory(name=self.shMemName, track=False)

    def subscribe(self):
        #self.MainVideoSubscriber = messageHandlerSubscriber(self.queuesList, MainVideo, "lastOnly", True)
        self.LaneDetectionStartSubscriber = messageHandlerSubscriber(self.queuesList, startLaneDetection, "lastOnly", True)
        self.shMemResponseSubscriber = messageHandlerSubscriber(self.queuesList, ShMemResponse, "lastOnly", True)
        self.distanceFrontSubscriber = messageHandlerSubscriber(self.queuesList, DistanceFront, "lastOnly", True)
        self.distanceRightSubscriber = messageHandlerSubscriber(self.queuesList, DistanceRight, "lastOnly", True)

        ###########
        self.semaphoresSubscriber = messageHandlerSubscriber( self.queuesList, Semaphores, "fifo", True)
        self.setKDSubscriber = messageHandlerSubscriber(self.queuesList, SetLaneKD, "lastOnly", True)
        self.setKPSubscriber = messageHandlerSubscriber(self.queuesList, SetLaneKP, "lastOnly", True)
        self.setKISubscriber = messageHandlerSubscriber(self.queuesList, SetLaneKI, "lastOnly", True)