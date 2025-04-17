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
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.driving.LaneDetection.utils import preprocessing as pre
from src.driving.LaneDetection.utils import PID
from src.driving.LaneDetection.utils import cannyandhough as CH
from src.driving.LaneDetection.utils.gamma import apply_gamma_on_frame
from src.driving.LaneDetection.utils.kalman_filter import KalmanFilter_LANE
from multiprocessing.shared_memory import SharedMemory
import numpy as np
import os
import cv2
#from ultralytics import YOLO
import numpy as np
import ast
from src.driving.LaneDetection.utils.signDetection import signDetection
from src.driving.LaneDetection.utils.signDetection import CLASSES

from src.driving.LaneDetection.utils.cannyandhough import LaneFollowing

CLASSES_ROI = [
    [0, 0, 960, 540], #car 0
    [0, 0, 960, 540], #closed-road-stand 1
    [690, 90, 900, 220], #crosswalk-sign 2
    [700, 90, 900, 220], #highway-entry-sign 3
    [700, 90, 900, 220], #highway-exit-sign 4
    [700, 90, 900, 220], #no-entry-road-sign 5 
    [700, 90, 900, 220], #one-way-road-sign 6
    [690, 70, 920, 220], #parking-sign 7
    [0, 0, 960, 540], #parking-spot 8
    [[150, 150, 650, 400],[50, 150, 900, 400]], #pedestrian 9
    [700, 90, 900, 220], #priority-sign 10
    [700, 90, 900, 220], #round-about-sign 11
    [[300, 400, 660, 540], [300, 200, 660, 540]],#[400, 400, 550, 540], #stop-line 12
    [480, 0, 960, 540], #stop-sign 13 
    [0, 0, 960, 540], #traffic-light 14
]

class threadLaneDetection(ThreadWithStop):
    """This thread handles laneDetection.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        #self.laneVideoSender = messageHandlerSender(self.queuesList, LaneVideo)

        self.subscribe()

        self.isLaneKeeping = False
        #self.vehicle = vehicleState( 15, 0, 0, 0 )
        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.warningSignalSender = messageHandlerSender(self.queuesList, WarningSignal)

        self.kf = KalmanFilter_LANE()

        # kalibrisani podaci
        dir_path = os.path.dirname(os.path.realpath(__file__))
        print("dir_path_lane_det: ", dir_path)
        self.ppData, self.maps = pre.loadPPData(dir_path + "/../utils/data")  

        self.pid = PID.PIDController( Kp = 0.13, Ki = 0.03, Kd = 0.09 )

        self.detector = signDetection(conf_thresh=0.5, iou_thresh=0.45)
        self.noOfDetections = [0] * 15
        self.frame = np.empty((540, 960, 3), dtype=np.uint8)

        self.ShMemConfigSender = messageHandlerSender(self.queuesList, ShMemConfig)

        self.shMemNameMainVideo = "mainVideoFrames"
        self.shMemNameLaneVideo = "laneVideoFrames"
        self.shMemMsgOwner = "threadLaneDetection"

        self.isOnHighway = False
        self.isStop = False

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
                    self.logging.info("LaneDetection MainVideoShMem Init Success!")
                    self.ShMemConfigSender.send({"action": "createShMem", "name": self.shMemNameLaneVideo, "shape": (540, 960, 3), "dtype": "uint8", "owner": self.shMemMsgOwner})

            if shMemResp is not None:
                if shMemResp["status"] == 0 and shMemResp["dest"] == self.shMemMsgOwner:
                    self.shMemNameLaneVideo = shMemResp["name"]
                    self.LaneVideolock = shMemResp["lock"]
                    self.shmLaneVideo = SharedMemory(self.shMemNameLaneVideo)
                    self.frameLaneBuffer = np.ndarray((540, 960, 3), dtype=np.uint8, buffer=self.shmLaneVideo.buf)
                    isLaneVideoMemoryConfigured = True
                    #print("LaneDetection LaneVideoShMem Init Success!")
                    self.logging.info("LaneDetection LaneVideoShMem Init Success!")

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
                    self.logging.info("LaneDetection VehicleState Initialized!")

    def isInsideROI(self, x, y, ROIBox):
        return ROIBox[0] <= x <= ROIBox[2] and ROIBox[1] <= y <= ROIBox[3]

    def makeDecision(self, detections, bufferDetections, distanceF, timer):
        
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


        if boolDetections[2]: #crosswalk-sign
            isDetected = True
            #self.vehicleState.setSpeed(10)
            self.isLaneKeeping = False ############################PROMENA
            desiredSpeed = 10
            self.warningSignalSender.send({"WarningName":"Crosswalk Ahead", "WarningID": 4})
        if boolDetections[12][0] and boolDetections[9][1]: #stop-line and pedestrian
            #self.vehicleState.setSpeed(0)
            self.isLaneKeeping = False ##################PROMENA
            desiredSpeed = 0
            isDetected = True
            self.warningSignalSender.send({"WarningName":"Pedestrian on Crosswalk", "WarningID": 11})
        if boolDetections[3]: #highwayEntry
            desiredSpeed = 40
            self.isLaneKeeping = True #############################################PROMENA

            self.isOnHighway = True
            self.warningSignalSender.send({"WarningName":"Highway Ahead", "WarningID": 6})
        if boolDetections[4]:
            self.isOnHighway = False
            self.warningSignalSender.send({"WarningName":"Highway Exit", "WarningID": 7})
        if boolDetections[9][0]: #pedestrian
            desiredSpeed = 0
            isDetected = True
            self.warningSignalSender.send({"WarningName":"Pedestrian On Road", "WarningID": 12})
        if boolDetections[12][1] and boolDetections[13]: #stop-sign and stop line
            self.isLaneKeeping = False
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
        if boolDetections[10]: #priority
            self.warningSignalSender.send({"WarningName":"Priority Ahead", "WarningID": 13})
            timer[2][0] = 1
            timer[2][1] = time.perf_counter()
            timer[2][2] = 6.0
        if boolDetections[11]: #roundabout
            self.isLaneKeeping = False
            self.warningSignalSender.send({"WarningName":"Roundabout Ahead", "WarningID": 15})
        if boolDetections[7]: #Parking Sign
            self.warningSignalSender.send({"WarningName":"Parking Ahead", "WarningID": 10})
            self.isLaneKeeping = True
        if boolDetections[8]: #Parking Spot
            self.warningSignalSender.send({"WarningName":"Parking Spot Ahead", "WarningID": 3})
        #print("Timer0 : ", timer[0])
        #print("Timer1 : ", timer[1])
        #print("Timer2 : ", timer[2])

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
        if timer[0][0]:
            isDetected = True
        if timer[2][0]:
            elapsed = time.perf_counter() - timer[2][1]
            if elapsed >= timer[2][2]:
                timer[2][0] = 0
                self.isLaneKeeping = True

        #print(f"DesiredSpeed: {desiredSpeed}, isDetected {isDetected}")

        if not isDetected and not self.isOnHighway:
            self.vehicleState.setSpeed(20)
            #desiredSpeed = 20
        else:
            self.vehicleState.setSpeed(desiredSpeed)
            

    def run(self):
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

        distanceF = 99

        LANEFOL = LaneFollowing()

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
                distanceF = distanceFRecv

            if self.frame is not None:
                frame_width = self.frame.shape[1] 

                counter += 1
                detection_frame = self.frame

                if counter >= counter_max:
                    filtered_boxes, class_ids, class_scores = self.detector.detect(detection_frame, showOutput= False)
                    self.makeDecision(zip(filtered_boxes, class_ids, class_scores), bufferDetections, distanceF, timer)

                    #print(self.vehicleState.getPosition())

                # frame_lines, lines = CH.CannyEdge( self.frame )

                frame_lines, lines = LANEFOL.CannyEdge( self.frame )

                frame_lines = apply_gamma_on_frame( frame_lines, gamma=0.45 )

                #frame_bev = pre.bev( frame_lines, self.ppData, self.maps )
                
                # left_lines, right_lines, lane_center = CH.get_lane_center( lines, frame_lines, prev_lane_center )

                lane_center = LANEFOL.get_lane_center( lines, frame_lines, prev_lane_center)

                if abs( prev_lane_center - lane_center ) < 20:
                    lane_center = prev_lane_center


                prev_lane_center = lane_center
                    
                error = lane_center - frame_width // 2
                if error < 20 and error > -20:
                    error = 0
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

                cv2.line( frame_lines, (frame_width // 2,0), (frame_width // 2, 540), (255,0,0),3  )

                with self.LaneVideolock:
                    np.copyto(self.frameLaneBuffer, frame_lines)
                    #self.frameLaneBuffer[:] = frame_lines
                    #self.frameLaneBuffer[:] = detection_frame


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
