import time
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (

    #MainVideo,
    #LaneVideo,
    startLaneDetection,
    SteerMotor,
    ImuData,
    createShMem,
    getShMem,
    ShMemResponse
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
import torch
import onnxruntime as ort
import numpy as np
import ast
from src.driving.LaneDetection.utils.signDetection import signDetection

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

        self.kf = KalmanFilter_LANE()

        # kalibrisani podaci
        dir_path = os.path.dirname(os.path.realpath(__file__))
        print("dir_path_lane_det: ", dir_path)
        self.ppData, self.maps = pre.loadPPData(dir_path + "/../utils/data")  

        self.pid = PID.PIDController( Kp = 0.1, Ki = 0.05, Kd = 0.003 )

        self.detector = signDetection(conf_thresh=0.5, iou_thresh=0.45)

        self.getShMemSender = messageHandlerSender(self.queuesList, getShMem)
        self.createShMemSender = messageHandlerSender(self.queuesList, createShMem)

        self.shMemNameMainVideo = "mainVideoFrames"
        self.shMemNameLaneVideo = "laneVideoFrames"

        self.init_shMem()

        super(threadLaneDetection, self).__init__()

    def init_shMem(self):
        self.getShMemSender.send(self.shMemNameMainVideo)

        shMemResp = self.shMemResponseSubscriber.receiveWithBlock()
        if shMemResp is not None:
            self.shMemNameMainVideo = shMemResp["name"]
            self.MainVideolock = shMemResp["lock"]
            self.shmMainVideo = SharedMemory(name=self.shMemNameMainVideo)
            self.frameBuffer = np.ndarray((540, 960, 3), dtype=np.uint8, buffer=self.shmMainVideo.buf)

        self.createShMemSender.send({"name": self.shMemNameLaneVideo, "shape": (540, 960, 3), "dtype": "uint8"})

        shMemRespLane = self.shMemResponseSubscriber.receiveWithBlock()
        if shMemRespLane is not None:
            self.shMemNameLaneVideo = shMemRespLane["name"]
            self.LaneVideolock = shMemRespLane["lock"]
            self.shmLaneVideo = SharedMemory(self.shMemNameLaneVideo)
            self.frameLaneBuffer = np.ndarray((540, 960, 3), dtype=np.uint8, buffer=self.shmLaneVideo.buf)


    def run(self):
        counter = 0
        counter_max = 5
        prev_lane_center = 960 // 2

        filtered_boxes = np.array([])
        class_ids = np.array([])
        class_scores = np.array([])

        while self._running:
            #frame = self.MainVideoSubscriber.receive()
            with self.MainVideolock:
                frame = self.frameBuffer.copy()
            #frame = None
            startLaneDet = self.LaneDetectionStartSubscriber.receive()
            if startLaneDet is not None:
                if startLaneDet == 'false':
                    self.isLaneKeeping = False
                else:
                    self.isLaneKeeping = True
                print("Start Lane Keeping: ", startLaneDet)
            if frame is not None:
                frame_width = frame.shape[1] 

                counter += 1
                detection_frame = frame
                edges, frame_lines, lines = CH.CannyEdge( frame )

                frame_lines = apply_gamma_on_frame( frame_lines, gamma=0.5 )

                #frame_bev = pre.bev( frame_lines, self.ppData, self.maps )
                
                left_lines, right_lines, lane_center = CH.get_lane_center( lines, frame_lines, prev_lane_center )

               
                prev_lane_center = lane_center

                    
                error = lane_center - frame_width // 2
                if error < 20 and error > -20:
                    error = 0
                compute_error = self.pid.pid_formula( error )
                compute_error = max( -25, min( compute_error, 25 ))
                self.kf.correct(compute_error)
                filtriran = self.kf.get_filtered_error()
            

                if counter >= counter_max:
                    #self.vehicle.steering_angle = compute_error
                    if self.isLaneKeeping:
                        self.steerMotorSender.send(str(round(filtriran)*10))
                    #print("steering angle filtriran je ", filtriran)
                    #print("steering angle je ", compute_error)
                    #print("Greska je: ", error)
                    filtered_boxes, class_ids, class_scores = self.detector.detect(detection_frame)
                    counter = 0

                if class_scores.size != 0:
                        frame_lines = self.detector.draw_detections(frame_lines, filtered_boxes, class_ids, class_scores)
                #detected = self.detectionModel.predict(source=frame_lines, conf=0.5)
                #for r in detected:
                #    print(r.boxes.data)
                #signDetectFrame = cv2.resize(frame, (640, 640))
                #signDetectFrame = signDetectFrame.astype(np.float32) / 255.0
                #signDetectFrame = np.transpose(signDetectFrame, (2, 0, 1))
                #signDetectFrame = np.expand_dims(signDetectFrame, axis=0)

                #outputs = self.detectionModelSession.run(self.detectionOutputNames, {"images": signDetectFrame})
                #print(outputs)
                #boxes, confidences, class_ids = self.decode_yolo_output(outputs, frame.shape[0], frame.shape[1])
                #print("Boxes: ", boxes)
                #print("Confidences: ", confidences)
                #print("Class IDs:", class_ids)
                #print("Output Shape:", np.array(outputs).shape)
                #print("Sample Output Data:", outputs[0][:5])  # Print first 5 predictions
                cv2.line( frame_lines, (frame_width // 2,0), (frame_width // 2, 540), (255,0,0),3  )
                # detected_image = self.detector.draw_detections(frame, filtered_boxes, class_ids, class_scores)

                #self.laneVideoSender.send(frame_lines)
                # self.laneVideoSender.send(detected_image)
                with self.LaneVideolock:
                    self.frameLaneBuffer[:] = frame_lines
                        #self.frameLaneBuffer[:] = detected_image

    def stop(self):
        super(threadLaneDetection, self).stop()
        with self.MainVideolock:    
            self.shmMainVideo.close()
        with self.LaneVideolock:
            self.shmLaneVideo.close()

    def subscribe(self):
        #self.MainVideoSubscriber = messageHandlerSubscriber(self.queuesList, MainVideo, "lastOnly", True)
        self.LaneDetectionStartSubscriber = messageHandlerSubscriber(self.queuesList, startLaneDetection, "lastOnly", True)
        self.shMemResponseSubscriber = messageHandlerSubscriber(self.queuesList, ShMemResponse, "lastOnly", True)
