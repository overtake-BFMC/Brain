from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (

    MainVideo,
    LaneVideo,
    startLaneDetection,
    SteerMotor,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.driving.LaneDetection.utils import preprocessing as pre
from src.driving.LaneDetection.utils import PID
from src.driving.LaneDetection.utils import cannyandhough as CH
from src.driving.LaneDetection.utils.gamma import apply_gamma_on_frame
import os
import cv2

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

        self.laneVideoSender = messageHandlerSender(self.queuesList, LaneVideo)

        self.subscribe()

        self.isLaneKeeping = False
        #self.vehicle = vehicleState( 15, 0, 0, 0 )
        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)

        # kalibrisani podaci
        dir_path = os.path.dirname(os.path.realpath(__file__))
        print("dir_path_lane_det: ", dir_path)
        self.ppData, self.maps = pre.loadPPData(dir_path + "/../utils/data")  

        self.pid = PID.PIDController( Kp = 0.27467977371505925, Ki = 0.0008108486849779399, Kd = 0.09678150213579352 )

        super(threadLaneDetection, self).__init__()

    def run(self):
        counter = 0
        counter_max = 5

        while self._running:
            frame = self.MainVideoSubscriber.receive()
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

                edges, frame_lines, lines = CH.CannyEdge( frame )

                frame_lines = apply_gamma_on_frame( frame_lines, gamma=0.5 )

                #frame_bev = pre.bev( frame_lines, self.ppData, self.maps )
                
                lane_center = CH.get_lane_center( lines, frame_lines )

                error = lane_center - frame_width // 2
                if error < 20 and error > -20:
                    error = 0
                compute_error = self.pid.pid_formula( error )
                compute_error = max( -25, min( compute_error, 25 ))

                if counter >= counter_max:
                    #self.vehicle.steering_angle = compute_error
                    if self.isLaneKeeping:
                        self.steerMotorSender.send(str(round(compute_error)*10))
                    print("steering angle je ", compute_error)
                    print("Greska je: ", error)
                    counter = 0

                cv2.line( frame_lines, (frame_width // 2,0), (frame_width // 2, 540), (255,0,0),3  )

                self.laneVideoSender.send(frame_lines)

    def subscribe(self):
        self.MainVideoSubscriber = messageHandlerSubscriber(self.queuesList, MainVideo, "lastOnly", True)
        self.LaneDetectionStartSubscriber = messageHandlerSubscriber(self.queuesList, startLaneDetection, "lastOnly", True)
