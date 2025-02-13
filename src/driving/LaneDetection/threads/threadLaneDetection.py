from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (

    MainVideo,
    LaneVideo,
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

        # kalibrisani podaci
        dir_path = os.path.dirname(os.path.realpath(__file__))
        print("dir_path_lane_det: ", dir_path)
        self.ppData, self.maps = pre.loadPPData(dir_path + "/../utils/data")  

        self.pid = PID.PIDController( Kp = 0.27467977371505925, Ki = 0.0008108486849779399, Kd = 0.09678150213579352 )
        self.total_error = 0

        super(threadLaneDetection, self).__init__()

    def run(self):
        while self._running:
            frame = self.MainVideoSubscriber.receive()
            #frame = None
            if frame is not None:
                frame_width = frame.shape[1] 

                edges, frame_lines, lines = CH.CannyEdge( frame )

                frame_lines = apply_gamma_on_frame( frame_lines, gamma=0.5 )

                frame_bev = pre.bev( frame_lines, self.ppData, self.maps )
                
                lane_center = CH.get_lane_center( lines, frame_lines )

                error = lane_center - frame_width // 2

                self.total_error += error

                compute_error = self.pid.pid_formula( error )
                compute_error = max( -25, min( compute_error, 25 ))

                #print("error je", error )
                #print("steering angle je ", compute_error)

                cv2.line( frame_lines, (frame_width // 2,0), (frame_width // 2, 540), (255,0,0),3  )

                self.laneVideoSender.send(frame_lines)

    def subscribe(self):
        self.MainVideoSubscriber = messageHandlerSubscriber(self.queuesList, MainVideo, "lastOnly", True)
