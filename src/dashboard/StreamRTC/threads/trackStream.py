from src.utils.messages.allMessages import (

    #MainVideo,
    #LaneVideo,
    ShMemConfig,
    ShMemResponse,
)

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from aiortc import MediaStreamTrack
import av
import time
import asyncio
from fractions import Fraction
from multiprocessing.shared_memory import SharedMemory
import numpy as np
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger
class trackStream(MediaStreamTrack):
    
    kind = 'video'

    def __init__(self, queueList, logger, debugging = False):
        super(trackStream, self).__init__()

        self.queueList = queueList
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)
        #self.track = track
        self.start_time = time.time()
        self.frame_count = 0
        self.isVideoConnected = False
        self.frameInterval = 1.0 / 20.0

        self.frame = np.empty((540, 960, 3), dtype=np.uint8)
        #self.MainVideoSubscriber = messageHandlerSubscriber(self.queueList, MainVideo, "lastOnly", True)
        #self.LaneVideoSubcscriber = messageHandlerSubscriber(self.queueList, LaneVideo, "lastOnly", True)
        self.shMemResponseSubscriber = messageHandlerSubscriber(self.queueList, ShMemResponse, "lastOnly", True)

        self.ShMemConfigSender = messageHandlerSender(self.queueList, ShMemConfig)
        self.shMemName = "laneVideoFrames"
        self.shMemMsgOwner = "trackStream"
        time.sleep(1)

        self.init_shMem()

    def init_shMem(self):
        self.ShMemConfigSender.send({"action": "getShMem", "name": self.shMemName, "owner": self.shMemMsgOwner})

        isMemoryConfigured = False
        while not isMemoryConfigured:
            shMemResp = self.shMemResponseSubscriber.receiveWithBlock()
            if shMemResp is not None:
                if shMemResp["status"] == 1 and shMemResp["dest"] == self.shMemMsgOwner:
                    self.shMemName = shMemResp["name"]
                    self.lock = shMemResp["lock"]
                    self.shm = SharedMemory(name=self.shMemName)
                    self.frameBuffer = np.ndarray((540, 960, 3), dtype=np.uint8, buffer=self.shm.buf)
                    isMemoryConfigured = True
                    #print("trackStream LaneVideoShMem Init Success!")
                    self.logger.info("trackStream LaneVideoShMem Init Success!")
                    self.isVideoConnected = True

    async def recv(self):
        while self.isVideoConnected:
            #frame = self.MainVideoSubscriber.receive()
            #frame = self.LaneVideoSubcscriber.receive()
            self.start_time = time.time()
            with self.lock:
                #frame = self.frameBuffer.copy()
                np.copyto(self.frame, self.frameBuffer)
            #frame = None
            if self.frame is not None:
                
                #SendFrame = await self.track.recv()
                #return SendFrame
                #return await self._generate_blank_frame()
                av_frame = av.VideoFrame.from_ndarray(self.frame, format='bgr24')
                self.frame_count += 1
                av_frame.pts = self.frame_count
                av_frame.time_base = Fraction(1, 15)
                return av_frame

            if self.debugging:
                self.logger.warning("No frame received from the message queue")
            elapsedTime = time.time() - self.start_time
            sleepTime = max(0, self.frameInterval - elapsedTime)
            await asyncio.sleep(sleepTime)
            #await asyncio.sleep(0.01)
            #blank_frame = await self._generate_blank_frame()
            #self.frame_count += 1
            #blank_frame.pts = self.frame_count
            #blank_frame.time_base = Fraction(1, 30)

            #return blank_frame

    def stop(self):
        #self.LaneVideoSubcscriber.unsubscribe()
        #self.shm.close()
        super().stop()
    
    async def _generate_blank_frame(self):
        """Generate a blank frame to avoid breaking the stream."""
        blank_frame = av.VideoFrame(width=640, height=480, format='bgr24')
        for plane in blank_frame.planes:
            plane.update(bytes(plane.buffer_size))
        return blank_frame