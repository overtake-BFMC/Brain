from src.utils.messages.allMessages import (

    MainVideo,
    LaneVideo,
)

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from aiortc import MediaStreamTrack
import av
import time
import asyncio
from fractions import Fraction
class trackStream(MediaStreamTrack):
    
    kind = 'video'

    def __init__(self, queueList, logging, debugging = False):
        super(trackStream, self).__init__()

        self.queueList = queueList
        self.logger = logging
        self.debugging = debugging
        #self.track = track
        self.start_time = time.time()
        self.frame_count = 0
        #self.MainVideoSubscriber = messageHandlerSubscriber(self.queueList, MainVideo, "lastOnly", True)
        self.LaneVideoSubcscriber = messageHandlerSubscriber(self.queueList, LaneVideo, "lastOnly", True)

    async def recv(self):
        while True:
            #frame = self.MainVideoSubscriber.receive()
            frame = self.LaneVideoSubcscriber.receive()
            if frame is not None:
                
                #SendFrame = await self.track.recv()
                #return SendFrame
                #return await self._generate_blank_frame()
                av_frame = av.VideoFrame.from_ndarray(frame, format='bgr24')
                self.frame_count += 1
                av_frame.pts = self.frame_count
                av_frame.time_base = Fraction(1, 30)
                return av_frame

            if self.debugging:
                self.logger.warning("No frame received from the message queue")
            await asyncio.sleep(0.01)
            #blank_frame = await self._generate_blank_frame()
            #self.frame_count += 1
            #blank_frame.pts = self.frame_count
            #blank_frame.time_base = Fraction(1, 30)

            #return blank_frame

    def stop(self):
        self.LaneVideoSubcscriber.unsubscribe()
        super().stop()
    
    async def _generate_blank_frame(self):
        """Generate a blank frame to avoid breaking the stream."""
        blank_frame = av.VideoFrame(width=640, height=480, format='bgr24')
        for plane in blank_frame.planes:
            plane.update(bytes(plane.buffer_size))
        return blank_frame