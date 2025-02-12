import subprocess
import os
from src.templates.threadwithstop import ThreadWithStop

from src.utils.messages.allMessages import (
    WebRTCAnswer,
    WebRTCOffer,
    ICECandidate,
)
import threading
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber

from aiortc import RTCPeerConnection, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc import RTCSessionDescription
from dashboard.webRTC.threads.trackStream import processStream
import asyncio

class threadStartWebRTC(ThreadWithStop):

    def __init__(self, queuesList, logger, debugger):
        super(threadStartWebRTC, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger

        self.pcs = set()
        self.RTCconfig = RTCConfiguration([])

        self.stream = processStream(self.queuesList, self.logger, debugging = True)
        
        self.WebRTCAnswerSender = messageHandlerSender(self.queuesList, WebRTCAnswer)
        #self.ICECandidateSender = messageHandlerSender(self.queuesList, ICECandidate)

        self.WebRTCOfferSubscriber = messageHandlerSubscriber(self.queuesList, WebRTCOffer, "lastOnly", True )
        self.ICECandidateSubscriber = messageHandlerSubscriber(self.queuesList, ICECandidate, "lastOnly", True )

# =============================== STOP ================================================
    def stop(self):
        super(threadStartWebRTC, self).stop()

# ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True. It captures the image from camera and makes the required modifications and sends the data to process gateway."""
        while self._running:
            try:
                RTCoffer = self.WebRTCOfferSubscriber.receive()
                if RTCoffer is not None:
                    pc = RTCPeerConnection(self.RTCconfig)
                    self.pcs.add(pc)

                    #stream = processStream(self.queuesList, self.logger, debugging = True)
                    pc.addTrack(self.stream)

                    @pc.on("icecandidate")
                    def on_ice_candidate(candidate):
                        if candidate:
                            print(f"ICE Candidate: {candidate.to_dict()}")
                        else:
                            print("ICE gathering complete, no candidates found.")

                    @pc.on("icegatheringstatechange")
                    def on_ice_gathering_state_change():
                        print(f"ICE Gathering State: {pc.iceGatheringState}")

                    async def process_offer():
                        await pc.setRemoteDescription(RTCSessionDescription(sdp=RTCoffer["sdp"], type=RTCoffer["type"]))
                        print(f"Received SDP: {RTCoffer['sdp']}")
                        print(f"Type: {RTCoffer['type']}")
                        answer = await pc.createAnswer()
                        await pc.setLocalDescription(answer)
                        print(f"Answer: {answer.sdp}")
                        await asyncio.sleep(2)
                        self.socketio.emit("answer", {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})
                        
                    asyncio.run(process_offer())

            except Exception as e:
                print(e)


# =============================== START ===============================================
    def start(self):
        super(threadStartWebRTC, self).start()

#MOVE THIS THREAD TO SEPERATE PROCESS AND THAN TALK WITH IT, proccessDashboard can only be on one thread 