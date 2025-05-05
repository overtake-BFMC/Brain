from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    WebRTCAnswer,
    WebRTCOffer,
    ICECandidate,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from aiortc import RTCPeerConnection, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc import RTCSessionDescription
from src.dashboard.StreamRTC.threads.trackStream import trackStream
import asyncio

class threadStreamRTC(ThreadWithStop):
    """This thread handles processStreamRTC.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logger, debugging=False):
        self.queuesList = queueList
        self.logger = logger
        self.debugging = debugging

        #self.pcs = set()
        #self.RTCconfig = RTCConfiguration([])

        #self.WebRTCAnswerSender = messageHandlerSender(self.queuesList, WebRTCAnswer)
        #self.subscribe()

        super(threadStreamRTC, self).__init__()

    def run(self):
        while self._running:
            pass
        #loop = asyncio.new_event_loop()
        #asyncio.set_event_loop(loop)
        #loop.run_until_complete(self.main_loop())

    #async def main_loop(self):
    #    while self._running:
    #        try:
    #            ICECandidate = self.ICECandidateSubscriber.receive()
    #            if ICECandidate is not None:
    #                await self._handleIceCandidateAsync(ICECandidate)
#
    #        except Exception as e:
    #            print(e)

#    async def _handleIceCandidateAsync(self, data):
#        print("Received ICE candidate from client:", data["candidate"])
#        pc = next(iter(self.pcs))
 #       candidate_dict = self.parse_ice_candidate(data["candidate"]["candidate"])

        # Create the RTCIceCandidate object
#        ice_candidate = RTCIceCandidate(
#            foundation=candidate_dict["foundation"],
##            component=candidate_dict["component"],
 #           protocol=candidate_dict["protocol"],
 #           priority=candidate_dict["priority"],
 #           ip=candidate_dict["ip"],
  #          port=candidate_dict["port"],
  #          type=candidate_dict["type"],
  #          sdpMid=data["candidate"]["sdpMid"],
  #          sdpMLineIndex=data["candidate"]["sdpMLineIndex"],
  #      )

        # Add the ICE candidate to the peer connection
 #       await pc.addIceCandidate(ice_candidate)

    def parse_ice_candidate(self, candidate_str):
        """
        Parse an ICE candidate string into its components.
        Example input: "candidate:1234567890 1 udp 2122194687 192.168.1.100 50005 typ host"
        """
        parts = candidate_str.split()
        if len(parts) < 8:
            raise ValueError("Invalid ICE candidate string")

        return {
            "foundation": parts[0].split(":")[1],  # Extract foundation from "candidate:1234567890"
            "component": int(parts[1]),  # Component ID (1 for RTP, 2 for RTCP)
            "protocol": parts[2],  # Protocol (udp/tcp)
            "priority": int(parts[3]),  # Priority
            "ip": parts[4],  # IP address
            "port": int(parts[5]),  # Port
            "type": parts[7],  # Candidate type (host/srflx/relay)
        }
    
    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.WebRTCOfferSubscriber = messageHandlerSubscriber(self.queuesList, WebRTCOffer, "lastOnly", True )
        self.ICECandidateSubscriber = messageHandlerSubscriber(self.queuesList, ICECandidate, "lastOnly", True )
