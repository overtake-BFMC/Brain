if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.dashboard.StreamRTC.threads.threadStreamRTC import threadStreamRTC

from src.utils.messages.allMessages import (
    WebRTCAnswer,
    WebRTCOffer,
    ICECandidate,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from aiortc import RTCPeerConnection, RTCIceCandidate, RTCConfiguration
from aiortc import RTCSessionDescription
from src.dashboard.StreamRTC.threads.trackStream import trackStream
import asyncio
import uuid
import os
import setproctitle
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger

class processStreamRTC(WorkerProcess):
    """This process handles processStreamRTC.
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
        super(processStreamRTC, self).__init__(self.queuesList)

        self.pcs = set()
        self.RTCconfig = RTCConfiguration([])

        self.WebRTCAnswerSender = messageHandlerSender(self.queuesList, WebRTCAnswer)

        self.subscribe()

        #self.running = True

        self.dir_path = os.path.dirname(os.path.realpath(__file__))

    def start(self):
        self.running = True
        super(processStreamRTC, self).start()
    
    def stop(self):
        self.running = False  # Stop the main loop
        super(processStreamRTC, self).stop()

    def run(self):
        """Apply the initializing methods and start the threads."""
        asyncio.run(self.main_loop())
        #super(processStreamRTC, self).run()

    async def main_loop(self):
        while self.running:
            try:
                RTCoffer = self.WebRTCOfferSubscriber.receive()
                if RTCoffer is not None:
                    await self.process_offer(RTCoffer)
                ICECandidateSTR = self.ICECandidateSubscriber.receive()
                if ICECandidateSTR is not None:
                    await self._handleIceCandidateAsync(ICECandidateSTR)
                await asyncio.sleep(0.01)
            except asyncio.CancelledError:
                print("Task was cancelled")
            except KeyboardInterrupt:
                print("Process interrupted")
        await self.cleanup()

    async def cleanup(self):
        for pc in self.pcs:
            await pc.close()
        self.pcs.clear()

    async def process_offer(self, RTCoffer):
        #logger = logging.getLogger("stream")

        pc = RTCPeerConnection(self.RTCconfig)
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        self.pcs.add(pc)

        def log_info(msg, *args):
            self.logger.info(pc_id + " " + msg, *args)

        log_info("Created for Angular")



        pc.addTrack(trackStream(self.queuesList, self.logger, self.debugging))
        #player = MediaPlayer(self.dir_path + '/demo-instruct.wav')
        #pc.addTrack(player.audio)
        log_info("Created track stream")
        await pc.setRemoteDescription(RTCSessionDescription(sdp=RTCoffer, type='offer'))
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        self.WebRTCAnswerSender.send({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})

        log_info("Connection state is %s", pc.connectionState)

    async def _handleIceCandidateAsync(self, data):
        print("Received ICE candidate from client:", data["candidate"])
        pc = next(iter(self.pcs))
        candidate_dict = self.parse_ice_candidate(data["candidate"]["candidate"])

        ice_candidate = RTCIceCandidate(
            foundation=candidate_dict["foundation"],
            component=candidate_dict["component"],
            protocol=candidate_dict["protocol"],
            priority=candidate_dict["priority"],
            ip=candidate_dict["ip"],
            port=candidate_dict["port"],
            type=candidate_dict["type"],
            sdpMid=data["candidate"]["sdpMid"],
            sdpMLineIndex=data["candidate"]["sdpMLineIndex"],
        )

        # Add the ICE candidate to the peer connection
        await pc.addIceCandidate(ice_candidate)

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

    def _init_threads(self):
        """Create the processStreamRTC Publisher thread and add to the list of threads."""
        pass
        # processStreamRTCTh = threadStreamRTC(
        #     self.queuesList, self.logger, self.debugging
        # )
        # self.threads.append(processStreamRTCTh)

    def subscribe(self):
        self.WebRTCOfferSubscriber = messageHandlerSubscriber(self.queuesList, WebRTCOffer, "lastOnly", True )
        self.ICECandidateSubscriber = messageHandlerSubscriber(self.queuesList, ICECandidate, "lastOnly", True )
