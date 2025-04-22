if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.dashboard.webRTC.threads.threadwebRTC import threadwebRTC

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

import argparse
import asyncio
import json
import logging
import os
import ssl
import uuid

import cv2
from aiohttp import web
import aiohttp_cors
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder, MediaRelay
from av import VideoFrame
from src.dashboard.StreamRTC.threads.trackStream import trackStream

logger = logging.getLogger("pc")

class processWebRTC(WorkerProcess):
    """This process handles webRTC.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processWebRTC, self).__init__(self.queuesList)

        self.ROOT = os.path.dirname(__file__)
        self.pcs = set()
        self.relay = MediaRelay()

        self.ssl_context = None

        self.app = web.Application()
        self.cors = aiohttp_cors.setup(self.app)
        self.app.on_shutdown.append(self.on_shutdown)
        self.cors.add(self.app.router.add_get("/", self.index)), {
            "http://192.168.66.155:4200": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            )
        }
        self.app.router.add_get("/client.js", self.javascript)
        self.cors.add(self.app.router.add_post("/offer", self.offer)), {
            "http://192.168.66.155:4200": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            )
        }

    def run(self):
        """Apply the initializing methods and start the threads."""
        web.run_app(
            self.app, access_log=None, host="0.0.0.0", port=8085, ssl_context=self.ssl_context
        )
        super(processWebRTC, self).run()

    async def index(self, request):
        content = open(os.path.join(self.ROOT, "index.html"), "r").read()
        return web.Response(content_type="text/html", text=content)
    
    async def javascript(self, request):
        content = open(os.path.join(self.ROOT, "client.js"), "r").read()
        return web.Response(content_type="application/javascript", text=content)
    
    async def offer(self, request):
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        self.pcs.add(pc)

        def log_info(msg, *args):
            logger.info(pc_id + " " + msg, *args)

        log_info("Created for %s", request.remote)

        @pc.on("datachannel")
        def on_datachannel(channel):
            @channel.on("message")
            def on_message(message):
                if isinstance(message, str) and message.startswith("ping"):
                    channel.send("pong" + message[4:])

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            log_info("Connection state is %s", pc.connectionState)
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)

        @pc.on("track")
        def on_track(track):
            log_info("Track %s received", track.kind)

            if track.kind == "audio":
                print("Audio")
            elif track.kind == "video":
                pc.addTrack(
                    trackStream(self.queuesList, self.logging, debugging = False)
                )

        # handle offer
        await pc.setRemoteDescription(offer)

        # send answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )
    
    async def on_shutdown(self, app):
        # close peer connections
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()

    def _init_threads(self):
        """Create the webRTC Publisher thread and add to the list of threads."""
        webRTCTh = threadwebRTC(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(webRTCTh)
