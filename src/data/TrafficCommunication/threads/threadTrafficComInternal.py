from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import (
    TrafficComInternal
)
import time

class threadTrafficCOmInternal(ThreadWithStop):
    """Thread which will handle processTrafficCommunication functionalities

    Args:
        shrd_mem (sharedMem): A space in memory for mwhere we will get and update data.
        queuesList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        deviceID (int): The id of the device.
        decrypt_key (String): A path to the decription key.
    """

    # ====================================== INIT ==========================================
    def __init__(self, shrd_mem, queueslist, debugging = False):
        super(threadTrafficCOmInternal, self).__init__()
        self.queueslist = queueslist
        self.sharedMemory = shrd_mem
        self.debugging = debugging

        self.subscribe()

    # ======================================= RUN ==========================================
    def run(self):
        while self._running:
            trafficRecv = self.trafficComInternalSubscriber.receive()
            if trafficRecv is not None:
                print(f"trafficSend: {trafficRecv}")
                self.sharedMemory.insert(trafficRecv["dataType"], trafficRecv["vals"])

            time.sleep(0.05)

    # ====================================== STOP ==========================================
    def stop(self):
        super(threadTrafficCOmInternal, self).stop()

    def subscribe(self):
        self.trafficComInternalSubscriber = messageHandlerSubscriber(self.queueslist, TrafficComInternal, "fifo", True)