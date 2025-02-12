from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    WebRTCAnswer,
    WebRTCOffer,
    ICECandidate,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

class threadwebRTC(ThreadWithStop):
    """This thread handles webRTC.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        super(threadwebRTC, self).__init__()
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        self.subscribe()
        #super(threadwebRTC, self).__init__()

    def run(self):
        while self._running:
            pass

    def subscribe(self):
        pass
