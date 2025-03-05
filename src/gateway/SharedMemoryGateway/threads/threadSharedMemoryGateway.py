from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    mainCamera
    )
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from multiprocessing.shared_memory import SharedMemory
from multiprocessing import Lock
import numpy as np
from typing import Dict, Tuple, Type

SharedMemoryBlock = Dict[str, Tuple[SharedMemory, Lock]]
LockType = Lock

class threadSharedMemoryGateway(ThreadWithStop):
    """This thread handles SharedMemoryGateway.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.sharedMemoryBlocks: SharedMemoryBlock = {}
        self.subscribe()
        super(threadSharedMemoryGateway, self).__init__()

    def creteSharedMemory(self, name: str, shape: Tuple[int, int, int], dtype: np.dtype):
        if name in self.sharedMemoryBlocks:
            raise ValueError(f"Shared memory lock '{name}' already exists.")
        
        size = np.prod(shape) * dtype.itemsize
        shm = SharedMemory(name=name, create=True, size=size)
        lock = lock()
        self.sharedMemoryBlocks[name]= (shm, lock)
        return shm, lock
    
    def getSharedMemory(self, name: str):
        if name not in self.sharedMemoryBlocks:
            raise ValueError(f"Shared Memory Block '{name}' does not exist.")
        return self.sharedMemoryBlocks[name]
    
    def releaseSharedMemory(self, name: str):
        if name not in self.sharedMemoryBlocks:
            raise ValueError(f"Shared Memory Block '{name}' does not exist.")
        shm, _ = self.sharedMemoryBlocks[name]
        shm.close()
        shm.unlink()
        del self.sharedMemoryBlocks[name]

    def handleSharedMemoryRequest(self, message: dict):
        pass
        #This function should contain something like waiting to recieve
        #The config through the seperate queue??? or the critical level queue to properly configure it
        #

    def run(self):
        while self._running:
            pass

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
