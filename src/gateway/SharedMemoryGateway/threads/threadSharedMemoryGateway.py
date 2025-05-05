from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    #createShMem,
    #getShMem,
    #releaseShMem,
    #getVehicleStateObj,
    ShMemConfig,
    ShMemResponse,
    )
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from multiprocessing.shared_memory import SharedMemory
from multiprocessing import Lock, Manager
import numpy as np
from typing import Dict, Tuple, Type
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger

SharedMemoryBlock = Dict[str, Tuple[SharedMemory, Lock]]
LockType = Lock

class threadSharedMemoryGateway(ThreadWithStop):
    """This thread handles SharedMemoryGateway.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, manager, vehicleManager, logger, debugging=False):
        self.queuesList = queueList
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)
        self.sharedMemoryBlocks: SharedMemoryBlock = {}
        self.manager = manager
        self.subscribe()

        self.vehicleManager = vehicleManager
        self.vehicleState = self.vehicleManager.vehicleState()

        self.ShMemResponseSender = messageHandlerSender(self.queuesList, ShMemResponse)
        
        super(threadSharedMemoryGateway, self).__init__()

    def creteSharedMemory(self, name: str, shape: Tuple[int, int, int], dtype: str):
        if name in self.sharedMemoryBlocks:
            raise ValueError(f"Shared memory lock '{name}' already exists.")
        
        np_dtype = np.dtype(dtype)
        
        size = np.prod(shape) * np_dtype.itemsize
        shm = SharedMemory(name=name, create=True, size=size)
        lock = self.manager.Lock()
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
        #shm.close()
        #shm.unlink()
        #del self.sharedMemoryBlocks[name]

        try:
            shm.close()  # ✅ Close first
            shm.unlink()  # ✅ Then remove from system
            del self.sharedMemoryBlocks[name]  # ✅ Remove from dictionary
        except FileNotFoundError:
            self.logger.warning(f"Shared Memory '{name}' was already removed.")
        except Exception as e:
            self.logger.warning(f"Error releasing shared memory '{name}': {e}")

    def handleSharedMemoryRequest(self, message: dict):
        pass
        #This function should contain something like waiting to recieve
        #The config through the seperate queue??? or the critical level queue to properly configure it
        #

    # def run(self):
    #     while self._running:
    #         createShMemResp = self.createShMemSubscriber.receive()
    #         if createShMemResp is not None:
    #             name = createShMemResp.get("name")
    #             shape = createShMemResp.get("shape")
    #             dtype = createShMemResp.get("dtype")
    #             dest = createShMemResp.get("owner")
    #             try:
    #                 shm, lock = self.creteSharedMemory(name, shape, dtype)
    #                 self.ShMemResponseSender.send({"status": 0 , "name": name, "lock": lock, "dest": dest})
    #             except Exception as e:
    #                 self.logging.warning(str(e))

    #         getShMemResp = self.getShMemSubscriber.receive()
    #         if getShMemResp is not None:
    #             try:
    #                 name = getShMemResp.get("name")
    #                 dest = getShMemResp.get("owner")
    #                 shm, lock = self.getSharedMemory(name)
    #                 self.ShMemResponseSender.send({"status": 1 , "name": name, "lock": lock, "dest": dest})
    #             except Exception as e:
    #                 self.logging.warning(str(e))

    #         releaseShMemResp = self.releaseShMemSubscriber.receive()
    #         if releaseShMemResp is not None:
    #             try:
    #                 name = getShMemResp
    #                 self.releaseSharedMemory(name)
    #                 self.ShMemResponseSender.send({"status": 2 , "name": name})
    #             except Exception as e:
    #                 self.logging.warning(str(e))

    #         #print(self.sharedMemoryBlocks.keys())

    #         getVehicleStateObjResp = self.getVehicleStateObjSubscriber.receive()
    #         if getVehicleStateObjResp is not None:
    #             try:
    #                 dest = getVehicleStateObjResp
    #                 self.ShMemResponseSender.send({"status": 3 , "vehicleState": self.vehicleState, "dest": dest})
    #             except Exception as e:
    #                 self.logging.warning(str(e))
    def run(self):
        while self._running:
            shMemConfigResp = self.shMemConfigSubscriber.receive()
            if shMemConfigResp is not None:
                action = shMemConfigResp.get("action")
                if action == "createShMem":
                    try:
                        name = shMemConfigResp.get("name")
                        shape = shMemConfigResp.get("shape")
                        dtype = shMemConfigResp.get("dtype")
                        dest = shMemConfigResp.get("owner")
                        shm, lock = self.creteSharedMemory(name, shape, dtype)
                        self.ShMemResponseSender.send({"status": 0 , "name": name, "lock": lock, "dest": dest})
                    except Exception as e:
                        self.logger.warning(str(e))
                elif action == "getShMem":
                    try:
                        name = shMemConfigResp.get("name")
                        dest = shMemConfigResp.get("owner")
                        shm, lock = self.getSharedMemory(name)
                        self.ShMemResponseSender.send({"status": 1 , "name": name, "lock": lock, "dest": dest})
                    except Exception as e:
                        self.logger.warning(str(e))
                elif action == "releaseShMem":
                    try:
                        name = shMemConfigResp.get("name")
                        self.releaseSharedMemory(name)
                        self.ShMemResponseSender.send({"status": 2 , "name": name})
                    except Exception as e:
                        self.logger.warning(str(e))
                elif action == "getVehicleState":
                    try:
                        dest = shMemConfigResp.get("owner")
                        self.ShMemResponseSender.send({"status": 3 , "vehicleState": self.vehicleState, "dest": dest})
                        #print(f"SharedMemoryGateway received for dest: {dest}")
                    except Exception as e:
                        self.logger.warning(str(e))


    def stop(self):
        for name in list(self.sharedMemoryBlocks.keys()):
            self.releaseSharedMemory(name)
        super(threadSharedMemoryGateway, self).stop()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        #self.createShMemSubscriber = messageHandlerSubscriber(self.queuesList, createShMem, "fifo", True)
        #self.getShMemSubscriber = messageHandlerSubscriber(self.queuesList, getShMem, "fifo", True)
        #self.releaseShMemSubscriber = messageHandlerSubscriber(self.queuesList, releaseShMem, "fifo", True)
        #self.getVehicleStateObjSubscriber = messageHandlerSubscriber(self.queuesList, getVehicleStateObj, "fifo", True)
        self.shMemConfigSubscriber = messageHandlerSubscriber(self.queuesList, ShMemConfig, "fifo", True)
        
