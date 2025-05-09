if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.gateway.SharedMemoryGateway.threads.threadSharedMemoryGateway import threadSharedMemoryGateway
from multiprocessing import Manager
from multiprocessing.managers import BaseManager
from src.driving.PathFollowing.utils.vehicleState import vehicleState
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger

class VehicleStateManager(BaseManager):
    pass

VehicleStateManager.register('vehicleState', vehicleState)

class processSharedMemoryGateway(WorkerProcess):
    """This process handles SharedMemoryGateway.
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
        self.manager = Manager()

        self.vehicleManager = VehicleStateManager()
        self.vehicleManager.start()

        super(processSharedMemoryGateway, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processSharedMemoryGateway, self).run()

    def _init_threads(self):
        """Create the SharedMemoryGateway Publisher thread and add to the list of threads."""
        SharedMemoryGatewayTh = threadSharedMemoryGateway(
            self.queuesList,
            self.manager, 
            self.vehicleManager, 
            self.logger,
            self.debugging
        )
        self.threads.append(SharedMemoryGatewayTh)
