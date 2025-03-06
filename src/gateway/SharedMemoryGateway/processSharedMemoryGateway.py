if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.gateway.SharedMemoryGateway.threads.threadSharedMemoryGateway import threadSharedMemoryGateway
from multiprocessing import Manager

class processSharedMemoryGateway(WorkerProcess):
    """This process handles SharedMemoryGateway.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.manager = Manager()
        super(processSharedMemoryGateway, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processSharedMemoryGateway, self).run()

    def _init_threads(self):
        """Create the SharedMemoryGateway Publisher thread and add to the list of threads."""
        SharedMemoryGatewayTh = threadSharedMemoryGateway(
            self.queuesList, self.logging, self.manager, self.debugging
        )
        self.threads.append(SharedMemoryGatewayTh)
