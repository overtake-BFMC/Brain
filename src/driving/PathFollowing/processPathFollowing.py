if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.driving.PathFollowing.threads.threadPathFollowing import threadPathFollowing
import logging
from src.utils.logger.loggerConfig import setupLogger

class processPathFollowing(WorkerProcess):
    """This process handles pathFollowing.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, mainLogLevel = logging.INFO, consoleLogLevel = logging.WARNING, debugging = False):
        self.queuesList = queueList
        self.mainLogLevel = mainLogLevel
        self.consoleLogLevel = consoleLogLevel
        self.debugging = debugging
        self.logger = setupLogger(name=__name__, level=self.mainLogLevel, consoleLevel=self.consoleLogLevel)
        super(processPathFollowing, self).__init__(self.queuesList)

        self.lookAheadDistance = 1
        self.dt = 0.2

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processPathFollowing, self).run()

    def _init_threads(self):
        """Create the pathFollowing Publisher thread and add to the list of threads."""
        pathFollowingTh = threadPathFollowing(
            self.queuesList, 
            self.lookAheadDistance, 
            self.dt, 
            self.mainLogLevel, 
            self.consoleLogLevel, 
            self.debugging
        )
        self.threads.append(pathFollowingTh)
