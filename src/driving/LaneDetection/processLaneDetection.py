if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.driving.LaneDetection.threads.threadLaneDetection import threadLaneDetection
import logging
from src.utils.logger.loggerConfig import setupLogger

class processLaneDetection(WorkerProcess):
    """This process handles laneDetection.
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
        super(processLaneDetection, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processLaneDetection, self).run()

    def _init_threads(self):
        """Create the laneDetection Publisher thread and add to the list of threads."""
        laneDetectionTh = threadLaneDetection(
            self.queuesList,
            self.mainLogLevel, 
            self.consoleLogLevel, 
            self.debugging
        )
        self.threads.append(laneDetectionTh)
