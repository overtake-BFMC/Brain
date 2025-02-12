if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.dashboard.StreamRTC.threads.threadStreamRTC import threadStreamRTC

class processStreamRTC(WorkerProcess):
    """This process handles processStreamRTC.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processStreamRTC, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processStreamRTC, self).run()

    def _init_threads(self):
        """Create the processStreamRTC Publisher thread and add to the list of threads."""
        processStreamRTCTh = threadStreamRTC(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(processStreamRTCTh)
