if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.driving.PathFollowing.threads.threadPathFollowing import threadPathFollowing

class processPathFollowing(WorkerProcess):
    """This process handles pathFollowing.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processPathFollowing, self).__init__(self.queuesList)

        self.lookAheadDistance = 1
        self.dt = 0.2

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processPathFollowing, self).run()

    def _init_threads(self):
        """Create the pathFollowing Publisher thread and add to the list of threads."""
        pathFollowingTh = threadPathFollowing(
            self.queuesList, self.logging, self.lookAheadDistance, self.dt, self.debugging
        )
        self.threads.append(pathFollowingTh)
