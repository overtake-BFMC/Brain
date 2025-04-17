if __name__ == "__main__":
    import sys
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from src.templates.workerprocess import WorkerProcess
from src.hardware.canhandler.threads.threadCANRead import threadCANRead
from src.hardware.canhandler.threads.threadCANWrite import threadCANWrite
import can
from src.hardware.canhandler.threads.filehandler import FileHandler

class processCANHandler(WorkerProcess):
    """This process handles canhandler.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        canName = 'can0'
        logFile = "historyCAN.txt"

        try:
            self.CAN0 = can.interface.Bus(channel=canName, interface='socketcan')
        except can.exceptions.CanInitializationError as e:
            print("Error")
            logging.error(f"Failed to initialize CAN Bus communication: {e}")
            self.CAN0 = None

        self.historyFile = FileHandler(logFile)
        self.queuesList = queueList
        self.logger = logging
        self.debugging = debugging

        super(processCANHandler, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processCANHandler, self).run()

    def stop(self):
        self.CAN0.shutdown()
        super(processCANHandler, self).stop()
    
    def _init_threads(self):
        """Create the canhandler Publisher thread and add to the list of threads."""
        readCANTh = threadCANRead(self.CAN0, self.historyFile, self.queuesList, self.logger, self.debugging)
        self.threads.append(readCANTh)
        writeCANTh = threadCANWrite(self.CAN0, self.historyFile, self.queuesList, self.logger, self.debugging)
        self.threads.append(writeCANTh)

if __name__ == "__main__":
    from multiprocessing import Queue, Pipe
    import logging
    import time

    allProcesses = list()
    debug = False

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }

    logger = logging.getLogger()
    pipeRecv, pipeSend = Pipe(duplex=False)

    process = processCANHandler(queueList, logger, debug)
    process.daemon = True
    process.start()
    time.sleep(20)
    process.stop()