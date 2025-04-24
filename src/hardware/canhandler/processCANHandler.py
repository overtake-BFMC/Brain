if __name__ == "__main__":
    import sys
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from src.templates.workerprocess import WorkerProcess
from src.hardware.canhandler.threads.threadCANRead import threadCANRead
from src.hardware.canhandler.threads.threadCANWrite import threadCANWrite
import can
from src.hardware.canhandler.threads.filehandler import FileHandler
from src.utils.logger.setupLogger import LoggerConfigs, configLogger
import subprocess
from typing import List

class CANOSCommands:
    @staticmethod
    def up(interface: str = 'can0') -> List[str]:
        return ['sudo', 'ip', 'link', 'set', interface, 'up']
    
    @staticmethod
    def down(interface: str = 'can0') -> List[str]:
        return ['sudo', 'ip', 'link', 'set', interface, 'down']
    
    @staticmethod
    def set_bitrate(interface: str = 'can0', bitrate: int = 500000) -> List[str]:
        return ['sudo', 'ip', 'link', 'set', interface, 'type', 'can', 'bitrate', str(bitrate)]

class processCANHandler(WorkerProcess):
    """This process handles canhandler.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, loggingQueue, debugging = False):
        self.interfaceName = 'can0'
        self.interfaceBitrate = 500000
        self.logFile = "historyCAN.txt"

        self.historyFile = FileHandler(self.logFile)
        self.queuesList = queueList
        self.loggingQueue = loggingQueue
        self.debugging = debugging
        self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)
        
        try:
            self.CAN0 = can.interface.Bus(channel=self.interfaceName, interface='socketcan')
        except can.exceptions.CanInitializationError as e:
            self.logger.error(f"Failed to initialize CAN Bus Interface! Setting CAN0 to None!")
            self.CAN0 = None

        # Check if CAN network is down
        self.isCANUp = self.checkCANStatus()

        if not self.isCANUp and self.CAN0 is not None:
            self.logger.warning("CAN Interface is down, trying to get it up!")
            self.runCANOSCommand(CANOSCommands.set_bitrate(self.interfaceName, self.interfaceBitrate))
            time.sleep(0.1)
            self.runCANOSCommand(CANOSCommands.up(self.interfaceName))
            time.sleep(0.1)
            self.isCANUp = self.checkCANStatus()
            if self.isCANUp:
                self.logger.info("CAN Interface brought up sucessfully!")
            else:
                self.logger.critical("Failed to start CAN Interface!")

        super(processCANHandler, self).__init__(self.queuesList)

    def checkCANStatus(self):
        try:
            self.CAN0.recv(timeout=0.001)
            status = True
        except can.exceptions.CanOperationError as e:
            if e.error_code == 100:
                self.logger.warning("CAN network is down!")
                status = False
        return status

    def runCANOSCommand(self, command):
        try:
            result = subprocess.run(
                command,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
                )
            status = True
        except subprocess.CalledProcessError as e:
            self.logger.error("Running CAN OS command failed!")
            status = False
        return status
        
    def run(self):
        """Apply the initializing methods and start the threads."""
        if self.isCANUp and self.CAN0 is not None:
            self.logger.info("CAN Network Initiated Sucessfully.")
            super(processCANHandler, self).run()
        else:
            if self.CAN0 is not None:
                self.CAN0.shutdown()
            self.logger.critical("CAN Read and Write threads not starting, CAN NETWORK IS DOWN!")

    def stop(self):
        if self.CAN0 is not None:
            self.CAN0.shutdown()
        super(processCANHandler, self).stop()
    
    def _init_threads(self):
        """Create the canhandler Publisher thread and add to the list of threads."""
        readCANTh = threadCANRead(self.CAN0, self.historyFile, self.queuesList, self.loggingQueue, self.debugging)
        self.threads.append(readCANTh)
        writeCANTh = threadCANWrite(self.CAN0, self.historyFile, self.queuesList, self.loggingQueue, self.debugging)
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

    #logger, mainLogger = setupLogger("Main", "main.log", logging.DEBUG, logging.DEBUG )
    pipeRecv, pipeSend = Pipe(duplex=False)

    process = processCANHandler(queueList, logging.DEBUG, logging.DEBUG, debug)
    process.daemon = True
    process.start()
    time.sleep(20)
    process.stop()