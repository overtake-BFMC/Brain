import serial
from src.templates.workerprocess import WorkerProcess
from src.hardware.serialhandler.threads.filehandler import FileHandler
from src.hardware.serialhandler.threads.threadRead import threadRead
from src.hardware.serialhandler.threads.threadWrite import threadWrite

#from src.utils.logger.setupLogger import LoggerConfigs, configLogger

class processSerialHandler(WorkerProcess):
    """This process handles the connection between NUCLEO and Raspberry PI.\n
    Args:
        queueList (dict of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Used for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
        example (bool, optional): A flag for running the example. Defaults to False.
    """

    # ===================================== INIT =========================================
    def __init__(self, queueList, logger, debugging = False, example=False):
        devFile = "/dev/ttyACM0"
        logFile = "historyFile.txt"

        # comm init
        try:
            self.serialCom = serial.Serial(devFile, 115200, timeout=0.1)
            self.serialCom.reset_input_buffer()
            self.serialCom.reset_output_buffer()
        except serial.SerialException as e:
            logging.error(f"Failed to initialize serial communication: {e}")
            self.serialCom = None

        # log file init
        self.historyFile = FileHandler(logFile)
        self.queuesList = queueList
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)
        self.example = example
        super(processSerialHandler, self).__init__(self.queuesList)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processSerialHandler, self).run()
        self.historyFile.close()

    # ===================================== INIT TH =================================
    def _init_threads(self):
        """Initializes the read and the write thread."""
        if self.serialCom:
            readTh = threadRead(self.serialCom, self.historyFile, self.queuesList, self.logger, self.debugging)
            self.threads.append(readTh)
            writeTh = threadWrite(self.queuesList, self.serialCom, self.historyFile, self.logger, self.debugging, self.example)
            self.threads.append(writeTh)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processSerialHandler.py

if __name__ == "__main__":
    from multiprocessing import Queue, Pipe
    import logging
    import time

    allProcesses = list()
    debugg = False
    # We have a list of multiprocessing.Queue() which individually represent a priority for processes.
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    logger = logging.getLogger()
    pipeRecv, pipeSend = Pipe(duplex=False)
    process = processSerialHandler(queueList, logger, debugg, True)
    process.daemon = True
    process.start()
    time.sleep(4)  # modify the value to increase/decrease the time of the example
    process.stop()
