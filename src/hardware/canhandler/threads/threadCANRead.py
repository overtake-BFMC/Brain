from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    ImuData,
    ImuAck,
    EnableButton,
    ResourceMonitor,
    CurrentSpeed,
    CurrentSteer,
    DistanceFront,
    WhiteLine,
    MsgACK,
    VCDTicks
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
import can
import struct
import threading
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger

class threadCANRead(ThreadWithStop):
    """This thread handles canhandler.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, f_CAN0: can.BusABC, f_logFile, queueList, logger, debugging = False):
        self.CAN0 = f_CAN0
        self.logFile = f_logFile
        self.queuesList = queueList
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)
        self.subscribe()

        self.enableButtonSender = messageHandlerSender(self.queuesList, EnableButton)
        self.imuDataSender = messageHandlerSender(self.queuesList, ImuData)
        self.imuAckSender = messageHandlerSender(self.queuesList, ImuAck)
        self.resourceMonitorSender = messageHandlerSender(self.queuesList, ResourceMonitor)
        self.currentSpeedSender = messageHandlerSender(self.queuesList, CurrentSpeed)
        self.currentSteerSender = messageHandlerSender(self.queuesList, CurrentSteer)
        self.distanceFrontSender = messageHandlerSender(self.queuesList, DistanceFront)
        self.whiteLineSender = messageHandlerSender(self.queuesList, WhiteLine)
        self.messageACKSender = messageHandlerSender(self.queuesList, MsgACK)
        self.VCDTicksSender = messageHandlerSender(self.queuesList, VCDTicks)


        self.Queue_Sending()
        
        super(threadCANRead, self).__init__()

    def Queue_Sending(self):
        """Callback function for enable button flag."""
        self.enableButtonSender.send(True)
        threading.Timer(1, self.Queue_Sending).start()

    def run(self):
        while self._running:
            CANResp = self.CAN0.recv(timeout= 0.001)
            if CANResp is not None:
                if CANResp.arbitration_id == 0x119: #"imu"
                    scaledAccelX, scaledAccelY = struct.unpack('<ii', CANResp.data)
                    ogAccelX = scaledAccelX / 1000.0
                    ogAccelY = scaledAccelY / 1000.0
                    data ={
                        "accelx": ogAccelX,
                        "accely": ogAccelY
                    }
                    if self.debugging:
                        self.logger.info(f"IMU:(Accel X: {ogAccelX}, Accel Y: {ogAccelY})")
                    #print(f"IMU: {str(data)}")
                    self.imuDataSender.send(str(data))
                elif CANResp.arbitration_id == 0x11E: #"distanceFront"
                    distanceF = struct.unpack('<i', CANResp.data)[0]
                    if self.debugging:
                        self.logger.info(f"distance Front : {distanceF}")
                    self.distanceFrontSender.send(float(distanceF))
                elif CANResp.arbitration_id == 0x123: #"currentSpeed"
                    currentSpeed = struct.unpack('<i', CANResp.data)[0]
                    #print(f"Speed: {CANResp.data}, data: {CANResp.dlc}")
                    if self.debugging:
                        self.logger.info(f"Speed ACK : {currentSpeed}")
                    self.currentSpeedSender.send(float(currentSpeed))
                elif CANResp.arbitration_id == 0x128: #"currentSteer"
                    currentSteer = struct.unpack('<i', CANResp.data)[0]
                    #print(f"Steer: {CANResp.data}, data: {CANResp.dlc}")
                    if self.debugging:
                        self.logger.info(f"Steer ACK : {currentSteer}")
                    self.currentSteerSender.send(float(currentSteer))
                elif CANResp.arbitration_id == 0x12D: #"HeapStackUsage"
                    scaledHeapUsage, scaledStackUsage = struct.unpack('<hh', CANResp.data)
                    ogHeapUsage = scaledHeapUsage / 100.0
                    ogStackUsage = scaledStackUsage / 100.0
                    message = {"heap": ogHeapUsage, "stack": ogStackUsage}
                    if self.debugging:
                        self.logger.info(f"Heap and Stack Usage : {str(message)}")
                    self.resourceMonitorSender.send(message)
                elif CANResp.arbitration_id == 0x11D: #"WhiteLine Detected"
                    whiteLine = not struct.unpack('<?', CANResp.data)[0]
                    if self.debugging:
                        self.logger.info(f"WhiteLineDetected: {str(whiteLine)}")
                    self.whiteLineSender.send(whiteLine)
                elif CANResp.arbitration_id == 0x141: #"breakState"
                    breakState = CANResp.data[0] != 0
                elif CANResp.arbitration_id == 0x146: #"vcdEnd"
                    #vcdState = CANResp.data[0] != 0
                    VCDTicks = struct.unpack("<H", CANResp.data)[0]
                    if self.debugging:
                        self.logger.info(f"VCD Exec Time: {VCDTicks}")
                    self.VCDTicksSender.send(VCDTicks)
                elif CANResp.arbitration_id == 0x99:
                    MsgACK = int.from_bytes(CANResp.data, byteorder='little')
                    if self.debugging:
                        self.logger.info(f"MsgACK: {str(MsgACK)}")
                    self.messageACKSender.send(MsgACK)
                    

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
