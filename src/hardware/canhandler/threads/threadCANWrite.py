from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    Klem,
    Control,
    SteerMotor,
    SpeedMotor,
    Brake,
    ToggleImuData,
    ToggleResourceMonitor,
    MsgACK,
    ToggleWhiteLineSensor,
    ToggleDistanceSensor,
    ManualPWMSpeedMotor,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
import can
import struct
import time
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger

class threadCANWrite(ThreadWithStop):
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

        self.running = False
        self.engineEnabled = False

        self.subscribe()
        super(threadCANWrite, self).__init__()

    def sendToCAN(self, msgID, command): #add command checks
        try:
            canMsg = can.Message(arbitration_id=msgID, data=command, is_extended_id=False, check=True)
        except ValueError as e:
            print(f"CAN Message Value Problem: {e}")
        try:
            self.CAN0.send(canMsg)
        except can.exceptions.CanError as e:
            print(f"Can transm failed: {e}")
            self.CAN0.flush_tx_buffer()
            self.logger.warning(f"Can transm failed: {str(canMsg)}")
            self.logger.warning(f"CAN0 Tx buffer cleared!")

    def run(self):
        while self._running:
            try:
                klRecv = self.klSubscriber.receive()
                if klRecv is not None:
                    if klRecv == "30":
                        self.running = True
                        self.engineEnabled = True
                        command = struct.pack("<i", 30)
                        self.sendToCAN(0x100, command)
                    elif klRecv == "15":
                        self.running = True
                        self.engineEnabled = False
                        command = struct.pack("<i", 15)
                        self.sendToCAN(0x100, command)
                    elif klRecv == "0":
                        self.running = False
                        self.engineEnabled = False
                        command = struct.pack("<i", 0)
                        self.sendToCAN(0x100, command)

                if self.running:
                    if self.engineEnabled:
                        breakRecv = self.brakeSubscriber.receive()
                        if breakRecv is not None:
                            command = struct.pack("<B", 1)
                            self.sendToCAN(0x105, command)

                        speedRecv = self.speedMotorSubscriber.receive()
                        if speedRecv is not None:
                            command = struct.pack("<i", int(speedRecv))
                            self.sendToCAN(0x10A, command)
                            if self.debugging:
                                self.logger.info(f"Speed CMD: {speedRecv}")

                        steerRecv = self.steerMotorSubscriber.receive()
                        if steerRecv is not None:
                            command = struct.pack("<i", int(steerRecv))
                            self.sendToCAN(0x10F, command)
                            if self.debugging:
                                self.logger.info(f"Steer CMD: {steerRecv}")

                        controlRecv = self.controlSubscriber.receive()
                        if controlRecv is not None:
                            command = struct.pack("<hhh", int(controlRecv["Time"]), int(controlRecv["Speed"]), int(controlRecv["Steer"]))
                            self.sendToCAN(0x114, command)

                        PWMSpeedRecv = self.manualPWMSpeedMotorSubscriber.receive()
                        if PWMSpeedRecv is not None:
                            command = struct.pack("<i", PWMSpeedRecv)
                            self.sendToCAN(0x091, command)
                            if self.debugging:
                                self.logger.info(f"ManualPWMSpeed CMD: {PWMSpeedRecv}")

                    resourceMonitorToggleRecv = self.resourceMonitorToggleSubscriber.receive()
                    if resourceMonitorToggleRecv is not None:
                        command = struct.pack("<B", int(resourceMonitorToggleRecv))
                        self.sendToCAN(0x132, command)
                    
                    imuToggleRecv = self.imuToggleSubscriber.receive()
                    if imuToggleRecv is not None:
                        command = struct.pack("<B", int(imuToggleRecv))
                        self.sendToCAN(0x137, command)

                    distanceToggleRecv = self.distanceToggleSubscriber.receive()
                    if distanceToggleRecv is not None:
                        command = struct.pack("<B", int(distanceToggleRecv))
                        self.sendToCAN(0x13C, command)

                    whiteLineToggleRecv = self.whiteLineToggleSubscriber.receive()
                    if whiteLineToggleRecv is not None:
                        command = struct.pack("<B", int(whiteLineToggleRecv))
                        self.sendToCAN(0x13E, command)
                    
            except Exception as e:
                pass

    def stop(self):
        command = struct.pack("<i", 0)
        self.sendToCAN(0x100, command)
        time.sleep(1)
        super(threadCANWrite, self).stop()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastOnly", True)
        self.controlSubscriber = messageHandlerSubscriber(self.queuesList, Control, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "fifo", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "fifo", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, Brake, "lastOnly", True)
        self.resourceMonitorToggleSubscriber = messageHandlerSubscriber(self.queuesList, ToggleResourceMonitor, "lastOnly", True)
        self.imuToggleSubscriber = messageHandlerSubscriber(self.queuesList, ToggleImuData, "lastOnly", True)
        self.messageACKSubscriber = messageHandlerSubscriber(self.queuesList, MsgACK, "fifo", False)
        self.distanceToggleSubscriber = messageHandlerSubscriber(self.queuesList, ToggleDistanceSensor, "lastOnly", True)
        self.whiteLineToggleSubscriber = messageHandlerSubscriber(self.queuesList, ToggleWhiteLineSensor, "lastOnly", True)
        self.manualPWMSpeedMotorSubscriber = messageHandlerSubscriber(self.queuesList, ManualPWMSpeedMotor, "lastOnly", True)
