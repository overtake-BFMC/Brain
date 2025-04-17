from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    Klem,
    Control,
    SteerMotor,
    SpeedMotor,
    Brake,
    ToggleImuData,
    ToggleResourceMonitor
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
import can
import struct
import time
class threadCANWrite(ThreadWithStop):
    """This thread handles canhandler.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, f_CAN0: can.BusABC, f_logFile, queueList, logging, debugging=False):
        self.CAN0 = f_CAN0
        self.logFile = f_logFile
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

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
                            self.sendToCAN(0x101, command)

                        speedRecv = self.speedMotorSubscriber.receive()
                        if speedRecv is not None:
                            command = struct.pack("<i", int(speedRecv))
                            self.sendToCAN(0x102, command)

                        steerRecv = self.steerMotorSubscriber.receive()
                        if steerRecv is not None:
                            command = struct.pack("<i", int(steerRecv))
                            self.sendToCAN(0x103, command)

                        controlRecv = self.controlSubscriber.receive()
                        if controlRecv is not None:
                            command = struct.pack("<hhh", int(controlRecv["Time"]), int(controlRecv["Speed"]), int(controlRecv["Steer"]))
                            self.sendToCAN(0x104, command)

                    resourceMonitorRecv = self.resourceMonitorSubscriber.receive()
                    if resourceMonitorRecv is not None:
                        command = struct.pack("<B", int(resourceMonitorRecv))
                        self.sendToCAN(0x110, command)
                    
                    imuRecv = self.imuSubscriber.receive()
                    if imuRecv is not None:
                        command = struct.pack("<B", int(imuRecv))
                        self.sendToCAN(0x111, command)
                    
                    #Distance sensor on off 0x112
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
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, Brake, "lastOnly", True)
        self.resourceMonitorSubscriber = messageHandlerSubscriber(self.queuesList, ToggleResourceMonitor, "lastOnly", True)
        self.imuSubscriber = messageHandlerSubscriber(self.queuesList, ToggleImuData, "lastOnly", True)
