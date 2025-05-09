import time
import threading
import re
import os

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    BatteryLvl,
    ImuData,
    ImuAck,
    InstantConsumption,
    EnableButton,
    ResourceMonitor,
    CurrentSpeed,
    CurrentSteer,
    WarningSignal,
    DistanceFront,

)
from src.utils.messages.messageHandlerSender import messageHandlerSender
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger

class threadRead(ThreadWithStop):
    """This thread reads the data that NUCLEO sends to Raspberry PI.\n
    Args:
        f_serialCon (serial.Serial): Serial connection between the two boards.
        f_logFile (FileHandler): The path to the history file where you can find the logs from the connection.
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
    """

    # ===================================== INIT =========================================
    def __init__(self, f_serialCon, f_logFile, queueList, logger, debugging = False):
        self.serialCon = f_serialCon
        self.logFile = f_logFile
        self.buff = ""
        self.isResponse = False
        self.queuesList = queueList
        self.acumulator = 0
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)
        self.currentSpeed = 0
        self.currentSteering = 0

        self.enableButtonSender = messageHandlerSender(self.queuesList, EnableButton)
        self.batteryLvlSender = messageHandlerSender(self.queuesList, BatteryLvl)
        self.instantConsumptionSender = messageHandlerSender(self.queuesList, InstantConsumption)
        self.imuDataSender = messageHandlerSender(self.queuesList, ImuData)
        self.imuAckSender = messageHandlerSender(self.queuesList, ImuAck)
        self.resourceMonitorSender = messageHandlerSender(self.queuesList, ResourceMonitor)
        self.currentSpeedSender = messageHandlerSender(self.queuesList, CurrentSpeed)
        self.currentSteerSender = messageHandlerSender(self.queuesList, CurrentSteer)
        self.warningSender = messageHandlerSender(self.queuesList, WarningSignal)
        self.distanceFrontSender = messageHandlerSender(self.queuesList, DistanceFront)

        self.expectedValues = {"kl": "0, 15 or 30", "instant": "1 or 0", "battery": "1 or 0",
                               "resourceMonitor": "1 or 0", "imu": "1 or 0", "steer": "between -25 and 25",
                               "speed": "between -500 and 500", "break": "between -250 and 250"}

        self.warningPattern = r'^(-?[0-9]+)H(-?[0-5]?[0-9])M(-?[0-5]?[0-9])S$'
        self.resourceMonitorPattern = r'Heap \((\d+\.\d+)\);Stack \((\d+\.\d+)\)'

        self.Queue_Sending()

        super(threadRead, self).__init__()

    # ====================================== RUN ==========================================
    def run(self):
        while self._running:
            # read_chr = self.serialCon.read()

            if self.serialCon.in_waiting > 0:
                try:
                    self.buff = self.serialCon.readline().decode("ascii")
                    self.sendqueue(self.buff)
                    #self.serialCon.reset_input_buffer()
                except Exception as e:
                    print("ThreadRead -> run method:", e)

            # try:
            #     read_chr = read_chr.decode("ascii")
            #     if read_chr == "@":
            #         self.isResponse = True
            #         self.buff = ""
            #     elif read_chr == "\r":
            #         self.isResponse = False
            #         if len(self.buff) != 0:
            #             self.sendqueue(self.buff)
            #     if self.isResponse:
            #         self.buff += read_chr
            # except Exception as e :
            #     print(e)

    # ==================================== SENDING =======================================
    def Queue_Sending(self):
        """Callback function for enable button flag."""
        self.enableButtonSender.send(True)
        threading.Timer(1, self.Queue_Sending).start()

    def sendqueue(self, buff):
        """This function selects which type of message we receive from NUCLEO and sends the data further."""

        if '@' in buff and ':' in buff:
            if buff.count('@') > 1 or buff.count(':') > 1 or not buff.startswith('@'):
                #raise ValueError("String contains more than two '@' symbols")
                return 1
            action, value = buff.split(":")  # @action:value;;
            action = action[1:]
            value = value[:-4]

            if self.debugging:
                self.logger.info(buff)

            if action == "imu":
                splittedValue = value.split(";")
                if len(buff) > 20:
                    data = {
                        "roll": splittedValue[0],
                        "pitch": splittedValue[1],
                        "yaw": splittedValue[2],
                        "accelx": splittedValue[3],
                        "accely": splittedValue[4],
                        "accelz": splittedValue[5],
                    }
                    self.imuDataSender.send(str(data))
                else:
                    self.imuAckSender.send(splittedValue[0])
            
            elif action == "distanceF":
                distance = value.split(",")[0]
                if self.isFloat(distance):
                    try:
                        self.distanceFrontSender.send(float(distance))
                    except:
                        print("Error in front distance")
            elif action == "speed":
                speed = value.split(",")[0]
                if self.isFloat(speed):
                    try:
                        self.currentSpeedSender.send(float(speed))
                    except:
                        print("Error in speed")

            elif action == "steer":
                steer = value.split(",")[0]
                if self.isFloat(steer):
                    try:
                        self.currentSteerSender.send(float(steer))
                    except:
                        print("Error in steer")

            elif action == "instant":
                if self.checkValidValue(action, value):
                    try:
                        self.instantConsumptionSender.send(float(value) / 1000.0)
                    except:
                        print("Error in instant consumption")

            elif action == "battery":
                if self.checkValidValue(action, value):
                    percentage = (int(value) - 7200) / 12
                    percentage = max(0, min(100, round(percentage)))
                    self.batteryLvlSender.send(percentage)

            elif action == "resourceMonitor":
                if self.checkValidValue(action, value):
                    data = re.match(self.resourceMonitorPattern, value)
                    if data:
                        message = {"heap": data.group(1), "stack": data.group(2)}
                        self.resourceMonitorSender.send(message)

            elif action == "warning":
                data = re.match(self.warningPattern, value)
                if data:
                    print(f"WARNING! Shutting down in {data.group(1)} hours {data.group(2)} minutes {data.group(3)} seconds")
                    self.warningSender.send(action, data)

            elif action == "shutdown":
                print("SHUTTING DOWN!")
                time.sleep(3)
                os.system("sudo shutdown -h now")

    def checkValidValue(self, action, message):
        if message == "syntax error":
            print(f"WARNING! Invalid value for {action.upper()} (expected {self.expectedValues[action]})")
            return False

        if message == "kl 15/30 is required!!":
            print(f"WARNING! KL set to 15 or 30 is required to perform {action.upper()} action")
            return False

        if message == "ack":
            return False
        return True

    def isFloat(self, string):
        try:
            float(string)
        except ValueError:
            return False
        return True

