from src.templates.threadwithstop import ThreadWithStop

from src.utils.messages.allMessages import (
    memory_channel,
    cpu_channel,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from jtop import jtop
import psutil
#from src.dashboard.threads.utils.ina226 import INA226
#from src.utils.logger.setupLogger import LoggerConfigs, configLogger
import json
import os

class threadSysInfo(ThreadWithStop):

    def __init__(self, queuesList, logger, debugging = False):
        super(threadSysInfo, self).__init__()
        self.queuesList = queuesList
        #self.loggingQueue = loggingQueue
        self.logger = logger
        self.debugging = debugging
        #self.logger = configLogger(LoggerConfigs.WORKER, __name__, self.loggingQueue)

        self.checkInterval = 2

        self.memory_usage = 0
        self.core_usage = 0
        self.cpu_temp = 0
        self.batteryDataFilePath = "/run/batteryMonitor/status.json"

        # self.INA226JetsonPresent = 0
        # self.jetsonBatt = 0
        # try:
        #     self.INA226Jetson = INA226(busnum=7, address=0x40, max_expected_amps=20.0, shunt_ohms=0.003)
        #     self.INA226Jetson.configure()
        #     self.INA226Jetson.set_low_battery(5)
        #     self.INA226JetsonPresent = 1
        # except Exception as e:
        #     self.logger.error("No battery logger attached at I2C bus 7 address 0x40 for jetson")
        #     #print("No battery logger attached at I2C bus 7 address 0x40 for jetson")
            
        # self.INA226NucleoPresent = 0
        # self.nucleoBatt = 0
        # try:
        #     self.INA226Nucleo = INA226(busnum=7, address=0x41, max_expected_amps=20.0, shunt_ohms=0.003)
        #     self.INA226Nucleo.configure()
        #     self.INA226Nucleo.set_low_battery(5)
        #     self.INA226NucleoPresent = 1
        # except Exception as e:
        #     self.logger.error("No battery logger attached at I2C bus 7 address 0x40 for nucleo")
        #     #print("No battery logger attached at I2C bus 7 address 0x40 for nucleo")
        self.jetsonBattVoltage = 0
        self.nucleoBattVoltage = 0

        self.cpuChannelSender = messageHandlerSender(self.queuesList, cpu_channel)
        self.memoryChannelSender = messageHandlerSender(self.queuesList, memory_channel)

# =============================== STOP ================================================
    def stop(self):
        super(threadSysInfo, self).stop()

# ================================ RUN ================================================
    def run(self):
        while self._running:
            try:
                with jtop(self.checkInterval) as jetson:
                    while jetson.ok():
                        self.cpu_temp = round(jetson.temperature['cpu']['temp'], 1)
                        self.memory_usage = round(jetson.stats['RAM'] * 100, 1)
                        #self.core_usage = jetson.cpu['cpu'][0]['user']
                        #self.core_usage = {i: core['user'] for i, core in enumerate(jetson.cpu['cpu'])}
                        self.core_usage = psutil.cpu_percent(interval=None, percpu=True)
                        #print("Core: ",self.core_usage)
                        #print("Memory: ", self.memory_usage)
                        #print("Temp: ", self.cpu_temp)
                        # if self.INA226JetsonPresent and self.INA226NucleoPresent:
                        #     self.INA226Jetson.wake(3)
                        #     self.INA226Nucleo.wake(3)
                        #     while 1:
                        #         if self.INA226Jetson.is_conversion_ready():
                        #             #print("===================================================Conversion ready")
                        #             self.readJetsonVolt()

                        #         if self.INA226Nucleo.is_conversion_ready():
                        #             self.readNucleoVolt()
                        #             break
                        self.readBatteryStatus()

                        self.memoryChannelSender.send(self.memory_usage)
                        self.cpuChannelSender.send({'usage': self.core_usage, 'temp': self.cpu_temp, 'jetsonBatt': self.jetsonBattVoltage, 'nucleoBatt': self.nucleoBattVoltage})
    
            except Exception as e:
                print(e)

    #def readJetsonVolt(self):
        #self.jetsonBatt = self.INA226Jetson.voltage()
        #print("Jetson Power : %.3f mW" % self.INA226Jetson.power())
        #print("Jetson Bus Voltage    : %.3f V" % self.INA226Jetson.voltage())
        #print("Jetson Supply Voltage : %.3f V" % self.INA226Jetson.supply_voltage())

    def readBatteryStatus(self):
        try:
            with open(self.batteryDataFilePath, 'r') as f:
                batteryData = json.load(f)

                jetsonBattery = batteryData.get('JetsonBattery', {})
                jetsonVoltageStr = jetsonBattery.get('voltage', None)
                if jetsonVoltageStr is not None:
                    self.jetsonBattVoltage = float(jetsonVoltageStr)
                else:
                    self.jetsonBattVoltage = 0

                nucleoBattery = batteryData.get('NucleoBattery', {})
                nucleoVoltageStr = nucleoBattery.get('voltage', None)
                if nucleoVoltageStr is not None:
                    self.nucleoBattVoltage = float(nucleoVoltageStr)
                else:
                    self.nucleoBattVoltage = 0
        except json.JSONDecodeError:
            self.logger.warn("Cannot decode battery status json!")
        except Exception as e:
            self.logger.error(f"Error reading JSON file: {e}")
            print(f"Error reading JSON file: {e}")


    #def readNucleoVolt(self):
        #self.nucleoBatt = self.INA226Nucleo.voltage()
        #print("Nucleo Power : %.3f mW" % self.INA226Nucleo.power())
        #print("Nucleo Bus Voltage    : %.3f V" % self.INA226Nucleo.voltage())
        #print("Nucleo Supply Voltage : %.3f V" % self.INA226Nucleo.supply_voltage())

# =============================== START ===============================================
    def start(self):
        super(threadSysInfo, self).start()
