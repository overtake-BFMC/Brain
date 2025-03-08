from src.templates.threadwithstop import ThreadWithStop

from src.utils.messages.allMessages import (
    memory_channel,
    cpu_channel,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from jtop import jtop
import psutil
from src.dashboard.threads.utils.ina226 import INA226
import logging

class threadSysInfo(ThreadWithStop):

    def __init__(self, queuesList, logger, debugger):
        super(threadSysInfo, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger

        self.checkInterval = 2

        self.memory_usage = 0
        self.core_usage = 0
        self.cpu_temp = 0

        self.INA226Jetson = INA226(busnum=7, address=0x40, max_expected_amps=0.5, shunt_ohms=0.1, log_level=logging.INFO)
        self.INA226Jetson.configure()
        self.INA226Jetson.set_low_battery(5)

        self.INA226Nucleo = INA226(busnum=7, address=0x41, max_expected_amps=0.5, shunt_ohms=0.1, log_level=logging.INFO)
        self.INA226Nucleo.configure()
        self.INA226Nucleo.set_low_battery(5)

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
                        self.memoryChannelSender.send(self.memory_usage)
                        self.cpuChannelSender.send({'usage': self.core_usage, 'temp': self.cpu_temp})

                        self.INA226Jetson.wake(3)
                        self.INA226Nucleo.wake(3)
                        while 1:
                            if self.INA226Jetson.is_conversion_ready():
                                #sleep(1)
                                print("===================================================Conversion ready")
                                self.readJetsonVolt()

                            if self.INA226Nucleo.is_conversion_ready():
                                self.readNucleoVolt()
                                break
    
            except Exception as e:
                print(e)

    def readJetsonVolt(self):
        print("Jetson Bus Voltage    : %.3f V" % self.INA226Jetson.voltage())
        print("Jetson Supply Voltage : %.3f V" % self.INA226Jetson.supply_voltage())

    def readNucleoVolt(self):
        print("Nucleo Bus Voltage    : %.3f V" % self.INA226Nucleo.voltage())
        print("Nucleo Supply Voltage : %.3f V" % self.INA226Nucleo.supply_voltage())

# =============================== START ===============================================
    def start(self):
        super(threadSysInfo, self).start()

#MOVE THIS THREAD TO SEPERATE PROCESS AND THAN TALK WITH IT, proccessDashboard can only be on one thread 