# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# To start the project: 
#
#       sudo apt update
#       sudo apt upgrade
#       xargs sudo apt install -y < "requirement.txt" 
#       cd src/dashboard/frontend/
#       curl -fsSL https://fnm.vercel.app/install | bash
#       source ~/.bashrc
#       fnm install --lts
#       npm install -g @angular/cli@17
#       npm install
#       if needed: npm audit fix
#
# ===================================== GENERAL IMPORTS ==================================
import sys
import subprocess
import time

sys.path.append(".")
from multiprocessing import Queue, Event
from src.utils.logger.loggerConfig import setupLogger
import logging

#Change here to see more or less info
LOG_LEVEL = logging.INFO
CONSOLE_LOG_LEVEL = logging.WARNING

#logging.basicConfig(filename='main.log', level=logging.INFO)
logger , mainLogger = setupLogger("Main", "main.log", LOG_LEVEL, CONSOLE_LOG_LEVEL)

# ===================================== PROCESS IMPORTS ==================================

from src.gateway.processGateway import processGateway
from src.dashboard.processDashboard import processDashboard
from src.hardware.camera.processCamera import processCamera
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.data.Semaphores.Semaphores import processSemaphores
from src.data.TrafficCommunication.processTrafficCommunication import processTrafficCommunication
from src.utils.ipManager.IpReplacement import IPManager
# ------ New component imports starts here ------#
from src.dashboard.webRTC.processWebRTC import processWebRTC
from src.dashboard.StreamRTC.processStreamRTC import processStreamRTC
from src.driving.LaneDetection.processLaneDetection import processLaneDetection
from src.driving.PathFollowing.processPathFollowing import processPathFollowing
from src.gateway.SharedMemoryGateway.processSharedMemoryGateway import processSharedMemoryGateway
from src.hardware.canhandler.processCANHandler import processCANHandler
# ------ New component imports ends here ------#
# ======================================== SETTING UP ====================================
allProcesses = list()

queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "ShMemConfig": Queue(),
    "Config": Queue(),
    "Video" : Queue(),
}
#logging = logging.getLogger()

Dashboard = True
toRecompileDashboard = False

Camera = True
Semaphores = False
TrafficCommunication = False
SerialHandler = False

# ------ New component flags starts here ------#
webRTC = False #JS Version
streamRTC = True
LaneDetection = True
PathFollowing = True
CANHandler = True
# ------ New component flags ends here ------#

#Log the start of the instance
mainLogger.info(f"Starting Brain instance...")

# ===================================== SETUP PROCESSES ==================================

# Initializing gateway
processGateway = processGateway(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
processGateway.start()

processSharedMemoryGateway = processSharedMemoryGateway(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
processSharedMemoryGateway.start()

# Ip replacement
path = './src/dashboard/frontend/src/app/webSocket/web-socket.service.ts'
IpChanger = IPManager(path)
IpChanger.replace_ip_in_file()

# Initializing dashboard
if Dashboard:
    processDashboard = processDashboard( queueList, toRecompileDashboard, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processDashboard)

# Initializing camera
if Camera:
    processCamera = processCamera(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processCamera)

if LaneDetection:
    processLaneDetection = processLaneDetection(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processLaneDetection)

if PathFollowing:
    processPathFollowing = processPathFollowing(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processPathFollowing)

if webRTC: #JS Version
    processWebRTC = processWebRTC(queueList, logging, debugging= False)
    allProcesses.append(processWebRTC)

if streamRTC:
    processStreamRTC = processStreamRTC(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processStreamRTC)

# Initializing semaphores
if Semaphores:
    processSemaphores = processSemaphores(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processSemaphores)

# Initializing GPS
if TrafficCommunication:
    processTrafficCommunication = processTrafficCommunication(queueList, 3, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processTrafficCommunication)

# Initializing serial connection NUCLEO - > PI
if SerialHandler:
    processSerialHandler = processSerialHandler(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processSerialHandler)

if CANHandler:
    processCANHandler = processCANHandler(queueList, mainLogLevel=LOG_LEVEL, debugging = False)
    allProcesses.append(processCANHandler)

# ------ New component runs starts here ------#

# ------ New component runs ends here ------#

# ===================================== START PROCESSES ==================================
for process in allProcesses:
    process.daemon = True
    process.start()
    time.sleep(2)

time.sleep(5)
c4_bomb = r"""
  _______________________
 /                       \
| [██████]    [██████]    |
| [██████]    [██████]    |
| [██████]    [██████]    |
|       TIMER: 00:10      |
|_________________________|
 \_______________________/
        LET'S GO!!!

        Press ctrl+C to close
"""

print(c4_bomb)

# ===================================== STAYING ALIVE ====================================
blocker = Event()
try:
    blocker.wait()
except KeyboardInterrupt:
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
    big_text = """
    PPPP   L        EEEEE    A    SSSS  EEEEE       W      W   A   III TTTTT
    P   P  L        E       A A   S     E           W      W  A A   I    T  
    PPPP   L        EEEE   A   A   SSS  EEEE        W  W   W A   A  I    T  
    P      L        E      AAAAA      S E           W W W W  AAAAA  I    T  
    P      LLLLLL   EEEEE  A   A  SSSS  EEEEE        W   W   A   A III   T  
    """

    print(big_text)

    for proc in reversed(allProcesses):
        print("Process stopped", proc)
        proc.stop()
    print("Process stopped", processSharedMemoryGateway)
    processSharedMemoryGateway.stop()
    print("Process stopped", processGateway)
    processGateway.stop()
    mainLogger.info("Stopped Brain instance...")

    big_text = """
    PPPP   RRRR   EEEEE  SSSS  SSSS       CCCC  TTTTT RRRR    L          ++      CCCC      !!! 
    P   P  R   R  E     S     S          C        T   R   R   L          ++      C         !!! 
    PPPP   RRRR   EEEE   SSS   SSS       C        T   RRRR    L      ++++++++++  C         !!! 
    P      R R    E         S     S      C        T   R R     L          ++      C         !!! 
    P      R  R   EEEEE  SSSS  SSSS       CCCC    T   R  R    LLLLL      ++      CCCC      !!!
    """

    print(big_text)
