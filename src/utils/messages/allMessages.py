# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

from enum import Enum

####################################### processCamera #######################################
class mainCamera(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 1
    msgType = "str"

class serialCamera(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 2
    msgType = "str"

class Recording(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 3
    msgType = "bool"

class Signal(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 4
    msgType = "str"

class LaneKeeping(Enum):
    Queue = "General"
    Owner = "threadCamera" # here you will send an offset of the car position between the lanes of the road + - from 0 point to dashboard
    msgID = 5
    msgType = "int"

# class MainVideo(Enum):
#     Queue = "Video"
#     Owner = "threadCamera"
#     msgID = 6
#     msgType = "ndarray"

# class LaneVideo(Enum):
#     Queue = "Video"
#     Owner = "threadLaneDetection"
#     msgID = 1
#     msgType = "ndarray"

################################# processCarsAndSemaphores ##################################
class Cars(Enum):
    Queue = "General"
    Owner = "threadCarsAndSemaphores"
    msgID = 1
    msgType = "dict"

class Semaphores(Enum):
    Queue = "General"
    Owner = "threadCarsAndSemaphores"
    msgID = 2
    msgType = "dict"

################################# From Dashboard ##################################
class SpeedMotor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 1
    msgType = "str"

class SteerMotor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 2
    msgType = "str"

class Control(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 3
    msgType = "dict"

class Brake(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 4
    msgType = "float"

class Record(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 5
    msgType = "str"

class Config(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 6
    msgType = "dict"

class Klem(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 7
    msgType = "str"

class DrivingMode(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 8
    msgType = "str"

class ToggleInstant(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 9
    msgType = "str"

class ToggleBatteryLvl(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 10
    msgType = "str"

class ToggleImuData(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 11
    msgType = "str"

class ToggleResourceMonitor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 12
    msgType = "str"

class State(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 13
    msgType = "str"

class Brightness(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 14
    msgType = "str"

class Contrast(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 15
    msgType = "str"

class DropdownChannelExample(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 16
    msgType = "str"

class SliderChannelExample(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 17
    msgType = "str"

class WebRTCOffer(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 18
    msgType = "str"

class ICECandidate(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 19
    msgType = "dict"

class startRun(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 20
    msgType = "str"

class startLaneDetection(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 21
    msgType = "str"

class SelectTrackNo(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 22
    msgType = "int"

################################# From WebRTC(Video Stream) ##################################
class WebRTCAnswer(Enum):
    Queue = "General"
    Owner = "threadWebRTC"
    msgID = 1
    msgType = "dict"

################################# From SysInfo ##################################
class memory_channel(Enum):
    Queue = "General"
    Owner = "threadSysInfo"
    msgID = 1
    msgType = "float"

class cpu_channel(Enum):
    Queue = "General"
    Owner = "threadSysInfo"
    msgID = 2
    msgType = "dict"

################################# From SharedMemoryGateway ##################################
# class createShMem(Enum):
#     Queue = "ShMemConfig"
#     Owner = "threadSharedMemoryGateway"
#     msgID = 1
#     msgType = "dict"

# class getShMem(Enum):
#     Queue = "ShMemConfig"
#     Owner = "threadSharedMemoryGateway"
#     msgID = 2
#     msgType = "dict"

# class releaseShMem(Enum):
#     Queue = "ShMemConfig"
#     Owner = "threadSharedMemoryGateway"
#     msgID = 3
#     msgType = "str"

# class getVehicleStateObj(Enum):
#     Queue = "ShMemConfig"
#     Owner = "threadSharedMemoryGateway"
#     msgID = 4
#     msgType = "str"

# class ShMemResponse(Enum):
#     Queue = "General"
#     Owner = "threadSharedMemoryGateway"
#     msgID = 5
#     msgType = "dict"

class ShMemConfig(Enum):
    Queue = "ShMemConfig"
    Owner = "threadSharedMemoryGateway"
    msgID = 1
    msgType = "dict"

class ShMemResponse(Enum):
    Queue = "General"
    Owner = "threadSharedMemoryGateway"
    msgID = 2
    msgType = "dict"

################################# From Nucleo ##################################
class BatteryLvl(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 1
    msgType = "int"

class ImuData(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 2
    msgType = "str"

class InstantConsumption(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 3
    msgType = "float"

class ResourceMonitor(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 4
    msgType = "dict"

class CurrentSpeed(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 5
    msgType = "float"

class CurrentSteer(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 6
    msgType = "float"

class ImuAck(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 7
    msgType = "str"
    
class WarningSignal(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 8
    msgType = "str"

class DistanceFront(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 8
    msgType = "float"

################################# From Locsys ##################################
class Location(Enum):
    Queue = "General"
    Owner = "threadTrafficCommunication"
    msgID = 1
    msgType = "dict"

######################    From processSerialHandler  ###########################
class EnableButton(Enum):
    Queue = "General"
    Owner = "threadWrite"
    msgID = 1
    msgType = "bool"

class WarningSignal(Enum):
    Queue = "General"
    Owner = "brain"
    msgID = 1
    msgType = "dict"

### It will have this format: {"WarningName":"name1", "WarningID": 1}
