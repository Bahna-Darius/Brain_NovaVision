# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# (license text unchanged...)

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

class serialCameraRaw(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 22
    msgType = "ndarray"


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
    Owner = "threadCamera"
    msgID = 5
    msgType = "int"


################ NovaVision 26.01.2026 ###############
# Context:
# - New lane output message that carries rich dict results for the new pipeline.
# - We keep the ORIGINAL LaneKeeping (int) for compatibility with BFMC code.
# - Camera/Lane modules should publish LaneKeepingNV (dict) for Perception.
#####################################################
class LaneKeepingNV(Enum):
    Queue = "General"
    Owner = "threadLaneModule"  # or "threadCamera" if you compute lane there; choose ONE and keep it consistent
    msgID = 100
    msgType = "dict"  # {"steer": int, "confidence": float, "timestamp": float, ...}


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
    msgType = "str"


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


class ControlCalib(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 18
    msgType = "dict"


class IsAlive(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 19
    msgType = "bool"


class RequestSteerLimits(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 20
    msgType = "bool"


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


class ShutDownSignal(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 8
    msgType = "str"


class CalibPWMData(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 9
    msgType = "dict"


class CalibRunDone(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 10
    msgType = "bool"


class AliveSignal(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 11
    msgType = "bool"


class SteeringLimits(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 12
    msgType = "dict"


################################# From Locsys ##################################
class Location(Enum):
    Queue = "General"
    Owner = "threadTrafficCommunication"
    msgID = 1
    msgType = "dict"


###################### From processSerialHandler ###########################
class EnableButton(Enum):
    Queue = "General"
    Owner = "threadWrite"
    msgID = 1
    msgType = "bool"


class WarningSignal(Enum):
    Queue = "General"
    Owner = "brain"
    msgID = 2
    msgType = "str"


class SerialConnectionState(Enum):
    Queue = "General"
    Owner = "processSerialHandler"
    msgID = 3
    msgType = "bool"


################################# From StateMachine ##################################
class StateChange(Enum):
    Queue = "Critical"
    Owner = "stateMachine"
    msgID = 1
    msgType = "str"


# ========================== NovaVision additions (new pipeline) ==========================

################ NovaVision 26.01.2026 ###############
# Context:
# - New TrafficSign output message for Perception input.
#####################################################
class TrafficSignDetections(Enum):
    Queue = "General"
    Owner = "threadTrafficSigns"
    msgID = 101
    msgType = "dict"  # {"timestamp": float, "dets": list[...], "source": "yolov8", ...}


################ NovaVision 26.01.2026 ###############
# Context:
# - PerceptionContext = fused dict that ALL downstream modules use.
#####################################################
class PerceptionContext(Enum):
    Queue = "General"
    Owner = "threadPerception"
    msgID = 102
    msgType = "dict"  # {"timestamp": float, "lane": dict|None, "traffic_signs": dict|None, "age": dict}


################ NovaVision 26.01.2026 ###############
# Context:
# - SafetyOverride output (hard authority).
#####################################################
class SafetyOverride(Enum):
    Queue = "Critical"
    Owner = "threadSafetyGuard"
    msgID = 103
    msgType = "dict"  # {"stop": bool, "max_speed": int|None, "max_steer": int|None, "ttl_ms": int, "reason": str}


################ NovaVision 26.01.2026 ###############
# Context:
# - ControlUnit outputs desired commands (AUTO).
#####################################################
class DesiredSpeed(Enum):
    Queue = "General"
    Owner = "threadControlUnit"
    msgID = 104
    msgType = "int"

class DesiredSteer(Enum):
    Queue = "General"
    Owner = "threadControlUnit"
    msgID = 105
    msgType = "int"

######### HC-SR04 sensor override and parking enable #########
class ParkingEnable(Enum):
    Queue = "General"
    Owner = "Brain"
    msgID = 200
    msgType = "int"

class USFrontOverride(Enum):
    Queue = "General"
    Owner = "Brain"
    msgID = 201
    msgType = "int"

