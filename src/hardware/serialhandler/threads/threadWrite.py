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

import json
import threading
import time
from datetime import datetime, timedelta

from src.hardware.serialhandler.threads.messageconverter import MessageConverter
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    Klem,
    Control,
    SteerMotor,
    SpeedMotor,
    Brake,
    ToggleBatteryLvl,
    ToggleImuData,
    ToggleInstant,
    ToggleResourceMonitor,
    SerialConnectionState,
    ControlCalib,
    IsAlive,
    RequestSteerLimits,
    # -------- NovaVision 26.01.2026 (new pipeline) --------
    ShapedSpeedMotor,
    ShapedSteerMotor,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender


################ NovaVision 26.01.2026 ###############
# Context:
# - Added support for "Shaped" actuator commands produced by:
#     ControlUnit -> CommandShaper -> ShapedSpeedMotor/ShapedSteerMotor -> SerialHandler(threadWrite)
# - threadWrite prefers shaped commands when they are fresh, otherwise falls back
#   to original manual SpeedMotor/SteerMotor (keeps BFMC manual behavior untouched).
#####################################################

SHAPED_TIMEOUT_S = 0.6  # seconds
ACTUATOR_SEND_HZ = 20
ACTUATOR_PERIOD_S = 1.0 / ACTUATOR_SEND_HZ
STEER_SPEED_GAP_S = 0.015   # 10ms gap (tune 0.005–0.02)


class threadWrite(ThreadWithStop):
    """This thread write the data that Raspberry PI send to NUCLEO.\n

    Args:
        queues (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        process (processSerialHandler): ProcessSerialHandler object.
        logFile (FileHandler): The path to the history file where you can find the logs from the connection.
        example (bool, optional): Flag for exmaple activation. Defaults to False.
    """

    # ===================================== INIT =========================================
    def __init__(self, process, logFile, queues, logger, debugger=False, example=False):
        super(threadWrite, self).__init__(pause=0.001)
        self.process = process
        self.queuesList = queues
        self.logFile = logFile
        self.exampleFlag = example
        self.logger = logger
        self.debugger = debugger

        self.running = False
        self.engineEnabled = False
        self.messageConverter = MessageConverter()
        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.configPath = "src/utils/table_state.json"

        # error rate limiting
        self.last_error_time = None
        self.error_cooldown = timedelta(seconds=3)

        ################ NovaVision 26.01.2026 ###############
        # Context:
        # - Shaped command cache + timestamps
        #####################################################
        self.shaped_speed = None
        self.shaped_steer = None
        self.last_shaped_speed_time = 0.0
        self.last_shaped_steer_time = 0.0
        self._last_actuator_send = 0.0
        self._last_manual_speed = 0
        self._last_manual_steer = 0


        self.load_config("init")
        self._init_subscribers()
        self._init_senders()

        if example:
            self.i = 0.0
            self.j = -1.0
            self.s = 0.0
            self.example()

    def _init_subscribers(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastOnly", True)
        self.controlSubscriber = messageHandlerSubscriber(self.queuesList, Control, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, Brake, "lastOnly", True)
        self.instantSubscriber = messageHandlerSubscriber(self.queuesList, ToggleInstant, "lastOnly", True)
        self.batterySubscriber = messageHandlerSubscriber(self.queuesList, ToggleBatteryLvl, "lastOnly", True)
        self.resourceMonitorSubscriber = messageHandlerSubscriber(self.queuesList, ToggleResourceMonitor, "lastOnly", True)
        self.imuSubscriber = messageHandlerSubscriber(self.queuesList, ToggleImuData, "lastOnly", True)
        self.controlCalibSubscriber = messageHandlerSubscriber(self.queuesList, ControlCalib, "lastOnly", True)
        self.isAliveSubscriber = messageHandlerSubscriber(self.queuesList, IsAlive, "lastOnly", True)
        self.requestSteerLimitsSubscriber = messageHandlerSubscriber(self.queuesList, RequestSteerLimits, "lastOnly", True)

        ################ NovaVision 26.01.2026 ###############
        # Context:
        # - Subscribe to shaped outputs from CommandShaper.
        #####################################################
        self.shapedSpeedSubscriber = messageHandlerSubscriber(self.queuesList, ShapedSpeedMotor, "lastOnly", True)
        self.shapedSteerSubscriber = messageHandlerSubscriber(self.queuesList, ShapedSteerMotor, "lastOnly", True)

    def _init_senders(self):
        self.serialConnectionStateSender = messageHandlerSender(self.queuesList, SerialConnectionState)

    # ==================================== SENDING =======================================

    def send_to_serial(self, msg):
        command_msg = self.messageConverter.get_command(**msg)
        if self.debugger:                       ###########################################DEBUGGER
            print(f"[WriteASCII] {command_msg.strip()}") ##########################################
        if command_msg != "error":
            try:
                if self.debugger:
                    print(f"[Write] -> {msg}")

                with self.process.serialLock:
                    serialCon = self.process.serialCon
                    if serialCon and self.process.serialConnected and serialCon.is_open:
                        serialCon.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)

            except Exception as e:
                if self._should_send_error():
                    self.serialConnectionStateSender.send(False)
                    print(
                        f"\033[1;97m[ Serial Handler ] :\033[0m "
                        f"\033[1;91mERROR\033[0m - Failed to write to serial ({e})"
                    )

    def load_config(self, configType):
        with open(self.configPath, "r") as file:
            data = json.load(file)

        if configType == "init":
            capacity = data["init"]["batteryCapacity"]["capacity"]
            command = {"action": "batteryCapacity", "capacity": capacity}
            self.send_to_serial(command)
        else:
            toggle_keys = [
                "ToggleInstant",
                "ToggleBatteryLvl",
                "ToggleImuData",
                "ToggleResourceMonitor",
            ]
            for key in toggle_keys:
                toggle = data[key]
                value_str = toggle["value"]
                value = 0 if str(value_str) == "False" else 1
                command_name = toggle["command"]
                command = {"action": command_name, "activate": value}
                self.send_to_serial(command)
                time.sleep(0.05)

    def convert_fc(self, instantRecv):
        if instantRecv == "True":
            return 1
        else:
            return 0

    # ===================================== RUN ==========================================
    def thread_work(self):
        """BFMC original structure preserved. NovaVision shaped support is injected minimally."""
        try:
            klRecv = self.klSubscriber.receive()
            if klRecv is not None:
                if self.debugger:
                    self.logger.info(klRecv)
                if klRecv == "30":
                    self.running = True
                    self.engineEnabled = True
                    command = {"action": "kl", "mode": 30}
                    self.send_to_serial(command)
                    self.load_config("sensors")
                elif klRecv == "15":
                    self.running = True
                    self.engineEnabled = False
                    command = {"action": "kl", "mode": 15}
                    self.send_to_serial(command)
                    self.load_config("sensors")
                elif klRecv == "0":
                    self.running = False
                    self.engineEnabled = False
                    command = {"action": "kl", "mode": 0}
                    self.send_to_serial(command)

            isAliveRecv = self.isAliveSubscriber.receive()
            if isAliveRecv is not None:
                if self.debugger:
                    self.logger.info(isAliveRecv)
                command = {"action": "alive", "activate": 0}
                self.send_to_serial(command)

            requestSteerLimitsRecv = self.requestSteerLimitsSubscriber.receive()
            if requestSteerLimitsRecv is not None:
                if self.debugger:
                    self.logger.info(requestSteerLimitsRecv)
                command = {"action": "steerLimits", "request": 0}
                self.send_to_serial(command)

            if self.running:
                if self.engineEnabled:

                    ################ NovaVision 26.01.2026 ###############
                    # Context:
                    # - Read shaped commands (AUTO pipeline) and track timestamps.
                    #####################################################
                    shapedSpeedRecv = self.shapedSpeedSubscriber.receive()
                    if shapedSpeedRecv is not None:
                        self.shaped_speed = int(shapedSpeedRecv)
                        self.last_shaped_speed_time = time.time()

                    shapedSteerRecv = self.shapedSteerSubscriber.receive()
                    if shapedSteerRecv is not None:
                        self.shaped_steer = int(shapedSteerRecv)
                        self.last_shaped_steer_time = time.time()

                    now = time.time()
                    use_shaped = (
                        self.shaped_speed is not None
                        and self.shaped_steer is not None
                        and (now - self.last_shaped_speed_time) < SHAPED_TIMEOUT_S
                        and (now - self.last_shaped_steer_time) < SHAPED_TIMEOUT_S
                    )
                    speedRecv = self.speedMotorSubscriber.receive()
                    if speedRecv is not None:
                        self._last_manual_speed = int(speedRecv)

                    steerRecv = self.steerMotorSubscriber.receive()
                    if steerRecv is not None:
                        self._last_manual_steer = int(steerRecv)

                    #####################################################

                    brakeRecv = self.brakeSubscriber.receive()
                    if brakeRecv is not None:
                        if self.debugger:
                            self.logger.info(brakeRecv)
                        command = {"action": "brake", "steerAngle": int(brakeRecv)}
                        self.send_to_serial(command)

                    # ---------------- SPEED (original) ----------------
                    # if use_shaped:
                    #     command = {"action": "speed", "speed": int(self.shaped_speed)}
                    #     self.send_to_serial(command)
                    # else:
                    #     speedRecv = self.speedMotorSubscriber.receive()
                    #     if speedRecv is not None:
                    #         if self.debugger:
                    #             self.logger.info(speedRecv)
                    #         command = {"action": "speed", "speed": int(speedRecv)}
                    #         self.send_to_serial(command)

                    # # ---------------- STEER (original) ----------------
                    # if use_shaped:
                    #     command = {"action": "steer", "steerAngle": int(self.shaped_steer)}
                    #     self.send_to_serial(command)
                    # else:
                    #     steerRecv = self.steerMotorSubscriber.receive()
                    #     if steerRecv is not None:
                    #         if self.debugger:
                    #             self.logger.info(steerRecv)
                    #         command = {"action": "steer", "steerAngle": int(steerRecv)}
                    #         self.send_to_serial(command)
                    # ---- Send actuator commands at a fixed rate ----
                    now = time.time()
                    if (now - self._last_actuator_send) >= ACTUATOR_PERIOD_S:
                        self._last_actuator_send = now

                        # Pick source
                        if use_shaped:
                            steer_val = int(self.shaped_steer)
                            speed_val = int(self.shaped_speed)
                        else:
                            steer_val = int(self._last_manual_steer)
                            speed_val = int(self._last_manual_speed)

                        # IMPORTANT for many firmwares: send STEER first, then SPEED last
                        self.send_to_serial({"action": "steer", "steerAngle": steer_val})
                        time.sleep(STEER_SPEED_GAP_S)
                        self.send_to_serial({"action": "speed", "speed": speed_val})


                    ################ NovaVision 26.01.2026 ###############
                    # Context:
                    # - Keep VCD / calib only when shaped is NOT active (avoid mixing sources).
                    #####################################################
                    if not use_shaped:
                        controlRecv = self.controlSubscriber.receive()
                        if controlRecv is not None:
                            if self.debugger:
                                self.logger.info(controlRecv)
                            command = {
                                "action": "vcd",
                                "time": int(controlRecv["Time"]),
                                "speed": int(controlRecv["Speed"]),
                                "steer": int(controlRecv["Steer"]),
                            }
                            self.send_to_serial(command)

                        controlCalibRecv = self.controlCalibSubscriber.receive()
                        if controlCalibRecv is not None:
                            if self.debugger:
                                self.logger.info(controlCalibRecv)
                            command = {
                                "action": "vcdCalib",
                                "time": int(controlCalibRecv["Time"]),
                                "speed": int(controlCalibRecv["Speed"]),
                                "steer": int(controlCalibRecv["Steer"]),
                            }
                            self.send_to_serial(command)

                instantRecv = self.instantSubscriber.receive()
                if instantRecv is not None:
                    if self.debugger:
                        self.logger.info(instantRecv)
                    command = {"action": "instant", "activate": int(instantRecv)}
                    self.send_to_serial(command)

                batteryRecv = self.batterySubscriber.receive()
                if batteryRecv is not None:
                    if self.debugger:
                        self.logger.info(batteryRecv)
                    command = {"action": "battery", "activate": int(batteryRecv)}
                    self.send_to_serial(command)

                resourceMonitorRecv = self.resourceMonitorSubscriber.receive()
                if resourceMonitorRecv is not None:
                    if self.debugger:
                        self.logger.info(resourceMonitorRecv)
                    command = {"action": "resourceMonitor", "activate": int(resourceMonitorRecv)}
                    self.send_to_serial(command)

                imuRecv = self.imuSubscriber.receive()
                if imuRecv is not None:
                    if self.debugger:
                        self.logger.info(imuRecv)
                    command = {"action": "imu", "activate": int(imuRecv)}
                    self.send_to_serial(command)

        except Exception as e:
            print(f"\033[1;97m[ Serial Handler ] :\033[0m \033[1;91mERROR\033[0m - {e}")
            self.serialConnectionStateSender.send(False)

    # ==================================== START =========================================
    def start(self):
        super(threadWrite, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        """This function will close the thread and will stop the car."""
        self.exampleFlag = False
        command = {"action": "kl", "mode": 0}
        self.send_to_serial(command)
        super(threadWrite, self).stop()

    # ================================== EXAMPLE =========================================
    def example(self):
        """This function simulte the movement of the car."""
        if self.exampleFlag:
            self.speedMotorSender.send({"Type": "Speed", "value": self.s})
            self.steerMotorSender.send({"Type": "Steer", "value": self.i})
            self.i += self.j
            if self.i >= 21.0:
                self.i = 21.0
                self.s = self.i / 7
                self.j *= -1
            if self.i <= -21.0:
                self.i = -21.0
                self.s = self.i / 7
                self.j *= -1.0
            threading.Timer(0.01, self.example).start()

    def _should_send_error(self):
        """Check if we should send an error message (rate limiting)."""
        now = datetime.now()
        if self.last_error_time is None or (now - self.last_error_time) >= self.error_cooldown:
            self.last_error_time = now
            return True
        return False
