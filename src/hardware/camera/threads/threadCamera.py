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

import cv2
import threading
import base64
import picamera2
import time
import os

from src.utils.messages.allMessages import (
    mainCamera,
    serialCameraRaw,
    Recording,
    Record,
    Brightness,
    Contrast,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import StateChange
from src.statemachine.systemMode import SystemMode


class threadCamera(ThreadWithStop):
    """Thread which will handle camera functionalities.\n
    Args:
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__(pause=0.001)
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 10
        self.PUBLISH_HZ = self.frame_rate
        self.recording = False

        self.video_writer = ""
        self._next_frame_deadline = 0.0

        self.dashboard_stream_enabled = True
        self.perception_stream_enabled = True
        self.yolo_stream_size = (1280, 720)
        self.perception_stream_size = (512, 270)
        self.record_stream_size = (1280, 720)

        self.frame_rate = 30              # lane camera cadence
        self._cam_tick = 0

        self.YOLO_EVERY_N = 6             # 30 / 6 = 5 FPS for YOLO
        self.MAIN_JPEG_QUALITY = 45       # lighter than 65
        self.main_encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.MAIN_JPEG_QUALITY]

        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraRawSender = messageHandlerSender(self.queuesList, serialCameraRaw)

        # frame count
        self._cam_fps_window_start = time.time()
        self._cam_fps_count = 0


        ################ NovaVision 26.01.2026 ###############
        # NEW ELEMENT:
        # - Track AUTO/MANUAL mode locally (visual/debug/perception context only)
        # - No actuator commands are sent from camera thread
        #####################################################
        self.is_auto_mode = False

        ################ NovaVision 26.01.2026 ###############
        # NEW ELEMENT:
        # - Controlled JPEG quality (stable bandwidth)
        # - Matches your newer camera thread behavior
        #####################################################
        # self.dashboard_encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
        self.perception_encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 55]

        self.subscribe()
        self._init_camera()
        self.queue_sending()
        self.configs()

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""

        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)
        self.stateChangeSubscriber = messageHandlerSubscriber(self.queuesList, StateChange, "lastOnly", True)

    def queue_sending(self):
        """Callback function for recording flag."""
        if self._blocker.is_set():
            return
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.queue_sending).start()

    # ================================ RUN ================================================
    def thread_work(self):
        """This function will run while the running flag is True.
        It captures the image from camera and make the required modifies
        and then it send the data to process gateway."""
        # if camera is not available, skip processing
        start = time.time()

        # if camera is not available, skip processing
        if self.camera is None:
            time.sleep(0.1)
            return

        ################ NovaVision 26.01.2026 ###############
        # NEW ELEMENT:
        # - Actually call state handler inside loop
        # - Original had the function but did not call it here
        #####################################################
        try:
            self.state_change_handler()
        except Exception:
            pass

        try:
            recordRecv = self.recordSubscriber.receive()
            if recordRecv is not None:
                self.recording = bool(recordRecv)
                if recordRecv == False:
                    self.video_writer.release()  # type: ignore
                else:
                    fourcc = cv2.VideoWriter_fourcc(  # type: ignore
                        *"XVID"
                    )  # You can choose different codecs, e.g., 'MJPG', 'XVID', 'H264', etc.
                    self.video_writer = cv2.VideoWriter(
                        "output_video" + str(time.time()) + ".avi",
                        fourcc,
                        self.frame_rate,
                        self.record_stream_size,
                    )

        except Exception as e:
            print(f"\033[1;97m[ Camera ] :\033[0m \033[1;91mERROR\033[0m - {e}")

        try:
            self._cam_tick += 1

            # Always lane stream

            need_main = self.recording or ((self._cam_tick % self.YOLO_EVERY_N) == 0)

            req = self.camera.capture_request()
            try:
                lores = req.make_array("lores")
                # lores = cv2.cvtColor(lores, cv2.COLOR_RGB2BGR)
                main = req.make_array("main") if need_main else None
            finally:
                req.release()

            if self.recording and main is not None:
                self.video_writer.write(main)

            if self._blocker.is_set():
                return

            # Send YOLO stream less often
            if self.dashboard_stream_enabled and main is not None:
                ok_dash, dashboard_encoded_img = cv2.imencode(".jpg", main, self.main_encode_param)
                if ok_dash:
                    dashboard_encoded_image_data = base64.b64encode(dashboard_encoded_img).decode("utf-8")
                    self.mainCameraSender.send(dashboard_encoded_image_data)

            # Send lane raw every loop
            if self.perception_stream_enabled:
                self.serialCameraRawSender.send(lores)
                self._cam_fps_count += 1

            now = time.time()
            cam_window_dt = now - self._cam_fps_window_start
            if cam_window_dt >= 1.0:
                cam_fps = self._cam_fps_count / cam_window_dt if cam_window_dt > 0 else 0.0
                print(f"[Camera] lane_publish_fps={cam_fps:.1f}")
                self._cam_fps_window_start = now
                self._cam_fps_count = 0

            # rate limit ca sa nu flood-uim gateway-ul
            period = 1.0 / self.frame_rate if self.frame_rate > 0 else 0.0
            if period > 0.0:
                elapsed = time.time() - start
                time.sleep(max(0.0, period - elapsed))

        except Exception as e:
            print(f"\033[1;97m[ Camera ] :\033[0m \033[1;91mERROR\033[0m - {e}")

    # ================================ STATE CHANGE HANDLER ========================================
    def state_change_handler(self):
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            modeDict = SystemMode[message].value["camera"]["thread"]

            if "resolution" in modeDict:
                print(
                    f"\033[1;97m[ Camera Thread ] :\033[0m \033[1;92mINFO\033[0m - Resolution changed to {modeDict['resolution']}")

            ################ NovaVision 26.01.2026 ###############
            # NEW ELEMENT:
            # - Track AUTO/MANUAL flag without affecting SystemMode behavior
            # - Useful for dashboard overlay or perception context
            #####################################################
            try:
                state_name = str(message)
                if "AUTO" in state_name:
                    self.is_auto_mode = True
                else:
                    self.is_auto_mode = False
            except Exception:
                pass

    # ================================ INIT CAMERA ========================================
    def _init_camera(self):
        """This function will initialize the camera object. It will make this camera object have two chanels "lore" and "main"."""

        try:
            # check if camera is available
            if len(picamera2.Picamera2.global_camera_info()) == 0:
                print(
                    f"\033[1;97m[ Camera Thread ] :\033[0m \033[1;91mERROR\033[0m - No camera detected. Camera functionality will be disabled.")
                self.camera = None
                return
            
            self.camera = picamera2.Picamera2()
            config = self.camera.create_video_configuration(   # use video config for FPS
                buffer_count=4,
                queue=True,
                main={"format": "RGB888", "size": self.yolo_stream_size},
                lores={"format": "RGB888", "size": self.perception_stream_size},
                controls={"FrameDurationLimits": (33333, 33333)},  # ~30 FPS
            )

            self.camera.configure(config)  # type: ignore
            self.camera.start()
            print(f"\033[1;97m[ Camera Thread ] :\033[0m \033[1;92mINFO\033[0m - Camera initialized successfully")
        except Exception as e:
            print(f"\033[1;97m[ Camera Thread ] :\033[0m \033[1;91mERROR\033[0m - Failed to initialize camera: {e}")
            self.camera = None

    # =============================== STOP ================================================
    def stop(self):
        if self.recording and self.video_writer:
            self.video_writer.release()  # type: ignore
        if self.camera is not None:
            self.camera.stop()
        super(threadCamera, self).stop()

    # =============================== CONFIG ==============================================
    def configs(self):
        """Callback function for receiving configs on the pipe."""
        if self._blocker.is_set():
            return

        ################ NovaVision 26.01.2026 ###############
        # NEW ELEMENT:
        # - Safety guard: if camera is missing, skip set_controls
        # - Prevents NoneType crashes if camera disconnects
        #####################################################
        if self.camera is None:
            threading.Timer(1, self.configs).start()
            return

        if self.brightnessSubscriber.is_data_in_pipe():
            message = self.brightnessSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "Brightness": max(0.0, min(1.0, float(message))),  # type: ignore
                }
            )
        if self.contrastSubscriber.is_data_in_pipe():
            message = self.contrastSubscriber.receive()  # de modificat marti uc camera noua
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "Contrast": max(0.0, min(32.0, float(message))),  # type: ignore
                }
            )
        threading.Timer(1, self.configs).start()