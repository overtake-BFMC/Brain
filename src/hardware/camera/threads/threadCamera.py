import cv2
import threading
import base64
import time
from src.utils.messages.allMessages import (
    mainCamera,
    serialCamera,
    Recording,
    Record,
    Brightness,
    Contrast,
    MainVideo,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.threadwithstop import ThreadWithStop

class threadCamera(ThreadWithStop):
    """Thread which will handle camera functionalities."""

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 15
        self.recording = False
        self.send_fps = 15
        self.frame_interval = 1.0 / self.send_fps
        self.last_sent_time = time.time()
        self.video_writer = ""

        #frame storing for multiple access
        self.latest_frame = None

        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        self.mainVideoSender = messageHandlerSender(self.queuesList, MainVideo)

        self.subscribe()
        self._init_camera()
        self.Queue_Sending()
        self.Configs()

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway."""
        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)

    def Queue_Sending(self):
        """Callback function for recording flag."""
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.Queue_Sending).start()

    # =============================== STOP ================================================
    def stop(self):
        if self.recording:
            self.video_writer.release()
        super(threadCamera, self).stop()

    # =============================== CONFIG ==============================================
    def Configs(self):
        """Callback function for receiving configs on the pipe."""
        if self.brightnessSubscriber.isDataInPipe():
            message = self.brightnessSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set(cv2.CAP_PROP_BRIGHTNESS, max(0.0, min(1.0, float(message))))

        if self.contrastSubscriber.isDataInPipe():
            message = self.contrastSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set(cv2.CAP_PROP_CONTRAST, max(0.0, min(32.0, float(message))))

        threading.Timer(1, self.Configs).start()

    # ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True. It captures the image from camera and makes the required modifications and sends the data to process gateway."""
        send = True
        frame_counter = 0  # Counter for frames
        recordingInitialized = False
        while self._running:
            try:
                recordRecv = self.recordSubscriber.receive()
                if recordRecv is not None:

                    if recordRecv == 'false':
                        self.recording = False
                    else:
                        self.recording = True

                    if self.recording:
                        if not recordingInitialized:

                            fourcc = cv2.VideoWriter_fourcc(*"H264")
                            self.video_writer = cv2.VideoWriter(
                                "output_video" + str(time.time()) + ".avi",
                                fourcc,
                                self.frame_rate,
                                (960, 540),
                            )
                            recordingInitialized = True
                    else:
                        if recordingInitialized:
                            self.video_writer.release()
                            self.video_writer = ""
                            recordingInitialized = False

                        
            except Exception as e:
                print(e)

            if send:
                ret, frame = self.camera.read()
                if ret:
                    #cv2.imshow('Frame',frame)
                    #cv2.waitKey(1)
                    self.latest_frame = frame
                    #frame_counter += 1  # Increment frame counter
                    #current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Get current time
                    #print(f"Processing frame: {frame_counter} at {current_time}")  # Print counter and time to terminal
                                        
                    #serialRequest = cv2.resize(frame, (960, 540))  # Lo-res capture
                    _, encoded_frame = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    decoded_frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)

                    #if self.recording:
                    #    self.video_writer.write(frame)

                    #_, mainEncodedImg = cv2.imencode(".jpg", frame)
                    #_, serialEncodedImg = cv2.imencode(".jpg", serialRequest)

                    #mainEncodedImageData = base64.b64encode(mainEncodedImg).decode("utf-8")
                    #serialEncodedImageData = base64.b64encode(serialEncodedImg).decode("utf-8")

                    #self.mainCameraSender.send(mainEncodedImageData)
                    #if (frame_counter % 5) == 0:
                    #    self.serialCameraSender.send(serialEncodedImageData)
                    current_time = time.time()
                    if current_time - self.last_sent_time >= self.frame_interval:
                        if self.recording:
                            self.video_writer.write(decoded_frame)
                        self.last_sent_time = current_time
                        self.mainVideoSender.send(decoded_frame)

        send = not send

    # =============================== START ===============================================
    def start(self):
        super(threadCamera, self).start()

    # ================================ INIT CAMERA ========================================
    def gstreamer_pipeline(
        self,
        sensor_id=0,
        capture_width=3280,
        capture_height=1848,
        display_width=960,
        display_height=540,
        framerate=30,
        flip_method=0,
    ):

        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )

    def _init_camera(self):
        """This function will initialize the camera object with GStreamer pipeline."""
        #self.show_camera()
        # Use GStreamer for CSI camera on Jetson
        gst_pipeline = self.gstreamer_pipeline(flip_method=0, framerate=self.frame_rate)

        self.camera = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.camera.isOpened():
            raise Exception("Could not open video device.")

# def gstreamer_pipeline(
#         self,
#         sensor_id=0,
#         capture_width=1920,
#         capture_height=1080,
#         display_width=960,
#         display_height=540,
#         framerate=30,
#         flip_method=0,
#     ):

#         return (
#             "nvarguscamerasrc sensor-id=%d ! "
#             "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
#             "nvvidconv flip-method=%d ! "
#             "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
#             "videoconvert ! "
#             "video/x-raw, format=(string)BGR ! appsink"
#             % (
#                 sensor_id,
#                 capture_width,
#                 capture_height,
#                 framerate,
#                 flip_method,
#                 display_width,
#                 display_height,
#             )
#         )