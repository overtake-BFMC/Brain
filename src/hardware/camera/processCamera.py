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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
from multiprocessing import Queue, Event
import time
import logging
import cv2
import base64
import numpy as np
from src.templates.workerprocess import WorkerProcess
from src.hardware.camera.threads.threadCamera import threadCamera
from src.utils.logger.loggerConfig import setupLogger

class processCamera(WorkerProcess):
    """
    This process handles the camera.

    Args:
        queueList (dict of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging.Logger): Logger object for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, mainLogLevel = logging.INFO, consoleLogLevel = logging.WARNING, debugging = False):
        self.queuesList = queueList
        self.mainLogLevel = mainLogLevel
        self.consoleLogLevel = consoleLogLevel
        self.debugging = debugging
        self.logger = setupLogger(name=__name__, level=self.mainLogLevel, consoleLevel=self.consoleLogLevel)
        super(processCamera, self).__init__(self.queuesList)

    def run(self):
        """
        Apply the initializing methods and start the threads.
        """
        self.logger.info("Starting processCamera...")
        try:
            super(processCamera, self).run()
        except Exception as e:
            self.logger.critical(f"Error in processCamera run: {e}", exc_info=True)

    def _init_threads(self):
        """
        Create the Camera Publisher thread and add it to the list of threads.
        """
        try:
            camTh = threadCamera(self.queuesList, self.mainLogLevel, self.consoleLogLevel, self.debugging)
            self.threads.append(camTh)
            self.logger.info("Camera thread initialized successfully.")
        except Exception as e:
            self.logger.critical(f"Error initializing camera thread: {e}", exc_info=True)

if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s")
    logger = logging.getLogger()

    # Debugging flag
    debugg = True

    # Queue setup
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }

    # Initialize processCamera
    process = processCamera(queueList, logger, debugg)
    process.daemon = True

    try:
        process.start()
        logger.info("processCamera started.")

        # Simulate some processing
        time.sleep(4)

        if debugg:
            logger.warning("Attempting to retrieve image...")

        img = {"msgValue": 1}
        while not isinstance(img["msgValue"], str):
            try:
                img = queueList["General"].get(timeout=5)  # Avoid indefinite blocking
            except Exception as e:
                logger.error(f"Error retrieving image from queue: {e}")
                break

        if "msgValue" in img:
            try:
                image_data = base64.b64decode(img["msgValue"])
                img_array = np.frombuffer(image_data, dtype=np.uint8)
                image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                cv2.imwrite("test.jpg", image)
                logger.info("Image saved as test.jpg.")
            except Exception as e:
                logger.error(f"Error decoding or saving image: {e}", exc_info=True)
        else:
            logger.warning("No valid image data received.")

    except KeyboardInterrupt:
        logger.info("Process interrupted by user.")

    finally:
        process.stop()
        logger.info("processCamera stopped.")

