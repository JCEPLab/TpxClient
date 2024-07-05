import numpy as np
import threading
import zmq
import struct
import time

from client import *

class TpxImageRepeater:

    def __init__(self, client:TpxClient, exposure_s, image_triggered_func=None):
        self._cancelled = False
        self._thread = None
        self._client = client
        self._image_triggered_func = image_triggered_func
        self._last_update_time = 0
        self._exposure_s = exposure_s

    def setExposure(self, exposure_s):
        self._exposure_s = exposure_s

    def launchThread(self):
        self._thread = threading.Thread(target=self.run)
        self._cancelled = False
        self._thread.start()

    def stopThread(self):
        self._cancelled = True
        self._thread.join()

    def run(self):
        self._client.restartTimers()
        while not self._cancelled:
            self._client.takeSingleImage(self._exposure_s, restart_timer=False, keep_existing_fname=True)
            if self._image_triggered_func is not None:
                self._image_triggered_func()