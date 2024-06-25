import numpy as np
import threading
import zmq
import struct
import time
import fast_histogram as fhist

class TpxRawDataPlotter:

    def __init__(self, client, update_hist_func, update_s, server_path):
        self._server_path = server_path
        self._cancelled = False
        self._thread = None
        self._subscriber = None
        self._zmq = client.getZmq()
        self._update_hist_func = update_hist_func
        self._last_update_time = 0
        self._hist = np.zeros((256, 256), dtype=int)
        self._update_time_s = update_s

    def launchThread(self):
        self._thread = threading.Thread(target=self.run)
        self._cancelled = False
        self._thread.start()

    def stopThread(self):
        self._cancelled = True
        self._thread.join()

    def run(self):
        self._subscriber = self._zmq.socket(zmq.SUB)
        self._subscriber.connect(self._server_path)
        self._subscriber.subscribe("")

        while not self._cancelled:
            has_msg = self._subscriber.poll(250)
            if has_msg:
                data = self._subscriber.recv()
                self.histogramData(data)

            self.updateExternalHist()

        self._subscriber.close()

    def histogramData(self, data):
        arr = np.array(struct.unpack('='+str(len(data))+'B', data)).astype(np.uint8)
        if len(arr) > 0:
            new_hist = fhist.histogram2d(arr[6::8], arr[7::8], bins=256, range=[[0, 256], [0, 256]])
            self._hist += new_hist.astype(int)

    def updateExternalHist(self):
        if (time.monotonic() - self._last_update_time) > self._update_time_s:
            self._update_hist_func(self._hist)
            self._hist *= 0
            self._last_update_time = time.monotonic()