from client import *
from raw_data_plotter import *
from image_repeater import *

import PySide6.QtWidgets as qt

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import numpy as np

class TpxImageCanvas(FigureCanvasQTAgg):

    def __init__(self, width=5, height=4, dpi=100):
        self._fig = Figure(figsize=(width,height), dpi=dpi)
        self._axes = self._fig.add_subplot(111)
        im = self._axes.imshow(np.random.random((256,256)))
        self._cbar = plt.colorbar(im)
        super().__init__(self._fig)

class TpxGuiMainWindow(qt.QMainWindow):

    def __init__(self):
        super().__init__()
        self._tpxplot = TpxImageCanvas()
        self.setCentralWidget(self._tpxplot)
        self.setWindowTitle("Timepix Viewer")

class TpxGui:

    def __init__(self, client):
        self._client = client
        self._app = qt.QApplication()
        self._window = TpxGuiMainWindow()
        self._image_thread = TpxImageRepeater(client, 1)
        self._raw_data_thread = TpxRawDataPlotter(client, self.updateRawHist, 0.1)

    def run(self):
        threads = [
            self._image_thread,
            self._raw_data_thread
        ]

        for thread in threads:
            thread.launchThread()

        self._window.show()
        self._app.exec()

        threads_reversed = threads.copy()
        threads_reversed.reverse()
        for thread in threads_reversed:
            thread.stopThread()

    def updateRawHist(self, data):
        self._window._tpxplot._axes.clear()
        im = self._window._tpxplot._axes.imshow(data)
        self._window._tpxplot._cbar.update_normal(im)
        self._window._tpxplot.draw()

if __name__ == '__main__':
    MASK_CONFIG_FILE = './config/maskBits.txt'
    THRESHOLD_CONFIG_FILE = './config/thlAdj.txt'
    TEST_CONFIG_FILE = './config/testBits.txt'
    DAC_CONFIG_FILE = './config/dacConfig.json'

    client = TpxClient()
    client.initialize(MASK_CONFIG_FILE, THRESHOLD_CONFIG_FILE, TEST_CONFIG_FILE, DAC_CONFIG_FILE)

    gui = TpxGui(client)
    gui.run()