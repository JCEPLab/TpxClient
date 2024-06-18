import zmq
import struct
import numpy as np
from scipy.stats import linregress

from time import sleep

import json

class ServerCommand:

    NOP = 0

    GET_SOFTWARE_VERSION = 1
    GET_FIRMWARE_VERSION = 2

    GET_CAMERA_ID = 3
    GET_DEVICE_IDS = 4
    GET_LOCAL_TEMP = 5
    GET_REMOTE_TEMP = 6
    GET_FPGA_TEMP = 7
    GET_FAN_SPEED = 8
    GET_PRESSURE = 9
    GET_HUMIDITY = 10

    GET_SPIDR_ADC = 11
    SET_BIAS_VOLTAGE = 12
    SET_BIAS_SUPPLY_ENABLED = 13

    GET_DEVICE_PORT = 14
    GET_SERVER_PORT = 15
    SET_SERVER_PORT = 16

    SET_TOA_DECODERS_ENABLED = 17

    RESTART_TIMERS = 18
    RESET_TIMERS = 19

    RESET_PIXELS = 20

    GET_SPIDR_REGISTER = 21
    SET_SPIDR_REGISTER = 22

    GET_PLL_CONFIG = 23
    SET_PLL_CONFIG = 24

    GET_HEADER_FILTER = 25
    SET_HEADER_FILTER = 26

    GET_GEN_CONFIG = 27
    SET_GEN_CONFIG = 28

    GET_PERIOD_PHASE = 29
    SET_PERIOD_PHASE = 30

    GET_TRIGGER_CONF = 31
    SET_TRIGGER_CONF = 32

    GET_DAC = 33
    SET_DAC = 34

    START_READOUT = 35
    STOP_READOUT = 36

    AUTOTRIG_START = 37
    AUTOTRIG_STOP = 38

    GET_PIXEL_CONFIG = 39
    SET_PIXEL_CONFIG = 40

    RESET_MODULE = 41
    GET_READOUT_SPEED = 42

    SET_SENSEDAC = 48

    SET_UDP_PORT = 500
    SET_RAW_TPX3_PATH = 501
    GET_RAW_DATA_SERVER_PATH = 502

    CANT_OPEN_FILE = 994
    THREAD_NOT_CONNECTED = 995
    THREAD_NOT_STARTED = 996
    SERVER_RESET = 997
    ERROR_OCCURED = 998
    UNKNOWN_COMMAND = 999

SERVER_ERROR_NAMES = {
    ServerCommand.CANT_OPEN_FILE: "Cannot open the specified file",
    ServerCommand.THREAD_NOT_CONNECTED: "Thread is not connected",
    ServerCommand.THREAD_NOT_STARTED: "Thread has not been started",
    ServerCommand.SERVER_RESET: "Server reset",
    ServerCommand.ERROR_OCCURED: "Error occurred while executing command",
    ServerCommand.UNKNOWN_COMMAND: "Unknown command code"
}

class SpidrRegister:
    SPIDR_CPU2TPX_WR_I = 0x01C8
    SPIDR_SHUTTERTRIG_CTRL_I = 0x0290
    SPIDR_SHUTTERTRIG_CNT_I = 0x0294
    SPIDR_SHUTTERTRIG_FREQ_I = 0x0298
    SPIDR_SHUTTERTRIG_LENGTH_I = 0x029C
    SPIDR_SHUTTERTRIG_DELAY_I = 0x02AC
    SPIDR_DEVICES_AND_PORTS_I = 0x02C0
    SPIDR_TDC_TRIGGERCOUNTER_I = 0x02F8
    SPIDR_FE_GTX_CTRL_STAT_I = 0x0300
    SPIDR_PIXEL_PKTCOUNTER_I = 0x0340
    SPIDR_IPMUX_CONFIG_I = 0x0380
    SPIDR_UDP_PKTCOUNTER_I = 0x0384
    SPIDR_UDPMON_PKTCOUNTER_I = 0x0388
    SPIDR_UDPPAUSE_PKTCOUNTER_I = 0x038C
    SPIDR_PIXEL_PKTCOUNTER_OLD_I = 0x0390
    SPIDR_PIXEL_FILTER_I = 0x0394

class SpidrDAC:
    IBIAS_PREAMP_ON = 1
    IBIAS_PREAMP_OFF = 2
    VPREAMP_NCAS = 3
    IBIAS_IKRUM = 4
    VFBK = 5
    VTHRESHOLD_FINE = 6
    VTHRESHOLD_COARSE = 7
    IBIAS_DISCS1_ON = 8
    IBIAS_DISCS1_OFF = 9
    IBIAS_DISCS2_ON = 10
    IBIAS_DISCS2_OFF = 11
    IBIAS_PIXELDAC = 12
    IBIAS_TPBUFFERIN = 13
    IBIAS_TPBUFFEROUT = 14
    VTP_COARSE = 15
    VTP_FINE = 16
    IBIAS_CP_PLL = 17
    PLL_VCNTROL = 18

class ShutterMode:
    EXTERNAL_RISING_FALLING = 0
    EXTERNAL_FALLING_RISING = 1
    EXTERNAL_RISING_TIMER = 2
    EXTERNAL_FALLING_TIMER = 3
    AUTO = 4
    PULSE_COUNTER = 5
    OPEN = 6

class PolarityModes:
    POSITIVE = 0x0
    NEGATIVE = 0x1

class OperationModes:
    TOA_AND_TOT = 0x0
    TOA = 0x2
    EVENT_TOA = 0x4
    MASK = 0x6

class GrayCounterModes:
    DISABLED = 0x0
    ENABLED = 0x8

class TestPulseModes:
    DISABLED = 0x0
    ENABLED = 0x20

class SuperPixModes:
    DISABLED = 0x0
    ENABLED = 0x40

class TimerOverflowModes:
    CYCLE = 0x0
    STOP = 0x80

class TestPulseTypes:
    FRONT_END_ANALOG = 0x0
    DISCRIMINATOR_DIGITAL = 0x200

class TestPulseGenerators:
    INTERNAL = 0x0
    EXTERNAL = 0x400

class ToAClockModes:
    PHASE_SHIFTED_GRAY = 0x0
    SYSTEM_CLOCK = 0x800

class TimepixGenConfig:
    opMode = OperationModes.TOA_AND_TOT
    polarity = PolarityModes.POSITIVE
    grayCounter = GrayCounterModes.ENABLED
    testPulse = TestPulseModes.ENABLED
    superPix = SuperPixModes.ENABLED
    timerOverflow = TimerOverflowModes.CYCLE
    testPulseType = TestPulseTypes.DISCRIMINATOR_DIGITAL
    testPulseGenerator = TestPulseGenerators.INTERNAL
    toaClock = ToAClockModes.PHASE_SHIFTED_GRAY

    def validate(self):
        valid = (
            (self.polarity & 0x1 == self.polarity)
            and (self.opMode & 0x6 == self.opMode)
            and (self.grayCounter & 0x8 == self.grayCounter)
            and (self.testPulse & 0x20 == self.testPulse)
            and (self.superPix & 0x40 == self.superPix)
            and (self.timerOverflow & 0x80 == self.timerOverflow)
            and (self.testPulseType & 0x200 == self.testPulseType)
            and (self.testPulseGenerator & 0x400 == self.testPulseGenerator)
            and (self.toaClock & 0x800 == self.toaClock)
        )

        return valid

    def toInt(self):
        if(not self.validate()):
            raise RuntimeError("Invalid TimepixGenConfig object")

        result = 0
        result |= self.polarity
        result |= self.opMode
        result |= self.grayCounter
        result |= self.testPulse
        result |= self.superPix
        result |= self.timerOverflow
        result |= self.testPulseType
        result |= self.testPulseGenerator
        result |= self.toaClock

        return result

    def fromInt(self, val:int):
        self.polarity = val & 0x1
        self.opMode = val & 0x6
        self.grayCounter = val & 0x8
        self.testPulse = val & 0x20
        self.superPix = val & 0x40
        self.timerOverflow = val & 0x80
        self.testPulseType = val & 0x200
        self.testPulseGenerator = val & 0x400
        self.toaClock = val & 0x800

        if(not self.validate()):
            raise RuntimeError("Failed to load TimepixGenConfig object from int")

        return self

class TpxClient:

    def __init__(self, port=48288, timeout_ms=10000):
        self.reconnect(port=port, timeout_ms=timeout_ms)
        self._bias_voltage_calibration = None

    def calibrateBiasVoltage(self, max_voltage=40):
        calibration = []
        set_bias = 12
        read_bias = 12
        while read_bias < max_voltage:
            self.setBiasVoltage(set_bias)
            sleep(0.05)
            read_bias = self.getBiasVoltage()

            calibration.append([set_bias, read_bias])

            set_bias += 1.5

        calibration = np.array(calibration)
        linfit = linregress(calibration[:,0], calibration[:,1])
        self._bias_voltage_calibration = [linfit.slope, linfit.intercept]

    def initialize(self, mask_image:str, thlAdj_image:str, test_img:str, dac_config:str):
        self.resetModule()
        print("Resetting SPIDR")
        self.setHeaderFilter(eth_mask=0xCD0, cpu_mask=0xF39F)
        self.resetTimers()
        self.setPeriodAndPhase(0x8001d)
        self.setBiasVoltageEnabled(True)
        print("Generating bias voltage calibration")
        self.calibrateBiasVoltage()
        self.setBiasVoltage(40)
        print("Loading pixel and DAC configuration")
        self.loadPixelConfig(mask_image, thlAdj_image, test_img)
        self.loadDacValues(dac_config)
        self.resetPixels()

        print("Starting UDP server")
        self.startUdpServer()

        config = TimepixGenConfig()
        config.opMode = OperationModes.TOA_AND_TOT
        config.polarity = PolarityModes.POSITIVE
        config.grayCounter = GrayCounterModes.DISABLED
        config.testPulse = TestPulseModes.DISABLED
        config.superPix = SuperPixModes.ENABLED
        config.timerOverflow = TimerOverflowModes.CYCLE
        config.testPulseType = TestPulseTypes.FRONT_END_ANALOG
        config.testPulseGenerator = TestPulseGenerators.INTERNAL
        config.toaClock = ToAClockModes.PHASE_SHIFTED_GRAY
        self.setGenConfig(config)

        self.setToADecodersEnabled(True)
        print("Timepix client initialized.")

    def takeSingleImage(self, exposure_s, fname=None):
        if fname is not None:
            self.setRawTpx3Path(fname)

        self.startReadout()

        self.setShutterParameters(exposure_s, exposure_s*1.05, 1)

        self.restartTimers()
        self.startAutoTrigger()

        sleep(exposure_s)

        self.stopAutoTrigger()
        self.stopReadout()

    def reconnect(self, port=None, timeout_ms=None, max_attempts=10):
        for i in range(max_attempts):
            try:
                self._reconnect_once(port=port, timeout_ms=timeout_ms)
                print(f"Connected to tcp://localhost:{port}")
                return
            except:
                pass

        raise RuntimeError(f"Unable to connect to Timepix server after {max_attempts} attempts")

    def _reconnect_once(self, port=None, timeout_ms=None):
        if port is not None:
            self._port = port

        if timeout_ms is not None:
            self._timeout = timeout_ms

        if hasattr(self, '_zmq'):
            self._zmq.destroy()

        self._zmq = zmq.Context()
        self._command_sock = self._zmq.socket(zmq.REQ)
        self._command_sock.RCVTIMEO = self._timeout

        socket_addr = 'tcp://localhost:' + str(self._port)
        self._command_sock.connect(socket_addr)

    def getZmq(self):
        return self._zmq

    def _send_req(self, command, data=[], fmt='I', wait_for_response=True):
        command_buffer = struct.pack("=I", int(command))

        if len(data) == 1:
            format_str = '=' + str(len(data)) + fmt
            data_buffer = struct.pack(format_str, data[0])
            command_buffer += data_buffer
        elif len(data) > 1:
            format_str = '=' + str(len(data)) + fmt
            data_buffer = struct.pack(format_str, *data)
            command_buffer += data_buffer

        self._command_sock.send(command_buffer)

        if wait_for_response:
            response = self._command_sock.recv()

            response_command = self._unpack_int(response[:4])[0]
            if response_command != command:
                raise RuntimeError("An error occurred while executing a Timepix command: " + SERVER_ERROR_NAMES[response_command])

            response = response[4:]

            return response

    def _unpack_int(self, data, size=1, type='I'):
        fmt = '=' + (type * size)
        return struct.unpack(fmt, data)

    def getSoftwareVersion(self):
        data = self._send_req(ServerCommand.GET_SOFTWARE_VERSION)

        return self._unpack_int(data)[0]

    def getFirmwareVersion(self):
        data = self._send_req(ServerCommand.GET_FIRMWARE_VERSION)

        return self._unpack_int(data)[0]

    def getCameraId(self):
        val = self._send_req(ServerCommand.GET_CAMERA_ID)
        name = (val[3] << 24) + (val[2] << 16) + (val[1] << 8) + val[0]

        return name

    def getDeviceIds(self):
        data = self._send_req(ServerCommand.GET_DEVICE_IDS)
        dev_array = self._unpack_int(data, 4)

        nonzero_devs = []
        for dev in dev_array:
            if dev != 2147483647:
                nonzero_devs.append(dev)

        return nonzero_devs

    def getLocalTemp(self):
        data = self._send_req(ServerCommand.GET_LOCAL_TEMP)

        return self._unpack_int(data)[0] / 1000

    def getRemoteTemp(self):
        data = self._send_req(ServerCommand.GET_REMOTE_TEMP)

        return self._unpack_int(data)[0] / 1000

    def getFpgaTemp(self):
        data = self._send_req(ServerCommand.GET_FPGA_TEMP)

        return self._unpack_int(data)[0] / 1000

    def getFanSpeed(self):
        data = self._send_req(ServerCommand.GET_FAN_SPEED)

        return self._unpack_int(data)[0]

    def getPressure(self):
        data = self._send_req(ServerCommand.GET_PRESSURE)

        return self._unpack_int(data)[0] / 1000

    def getHumidity(self):
        data = self._send_req(ServerCommand.GET_HUMIDITY)

        return self._unpack_int(data)[0]

    def getSpidrAdc(self, val):
        data = self._send_req(ServerCommand.GET_SPIDR_ADC, [val])

        return self._unpack_int(data)[0]

    def getBiasVoltage(self):
        raw_val = self.getSpidrAdc(1)

        return (((raw_val & 0xFFF) * 1500 + 4095) / 4096) / 10

    def setBiasVoltage(self, volts):
        if self._bias_voltage_calibration is not None:
            volts = (volts - self._bias_voltage_calibration[1])/self._bias_voltage_calibration[0]

        volts = np.clip(volts, 12, 104)

        adc_code = int(((volts - 12) * 4095) / (104 - 12))
        data = self._send_req(ServerCommand.SET_BIAS_VOLTAGE, [adc_code])

        raw_val = self._unpack_int(data)[0]

        sleep(0.1)

        return (float(raw_val) * (104 - 12)) / 4095 + 12

    def setBiasVoltageEnabled(self, enabled):
        data = self._send_req(ServerCommand.SET_BIAS_SUPPLY_ENABLED, [int(enabled)])

        return self._unpack_int(data)[0]

    def getDevicePort(self):
        data = self._send_req(ServerCommand.GET_DEVICE_PORT)

        return self._unpack_int(data)[0]

    def getServerPort(self):
        data = self._send_req(ServerCommand.GET_SERVER_PORT)

        return self._unpack_int(data)[0]

    def setServerPort(self, val):
        data = self._send_req(ServerCommand.SET_SERVER_PORT, [val])

        return self._unpack_int(data)[0]

    def setToADecodersEnabled(self, enabled):
        data = self._send_req(ServerCommand.SET_TOA_DECODERS_ENABLED, [int(enabled)])

        return self._unpack_int(data)[0]

    def restartTimers(self):
        self._send_req(ServerCommand.RESTART_TIMERS)

    def resetTimers(self):
        self._send_req(ServerCommand.RESET_TIMERS)

    def resetPixels(self):
        self._send_req(ServerCommand.RESET_PIXELS)

    def getRegister(self, register):
        data = self._send_req(ServerCommand.GET_SPIDR_REGISTER, [register])

        parsed = self._unpack_int(data, 2)
        if(parsed[0] != register):
            raise RuntimeError(f"Timepix returned value for register {parsed[0]}, expected data from {register}")

        return parsed[1]

    def setRegister(self, register, val):
        data = self._send_req(ServerCommand.SET_SPIDR_REGISTER, [register, val])

        return self._unpack_int(data)[0]

    def getPLLConfig(self):
        data = self._send_req(ServerCommand.GET_PLL_CONFIG)

        return self._unpack_int(data)[0]

    def setPLLConfig(self, val):
        data = self._send_req(ServerCommand.SET_PLL_CONFIG, [val])

        return self._unpack_int(data)[0]

    def getHeaderFilter(self):
        data = self._send_req(ServerCommand.GET_HEADER_FILTER)

        val = self._unpack_int(data)[0]
        cpu_mask = (val & 0xFFFF0000) >> 16
        eth_mask = (val & 0x0000FFFF)

        return cpu_mask, eth_mask

    def setHeaderFilter(self, cpu_mask, eth_mask):
        val = ((cpu_mask & 0xFFFF) << 16) | (eth_mask & 0xFFFF)
        data = self._send_req(ServerCommand.SET_HEADER_FILTER, [val])

        new_val = self._unpack_int(data)[0]

        cpu_mask = (new_val & 0xFFFF0000) >> 16
        eth_mask = (new_val & 0x0000FFFF)

        return cpu_mask, eth_mask

    def getGenConfig(self):
        data = self._send_req(ServerCommand.GET_GEN_CONFIG)

        config = TimepixGenConfig()
        config.fromInt(self._unpack_int(data)[0])
        return config

    def setGenConfig(self, config : TimepixGenConfig):
        val = config.toInt()
        data = self._send_req(ServerCommand.SET_GEN_CONFIG, [val])

        return self._unpack_int(data)[0]

    def getPeriodAndPhase(self):
        data = self._send_req(ServerCommand.GET_PERIOD_PHASE)

        return self._unpack_int(data)[0]

    def setPeriodAndPhase(self, period_and_phase):
        data = self._send_req(ServerCommand.SET_PERIOD_PHASE, [period_and_phase])

        return self._unpack_int(data)[0]

    def getTriggerConf(self):
        data = self._send_req(ServerCommand.GET_TRIGGER_CONF)

        return self._unpack_int(data, 5)

    def setTriggerConf(self, shutter_mode, exposure_us, freq_hz, trig_count, delay_ns):
        self._send_req(ServerCommand.SET_TRIGGER_CONF, [shutter_mode, exposure_us, freq_hz, trig_count, delay_ns])

    def getDac(self, dac):
        data = self._send_req(ServerCommand.GET_DAC, [dac])

        return self._unpack_int(data)[0]

    def setDac(self, dac, val):
        dac_val = ((dac & 0xFFFF) << 16) | (val & 0xFFFF)
        data = self._send_req(ServerCommand.SET_DAC, [dac_val])

        return self._unpack_int(data)[0]

    def startReadout(self):
        self._send_req(ServerCommand.START_READOUT)

    def stopReadout(self):
        self._send_req(ServerCommand.STOP_READOUT)

    def startAutoTrigger(self):
        self._send_req(ServerCommand.AUTOTRIG_START)

    def stopAutoTrigger(self):
        self._send_req(ServerCommand.AUTOTRIG_STOP)

    def _get_pixel_config_row(self, row):
        data = self._send_req(ServerCommand.GET_PIXEL_CONFIG, [row])

        col = self._unpack_int(data[:4])
        row_data = self._unpack_int(data[4:], 256, 'B')

        mask = [x & 0x1 for x in row_data]
        threshold = [((x >> 1) & 0xF) for x in row_data]
        test = [((x >> 5) & 0x1) for x in row_data]

        return mask, threshold, test

    def getPixelConfig(self):
        mask = np.zeros((256, 256), dtype=int)
        threshold = np.zeros_like(mask, dtype=int)
        test = np.zeros_like(threshold, dtype=int)

        for i in range(256):
            mask[:, i], threshold[:, i], test[:, i] = self._get_pixel_config_row(i)

        return mask, threshold, test

    def _send_pixel_rows(self, pixel_data, start_row, rows):
        byte_vals = np.zeros(4 + int(256*3*rows/4), int)  # 256 pixels/row * 3 bytes/4 pixels
        byte_vals[3] = start_row
        # process bytes in groups of three
        for i in np.arange(64*rows):
            three_bytes = 0
            for j in np.arange(4):
                byte_ix = i * 4 + j
                row = start_row + (byte_ix // 256)
                col = (byte_ix % 256)
                pixel_val = pixel_data[row, col]
                three_bytes = three_bytes | (pixel_val << (18 - j * 6))
            byte_vals[4 + i * 3] = (three_bytes >> 16) & 0xFF
            byte_vals[4 + i * 3 + 1] = (three_bytes >> 8) & 0xFF
            byte_vals[4 + i * 3 + 2] = three_bytes & 0xFF

        byte_vals_le = np.zeros(len(byte_vals), dtype=int)
        for ix in range(0,len(byte_vals),4):
            byte_vals_le[ix] = byte_vals[ix+3]
            byte_vals_le[ix+1] = byte_vals[ix+2]
            byte_vals_le[ix+2] = byte_vals[ix+1]
            byte_vals_le[ix+3] = byte_vals[ix]

        response = self._send_req(ServerCommand.SET_PIXEL_CONFIG, byte_vals_le.tolist(), fmt='B')

    def setPixelConfig(self, mask : np.ndarray, threshold : np.ndarray, test : np.ndarray):
        mask = 1-(mask & 0x1)

        threshold = threshold & 0xF
        threshold = ((threshold & 0x01) << 3) | ((threshold & 0x02) << 1) | ((threshold & 0x04) >> 1) | ((threshold & 0x08) >> 3)

        test = 1-(test & 0x1)

        pixel_vals = (test << 5) | (threshold << 1) | mask

        pixel_vals = np.flipud(pixel_vals).transpose()

        for start_row in range(0, 252, 3):
            self._send_pixel_rows(pixel_vals, start_row, 3)

        self._send_pixel_rows(pixel_vals, 252, 2)
        self._send_pixel_rows(pixel_vals, 254, 2)

    def loadPixelConfig(self, mask_fname:str, thl_fname:str, test_fname:str):
        mask = np.loadtxt(mask_fname, delimiter=' ', dtype=int)
        thl = np.loadtxt(thl_fname, delimiter=' ', dtype=int)
        test = np.loadtxt(test_fname, delimiter=' ', dtype=int)

        self.setPixelConfig(mask, thl, test)

    def resetModule(self, readout_speed=0):
        self._send_req(ServerCommand.RESET_MODULE, [readout_speed])
        sleep(3)
    
    def getReadoutSpeed(self):
        data = self._send_req(ServerCommand.GET_READOUT_SPEED)
        
        return self._unpack_int(data)[0]

    def startUdpServer(self):
        host_port = self.getServerPort()
        self._send_req(ServerCommand.SET_UDP_PORT, [host_port])

    def getShutterParameters(self):
        frames = self.getRegister(SpidrRegister.SPIDR_SHUTTERTRIG_CNT_I)
        time_int = self.getRegister(SpidrRegister.SPIDR_SHUTTERTRIG_LENGTH_I)
        freq_int = self.getRegister(SpidrRegister.SPIDR_SHUTTERTRIG_FREQ_I)

        return float(time_int)/1e9*25, 1e9/float(freq_int)/25, frames

    def setShutterParameters(self, time_s:float, period_s:float, frames:int):
        time_int = int(time_s * 40000000)
        freq_int = int(period_s * 40000000)

        self.setRegister(SpidrRegister.SPIDR_SHUTTERTRIG_CTRL_I, 0x804) # auto trigger
        self.setRegister(SpidrRegister.SPIDR_SHUTTERTRIG_CNT_I, frames)
        self.setRegister(SpidrRegister.SPIDR_SHUTTERTRIG_LENGTH_I, time_int)
        self.setRegister(SpidrRegister.SPIDR_SHUTTERTRIG_FREQ_I, freq_int)

    def setSenseDac(self, val):
        data = self._send_req(ServerCommand.SET_SENSEDAC, [val])

        return self._unpack_int(data)[0]

    def loadDacValues(self, json_path:str):
        with open(json_path, 'r') as f:
            data = json.load(f)
            dacs = data['ChipConfigs'][0]['dacs']

            dac_str_map = {
                'Ibias_Preamp_ON': SpidrDAC.IBIAS_PREAMP_ON,
                'Ibias_Preamp_OFF': SpidrDAC.IBIAS_PREAMP_OFF,
                'VPreamp_NCAS': SpidrDAC.VPREAMP_NCAS,
                'Ibias_Ikrum': SpidrDAC.IBIAS_IKRUM,
                'Vfbk': SpidrDAC.VFBK,
                'Vthreshold_fine': SpidrDAC.VTHRESHOLD_FINE,
                'Vthreshold_coarse': SpidrDAC.VTHRESHOLD_COARSE,
                'Ibias_DiscS1_ON': SpidrDAC.IBIAS_DISCS1_ON,
                'Ibias_DiscS1_OFF': SpidrDAC.IBIAS_DISCS1_OFF,
                'Ibias_DiscS2_ON': SpidrDAC.IBIAS_DISCS2_ON,
                'Ibias_DiscS2_OFF': SpidrDAC.IBIAS_DISCS2_OFF,
                'Ibias_PixelDAC': SpidrDAC.IBIAS_PIXELDAC,
                'Ibias_TPbufferIn': SpidrDAC.IBIAS_TPBUFFERIN,
                'Ibias_TPbufferOut': SpidrDAC.IBIAS_TPBUFFEROUT,
                'VTP_coarse': SpidrDAC.VTP_COARSE,
                'VTP_fine': SpidrDAC.VTP_FINE,
                'Ibias_CP_PLL': SpidrDAC.IBIAS_CP_PLL,
                'PLL_Vcntrl': SpidrDAC.PLL_VCNTROL
            }

            for dac in dacs:
                self.setDac(dac_str_map[dac], dacs[dac])

    def setRawTpx3Path(self, path:str):
        self._send_req(ServerCommand.SET_RAW_TPX3_PATH, list((path + '\0').encode('ascii')))

    def getRawDataServerPath(self):
        response = self._send_req(ServerCommand.GET_RAW_DATA_SERVER_PATH)
        return response.decode('ascii').strip('\0')