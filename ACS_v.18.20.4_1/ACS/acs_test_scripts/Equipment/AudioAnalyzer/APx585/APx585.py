"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: implementation of Apx 585 audio analyzer
:since:27/11/2012
:author: nprecigx
"""

import time
import subprocess

from threading import Thread
from ctypes import *
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import ExeRunner
from acs_test_scripts.Equipment.AudioAnalyzer.Interface.IAudioAnalyzer import IAudioAnalyzer
from Device.DeviceManager import DeviceManager


class APx585(IAudioAnalyzer, ExeRunner):

    """
    Implementation of APx585 audio analyzer
    """

    # Executable timeout
    TIMEOUT = 1200
    PIPE_ACCESS_DUPLEX = 0x3
    PIPE_TYPE_MESSAGE = 0x4
    PIPE_READMODE_MESSAGE = 0x2
    PIPE_WAIT = 0
    PIPE_NOWAIT = 1
    PIPE_UNLIMITED_INSTANCES = 255
    BUFFERSIZE = 512
    NMPWAIT_USE_DEFAULT_WAIT = 5000
    INVALID_HANDLE_VALUE = -1
    ERROR_PIPE_CONNECTED = 535

    TIMEOUT_PIPE = 60
    PIPENAME = "\\\\.\\pipe\\APxSync"

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        IAudioAnalyzer.__init__(self)
        ExeRunner.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        self._device = None
        self.__handle = None
        self.__pipeHandle = None
        self.__charBuffer = None
        self.__bNbrRead = None

    def __manage_apx585(self,
                        execution_type,
                        board_type=None,
                        test_type=None,
                        call_type=None,
                        accessories_type=None,
                        signal_tested_direction=None,
                        dut_bt_address=None):
        """
        launch a command to APx585.

        :type execution_type: str
        :param execution_type: Init, pair_bt, run or calibration

        :type board_type: str
        :param board_type: board_type from device_catalog

        :type test_type: str
        :param test_type: test executed on device

        :type call_type: str
        :param call_type: 2G, 3G, VOIP

        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp
        :type signal_tested_direction: str
        :param signal_tested_direction: UL, DL

        :type dut_bt_address: str
        :param dut_bt_address: dut mac bluetooth address

        :rtype: str
        :return: executable return code

        """
        waitendprocess = True

        if execution_type == "init":
            if board_type is not None and test_type is not None:
                cmd_line = "%s %s %s" % (execution_type, board_type, test_type)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER)

        elif execution_type == "run":
            if accessories_type is not None and call_type is not None \
                    and signal_tested_direction is not None:
                cmd_line = "%s %s %s %s" % (execution_type, call_type,
                                            accessories_type, signal_tested_direction)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER)

        elif execution_type == "pairbt":
            if dut_bt_address is not None:
                cmd_line = "%s %s" % (execution_type, dut_bt_address)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER)

        elif execution_type == "connectbt":
            if dut_bt_address is not None:
                cmd_line = "%s %s %s" % (execution_type,
                                         accessories_type,
                                         dut_bt_address)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER)

        elif execution_type == "calibration":
            if dut_bt_address is not None:
                cmd_line = "%s %s %s %s %s %s" % (execution_type,
                                                  board_type,
                                                  call_type,
                                                  accessories_type,
                                                  signal_tested_direction,
                                                  dut_bt_address)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER)
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER)

        apx585_result = ExeRunner.start_exe(self, cmd_line, self.TIMEOUT, waitendprocess)

        if not waitendprocess:
            exec_return_code = apx585_result[2].wait()
        else:
            exec_return_code = apx585_result[1]

        # if exception occured during execution => kill AudioPrecision application
        if exec_return_code == ExeRunner.NOT_TERMINATED:
            self._logger.info("Apx executable timeout")
            subprocess.Popen("taskkill /F /IM AudioPrecision.APx500.exe /T", shell=True)
        return exec_return_code

    def initialization(self,
                       board_type,
                       test_type):
        """
        Init Apx585.

        :type board_type: str
        :param board_type: board_type from device_catalog

        :type test_type: str
        :param test_type: test executed on device

        :rtype: str
        :return: executable return code
        """
        exec_return_code = self.__manage_apx585("init", board_type, test_type)

        return exec_return_code

    def pair_bt(self,
                dut_bt_address):
        """
        Pair Bluetooth between Apx585 and Dut

        :param dut_bt_address: dut mac bluetooth address
        :rtype: str

        :rtype: str
        :return: executable return code

        """
        exec_return_code = self.__manage_apx585("pairbt",
                                                None,
                                                None,
                                                None,
                                                None,
                                                None,
                                                dut_bt_address)

        return exec_return_code

    def connect_bt(self,
                   accessories_type,
                   dut_bt_address):
        """
        Connect Bluetooth between Apx585 and Dut
        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp
        :type accessories_type: str
        :param dut_bt_address: dut mac bluetooth address

        :rtype: str
        :return: executable return code
        """
        exec_return_code = self.__manage_apx585("connectbt",
                                                None,
                                                None,
                                                None,
                                                accessories_type,
                                                None,
                                                dut_bt_address)

        return exec_return_code

    def run(self,
            call_type,
            accessories_type,
            signal_tested_direction):
        """
        Run Apx585 test
        :type call_type: str
        :param call_type: 2G, 3G, VOIP
        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp
        :type signal_tested_direction: str
        :param signal_tested_direction: UL, DL

        :rtype: str
        :return: executable return code

        """
        exec_return_code = self.__manage_apx585("run",
                                                None,
                                                None,
                                                call_type,
                                                accessories_type,
                                                signal_tested_direction)

        return exec_return_code

    def calibration(self,
                    board_type,
                    call_type,
                    acc_type,
                    signal_tested_direction,
                    dut_bt_address):
        """
        Run Apx585 calibration
        :type board_type: str
        :param board_type: board_type from device_catalog
        :type call_type: str
        :param call_type: 2G, 3G, VOIP
        :type acc_type: str
        :param acc_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp
        :type signal_tested_direction: str
        :param signal_tested_direction: UL, DL
        :type dut_bt_address: str
        :param dut_bt_address: dut mac bluetooth address

        :rtype: str
        :return: executable return code

        """
        exec_return_code = self.__manage_apx585("calibration",
                                                board_type,
                                                None,
                                                call_type,
                                                acc_type,
                                                signal_tested_direction,
                                                dut_bt_address)

        return exec_return_code

    def get_volume(self,
                   call_type=None,
                   accessories_type=None):
        """
        launch a command to APx585
        :type call_type: str
        :param call_type: 2G, 3G, VOIP
        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp

        :rtype: (str, str)
        :return: tuple of (executable return code, volume)
        """

        waitendprocess = False

        # Pipe handle creation
        self.create_pipe()

        cmd_line = "%s %s %s" % ("getvolume",
                                 call_type,
                                 accessories_type)

        apx585_result = ExeRunner.start_exe(self, cmd_line, self.TIMEOUT, waitendprocess)

        # Waiting message from APx
        l_volume = self.read_pipe()

        if not waitendprocess:
            exec_return_code = apx585_result[2].wait()
        else:
            exec_return_code = apx585_result[1]

        # if exception occured during execution => kill AudioPrecision application
        if exec_return_code == 3:
            self._logger.info("Apx executable timeout")
            subprocess.Popen("taskkill /F /IM AudioPrecision.APx500.exe /T", shell=True)

        self.close_pipe()

        return exec_return_code, l_volume

    def ringtone_detection(self,
                           accessories_type=None):
        """
        launch a command to APx585 to detect ringtone
        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp

        :rtype: str
        :return: executable return code
        """

        waitendprocess = True

        cmd_line = "%s %s " % ("ringtonedetection", accessories_type)

        apx585_result = ExeRunner.start_exe(self, cmd_line, self.TIMEOUT, waitendprocess)

        exec_return_code = apx585_result[1]

        # if exception occured during execution => kill AudioPrecision application
        if exec_return_code == 3:
            self._logger.info("Apx executable timeout")
            subprocess.Popen("taskkill /F /IM AudioPrecision.APx500.exe /T", shell=True)

        return exec_return_code

    def write_pipe(self, message="none"):
        """
        Write a message in a pipe
        :type message: str
        :param message: message write in the pipe

        :rtype: None
        :raise UECommandException: if message superior at the buffer size
                                   or message is not write on the pipe
        """

        if len(message) < self.BUFFERSIZE:
            bNbrWritten = c_ulong(0)
            fSuccess = windll.kernel32.WriteFile(self.__pipeHandle,
                                                 c_char_p(message),
                                                 len(message),
                                                 byref(bNbrWritten),
                                                 None)
            windll.kernel32.FlushFileBuffers(self.__pipeHandle)
            if fSuccess != 1 or bNbrWritten.value == 0:
                raise TestEquipmentException(
                    TestEquipmentException.OPERATION_FAILED,
                    "Message %s is not write in the pipe"
                    % str(message))
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Size of message %s is superior at the buffer size"
                % str(message))

    def read_pipe(self):
        """
        Read a message in a pipe

        :rtype: str
        :return: the message read on the pipe
        :raise UECommandException: if timeout is reached
        """

        self.__charBuffer = create_string_buffer(self.BUFFERSIZE)
        self.__bNbrRead = c_ulong(0)

        def target():
            fSuccess = 0
            while (fSuccess != 1) or (self.__bNbrRead.value == 0):
                fSuccess = windll.kernel32.ReadFile(self.__pipeHandle,
                                                    self.__charBuffer,
                                                    self.BUFFERSIZE,
                                                    byref(self.__bNbrRead),
                                                    None)
                time.sleep(0.2)

        # Execution threader => if executable don't respond before timeout, it is killed
        thread = Thread(target=target)
        thread.start()
        thread.join(self.TIMEOUT_PIPE)
        if thread.is_alive():
            windll.kernel32.FlushFileBuffers(self.__pipeHandle)
            self.close_pipe()
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, "Read message failed")

        return self.__charBuffer.value

    def create_pipe(self):
        """
        Create and connect pipe

        :rtype: None
        :raise UECommandException: if message superior at the buffer size
        """
        # Pipe creation
        pipeHandle = windll.kernel32.CreateNamedPipeA(self.PIPENAME,
                                                      self.PIPE_ACCESS_DUPLEX,
                                                      self.PIPE_TYPE_MESSAGE | self.PIPE_READMODE_MESSAGE | self.PIPE_WAIT,
                                                      self.PIPE_UNLIMITED_INSTANCES,
                                                      self.BUFFERSIZE,
                                                      self.BUFFERSIZE,
                                                      self.NMPWAIT_USE_DEFAULT_WAIT,
                                                      None)
        self.__pipeHandle = pipeHandle

        def connect_pipe():
            """
            connect pipe
            """
            if self.__pipeHandle == self.INVALID_HANDLE_VALUE:
                raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, "Invalid pipe handle")
            # Connecting pipe
            self._logger.info("Connect Pipe")
            pConnected = windll.kernel32.ConnectNamedPipe(self.__pipeHandle, None)
            if ((pConnected == 0 and windll.kernel32.GetLastError() == self.ERROR_PIPE_CONNECTED) or
                  pConnected == 1):
                # self.__pipeHandle = pipeHandle
                self._logger.info("Pipe Connected => " + str(self.__pipeHandle))
            # If the connection failed then the handle is closed
            else:
                self.__pipeHandle = pipeHandle
                windll.kernel32.CloseHandle(self.__pipeHandle)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Pipe creation failed")

            self._logger.info("Dans Create pipe => " + str(self.__pipeHandle))

        # Execution threader => if executable don't respond before timeout, it is killed
        thread = Thread(target=connect_pipe)
        thread.start()

        return self.__pipeHandle

    def close_pipe(self):
        """
        Create and connect pipe

        :rtype: None
        :raise UECommandException: Pipe handle is None
        """
        # Pipe Handle Closing
        if self.__pipeHandle is not None:
            windll.kernel32.FlushFileBuffers(self.__pipeHandle)
            windll.kernel32.DisconnectNamedPipe(self.__pipeHandle)
            windll.kernel32.CloseHandle(self.__pipeHandle)
        else:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Pipe handle none close error")

    def get_dtmf(self,
                 accessories_type=None,
                 signal_tested_direction=None,
                 key_touch=None):
        """
        launch a command to APx585
        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp
        :type signal_tested_direction: str
        :param signal_tested_direction: UL, DL

        :rtype: str
        :return: executable return code

        """

        waitendprocess = False
        exec_return_code = 3
        l_touch_press2 = -1

        if signal_tested_direction == "UL":
            self._device = DeviceManager().get_device("PHONE1")
        else:
            self._device = DeviceManager().get_device("PHONE2")

        # Pipe handle creation
        l_create_pipe = self.create_pipe()
        time.sleep(0.5)
        self._logger.info("Create pipe => " + str(l_create_pipe))

        cmd_line = "%s %s %s" % ("getdtmf", accessories_type, signal_tested_direction)

        apx585_result = ExeRunner.start_exe(self, cmd_line, self.TIMEOUT_PIPE, waitendprocess)
        self._logger.info("Create pipe => " + str(self.__pipeHandle))

        # Waiting message from APx
        self._logger.info("Start read pipe")
        l_ready = self.read_pipe()
        self._logger.info("End read pipe => " + str(l_ready))
        time.sleep(0.5)

        if l_ready == "READY":
            self.write_pipe("GO")
            time.sleep(1)
            # Key press
            self._logger.info("DTMF press touch " + key_touch)
            self._device.run_cmd("adb shell input text " + key_touch, 1)
            self._logger.info("End press touch")
            self._logger.info("Read pipe")
            l_touch_press = self.read_pipe()
            self._logger.info("End read pipe (touch press) => " + str(l_touch_press))
            exec_return_code = apx585_result[2].wait()
            self._logger.info("End wait pipe (exec_return_code) => " + str(exec_return_code))
            if l_touch_press != "READY":
                l_touch_press2 = int(l_touch_press) - 48
        else:
            apx585_result[2].terminate()

        self._logger.info("Start close pipe")
        self.close_pipe()
        self._logger.info("End close pipe")

        # if exception occured during execution => kill AudioPrecision application
        if exec_return_code == 3:
            self._logger.info("Apx executable timeout")
            subprocess.Popen("taskkill /F /IM AudioPrecision.APx500.exe /T", shell=True)

        return exec_return_code, str(l_touch_press2)

    def glitch_detection_before_switch(self,
                                       test_type,
                                       board_type,
                                       call_type,
                                       accessories_type,
                                       signal_tested_direction):
        """
        Run Apx585 glitch detection in call.

        :type test_type: str
        :param test_type: glitch detection type glitchdetectiononswitch or glitchdetectiononswap

        :type board_type: str
        :param board_type: board_type from device_catalog

        :type call_type: str
        :param call_type: 2G, 3G, VOIP

        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset, bluetootha2dp, bluetoothhsp

        :type signal_tested_direction: str
        :param signal_tested_direction: UL, DL

        :rtype: tuple
        :return: A tuple containing

        ::
            - stdout as C{in}: 0 if ready -1 else
            - return code as C{Popen}: the executable process
        """

        waitendprocess = False

        if signal_tested_direction == "UL":
            self._device = DeviceManager().get_device("PHONE1")
        else:
            self._device = DeviceManager().get_device("PHONE2")

        # Pipe handle creation
        l_create_pipe = self.create_pipe()
        time.sleep(0.5)
        self._logger.info("Create pipe => " + str(l_create_pipe))

        cmd_line = "%s %s %s %s %s" % (test_type, board_type, call_type,
                                       accessories_type, signal_tested_direction)

        apx585_result = ExeRunner.start_exe(self, cmd_line, self.TIMEOUT_PIPE, waitendprocess)
        self._logger.info("Create pipe => " + str(self.__pipeHandle))

        # Waiting message from APx
        self._logger.info("Start read pipe")
        l_ready = self.read_pipe()
        self._logger.info("End read pipe => " + str(l_ready))
        time.sleep(0.5)

        if l_ready == "READY":
            self.write_pipe("GO")
            return 0, apx585_result[2]
        else:
            apx585_result[2].terminate()
            return -1, None

    def glitch_detection_after_switch(self, process):
        """
        Run Apx585 glitch detection in call
        :type process: Popen
        :param process: Apx585 executable processus

        :rtype: str
        :return: executable return code

        """
        exec_return_code = process.wait()

        self._logger.info("Start close pipe")
        self.close_pipe()
        self._logger.info("End close pipe")

        # if exception occured during execution => kill AudioPrecision application
        if exec_return_code == 3:
            self._logger.info("Apx executable timeout")
            subprocess.Popen("taskkill /F /IM AudioPrecision.APx500.exe /T", shell=True)

        return exec_return_code
