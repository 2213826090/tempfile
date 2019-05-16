"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the driver for the Saleae Logic16 logic analyzer.
:since: 2014-11-14
:author: emarchan

"""
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.LogicAnalyzer.Interface.ILogicAnalyzer import ILogicAnalyzer
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time
import subprocess
import os
import shlex
from acs_test_scripts.Equipment.Computer.Common.Common import GenericComputer
from platform import system
import sys
from threading  import Thread
from Queue import Queue, Empty
from ErrorHandling.DeviceException import DeviceException

class SaleaeLogic16(EquipmentBase, ILogicAnalyzer):

    """
    Command list for to send to the logic analyzer
    Format is
    "<Alias>":["command without \n", [<list of possible answers to the command>], <timeout>],
    """
    AVAILABLE_COMMANDS = {"ENTER":["", ['host port', 'Enter'], 3],
                          "SET_SAMPLE_RATE":["set_sample_rate", ['Enter'], 3],
                          "SET_SAMPLE_NR":["set_num_samples", ['Enter'], 3],
                          "GET_CONNECTED_DEV":["get_connected_devices", ['Enter'], 3],
                          "GET_ACTIVE_CHANNELS":["get_active_logic16_channels", ['Enter'], 3],
                          "SET_ACTIVE_CHANNELS":["set_active_logic16_channels", ['Enter'], 3],
                          "START_CAPTURE_TO_MEM":["capture", ['Wrote data'], 2],
                          "START_CAPTURE_TO_FILE":["capture_to_file", ['Wrote data'], 2],
                          "SAVE_RAW_CAPTURE_TO_FILE":["save_to_file", ['Enter'], 60],
                          "LOAD_RAW_CAPTURE_TO_FILE":["load_from_file", ['Enter'], 60],
                          "GET_AVAILABLE_FORMATTERS":["get_analyzers", ['Enter'], 10],
                          "SAVE_FORMATTED_CAPTURE_TO_FILE":["export_analyzer", ['Enter'], 60],
                          "SAVE_RAW_CSV_CAPTURE_TO_FILE":["export_data", ['Enter'], 60],
                          "SET_TRIGGER":["set_trigger", ['Enter'], 3],
                          "GET_PRETRIGGER_BUF_SIZE":["get_capture_pretrigger_buffer_size", ['Enter'], 3],
                          "SET_PRETRIGGER_BUF_SIZE":["set_capture_pretrigger_buffer_size", ['Enter'], 3]}

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        ILogicAnalyzer.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self._socket_client = None
        self._socket_server = None
        self._clientSW = None
        self._serverSW = None
        self._bench_params = bench_params
        self._out_q = None
        self.READ_TIMEOUT = 5
        self._stdout_thread = None
        self._avail_formatters = {}
        self._cur_os = None

    def __del__(self):
        """
        Destructor
        """
        self.release()

    def release(self):
        """
        Kills the server and client.
        """
        self._socket_client.terminate()
        self._socket_server.terminate()
        if self._stdout_thread is not None:
            self._stdout_thread.stop()

    def init(self):
        self._logger.info("Initialization")
        self._clientSW = str(self._bench_params.get_param_value("ClientSWLocation"))
        self._serverSW = str(self._bench_params.get_param_value("ServerSWLocation"))
        self._cur_os = system()

    def connect(self):
        """
        Launches the server and client and establish a connection between them.
        """
        self._socket_server = self._launch_server()
        self._socket_client = self._launch_client()
        time.sleep(10)
        self._send_command("ENTER")
        self._send_command("ENTER")
        # Get the list of formatters if needed later ons
        self._avail_formatters = self.get_capture_formatters()
        self._logger.info("Available formatter(s) are: %s" % self._avail_formatters.keys())


    def start_capture_to_mem(self):
        """
        Starts a capture (destination is memory).
        """
        self._send_command("START_CAPTURE_TO_MEM")

    def start_capture_to_file(self, dest_file):
        """
        Starts a capture (destination is file).

        :type dest_file: string
        :param dest_file: Destination file where the capture will be written.
        """
        self._send_command("START_CAPTURE_TO_FILE", dest_file)

    def stop_capture(self):
        """
        Stops the current capture.
        Since there's no API to do this currently, we've got to simulate a key press 'ENTER' into the UI
        """
        if self._cur_os == GenericComputer.LINUX:
            cmd = 'xdotool search "Saleae Logic" windowactivate'
            subprocess.call(shlex.split(cmd))
            time.sleep(2)
            cmd = 'xdotool key Return'
            subprocess.call(shlex.split(cmd))
        else:
            self._logger.error('stop_capture is not available for your platform')
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def export_raw_capture_to_csv_file(self, dest_file):
        """
        Exports the previously captured file in a RAW, unformatted CSV file.

        :type dest_file: string
        :param dest_file: CSV Destination file where the capture will be written.
        """
        self._send_command("SAVE_RAW_CSV_CAPTURE_TO_FILE", dest_file + ", all_channels, all_time, csv, headers, comma, time_stamp, separate, row_per_sample")

    def export_formatted_capture_to_file(self, dest_file, formatter_name):
        """
        Exports the previously captured file in a formatted format.

        :type dest_file: string
        :param dest_file: Destination file where the formatted capture will be written.

        :type formatter_name: string
        :param formatter_name: Name of the formatter that will parse the capture.
        """
        if formatter_name not in self._avail_formatters:
            msg = "Formatter %s can't be found in the logic analyzer's available ones." % formatter_name
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

        self._send_command("SAVE_FORMATTED_CAPTURE_TO_FILE", "%s, %s" % (self._avail_formatters[formatter_name], dest_file))

    def set_capture_rate(self, rate):
        """
        Sets the capture rate.

        :param rate: Capture rate
        :type rate: string
        """
        self._send_command("SET_SAMPLE_RATE", rate)

    def set_number_of_samples(self, number):
        """
        Sets the number of samples to capture.

        :param number: Number of samples to capture.
        :type number: string
        """
        self._send_command("SET_SAMPLE_NR", number)

    def set_capture_channels(self, capture_channels):
        """
        Sets the channels you want to capture the data from.

        :param capture_channels: List of channels to capture data from.
        :type capture_channels: List of string
        """
        str_capture_channels = ""
        for channel in capture_channels:
            str_capture_channels = str_capture_channels + channel + ', '
        # Remove last ,
        str_capture_channels = str_capture_channels[:-1]
        self._send_command("SET_ACTIVE_CHANNELS", str_capture_channels)

    def get_capture_formatters(self):
        """
        Gets the available capture formatters

        :return: The available formatters
        :rtype: Dict Format {"name": "ID"}
        """
        result = {}
        cmd_res = self._send_command("GET_AVAILABLE_FORMATTERS")
        for cur_line in cmd_res:
            cur_line = cur_line.replace(', ', ',')
            formatters = cur_line.split(',')
            if len(formatters) == 2:
                result[formatters[0]] = formatters[1]
        return result

    def _send_command(self, command, value=None):
        """
        Send a command from the client to the server.

        :param command: command to send
        :type command: string
        :param command: value to send after the command
        :type value: string

        :rtype: list of strings
        :return: The client answer
        """
        result = []
        if command not in self.AVAILABLE_COMMANDS:
            msg = "Invalid command %s" % command
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if self._socket_client is None:
            msg = "Invalid socket, client not connected!"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if value not in [None, ""]:
            cmd_line = "%s, %s\n" % (self.AVAILABLE_COMMANDS[command][0], value)
        else:
            cmd_line = self.AVAILABLE_COMMANDS[command][0] + '\n'

        self._logger.debug("Sending command %s to server." % (cmd_line[:-1]))
        self._socket_client.stdin.write(cmd_line)

        # read output without blocking
        line = ""
        possible_answers = self.AVAILABLE_COMMANDS[command][1]
        timeout = self.AVAILABLE_COMMANDS[command][2]

        while timeout >= 0 and not any([x in line for x in possible_answers]):
            try:
                line = self._out_q.get_nowait()  # or q.get(timeout=.1)
            except Empty:
                timeout = timeout - 1
                time.sleep(1)
            else:  # got line
                timeout = self.AVAILABLE_COMMANDS[command][2]
                if 'Enter a string to xmit, q to exit.' not in line:
                    if line.endswith('\n'):
                        line = line[:-1]
                    self._logger.debug("ANSWER is: %s" % line)
                    result.append(line)
        return result

    def _enqueue_output(self, out, queue):
        """
        Enqueues stdout for non blocking reading.
        """
        for line in iter(out.readline, b''):
            queue.put(line)
        out.close()

    def _launch_server(self):
        """
        Launches the server.

        :rtype: Popen
        :return The process ID for later communication.
        """
        self._logger.info("Launching saleae server (%s)" % self._serverSW)

        if not os.path.exists(self._serverSW):
            msg = "Can't find Saleae server %s" % self._serverSW
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR, msg)

        # Kill previous server instances
        self._logger.info("Killing previous Saleae server instances")
        if self._cur_os == GenericComputer.LINUX:
            subprocess.call(["/usr/bin/killall", self._serverSW])
        else:
            cmd = "taskkill /F /IM %s" % os.path.basename(self._serverSW)
            subprocess.call(shlex.split(cmd))

        # Remove error files to avoid a popup
        server_path = os.path.dirname(self._serverSW)
        error_path = os.path.join(server_path, 'Errors')
        if os.path.exists(error_path):
            error_files = os.listdir(error_path)
            for cur_file in error_files:
                os.remove(os.path.join(error_path, cur_file))

        socket = subprocess.Popen(self._serverSW, stdout=subprocess.PIPE)
        return socket

    def _launch_client(self):
        """
        Launches the client.

        :rtype: Popen
        :return The process ID for later communication.
        """
        self._logger.info("Launching saleae client (%s)" % self._clientSW)

        if not os.path.exists(self._clientSW):
            msg = "Can't find Saleae server %s" % self._serverSW
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR, msg)


        ON_POSIX = 'posix' in sys.builtin_module_names
        socket = subprocess.Popen(self._clientSW, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1, close_fds=ON_POSIX)

        self._out_q = Queue()
        self._stdout_thread = Thread(target=self._enqueue_output, args=(socket.stdout, self._out_q))
        self._stdout_thread.daemon = True  # thread dies with the program
        self._stdout_thread.start()

        return socket
