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
:summary: implementation of MP manager software
:since:04/06/2013
:author: lpastor
"""

import time
import telnetlib
import os
import threading

from acs_test_scripts.Equipment.IEquipment import ExeRunner
from acs_test_scripts.Equipment.NFCTools.Interface.INFCTools import INFCTools
from ErrorHandling.TestEquipmentException import TestEquipmentException


class MPManager(ExeRunner, INFCTools):

    """
    Implementation of MP manager
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor

        :type name: str
        :param name: the bench name of the equipment

        :type model: str
        :param model: the model of the equipment

        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment catalog parameters
        """
        INFCTools.__init__(self)
        ExeRunner.__init__(self, name, model, eqt_params)

        self._handle = None

        # Retrieve parameters from BenchConfig for connection
        self._host = str(bench_params.get_param_value("HostIP"))
        self._port = str(bench_params.get_param_value("PortNumber"))
        self._mp300_device = str(bench_params.get_param_value("DeviceToUse"))
        self._connection_type = str(bench_params.get_param_value("ConnectionType"))
        self._script_path = str(bench_params.get_param_value("ScriptFolder"))
        self._log_path = str(bench_params.get_param_value("LogFolder"))

        self._mp_manager_thread = None

    def _launch_in_server_mode(self):
        """
        Launch MP manager in server mode
        """
        cmd_line = "/s:%s:%s" % (self._port, self._host)
        ExeRunner.start_exe(self, cmd_line)

    def run_script(self, script_name):
        """
        Run script using MP300 TCL2 from micropross

        :type script_name: str
        :param script_name: name of the script to run
        """
        if self._handle is None:
            msg = "No telnet session detected"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)

        # connect MP300
        self._connect_mp300()
        # run script
        self._execute_script(script_name)
        # check result
        self._check_script_result()

    def _connect_mp300(self):
        """
        Connect MP300 to MPManager
        """
        cmd = "CONNECT /a:" + self._connection_type + ":" + self._mp300_device + "\n"
        self._write(cmd)
        self._read_until("CONNECT OK", "Connection command", 5)
        self._read_until("CONNECT DONE", "Connection establishment", 5)

    def _execute_script(self, script_name):
        """
        Execute script on MP300 TCL2
        """
        script_full_path = os.path.join(self._script_path, script_name)
        if not os.path.exists(script_full_path):
            msg = "Script not found!"
            raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR, msg)
        cmd = "SCRIPT /p:" + script_full_path + " /r:" + self._log_path + "/n:1" + "\n"
        self._write(cmd)
        self._read_until("SCRIPT OK", "Script execution command", 5)

    def _check_script_result(self,):
        """
        Check script execution result
        """
        out = self._handle.read_until("SCRIPT DONE", 15)
        if "SCRIPT FAIL" in out:
            msg = "SCRIPT fails : " + out
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, msg)

    def _set_handle(self, handle):
        """
        Sets the connection handle of the equipment
        """
        self._handle = handle

    def _connect_via_telnet(self):
        """
        connect MP manager via telnet
        """
        self.get_logger().debug("Open telnet connection to equipment.")

        # Initialize telnet session
        telnet_session = telnetlib.Telnet()

        try:
            telnet_session = telnetlib.Telnet(self._host, self._port)
            # debug level: 0->disable / 1->enable
            telnet_session.set_debuglevel(0)
            out = telnet_session.read_until("READY", 5)
            if "READY" not in out:
                msg = "Telnet connection is out of order. "
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            msg = "connect to equipment via telnet failed"
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)

        # Update handle value
        self._set_handle(telnet_session)

    def _disconnect_via_telnet(self):
        """
        disconnect MP manager via telnet
        """
        self.get_logger().debug("Close telnet connection from the equipment.")
        if self._handle is not None:
            self._handle.close()
            self._set_handle(None)

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self.get_logger().info("Initialization")

        # check parameters
        if self._host == "":
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Host IP address is not specified")

        if not self._port.isdigit():
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Invalid port number parameter")

        if self._mp300_device == "":
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Device to connect is not specified")

        if self._connection_type.upper() != "USB":
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Invalid connection type parameter")

        script_path = os.path.join(self._script_path)
        if not os.path.exists(script_path):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Unknown script path")

        log_path = os.path.join(self._log_path)
        if not os.path.exists(log_path):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Unknown log path")

        if self._handle is not None:
            return

        self._mp_manager_thread = threading.Thread(target=self._launch_in_server_mode, args=())
        self._mp_manager_thread.start()
        time.sleep(15)

        # Open telnet session
        connection_attempts = 5
        while connection_attempts > 0:
            try:
                self._connect_via_telnet()
                break
            except TestEquipmentException as e:
                self._logger.info(e)
                connection_attempts -= 1
                if connection_attempts <= 0:
                    raise e
                self.release()

    def release(self):
        """
        Release the equipment and all associated resources and kill MPManager child process
        """
        self.get_logger().info("Release")
        self._disconnect_via_telnet()
        os.system("taskkill /im MPManager.exe")

    def _write(self, cmd):
        """
        Sends command on telnet connection

        :type cmd: str
        :param cmd: command line to execute
        """
        self._handle.write(cmd)
        time.sleep(0.3)

    def _read_until(self, until, label, timeout=5):
        """
        Reads telnet ouput until meet the str "until".
        The function stops after timeout expiration and raise an exception if
        the str has not been read.

        :type until: str
        :param until: reads until this str
        :type label: str
        :param label: error message includes that str
        :type timeout: int
        :param timeout: time to wait for the output to contain "until" str
        """
        out = self._handle.read_until("%s" % (str(until)), timeout)
        if until not in out:
            msg = "%s error (%s)" % (label, out)
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)
