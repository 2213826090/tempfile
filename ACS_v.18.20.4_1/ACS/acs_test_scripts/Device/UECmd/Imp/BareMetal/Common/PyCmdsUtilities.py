"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW
:summary: UECmd based file for all Linux Application Framework Base
:since: 13/03/2014
:author: jreynaux
"""
from os import path, kill
import subprocess
import signal
import imp
import time
from Core.PathManager import Paths

LINK_OK = 51966


class PyCmdsUtilities(object):
    """
    Utility class that handle operation with python module used to interact with diana-class devices.
    """
    __instance = None

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(PyCmdsUtilities, cls).__new__(cls, *args, **kwargs)
            cls.__instance.__initialized = False
        return cls.__instance

    def __init__(self, logger=None, serial_port="/dev/ttyUSB0"):
        if self.__initialized: return

        self.__initialized = False
        self._cmds = None
        self._logger = logger
        self._serial_port = serial_port
        self.__initialized = True

    def init_pycmds(self, silent_mode=True, serial_port="/dev/ttyUSB0"):
        """
        Import and Load module

        :type silent_mode: bool
        :param silent_mode: Whether display log or not

        :rtype: bool
        :return: Whether import and load successful
        """
        py_cmds_path = path.join(Paths.EXTRA_LIB_FOLDER, "pyCmds.py")
        cmd_tbl_h_path = Paths.EXTRA_LIB_FOLDER

        # Kill any others instances runnning
        self.kill_pycmds_instances()

        if not silent_mode:
            self._logger.debug("Importing pyCmds module ...")

        try:
            # Import the pyCmds module from its path
            py_cmds = imp.load_source('pyCmds', py_cmds_path)
        except Exception:
            self._logger.error("Unable to load pyCmds module from pyCmd.py python file,"
                               " should be located on {0} folder.".format(Paths.EXTRA_LIB_FOLDER))
            return False

        if not silent_mode:
            self._logger.debug("Instanciate module ...")

        # TODO: Try to catch exception raised py pyCmds if no com port for ex (Exception in thread device_rx)
        self._cmds = py_cmds.pyCmds(argv=[path.realpath(__file__), "-c", serial_port, cmd_tbl_h_path])

        # Left time to connect to UART before test link
        wait_time = 5
        if not silent_mode:
            self._logger.debug("Wait {0}s before test link ...".format(wait_time))
        time.sleep(wait_time)

        # Because unable to catch error yet, may test link to get verdict of init
        if not self.test_link_to_dut(silent_mode=silent_mode):
            self.unload_pycmds()
            return False
        else:
            return True

    def is_pycmds_loaded(self):
        return True if self._cmds is not None else False

    def unload_pycmds(self):
        """
        Force the unload of module to free resource.

        :return:
        """
        self._logger.debug("Unload pyCmds module")
        if self._cmds is not None:
            self._logger.debug("Exiting from current instance")
            self._cmds.do_exit(0)
            self._cmds = None

        self.kill_pycmds_instances()

    def kill_pycmds_instances(self):
        """
        Kill remaining any others pyCmds instances.

        :return:
        """
        self._logger.debug("Attempting to kill others pyCmds running instances")
        p = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        out, err = p.communicate()
        for line in out.splitlines():
            if 'pyCmds' in line:
                self._logger.debug("Running instance found: {0}".format(line))
                lspl = line.split()
                pid = int(lspl[1])
                kill(pid, signal.SIGKILL)

    def test_link_to_dut(self, silent_mode=True):
        """
        Execute a test command

        :type silent_mode: bool
        :param silent_mode: Whether display log or not

        :rtype: tuple
        :return: The output status and result tuple (size may vary)
        """
        status = False
        try:
            output = self._cmds.test_link()
        except Exception as e:
            self._logger.warning("Unable to establish link to DUT !({0})".format(e))
            return False

        if output == LINK_OK:
            if not silent_mode:
                self._logger.debug("Link established, excellent work.")
            status = True
        else:
            if not silent_mode:
                self._logger.debug("Unable to establish link to DUT !")
            self.unload_pycmds()

        return status

    def do_py_cmd_command(self, cmd, silent_mode=True):
        """
        Execute a command

        :type cmd: str
        :param cmd:

        :type silent_mode: bool
        :param silent_mode: Whether display log or not

        :rtype: tuple
        :return: The output status and result tuple (size may vary)
        """
        status = False
        rv = ()

        if not self.is_pycmds_loaded():
            # Load pyCmds module
            if not silent_mode:
                message = "Python module was not yet loaded, loading it ..."
                self._logger.warning(message)

            # If not possible to load for any reason, return False and empty tuple
            if not self.init_pycmds(silent_mode=False, serial_port=self._serial_port):
                message = "Cannot run {0}, python module not loaded !".format(cmd)
                self._logger.warning(message)
                return status, rv

        if not silent_mode:
            self._logger.debug("Extract command and parameters ...")

        # Extract command and arguments
        full_cmd = cmd.rstrip().split(' ')
        cmd = full_cmd[0]
        args = full_cmd[1:]

        # convert all elements to integer if necessary
        for i, val in enumerate(args):
            if val.isdigit():
                args[i] = int(val)

        if not silent_mode:
            self._logger.debug("Run command {0} ...".format(cmd))

        try:
            # Call command
            method_to_call = getattr(self._cmds, cmd)
            rv = method_to_call(*args)

            status = True
            self._logger.debug("Output ({0}): {1}".format(type(rv), rv))

            # TODO: To use with care because may be reworked
            # if not self.check_data_consistency(rv, silent_mode=True):
            #     status = False
            #     rv = ()

        except Exception as e:
            self._logger.error("Unable to run command [{0}] !({1})".format(cmd, e.message))

        # finally:
        #     self.unload_pycmds()

        return status, rv

    def do_py_cmd_command_with_retry(self, cmd, max_retry=3, silent_mode=True):
        """
        Execute a command with given retry in case of failure

        :type cmd: str
        :param cmd:

        :type silent_mode: bool
        :param silent_mode: Whether display log or not

        :rtype: tuple
        :return: The output status and result tuple (size may vary)
        """
        status = False
        output = ()
        run = 1

        while run <= max_retry and not status:
            if not silent_mode:
                self._logger.debug("Run command {0} try {1}/{2}...".format(cmd, run, max_retry))

            status, output = self.do_py_cmd_command(cmd=cmd, silent_mode=silent_mode)
            if run < max_retry and not status:
                time.sleep(5)
            run += 1

        if not status:
            self._logger.error("Unable to run command {0} after {1} tries !".format(cmd, max_retry))
        elif not silent_mode:
                self._logger.debug("Command {0} succeed after {1} tries ...".format(cmd, run))

        return status, output

    def check_data_consistency(self, data, silent_mode=True):
        """
        Due to lack of consistency of data on tuple.

        :param data:
        :return:
        """
        status = True
        if not silent_mode:
            self._logger.debug("Checking data consistency...")

        if type(data) is not tuple:
            data = (data,)

        # Data is tuple continuing
        for index, element in enumerate(data):
            if not silent_mode:
                self._logger.debug("Checking element {0} ...".format(index))

            try:
                if type(element) is int:
                    continue

                if not all(ord(str(e)) >= 32 for e in element):
                    self._logger.error("An error occurred during check of element {0} in output data".format(index))
                    status = False
                else:
                    # If previous was ok, still ok, otherwise not ok
                    if not silent_mode:
                        self._logger.debug("OK")
                    status = True if status else False
            except Exception as ude:
                self._logger.error("An error occurred during check of element {0} in output data ({1})"
                                   .format(index, ude))
                status = False

        return status