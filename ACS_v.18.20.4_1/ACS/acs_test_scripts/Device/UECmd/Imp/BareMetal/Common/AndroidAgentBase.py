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

:organization: INTEL NDG SW DEV
:summary: UECmd based file for all Android Application Framework Base
:since: 21/03/2014
:author: jreynaux
"""
import re
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base as AndroidCommonBase
from UtilitiesFWK.Utilities import Global, run_local_command, internal_shell_exec
import UtilitiesFWK.Utilities as Util
from Device.DeviceLogger.LogCatLogger.LogCatLogger import LogCatLogger
from Core.Report.ACSLogging import LOGGER_WD
import os


class AndroidAgentBase(AndroidCommonBase):

    """
    Class that handle all generic operations on AcsAgent in "standalone mode"
    mean not linked to any DUT.
    e.i. Acs-Agent-like apk used as test equipment

    :note: NEED to be a singleton-type class in order to share logs data between steps
    """
    DEVICE_LOGGER = None

    def __init__(self, device):
        """
        Constructor
        """
        super(AndroidAgentBase, self).__init__(device)

        # Init LogCatLogger for local use
        self._device_logger = None

        # AndroidAgent specific
        self._app_package = "com.example.clarktestapp"
        self._acs_service_command = "adb shell am {0}service -n " + self._app_package + "/.AcsService"
        self.__app_name = "ClarkTestApp"

        # Reused logcat reader
        if AndroidAgentBase.DEVICE_LOGGER is None:
            # Initialize AndroidAgent
            self._init_logger()
            self._start_logging()
            self._start_acs_service()
        else:
            self._device_logger = AndroidAgentBase.DEVICE_LOGGER

    def _init_logger(self):
        """
         Initialize loggers
        """
        # Logcat init
        self.__log_file_name = None
        self._device_logger = LogCatLogger(self._device, True)
        AndroidAgentBase.DEVICE_LOGGER = self._device_logger

        # Watchod init
        self.__logger_wd = LOGGER_WD

    def get_android_agent_logger(self):
        return self._device_logger

    def get_phone_model(self):
        return self.__app_name

    @property
    def write_logcat_enabled(self):
        return True

    def _start_acs_service(self):

        # Kill apk before to avoid concurrent access
        cmd = "adb shell am force-stop " + self._app_package
        internal_shell_exec(cmd=cmd, timeout=self._device.get_uecmd_timeout(), silent_mode=True)

        self._logger.debug("Starting AcsService ...")
        return internal_shell_exec(cmd=self._acs_service_command.format("start"),
                                   timeout=self._device.get_uecmd_timeout(), silent_mode=True)

    def _stop_acs_service(self):
        self._logger.debug("Stopping AcsService ...")
        return internal_shell_exec(cmd=self._acs_service_command.format("stop"),
                                   timeout=self._device.get_uecmd_timeout(), silent_mode=True)

    def _start_logging(self):
        """
        Starts logging required by ACS for UECmd response
        :return: None
        """
        remote_app_serial_number = ""

        # Start logging thread required for UECmd response
        file_path = ""
        if self.write_logcat_enabled or self._device.get_config("writeAcsLogcat", "False", "str_to_bool"):
            timestamp = Util.get_timestamp()
            if remote_app_serial_number not in ["", None]:
                file_name = "%s_%s_%s_logcat.log" % (self.get_phone_model(),
                                                     re.sub(':', '_', remote_app_serial_number), timestamp,)
            else:
                file_name = "%s_%s_logcat.log" % (self.get_phone_model(), timestamp,)

            file_path = os.path.join(self._device.get_report_tree().
                                     get_subfolder_path(subfolder_name="LOGCAT",
                                     device_name=self.get_phone_model()),
                                     file_name)

        self.__log_file_name = file_path
        self._device_logger.set_output_file(self.__log_file_name)

        self._device_logger.start()

    def _stop_logging(self):
        """
        Stops logging required by ACS for UECmd response
        :return: None
        """
        # Stopping reader and analyser & writer threads
        self._device_logger.stop()

    def _exec(self, cmd, timeout=None, force_execution=False, wait_for_response=True):
        """
        Internal method that execute std command on device

        :type  cmd: str
        :param cmd: cmd to be executed

        :type  timeout: integer
        :param timeout: Script execution timeout in ms

        :type force_execution: Boolean
        :param force_execution: Force execution of command
                    without check phone connected (dangerous)

        :type wait_for_response: Boolean
        :param wait_for_response: Wait response from adb before
                                        statuing on command

        :return: output str
        :rtype: string
        """
        silent_mode = False
        if timeout is None:
            timeout = self._uecmd_default_timeout

        # (result, output) = self._device.run_cmd(cmd, timeout, force_execution, wait_for_response)
        if wait_for_response:
            return_code, output = internal_shell_exec(cmd=cmd, timeout=timeout, silent_mode=silent_mode)
        else:
            # Async operation is going to be started, we cannot provide a return_code
            # Return SUCCESS by default
            return_code = Global.SUCCESS
            output = ""
            run_local_command(args=cmd, get_stdout=not silent_mode)

        if return_code == Global.SUCCESS:
            if output is not None:
                return output
            else:
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "\"" + cmd + "\" returned null output !")
        else:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, output)

    def finalize(self):
        super(AndroidAgentBase, self).finalize()

