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

:organization: INTEL OTC QA Automation
:summary: This file implements the UC base class
:since: 29/06/2015
:author: rcstamat
"""

import subprocess
import os
import time
import shlex
import threading

import psutil

from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from Device.Model.DeviceBase import DeviceBase
from Device.DeviceController import DeviceController
from UtilitiesFWK.Utilities import Global
from Core.Report.ACSLogging import LOGGER_FWK


class AndroidQemuVM(IntelDeviceBase):
    """
        Definition for the QEMU instance with minimal Android inside a VM.
        This instance is dependent on being run in the same environment as the
        Android build tree and triggered as root since the startvm.sh script for
        QEMU start up requires root access rights.
    """

    def __init__(self, config, device_config):
        self.phone_ = """
        Constructor

        :type  phone_name: str
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """
        DeviceBase.__init__(self, config, device_config)
        self.pid = -1
        self.__logger = LOGGER_FWK
        self.android_build_top = os.environ.get('ANDROID_BUILD_TOP')
        # Just to avoid error message
        self._embedded_log = None
        self.startvm_path = None
        self._android_version = (self.get_config("AndroidVersion") or self.get_config("OSVersion", "Unknown")).title()
        self._supported_android_versions = self.retrieve_os_version()
        self._connection_lock = threading.Lock()
        self._phone_handle = None
        self._serial_number = self.get_config("serialNumber", "")
        self._is_device_connected = True
        self._uecmd_default_timeout = self.get_config("defaultTimeout", 50, int)
        self._phone_handle = None
        self._adb_server_port = self.get_config("adbServerPort", 5037, int)
        self._device_logger = None
        self._use_adb_socket = self.get_config("useAdbSocket", "False", "str_to_bool")
        self._eqts_controller = DeviceController.DeviceController(config.device_name, self._device_config,
                                                                  self.get_em_parameters(), self.get_logger())
        if self._serial_number in (None, "None", ""):
            # Retrieve current value of serial number
            self._serial_number = self.retrieve_serial_number()
        try:
            self.startvm_path = os.path.join(self.android_build_top,
                                             "device/intel/minimal/startvm.sh")
            self.__logger.info("startvm path: " + self.startvm_path)
        except Exception as e:
            self.__logger.info(
                "ANDROID_BUILD_TOP environment variable is not set. " +
                "Use build/envsetup.sh from the minimal root directoy")

    def switch_on(self, boot_timeout=10, settledown_duration=None, simple_switch_mode=False):
        """
        Start the QEMU using the startvm.sh script or retrieve it's pid if it is already up.
        """
        message = ""
        self.state = self.get_state()
        if self.state != "alive":
            self.__logger.info("Adb status is not alive: " + self.state + " Starting QEMU. ")
            output, message = self._run_adb_cmd("adb -s emulator-5554 kill-server", 5, True)
            if self.startvm_path:
                proc = subprocess.Popen(shlex.split(self.startvm_path), stdin=subprocess.PIPE)
                self.pid = proc.pid
            else:
                self.__logger.info("QEMU startvm path not set. Searching throught pids to check it already started ")
        if self.pid == -1:
            for process_entry in psutil.process_iter():
                if "qemu-system" in process_entry.name():
                    self.pid = process_entry.pid
        if self.pid == -1:
            status = Global.FAILURE
            message = "Emulator is not alive, qemu-system is not in the process tree. "
            return status, message
        state = ""
        boot_time_elapsed = 0
        self.state = self.get_state()
        while self.state != "alive":
            if boot_time_elapsed > boot_timeout:
                self._is_device_connected = False
                status = Global.FAILURE
                message = "Can't establish start up connection with the emulator"
                self.__logger.error("Emulator is not alive, qemu-system is not in the process tree.")
                return status, message

            # retry connection every 2 seconds
            time.sleep(2)
            boot_time_elapsed = boot_time_elapsed + 2
            self.state = self.get_state()
        self._is_device_connected = True
        status = Global.SUCCESS
        return status, message

    def get_state(self):
        """
        Adapted from AndroidDeviceBase
        """
        state = "unknown"
        try:
            retry = 2
            try_index = 0
            # Implement a retry mechanism as on some benches as on first
            # run of adb (if shutdown previously), it always return unknown
            while try_index < retry and state != "alive":
                try_index += 1
                output = self.run_cmd("adb get-state", 10, True)
            if output[0] != Global.FAILURE:
                state = (output[1].strip()).lower()
                if state.endswith("device"):
                    state = "alive"
                    if self.run_cmd("adb shell echo", 10, True)[0] \
                            == Global.FAILURE:
                        state = "unknown"
        except Exception as error:  # pylint: disable=W0703
            self.get_logger().debug("get_state: error happened: %s" % str(error))
            state = "unknown"
        return state

    def get_apk_version(self, package_name):
        """
        No need
        """
        return None

    def get_device_info(self):
        """
        No need
        """
        return []

    def init_device_connection(self, skip_boot_required, first_power_cycle, power_cycle_retry_number):
        """
        Always skip initial reboot if necessary
        """
        return IntelDeviceBase.init_device_connection(self, True, first_power_cycle, power_cycle_retry_number)

    def switch_off(self):
        return Global.SUCCESS
