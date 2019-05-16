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
:summary: This file implements the GMIN TCP-IP target platforms
:since: 5/27/2014
:author: rcstamat
"""

from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from ErrorHandling.DeviceException import DeviceException
from Device.Module.DeviceModuleFactory import DeviceModuleFactory
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException

import subprocess
import os
import signal

class AndroidEmulator(AndroidDeviceBase):
    """
        Definition for android emulator
    """

    def __init__(self, phone_name, device_config):
        """
        Constructor

        :type  phone_name: str
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """
        AndroidDeviceBase.__init__(self, phone_name, device_config)

        self._multiple_devices = self.config.get_value("multipleDevices", "False", "str_to_bool")
        #The process id. If none is provided have a negative value so we don't kill a real process
        self.pid = -1
        self.timeout = self.config.get_value("flashTimeout", 120 ,int)

        flash_file = self._acs_flash_file_path

    def switch_on(self, boot_timeout=None, settledown_duration=None, simple_switch_mode=False):

        cur_dir = os.getcwd()
        #convert the path to a string a take only the part after the :
        #Using os.environ to pass values from flashing tool to device since not other method is available
        path_to_x86 = "".join(os.environ.get('DIR_PATH').strip(":")[0:])
        os.chdir(path_to_x86)

        #Run the emulator
        emulator = "".join(os.environ.get('EMULATOR_PATH').strip(":")[0:])
        proc = subprocess.Popen(
            [emulator, '-verbose', '-kernel', 'kernel-qemu', '-ramdisk', 'ramdisk.img', '-system', 'system.img',
             '-no-snapshot', '-gpu', 'on', '-avd', 'x86'], stdin=subprocess.PIPE)

        status = Global.SUCCESS
        message = 0
        while self.timeout > 0 and proc is not None and proc.poll() is None:
            self.timeout = self.timeout - 1
            if self.timeout < 0:
                status = Global.FAILURE
                message = "Time out while switching emulator on"
        self.pid = proc.pid
        os.chdir(cur_dir)

        return status, message

    def is_available(self):
        return True

    def switch_off(self):
        if self.pid != -1:
            try:
                os.kill(self.pid, signal.SIGQUIT)
                return Global.SUCCESS
            except:
                return Global.FAILURE
        return Global.FAILURE

