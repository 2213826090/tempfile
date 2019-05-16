"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements the Flash UC
:since: 18/06/2014
:author: rcstamat
"""
import platform
from time import sleep

from acs_test_scripts.Device.Module.Common.Flash.FlashManager.Tool.CmdExecManager import CmdExecManager
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LIVE_ANDROID_EMULATOR(UseCaseBase):
    """
    Emulator control class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

    class CmdExec(CmdExecManager):
        """
        Class for overwriting the get_cmd method
        """

        def __init__(self, logger):
            CmdExecManager.__init__(self, logger)
            self._timeout = 30

        def _get_cmd(self):
            if platform.system() != "Windows":
                exec_cmd = "adb shell ps"
                return exec_cmd
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED, "OS not supported")

    def run_test(self):
        """
        Triggers switching the emulator on/off by calling the apropiate method from
        the emulator device model.
        """
        return_status = None
        error_message = ""
        switch = self._tc_parameters.get_param_value("SWITCH")
        start_up_time = self._tc_parameters.get_param_value("START_UP_TIME",200,int)
        if (switch == "On"):
            self._device.switch_on()
            # without haxm and latest graphics driver it takes around 2 min to start
            sleep(start_up_time)
            cmd = self.CmdExec("switch_thread")
            return_code, error_message = cmd._run()
            if return_code == 0:
                return_status = Global.SUCCESS
            else:
                return_status = self._device.switch_off()
                if return_code == Global.FAILURE:
                    error_message = "Can't execute the verification command"
                    return_status = Global.FAILURE
        elif (switch == "Off"):
            return_status = self._device.switch_off()
            error_message = "No pid for the emulator found"
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The SWITCH parameter is None, please provide a value in TC".format(
                                         self._optional_module_config))

        return return_status, error_message
