"""

:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: This file implements the IPC sleep UC to measure residency
:since: 07/07/2015
:author: amurarux
"""
import time
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException
from UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveSystemIpcSleepResidencyMeasurement(UseCaseBase):

    """
    IPC Sleep mode residency measurement class.
    """

    IPC_SSIC_DIR_PATH = "/d/pmc_atom/"
    IPC_SSIC_PMU_STATE_CMD = "adb shell cat /d/pmc_atom/dev_state | grep USB_EHCI"
    IPC_HSIC_DIR_PATH = "/d"
    IPC_HSIC_PMU_STATE_CMD = "adb shell cat d/mid_pmu_states | grep hsi"
    """
    Original platform configuration file
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCaseBase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        self._initial_screen_timeout = None
        # Sleep time
        if self._tc_parameters.get_param_value("DURATION") is not None:
            self._duration = \
                int(self._tc_parameters.get_param_value("DURATION"))

        # time where to control IPC residency
        self._time_period = int(self._tc_parameters.
                                get_param_value("TIME_PERIOD"))

        # target percentage of time period where to control IPC residency
        self._target_rate = int(self._tc_parameters.
                                get_param_value("TARGET_RATE"))

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

    def set_up(self):
        """
        Initialize the test.
        """
        # Run UC base set_up step
        UseCaseBase.set_up(self)

        # Check the duration before going any further
        if self._duration is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._duration),
                "DURATION")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check the time where to control IPC residency before going any further
        if self._time_period is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._time_period),
                "TIME_PERIOD")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check target percentage of time period where to control IPC residency before going any further
        if self._target_rate is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._target_rate),
                "TARGET_RATE")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        self._initial_screen_timeout = self._phonesystem_api.get_screen_timeout()

        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Execute the test
        """

        # Call use case base run_test
        UseCaseBase.run_test(self)

        # Get the modem version from target system properties
        modem_fw_cmd = "adb shell getprop gsm.version.baseband"
        (return_code, output) = self._device.run_cmd(modem_fw_cmd,
                                                    10,
                                                    force_execution=True)
        if return_code is Global.FAILURE:
            return_msg = "Command %s has failed" % modem_fw_cmd
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)
        elif output is "":
            return_msg = "Modem state is DOWN"
            raise DeviceException(AcsConfigException.OPERATION_FAILED, return_msg)

        # Set phone screen off
        self._phonesystem_api.set_screen_timeout(1)

        self._logger.info("Wait for %s s before IPC sleep check" %
                        str(self._duration))
        time.sleep(self._duration)

        ipc_sleep_cmd = None

        # Checks whether the IPC used is HSIC or SSIC
        if self._phonesystem_api.check_directory_exist_from_shell(self.IPC_SSIC_DIR_PATH) is True:
            self._logger.info("The device under test is SSIC IPC based")
            ipc_sleep_cmd = self.IPC_SSIC_PMU_STATE_CMD
        elif self._phonesystem_api.check_directory_exist_from_shell(self.IPC_HSIC_DIR_PATH) is True:
            self._logger.info("The device under test is HSI/HSIC IPC based")
            ipc_sleep_cmd = self.IPC_HSIC_PMU_STATE_CMD
        else:
            raise DeviceException(AcsConfigException.OPERATION_FAILED, "IPC used is neither SSIC nor HSI/SSIC")

        (status, output) = \
            self._phonesystem_api.check_ipc_sleep_mode(
                ipc_sleep_cmd,
                self._time_period,
                self._target_rate)

        # Return the status
        return status, output

    def tear_down(self):
        """
        Execute tear_down method
        """

        # Call use case base run_test
        UseCaseBase.tear_down(self)

        #set back inital screen timeout
        self._phonesystem_api.set_screen_timeout(self._initial_screen_timeout)

        return Global.SUCCESS, "No errors"
