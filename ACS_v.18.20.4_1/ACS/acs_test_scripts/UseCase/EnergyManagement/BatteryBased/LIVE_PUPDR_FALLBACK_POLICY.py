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
:summary: PUPDR Energy Management fallback policy
:author: jvauchex
:since: 04/02/2013
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.DeviceException import DeviceException


class LivePupdrFallbackPolicy(EmUsecaseBase):

    """
    Live Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read parameters from TC parameters
        self._wait_end_freeze_timeout = \
            int(self._tc_parameters.get_param_value("WAIT_END_FREEZE_TIMEOUT", 900))
        self._start_os = \
            self._tc_parameters.get_param_value("START_OS", "MOS")
        self._crash_type = \
            self._tc_parameters.get_param_value("CRASH_TYPE", "SECURITY")
        self._logger.info("Value of timeout : %d" % self._wait_end_freeze_timeout)
        self._logger.info("Start OS is : %s" % self._start_os)
        self._logger.info("Crash type is : %s" % self._crash_type)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LIVE_PUPDR_FALLBACK_POLICY", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # check for boot mode
        boot_mode = self._device.get_boot_mode()

        # If OS is different with the parameter => reboot
        if boot_mode != self._start_os:
            self._logger.info("Board isn't in %s : Reboot Board" % self._start_os)
            result = self._device.reboot(self._start_os)
            if not result:
                txt = "OS isn't correct !"
                self._logger.error(txt)
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, txt)

        # Disconnect the board in order to remove the logcat
        self._device.disconnect_board()
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)
        crash_counter = 0

        # Call the fast freeze board function
        while crash_counter <= 2:
            self.phonesystem_api.freeze_boot_board(self._wait_end_freeze_timeout, self._crash_type)
            crash_counter += 1
        self._logger.info("Crashes are OK, the board should be in ROS !")
        time.sleep(30)
        time_value = time.time()

        # Check the OS
        boot_mode = self._device.get_boot_mode()
        self._meas_list.add("BOOT_MODE", boot_mode, "none")

        self._device.reboot(self._start_os)

        value_logs = self.phonesystem_api.check_message_in_log("BOOT_REASON", time_value - 30, time_value, True)
        self._logger.info("The value of the boot is equal to %s " % value_logs[0])
        self._meas_list.add("BOOT_REASON", value_logs[0], "none")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
        self._em_meas_verdict.judge()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)
        boot_mode = self._device.get_boot_mode()

        # re-establish MOS
        if boot_mode in ["ROS"]:
            self._logger.info("Board in Recovery OS")
            self._device.reboot()

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
