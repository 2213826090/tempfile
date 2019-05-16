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
:summary: PUPDR Energy Management boot transition usecase
:author: vgomberx
:since: 27/09/2012
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabPupdrBootTransition(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Initialize variables
        self.__transition_combo = None

        # Read parameters from TC parameters
        self.__transition_timeout = \
            self._tc_parameters.get_param_value("TRANSITION_TIMEOUT")
        if self.__transition_timeout not in [None, ""]:
            # convert to int if not None else leave it to None
            self.__transition_timeout = int(self.__transition_timeout)
        else:
            self.__transition_timeout = self._device.get_boot_timeout()

        self.__transition_combo_raw = \
            str(self._tc_parameters.get_param_value("TRANSITION_COMBO"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_PUPDR_BOOT_TRANSITION", self._tc_parameters.get_params_as_dict(),
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

        local_transition = []
        self.__transition_combo = self.__transition_combo_raw .split(",")

        for mode in self.__transition_combo:
            mode = mode.upper().strip()
            if mode not in ["POS", "MOS", "COS", "ROS", ""]:
                txt = "not supported boot mode %s" % mode
                self._logger.error(txt)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)
            elif mode != "":
                local_transition.append(mode)

        if len(local_transition) == 0:
            txt = "no boot mode detected, aborting test"
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)
        else:
            self.__transition_combo = local_transition

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)
        # iteration for test tag
        boot_mode = []
        time_before = []
        time_after = []
        actual_mode = self._device.get_boot_mode()
        host_start_time = time.time()
        phone_start_time = self.get_time_from_board()

        for mode in self.__transition_combo:
            # get next mode
            next_mode = mode
            # reboot board
            time_before.append(phone_start_time + (time.time() - host_start_time))
            # Go in COS from MOS using end user way
            if actual_mode == "MOS" and next_mode == "COS":
                self._device.inject_device_log("i", "ACS",
                                               "trying to reboot board in %s" % mode)
                self._device.disconnect_board()
                self._io_card.press_power_button(0.5)
                time.sleep(0.5)
                self._io_card.press_power_button(self.phone_info["GENERAL"].get("PRESS_SOFT_SHUTDOWN"))

                # loop to see COS state
                start_time = time.time()
                while(time.time() - start_time) < self.__transition_timeout:
                    if mode == self._device.get_boot_mode():
                        self._logger.info("Board has been seen booted in %s after %s seconds" %
                                         (mode, str((time.time() - start_time))))
                        self._device.inject_device_log("i", "ACS",
                                                       "Board successfully booted in %s" % mode)
                        break

            # Go in MOS from COS using end user way
            elif actual_mode == "COS" and next_mode == "MOS":
                self._device.inject_device_log("i", "ACS",
                                               "trying to reboot board in %s" % mode)
                self._io_card.press_power_button(self.pwr_btn_boot)
                # loop to see MOS state
                start_time = time.time()
                while(time.time() - start_time) < self.__transition_timeout:
                    if mode == self._device.get_boot_mode():
                        self._logger.info("Board has been seen booted in %s after %s seconds" %
                                         (mode, str((time.time() - start_time))))
                        # wait some sec to settle down and connect board
                        time.sleep(self.usb_sleep)
                        self._device.connect_board()
                        self._device.inject_device_log("i", "ACS",
                                                       "Board successfully booted in %s" % mode)
                        break
            else:
                self._device.reboot(mode, wait_for_transition=True,
                                    transition_timeout=self.__transition_timeout,
                                    skip_failure=True)

            # get boot mode
            actual_mode = self._device.get_boot_mode()
            boot_mode.append(actual_mode)
            time_after.append(phone_start_time + (time.time() - host_start_time))

        # re-establish MOS
        actual_boot_mode = self._device.get_boot_mode()
        if actual_boot_mode not in ["UNKNOWN", "MOS"]:
            self._device.reboot()
        # if board is in unknown state, force shutdown and reboot
        elif actual_boot_mode == "UNKNOWN":
            self.em_core_module.reboot_board(switch_off_mode="HARD")

        for i in range(len(time_before)):
            # get shutdown reason
            shutdown = self.phonesystem_api.check_message_in_log(
                "SHUTDOWN_REASON", time_before[i],
                time_after[i], True)[0]
            self._meas_list.add("SHUTDOWN_REASON" + str(i + 1),
                                (shutdown, "none"))
            # get boot mode
            self._meas_list.add("BOOT_MODE" + str(i + 1),
                                (boot_mode[i], "none"))
            # get boot reason
            boot_reason = self.phonesystem_api.check_message_in_log(
                "BOOT_REASON", time_before[i],
                time_after[i], True)[0]
            self._meas_list.add("BOOT_REASON" + str(i + 1),
                                (boot_reason, "none"))
        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_global_result()
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
        if boot_mode not in ["UNKNOWN", "MOS"]:
            self._device.reboot()
        # if board is in unknown state, force shutdown and reboot
        elif boot_mode == "UNKNOWN":
            self.em_core_module.reboot_board(switch_off_mode="HARD")

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
