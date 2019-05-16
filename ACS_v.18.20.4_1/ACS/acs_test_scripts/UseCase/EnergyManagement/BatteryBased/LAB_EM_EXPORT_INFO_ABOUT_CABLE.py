"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL QCTV
:summary: EM - test export information about cable to userapp
:author: jortetx
:since: 13/03/2014
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
import time


class LabEmExportInfoAboutCable(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"
    __POSSIBLE_ACTION = ["BOOT_BOARD", "PLUG_CABLE"]

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call the LAB_EM_USE_CASE init method
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read mandatory cable info
        tmp_cable_minfo = self._tc_parameters.get_param_value("SYSFS_MANDATORY_INFO")
        self.__sysfs_mandatory_cable_info = filter(None, tmp_cable_minfo.split(';'))
        self.__sysfs_mandatory_cable_info = map(str.strip, self.__sysfs_mandatory_cable_info)

        self.__cable_type = self._tc_parameters.get_param_value("CABLE_TYPE")
        self.__action_to_perform = self._tc_parameters.get_param_value("ACTION")
        self.__delay = self._tc_parameters.get_param_value("DELAY_BEFORE_CHECKING_INFO", 0, default_cast_type=int)
        self.__boot_mode = self._tc_parameters.get_param_value("BOOT_MODE_IF_BOOT_BOARD")

        # get target and initialize measurement
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_EXPORT_INFO_ABOUT_CABLE", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.__sysfs_mandatory_cable_info)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Execute the set up
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # Check action first
        if self.__action_to_perform not in self.__POSSIBLE_ACTION:
            txt = "wrong action to perform, can only be %s and not %s" % (self.__POSSIBLE_ACTION, self.__action_to_perform)
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # Check specific option if they are chosen
        if self.__cable_type not in self._io_card.SUPPORTED_DEVICE_TYPE and self.__action_to_perform == "PLUG_CABLE":
            txt = "io card does not support cable type %s " % self.__cable_type
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # Check specific option if they are chosen
        if self.__boot_mode not in ["COS", "MOS", "COS_MOS", "MOS_COS"] and self.__action_to_perform == "BOOT_BOARD":
            txt = "boot mode cant be %s but only [COS, MOS, COS_MOS, MOS_COS] " % self.__boot_mode
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        self._device.disconnect_board()
        self._io_card.simulate_insertion(self.__cable_type)
        time.sleep(self.usb_sleep)

        # check if we still have data after changing the cable , if not, abort usecase
        if self._device.get_state() != "alive":
            txt = "DATA connection lost after switching cable to %s ,you need data connection for this test, aborting usecase." % self.__cable_type
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, txt)
        else:
            self._device.connect_board()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """

        if self.__action_to_perform == "BOOT_BOARD":
            self.__boot_checking(self.__boot_mode, self.__delay)

        elif self.__action_to_perform == "PLUG_CABLE":
            self.__cable_insertion_checking(self.__cable_type, self.__delay)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge()

        return self._em_meas_verdict.get_current_result_v2()

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # clean the board state and retrieve logs
        if self.em_core_module.TYPE == "BATTERY_BENCH":
            self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __boot_checking(self, boot_mode, delay):
        """
        The goal of this test is to check the cable information export to user app :
        check presence in sysfs interface of information when the platform have been just booted
        """
        if boot_mode in ["COS_MOS", "MOS_COS"]:
            self._device.reboot(boot_mode.split("_")[0])
            self._device.reboot(boot_mode.split("_")[1])
        else:
            self._device.reboot(boot_mode)

        # set like this to cause negative delay to crash
        if delay != 0:
            self._logger.info("waiting %ss before getting EM INFO" % delay)
            # waiting x minutes
            time.sleep(delay)

        # get register value
        em_info = self.em_api.get_msic_registers()
        # get all the battery info type contains in register
        cable_infos = em_info["CHARGER"].keys()
        # verify that all mandatory info are in register
        for minfo in self.__sysfs_mandatory_cable_info:
            if minfo not in cable_infos:
                self._logger.info("%s not found in your CABLE info" % minfo)
            else:
                self._meas_list.add(minfo, em_info["CHARGER"][minfo][0],
                                    em_info["CHARGER"][minfo][1])

    def __cable_insertion_checking(self, cable_type, delay):
        """
        The goal of this test is to check the cable information export to user app :
        check the change on sysfs interface of all mandatory information when an cable is inserted
        """
        # set like this to cause negative delay to crash
        if delay != 0:
            self._logger.info("waiting %ss before getting EM INFO" % delay)
            # waiting x minutes
            time.sleep(delay)

        # get register value
        em_info = self.em_api.get_msic_registers()
        # get all the battery info type contains in register
        cable_infos = em_info["CHARGER"].keys()
        # verify that all mandatory info are in register
        for minfo in self.__sysfs_mandatory_cable_info:
            if minfo not in cable_infos:
                self._logger.info("%s not found in your CABLE info" % minfo)
            else:
                self._meas_list.add(minfo, em_info["CHARGER"][minfo][0],
                                    em_info["CHARGER"][minfo][1])
