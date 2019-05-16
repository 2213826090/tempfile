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

:organization: INTEL QCTV
:summary: EM - test export information about charger to userapp
:author: jortetx
:since: 13/02/2014
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
import time
import os


class LabEmExportInfoAboutCharger(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"
    __POSSIBLE_ACTION = ["BOOT_BOARD", "PLUG_CHARGER"]
    __VARIATION_TAG = "_VARIATION"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call the LAB_EM_USE_CASE init method
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read mandatory charger info
        tmp_charger_minfo = self._tc_parameters.get_param_value("SYSFS_MANDATORY_INFO")
        tmp_charger_minfo = filter(None, tmp_charger_minfo.split(';'))
        tmp_charger_minfo = map(str.strip, tmp_charger_minfo)
        self.__sysfs_mandatory_charger_info = []

        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE")
        self.__action_to_perform = self._tc_parameters.get_param_value("ACTION")
        self.__delay = self._tc_parameters.get_param_value("DELAY_BEFORE_CHECKING_INFO", 0, default_cast_type=int)

        if self.__action_to_perform == "BOOT_BOARD":
            for ele in tmp_charger_minfo:
                self.__sysfs_mandatory_charger_info.append(ele + self.__VARIATION_TAG)
        else:
            self.__sysfs_mandatory_charger_info = tmp_charger_minfo

        # get target and initialize measurement
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_EXPORT_INFO_ABOUT_CHARGER", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.__sysfs_mandatory_charger_info)
        # measurement file
        meas_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

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
        if self.__charger_type not in self._io_card.SUPPORTED_DEVICE_TYPE and self.__action_to_perform == "PLUG_CHARGER":
            txt = "io card does not support charger type %s " % self.__charger_type
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """

        if self.__action_to_perform == "BOOT_BOARD":
            self.__boot_checking(self.__delay)

        elif self.__action_to_perform == "PLUG_CHARGER":
            self.__charger_insertion_checking(self.__charger_type, self.__delay)

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

        # clean autolog
        if self.is_board_and_acs_ok():
            self.em_api.clean_autolog()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __boot_checking(self, delay):
        """
        The goal of this test is to check the charger information export to user app :
        check presence in sysfs interface of information when the platform have been just booted
        """
        # prepare the logger to collect info after next reboot
        self.em_api.clean_autolog()
        hardcoded_time = 70
        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        if delay > 0:
            self.em_api.set_autolog_duration(delay)
        # start  non persistent autolog with a short period of data polling
        self.em_api.start_auto_logger_on_reboot(5, "sequenced")
        # then turn off board
        self._device.switch_off()
        # then boot in COS by inserting charger
        self._io_card.simulate_insertion(self.__charger_type)
        # wait a while then  press power button to boot in next boot mode
        self._logger.info("waiting 90s to let the board boot in COS")
        time.sleep(90)
        self._logger.info("trying to boot in MOS by pressing power button during %ss" % self.pwr_btn_boot)
        self._io_card.press_power_button(self.pwr_btn_boot)
        # in  MOS at this time logger should start asap, wait for a while
        # set like this to cause negative delay to crash
        if delay > 0:
            self._logger.info("waiting %ss + %ss before getting EM INFO" % (delay + hardcoded_time, hardcoded_time))
            # waiting x minutes
            time.sleep(delay + hardcoded_time)

        # disconnecting charger
        self._logger.info("Disconnecting %s" % self.__charger_type)
        self._io_card.remove_cable(self.__charger_type)
        time.sleep(self.usb_sleep)

        # reconnect data cable to collect info
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        self._device.connect_board()

        # check if we still  have data, if no, abort test
        if not self.is_board_and_acs_ok():
            txt = "board connection lost after booting from COS to MOS, cant compute any results"
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, txt)

        # get register value
        all_info = self.em_api.get_autolog_msic_registers()
        # remove the last element to avoid false FAIL measurement
        all_info.pop()
        em_info_at_boot = all_info[0][1]
        em_info_few_time_later = all_info[-1][1]
        self.em_api.clean_autolog()
        # calculate target
        for minfo in self.__sysfs_mandatory_charger_info:
            real_name = minfo.replace(self.__VARIATION_TAG, "")
            if real_name in em_info_few_time_later["CHARGER"].keys():
                # case where we compute a non numeric value , we take the last value in this case
                if type(em_info_few_time_later["CHARGER"][real_name][0]) is str:
                    tuple_result = em_info_few_time_later["CHARGER"][real_name]
                # else calculate the variation
                else:
                    delta_value = float(em_info_few_time_later["CHARGER"][real_name][0]) - float(em_info_at_boot["CHARGER"][real_name][0])
                    unit = em_info_few_time_later["CHARGER"][real_name][1]
                    tuple_result = (delta_value, unit)

                self._meas_list.add(minfo, tuple_result)
            else:
                self._logger.info("%s is missing in info to export" % real_name)
        # store measurement for debugging purpose
        self.em_core_module.fill_autolog_result(all_info, [], self.__em_meas_tab, "log from MOS boot after COS")

    def __charger_insertion_checking(self, charger_type, delay):
        """
        The goal of this test is to check the charger information exported to user app :
        check the change on sysfs interface of all mandatory information when an charger is inserted
        """
        em_id_init = self.em_api.get_msic_registers("scheduled", delay + 30)

        self._device.disconnect_board()
        self._io_card.simulate_insertion(charger_type)
        self._logger.info("waiting %ss before getting EM INFO" % (delay + 45))
        time.sleep(delay + 45)

        # disconnecting charger
        self._logger.info("Disconnecting %s" % charger_type)
        self._io_card.remove_cable(charger_type)
        time.sleep(self.usb_sleep)

        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        self._device.connect_board()

        # get register value
        em_info = self.em_api.get_msic_registers("read", em_id_init)
        # get all the battery info type contains in register
        charger_infos = em_info["CHARGER"].keys()
        # verify that all mandatory info are in register
        for minfo in self.__sysfs_mandatory_charger_info:
            if minfo not in charger_infos:
                self._logger.info("%s not found in your CHARGER info" % minfo)
            else:
                self._meas_list.add(minfo, em_info["CHARGER"][minfo][0],
                                    em_info["CHARGER"][minfo][1])
