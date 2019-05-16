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
:summary: EM - test export information about battery to userapp
:author: jortetx
:since: 07/01/2014
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
import time


class LabEmExportInfoAboutBatt(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"
    __POSSIBLE_ACTION = ["BOOT_BOARD", "BATTERY_INSERTION", "PLUG_CHARGER"]
    __VARIATION_TAG = "_VARIATION"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call the LAB_EM_USE_CASE init method
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read mandatory battery info
        tmp_batt_minfo = self._tc_parameters.get_param_value("SYSFS_MANDATORY_INFO")
        tmp_batt_minfo = tmp_batt_minfo.split(';')
        tmp_batt_minfo = map(str.strip, tmp_batt_minfo)
        self.__sysfs_mandatory_batt_info = []

        self.__action_to_perform = self._tc_parameters.get_param_value("ACTION")
        self.__delay = self._tc_parameters.get_param_value("DELAY_BEFORE_CHECKING_INFO", 0, default_cast_type=int)
        self.__charger_type = self._tc_parameters.get_param_value("CABLE_TYPE_IF_PLUG_CHARGER", self._io_card.USB_HOST_PC)
        self.__boot_mode = self._tc_parameters.get_param_value("BOOT_MODE_IF_BOOT_BOARD")

        if self.__action_to_perform == "PLUG_CHARGER":
            for ele in tmp_batt_minfo:
                self.__sysfs_mandatory_batt_info.append(ele + self.__VARIATION_TAG)
        else:
            self.__sysfs_mandatory_batt_info = tmp_batt_minfo

        if self.__action_to_perform == "BATTERY_INSERTION" and self.em_core_module.TYPE == "POWER_SUPPLY_BENCH":
            self.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
            self.io_card_init_state["Battery"] = False
            self.io_card_init_state["Platform"] = "OFF"
            self.io_card_init_state["USBChargerType"] = self._io_card.USB_HOST_PC
            self.io_card_init_state["USBCharger"] = False

        # get target and initialize measurement
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_EXPORT_INFO_ABOUT_BATT", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.__sysfs_mandatory_batt_info)

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
            txt = "io card does not support cable type %s " % self.__charger_type
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # Check specific option if they are chosen
        if self.__boot_mode not in ["COS", "MOS"] and self.__action_to_perform == "BOOT_BOARD":
            txt = "boot mode cant be %s but only [COS, MOS] " % self.__boot_mode
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # Check specific option if they are chosen
        if self.__action_to_perform == "BATTERY_INSERTION" and self.em_core_module.TYPE != "POWER_SUPPLY_BENCH":
            txt = "to perform BATTERY_INSERTION action you need to use a power supply bench and declare it on your EM testcase.xml"
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        if self.__action_to_perform == "BATTERY_INSERTION":
            self.__batt_insertion_checking(self.__delay)

        elif self.__action_to_perform == "BOOT_BOARD":
            self.__boot_checking(self.__boot_mode, self.__delay)

        elif self.__action_to_perform == "PLUG_CHARGER":
            self.__cable_insertion_checking(self.__charger_type, self.__delay)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge()

        return self._em_meas_verdict.get_current_result_v2()

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)

        if self.em_core_module.TYPE == "BATTERY_BENCH":
            self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __batt_insertion_checking(self, delay):
        """
        The goal of this test is to check the battery information export to user app :
         check presence in sysfs interface of information when the platform have been booted after a battery insertion
        """
        # when we are it means that battery has been removed and not charger plugged
        self._io_card.battery_connector(True)
        time.sleep(5)
        # power on the device
        self._device.switch_on(simple_switch_mode=True)
        # set like this to cause negative delay to crash
        if delay != 0:
            self._logger.info("waiting %ss before getting EM INFO" % delay)
            # waiting x minutes
            time.sleep(delay)

        # get board state
        if self._device.get_boot_mode() != "UNKNOWN":
            # get register value
            em_info = self.em_api.get_msic_registers()
            # get all the battery info type contains in register
            battery_infos = em_info["BATTERY"].keys()
            # verify that all mandatory info are in register
            for minfo in self.__sysfs_mandatory_batt_info:
                if minfo not in battery_infos:
                    self._logger.info("%s is missing in mandatory info to export" % minfo)
                else:
                    self._meas_list.add(minfo, em_info["BATTERY"][minfo][0], em_info["BATTERY"][minfo][1])
        else:
            txt = "Board fail to been seen booted after battery insertion"
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

    def __boot_checking(self, boot_mode, delay):
        """
        The goal of this test is to check the battery information export to user app :
        check presence in sysfs interface of information when the platform have been just booted
        """
        self._device.reboot(boot_mode)

        # set like this to cause negative delay to crash
        if delay != 0:
            self._logger.info("waiting %ss before getting EM INFO" % delay)
            # waiting x minutes
            time.sleep(delay)

        # get register value
        em_info = self.em_api.get_msic_registers()
        # get all the battery info type contains in register
        battery_infos = em_info["BATTERY"].keys()
        # verify that all mandatory info are in register
        for minfo in self.__sysfs_mandatory_batt_info:
            if minfo not in battery_infos:
                self._logger.info("%s is missing in mandatory info to export" % minfo)
            else:
                self._meas_list.add(minfo, em_info["BATTERY"][minfo][0], em_info["BATTERY"][minfo][1])

    def __cable_insertion_checking(self, cable_type, delay):
        """
        The goal of this test is to check the battery information export to user app :
        check the change on sysfs interface of all mandatory information when an charger is inserted
        """
        em_id_init = self.em_api.get_msic_registers("scheduled", 30)
        em_id_after_some_min = self.em_api.get_msic_registers("scheduled", delay + 30)

        self._device.disconnect_board()
        self._io_card.remove_cable(cable_type)
        time.sleep(45)
        self._io_card.simulate_insertion(cable_type)
        # wait for the first cable insertion then wait for the one that happen later
        self._logger.info("waiting %ss before getting EM INFO" % (delay + 45))
        time.sleep(delay + 45)
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        self._device.connect_board()

        # get register value
        em_info_init = self.em_api.get_msic_registers("read", em_id_init)
        em_info_after_some_min = self.em_api.get_msic_registers("read", em_id_after_some_min)

        # calculate target
        for minfo in self.__sysfs_mandatory_batt_info:
            real_name = minfo.replace(self.__VARIATION_TAG, "")
            if real_name in em_info_after_some_min["BATTERY"].keys():
                # case where we compute a non numeric value , we take the last value in this case
                if type(em_info_after_some_min["BATTERY"][real_name][0]) is str:
                    tuple_result = em_info_after_some_min["BATTERY"][real_name]
                # else calculate the variation
                else:
                    delta_value = float(em_info_after_some_min["BATTERY"][real_name][0]) - float(em_info_init["BATTERY"][real_name][0])
                    unit = em_info_after_some_min["BATTERY"][real_name][1]
                    tuple_result = (delta_value, unit)

                self._meas_list.add(minfo, tuple_result)
            else:
                self._logger.info("%s is missing in info to export" % real_name)
