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
:summary: Energy Management plug usb when there is no battery Use case
:author: dbatutx 11/20/2011
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmPlugNoBatt(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read VBATT from TC parameters
        self._vbatt_list = self._tc_parameters.get_param_value("VBATT_LIST").split(";")
        if len(self._vbatt_list) != 5:
            tmp_txt = "        Need 5 vbatt parameters"
            self._logger.info(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PLUG_NO_BATT", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = False (OFF)
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        # - USBChargerType = SDP
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.DCP
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature = 25
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = float(self._vbatt_list[0])

        # - VoltageLevel = VUSB
        self.em_core_module.eqp_init_state["USB"]["VoltageLevel"] = self.em_core_module.vusb

        # get power supplies
        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")
        self.em_core_module.pwrs_vusb = self._em.get_power_supply("USB")

#------------------------------------------------------------------------------

    def set_up(self):
        EmUsecaseBase.set_up(self)

        self._logger.info("Wait 40s for the phone to be switched off")
        time.sleep(40)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.run_test_body(self)

        # turn board ON
        self._io_card.press_power_button(2)
        # Plug USB_DCP
        self.em_core_module.plug_charger(self._io_card.DCP, ext_ps=True)

        # wait & check current
        time.sleep(30)
        self.check_current("IBATT0", "IUSB0")

        # reboot phone with appropriate vbatt
        self.em_core_module.pwrs_vbatt.set_current_voltage(float(self._vbatt_list[1]), self.em_core_module.ps_properties["BATT"]["PortNumber"])
        # disconnect usb
        self.em_core_module.unplug_charger(self._io_card.DCP)
        time.sleep(5)
        # turn board ON
        self._io_card.press_power_button(2)

        # Plug USB_DCP
        self.em_core_module.plug_charger(self._io_card.DCP, ext_ps=True)

        # wait & check current
        time.sleep(30)
        self.check_current("IBATT1", "IUSB1")

        # reboot phone with appropriate vbatt
        self.em_core_module.pwrs_vbatt.set_current_voltage(float(self._vbatt_list[2]), self.em_core_module.ps_properties["BATT"]["PortNumber"])
        # disconnect usb
        self.em_core_module.unplug_charger(self._io_card.DCP)
        time.sleep(5)
        # turn board ON
        self._io_card.press_power_button(2)

        # Plug USB_DCP
        self.em_core_module.plug_charger(self._io_card.DCP, ext_ps=True)

        # press power on button during 3 second
        self._io_card.press_power_button(self.pwr_btn_boot)

        # wait & check current
        time.sleep(30)
        self.check_current("IBATT2", "IUSB2")

        # reboot phone with appropriate vbatt
        self.em_core_module.pwrs_vbatt.set_current_voltage(float(self._vbatt_list[3]), self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # disconnect usb
        self.em_core_module.unplug_charger(self._io_card.DCP)
        time.sleep(30)

        # Plug USB_DCP
        before_cos_time = time.time()
        self.em_core_module.plug_charger(self._io_card.DCP, ext_ps=True)

        # wait boot in cos
        time.sleep(40)

        # check current
        self.check_current("IBATT3", "IUSB3")

        # reboot phone with appropriate vbatt
        self.em_core_module.pwrs_vbatt.set_current_voltage(float(self._vbatt_list[4]), self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # disconnect usb
        self.em_core_module.unplug_charger(self._io_card.DCP)
        time.sleep(30)
        after_cos_time = time.time()

        # Plug USB_DCP
        self.em_core_module.plug_charger(self._io_card.DCP, ext_ps=True)

        # sleep(30s)
        time.sleep(30)

        # boot board
        self.em_core_module.boot_board()

        # check for android boot mode
        result = self.phonesystem_api.check_message_in_log("COS_MODE",
                                                            before_cos_time, after_cos_time, False)
        if result[0] == "TRUE":
            os = "COS"
        else:
            os = "UNKNOW"
        # Compare boot mode cos value with limit parameters
        self._meas_list.add("STATE_COS", os, "none")

        # Compare boot mode mos value with limit parameters
        self._meas_list.add("STATE_MOS",
                            self._device.get_boot_mode(), "none")

        # unplug the charger
        self.em_core_module.plug_charger(self._io_card.WALL_CHARGER, ext_ps=True)
        time.sleep(5)

        # wait & check current
        self.check_current("IBATT4", "IUSB4")

        # unplug the WALL CHARGER
        self.em_core_module.unplug_charger(self._io_card.WALL_CHARGER)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def check_current(self, ibatt, iusb):
        """
        Function that checks the current ibatt and iusb and compare it with result

        :type  ibatt: str
        :param ibatt: str that give an enter for the compare fonction
        :type  iusb: str
        :param iusb: str that give an enter for the compare fonction
        """

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from Vbatt
        meas_iusb = self.em_core_module.get_charger_current(self._io_card.WALL_CHARGER)

        self._logger.info("        Current : IBatt = %s %s , IUsb = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

        # Compare Vbatt value with limit parameters
        self._meas_list.add(ibatt, meas_ibatt)

        # Compare Vusb value with limit parameters
        self._meas_list.add(iusb, meas_iusb)
