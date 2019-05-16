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
:summary: Energy Management battery over temperature Use case
:author: vgombert /dbatutx
:since: 11/25/2011
"""
import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmBattOverTemp(EmUsecaseBase):

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

        # Read all temperatures from TC parameters
        self._batt_temp_normal = int(self._tc_parameters.get_param_value("BATT_TEMP_NORMAL"))
        self._batt_temp_over = int(self._tc_parameters.get_param_value("BATT_TEMP_OVER"))
        self._batt_temp_under = int(self._tc_parameters.get_param_value("BATT_TEMP_UNDER"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_OVER_TEMP", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = ON
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        # - USBChargerType = USB_HOST_PC
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        # - USBCharger = True (inserted)
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # - BatteryTemperature = BatteryTemperature
        self.em_core_module.io_card_init_state["BatteryTemperature"] = self._batt_temp_normal

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

        # - VoltageProtectLevel = VUSB_OVERVOLTAGE_LEVEL
        self.em_core_module.eqp_init_state["USB"]["VoltageProtectLevel"] = 14

        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")
        self.em_core_module.pwrs_vusb = self._em.get_power_supply("USB")

        # - Delay to wait for scheduled commands
        self._scheduled_timer_1 = \
            int(self._em_targets["MSIC_REGISTER1"]["scheduled_time"])
        self._scheduled_timer_2 = \
            int(self._em_targets["MSIC_REGISTER2"]["scheduled_time"])
        self._scheduled_timer_3 = \
            int(self._em_targets["MSIC_REGISTER3"]["scheduled_time"])
        self._scheduled_timer_4 = \
            int(self._em_targets["MSIC_REGISTER4"]["scheduled_time"])
        self._scheduled_timer_5 = \
            int(self._em_targets["MSIC_REGISTER5"]["scheduled_time"])

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # start interrupt measurement after current connection stop
        interrupt_pid = self.em_api.get_proc_interrupt("scheduled", 0)

        # Set API to start in x seconds to read and store all MSIC registers
        time1_before = time.time()
        pid1 = self.em_api.get_msic_registers("scheduled",
                                               self._scheduled_timer_1)
        interrupt_pid1 = self.em_api.get_proc_interrupt("scheduled",
                                                         self._scheduled_timer_1)
        time1_after = time.time()

        # Set API to start in x seconds to read and store all MSIC registers
        time2_before = time.time()
        pid2 = self.em_api.get_msic_registers("scheduled",
                                               self._scheduled_timer_2)
        interrupt_pid2 = self.em_api.get_proc_interrupt("scheduled",
                                                         self._scheduled_timer_2)
        time2_after = time.time()

        # Set API to start in x seconds to read and store all MSIC registers
        time3_before = time.time()
        pid3 = self.em_api.get_msic_registers("scheduled",
                                               self._scheduled_timer_3)
        interrupt_pid3 = self.em_api.get_proc_interrupt("scheduled",
                                                         self._scheduled_timer_3)
        time3_after = time.time()

        # Set API to start in x seconds to read and store all MSIC registers
        time4_before = time.time()
        pid4 = self.em_api.get_msic_registers("scheduled",
                                               self._scheduled_timer_4)
        interrupt_pid4 = self.em_api.get_proc_interrupt("scheduled",
                                                         self._scheduled_timer_4)
        time4_after = time.time()

        # Set API to start in x seconds to read and store all MSIC registers
        time5_before = time.time()
        pid5 = self.em_api.get_msic_registers("scheduled",
                                               self._scheduled_timer_5)
        interrupt_pid5 = self.em_api.get_proc_interrupt("scheduled",
                                                         self._scheduled_timer_5)
        time5_after = time.time()

        # Close connection
        self._device.disconnect_board()
        self._io_card.usb_connector(False)

        # Select the wall charger
        self.em_core_module.plug_charger(self._io_card.WALL_CHARGER, ext_ps=True)

        # Wait time between command
        if (time.time() - time1_before) > self._scheduled_timer_1:
            return (Global.FAILURE, "Test implementation incorrect " +
                    "(increase MSIC_REGISTER1 / scheduled_time constant)")
        self._wait_smart_time(time1_after, self._scheduled_timer_1)
        time.sleep(3)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "Battery temperature=%s : IBatt = %s %s , IUsb = %s %s " %
            (str(self._batt_temp_normal), meas_ibatt[0], meas_ibatt[1],
             meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT_NORMAL1", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB_NORMAL1", meas_iusb)

        # Change the battery temperature to the over value
        self._io_card.set_battery_temperature(self._batt_temp_over)

        # Wait time between command
        if (time.time() - time2_before) > self._scheduled_timer_2:
            return (Global.FAILURE, "Test implementation incorrect " +
                    "(increase MSIC_REGISTER2 / scheduled_time constant)")
        self._wait_smart_time(time2_after, self._scheduled_timer_2)
        time.sleep(3)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info("Battery temperature=%s : IBatt = %s %s , IUsb = %s %s " %
                          (str(self._batt_temp_over), meas_ibatt[0], meas_ibatt[1],
                           meas_iusb[0], meas_iusb[1]))

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT_OVER", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB_OVER", meas_iusb)

        # Change the battery temperature to the normal value
        self._io_card.set_battery_temperature(self._batt_temp_normal)

        # Wait time between command
        if (time.time() - time3_before) > self._scheduled_timer_3:
            return (Global.FAILURE, "Test implementation incorrect " +
                    "(increase MSIC_REGISTER3 / scheduled_time constant)")
        self._wait_smart_time(time3_after, self._scheduled_timer_3)
        time.sleep(3)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "Battery temperature=%s : IBatt = %s %s , IUsb = %s %s " %
            (str(self._batt_temp_normal), meas_ibatt[0], meas_ibatt[1],
             meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT_NORMAL2", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB_NORMAL2", meas_iusb)

        # Change the battery temperature to the under value
        self._io_card.set_battery_temperature(self._batt_temp_under)

        # Wait time between command
        if (time.time() - time4_before) > self._scheduled_timer_4:
            return (Global.FAILURE, "Test implementation incorrect " +
                    "(increase MSIC_REGISTER3 / scheduled_time constant)")
        self._wait_smart_time(time4_after, self._scheduled_timer_4)
        time.sleep(3)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "Battery temperature=%f : IBatt = %s %s , IUsb = %s %s " %
            (self._batt_temp_under, meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT_UNDER", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB_UNDER", meas_iusb)

        # Change the battery temperature to the normal value
        self._io_card.set_battery_temperature(self._batt_temp_normal)

        # Wait time between command
        if (time.time() - time5_before) > self._scheduled_timer_5:
            return (Global.FAILURE, "Test implementation incorrect " +
                    "(increase MSIC_REGISTER3 / scheduled_time constant)")
        self._wait_smart_time(time5_after, self._scheduled_timer_5)
        time.sleep(3)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "Battery temperature=%f : IBatt = %s %s , IUsb = %s %s " %
            (self._batt_temp_normal, meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT_NORMAL3", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB_NORMAL3", meas_iusb)

        # Connect USB PC/Host and DUT
        self.em_core_module.check_board_connection(only_reconnect=True)

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid1)
        self._meas_list.add_dict("MSIC_REGISTER1", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Calculate the interrupt numbers

        it1 = self.em_api.get_proc_interrupt("read", interrupt_pid1)
        it0 = self.em_api.get_proc_interrupt("read", interrupt_pid)
        interrupt = (it1 - it0, "none")
        self._meas_list.add("INTERRUPT1", interrupt)

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid2)
        self._meas_list.add_dict("MSIC_REGISTER2", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Calculate the interrupt numbers
        it2 = self.em_api.get_proc_interrupt("read", interrupt_pid2)
        interrupt = (it2 - it1, "none")
        self._meas_list.add("INTERRUPT2", interrupt)

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid3)
        self._meas_list.add_dict("MSIC_REGISTER3", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Calculate the interrupt numbers
        it3 = self.em_api.get_proc_interrupt("read", interrupt_pid3)
        interrupt = (it3 - it2, "none")
        self._meas_list.add("INTERRUPT3", interrupt)

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid4)
        self._meas_list.add_dict("MSIC_REGISTER4", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Calculate the interrupt numbers
        it4 = self.em_api.get_proc_interrupt("read", interrupt_pid4)
        interrupt = (it4 - it3, "none")
        self._meas_list.add("INTERRUPT4", interrupt)

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid5)
        self._meas_list.add_dict("MSIC_REGISTER5", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Calculate the interrupt numbers
        it5 = self.em_api.get_proc_interrupt("read", interrupt_pid5)
        interrupt = (it5 - it4, "none")
        self._meas_list.add("INTERRUPT5", interrupt)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
