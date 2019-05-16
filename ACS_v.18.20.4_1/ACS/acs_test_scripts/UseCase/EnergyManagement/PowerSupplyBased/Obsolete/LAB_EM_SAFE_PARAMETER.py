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
:summary: Energy Management Safe Parameter Use case
:author: dbatutx
:since: 04/04/2012
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabEmSafeParameter(EmUsecaseBase):

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
        self._vbatt_full = float(self._tc_parameters.get_param_value("VBATT_FULL"))

        # Initialize flash attributes
        self._flash_fail = False
        self._flash_file_path = None
        self._flash_timeout = None
        self._flash_max_retry = None
        self._setup_path = None
        self._wpa_certificate_file = None

        # read the path of the fake build with fake IFWI
        if self._tc_parameters.get_param_value("FAKE_BUILD_PATH") != "":
            self._fake_build_path = self._tc_parameters.get_param_value(
                "FAKE_BUILD_PATH")
        else:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                                   "missing fake build for flash")

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_SAFE_PARAMETER", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = True (ON)
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        # - USBChargerType = USB_HOST_PC
        self.em_core_module.io_card_init_state["USBChargerType"] = "DCP"
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature = BATTERY_TEMPERATURE
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:

        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

        # VBATT and VUSB power supplies
        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")
        self.em_core_module.pwrs_vusb = self._em.get_power_supply("USB")

        # - Delay to wait for scheduled commands
        self._scheduled_timer_1 = \
            int(self._em_targets["MSIC_REGISTER_SIGNED"]["scheduled_time"])
        self._scheduled_timer_2 = \
            int(self._em_targets["MSIC_REGISTER_UNSIGNED"]["scheduled_time"])

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "Charger DCP plugged : IBatt = %s %s , IUsb = %s %s " %
            (meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT0", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB0", meas_iusb)

        # call check process of ibatt and misc
        self.check_process("SIGNED", self.em_core_module.vbatt)

        # Call flash sequence
        try:
            self._device.flash(self._fake_build_path, self._flash_timeout, self._flash_max_retry)

        except AcsBaseException as ex:
            self._error.Code = ex.get_error_code()
            self._error.Msg = ex.get_error_message()
            return self._error.Code, self._error.Msg

        time.sleep(250)

        # Setup embedded - DEPREACTED CODE TO BE REPLACED
        # self._device.setup_embedded(self._setup_path,
        #                             self._wpa_certificate_file)
        time.sleep(10)

        # set the vbatt voltage
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._vbatt_full,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # reboot the board
        self.em_core_module.reboot_board()

        # call check process of ibatt and misc
        self.check_process("UNSIGNED", self._vbatt_full)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def check_process(self, safety, vbatt_charge):
        """
        Execute the check procedure

        :type safety: str
        :param safety: type of safety build: UNSIGNED or SIGNED
        """
        # set the vbatt voltage at normal voltage to enable usb com
        self.em_core_module.pwrs_vbatt.set_current_voltage(self.em_core_module.vbatt,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Connect USB PC/Host and DUT
        self.em_core_module.check_board_connection(only_reconnect=True)

        # start interrupt measurement after current connection stop
        interrupt_pid = self.em_api.get_proc_interrupt("scheduled", 0)

        # Set API to start in x seconds to read and store all MSIC registers
        time1_before = time.time()
        pid1 = self.em_api.get_msic_registers("scheduled", self._scheduled_timer_1)
        interrupt_pid1 = self.em_api.get_proc_interrupt("scheduled",
                                                         self._scheduled_timer_1)
        time1_after = time.time()

        # Close connection
        self._device.disconnect_board()

        # set the vbatt voltage at the test charge voltage
        self.em_core_module.pwrs_vbatt.set_current_voltage(vbatt_charge,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Select usb device DCP and connect usb
        self._io_card.simulate_insertion(self._io_card.DCP)

        # Wait time between command
        if (time.time() - time1_before) > self._scheduled_timer_1:
            return Global.FAILURE, "Test implementation incorrect (increase MSIC_REGISTER1 / scheduled_time constant)"
        self._wait_smart_time(time1_after, self._scheduled_timer_1)
        time.sleep(3)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "%s case / Charger DCP plugged : IBatt = %s %s , IUsb = %s %s " %
            (safety, meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT_%s" % safety, meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB_%s" % safety, meas_iusb)

        # set the vbatt voltage at normal voltage to enable usb com
        self.em_core_module.pwrs_vbatt.set_current_voltage(self.em_core_module.vbatt,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Connect USB PC/Host and DUT
        self.em_core_module.check_board_connection(only_reconnect=True)

        self._logger.debug("        get misc register")
        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid1)
        self._meas_list.add_dict("MSIC_REGISTER_%s" % safety, msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Calculate the interrupt numbers
        it1 = self.em_api.get_proc_interrupt("read", interrupt_pid1)
        it0 = self.em_api.get_proc_interrupt("read", interrupt_pid)
        interrupt = (it1 - it0, "none")
        self._meas_list.add("INTERRUPT_%s" % safety, interrupt)
