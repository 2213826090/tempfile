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
:summary: Energy Management battery temperature measurement Usecase for battery bench
:author: vgombert
:since: 17/04/2012 (April)
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.ThermalChamberModule import ThermalChamberModule
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.EMUtilities as EMUtil
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattTempMeasCycle(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read READ_INFO_TIMEOUT from test case xml file
        self._read_info_timeout = \
            int(self._tc_parameters.get_param_value("READ_INFO_TIMEOUT"))

        # Read CHARGER_TYPE from TC parameters
        self._charger_type = \
            str(self._tc_parameters.get_param_value("CHARGER_TYPE"))

        # Read all temperatures from TC parameters
        temperatures = str(self._tc_parameters.get_param_value("TEMPERATURE_CYCLE"))

        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_TEMP_MEAS_CYCLE", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # get temperature chamber equipment
        self.tc_module = ThermalChamberModule(self._ambient_temperature)
        self.tc_module.set_test_type("SPECIFIC")

        # Initialize EM  xml object
        # measurement file
        name = os.path.join(self._saving_directory,
                            "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(name)

        self._temperature_cycle = []
        for temperature in temperatures.split(","):
            self._temperature_cycle.append(int(temperature))

#-----------------------------------------------------------------------

    def set_up(self):
        """
        set environment temperature to ambient.
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)
        # init temperature chamber
        self.tc_module.get_eq().init()
        # turn ON regulation
        self.tc_module.get_eq().set_regulation(True)

        if self._charger_type != "NONE":
            # Check if charger type is supported by your io card
            if self._charger_type not in self._io_card.SUPPORTED_DEVICE_TYPE:
                txt = "io card does not support cable type %s " % self._charger_type
                self._logger.info(txt)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        set temperature to wanted one then wait a while with wanted usb plugged or unplugged.
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # init var
        capability_nb = 0

        # do measurement for each temperature
        for temperature in self._temperature_cycle:
            # set temperature to wanted one
            self.tc_module.get_eq().set_temperature(temperature)
            if not self.tc_module.get_eq().wait_for_temperature(temperature):
                msg = "timeout exceed for changing temperature"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # increment capabilities number
            capability_nb += 1

            # start interrupt measurement after current connection stop
            start_interrupt = self.em_api.get_proc_interrupt()

            # schedule measurement
            msic_pid = self.em_api.get_msic_registers("scheduled",
                                                       self._read_info_timeout)
            thermal_pid = self.em_api.get_thermal_sensor_info("scheduled",
                                                               self._read_info_timeout)
            interrupt_pid = self.em_api.get_proc_interrupt("scheduled",
                                                            self._read_info_timeout)

            self._device.disconnect_board()
            # connect chosen cable
            if self._charger_type in [self._io_card.SDP, self._io_card.DCP,
                                      self._io_card.CDP, self._io_card.AC_CHGR]:
                msg = "Plug %s during %s seconds" % (self._charger_type,
                                                     self._read_info_timeout)
                # Connect charger
                self._io_card.simulate_insertion(self._charger_type)

            elif self._charger_type == "NONE":
                msg = "Unplug USB during %s seconds" % self._read_info_timeout
                self._io_card.usb_host_pc_connector(False)

            # wait x min
            self._logger.info(msg)
            time.sleep(self._read_info_timeout)

            if self._charger_type == self._io_card.AC_CHGR:
                # Disconnect AC CHARGER
                self._io_card.ac_charger_connector(False)

            # connect USB HOST
            self._io_card.usb_connector(False)
            self._io_card.usb_host_pc_connector(True)

            # wait x seconds
            time.sleep(self.usb_sleep)
            # connect board
            self._device.connect_board()
            # check adb connection
            self.em_core_module.check_board_connection()

            msic_reg = self.em_api.get_msic_registers("read", msic_pid)
            msic_batt = msic_reg["BATTERY"]

            self.batt_capacity = msic_batt["CAPACITY"][0]
            self.batt_voltage = msic_batt["VOLTAGE"][0]

            # get thermal info
            thermal_conf = self.em_api.get_thermal_sensor_info("read",
                                                                thermal_pid)
            # get interrupt info
            stop_interrupt = self.em_api.get_proc_interrupt("read",
                                                             interrupt_pid)
            # Calculate the interrupt numbers
            interrupt = stop_interrupt - start_interrupt

            # store result on xml
            self.__em_meas_tab.add_dict_measurement(msic_reg)
            self.__em_meas_tab.add_dict_measurement(thermal_conf)
            self.__em_meas_tab.add_measurement(
                [self.get_time_tuple(),
                 (self._em_cst.COMMENTS, "Temperature test at %s" % str(temperature)),
                 ("INTERRUPT", interrupt)])
            self.__em_meas_tab.switch_to_next_meas()

            # Read Platform OS and compare MSIC registers with expected values
            self._meas_list.add_dict("MSIC_REGISTER" + str(capability_nb),
                                     msic_reg, msic_reg["TIME_STAMP"][0])
            self._meas_list.add("INTERRUPT" + str(capability_nb),
                               (interrupt, "none"))
            self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
            self._meas_list.clean()
            # stop all daemonized ACS process
            self.phonesystem_api.clean_daemon_files()

        # compute verdict
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.save_data_report_file())

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # release equipment
        if self.tc_module is not None:
            self.tc_module.release_eq()

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        # stop all daemonized ACS process
        self.phonesystem_api.clean_daemon_files()

        return Global.SUCCESS, "No errors"
