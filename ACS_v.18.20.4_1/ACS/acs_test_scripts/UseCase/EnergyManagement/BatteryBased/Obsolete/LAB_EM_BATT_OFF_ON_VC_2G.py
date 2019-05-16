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
:summary: Energy Management Battery Monitor - OFF ON cycle 2G
:author: vgombert
:since: 11/14/2011
"""
import time
import os
import numpy

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase

from UtilitiesFWK.Utilities import Global, str_to_bool
import acs_test_scripts.Utilities.EMUtilities as EMUtil

from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattOffOnVc2g(EmUsecaseBase):

    """
    Lab Energy Management base class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read Band from test case xml file (str)
        self._cell_band = str(self._tc_parameters.get_param_value("CELL_BAND"))
        # Read CELL_SERVICE from test case xml file
        self._cell_service = \
            str(self._tc_parameters.get_param_value("CELL_SERVICE"))
        # Read TCH_ARFCN from test case xml file
        self._tch_arfcn = int(self._tc_parameters.get_param_value("TCH_ARFCN"))
        # Read UPLINK_CHANNEL from test case xml file
        self._uplink_channel = int(self._tc_parameters.get_param_value("UPLINK_CHANNEL"))
        # Read CELL_POWER from test case xml file
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read DATA_CALL from test case xml file
        self._data_call = str_to_bool(self._tc_parameters.get_param_value("DATA_CALL"))
        # Read DATA_CALL_MODE from test case xml file
        self._data_call_mode = str(self._tc_parameters.get_param_value("DATA_CALL_MODE"))

        # Read REBOOT_MODE from test case xml file
        self._reboot_mode = str(self._tc_parameters.get_param_value("REBOOT_MODE"))
        # Init fuel gauging param
        self.em_core_module.init_fg_param()

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Create cellular network simulator and retrieve 2G APIs
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_2g = self._ns.get_cell_2g()
        self._data_2g = self._ns_2g.get_data()

        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(meas_file_name)
        # result file
        ocv_file_name = os.path.join(self._saving_directory,
                                     "Energy_Management_result_report.xml")
        self._em_meas_result = EMUtil.XMLMeasurementFile(ocv_file_name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        self.__em_meas_tab.enable_global_meas(name, self._name)
        # init variables
        self._ocv_list = []
        self._slots_conf = []

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # set usb charging on
        self.em_api.set_usb_charging("on")

        # Configure CMU
        # Connect to cellular network simulator
        self._ns.init()
        # Perform Full Preset
        self._ns.perform_full_preset()

        # Set cell band using CELL_BAND parameter
        self._ns_2g.set_band(self._cell_band)

        # Set cell off
        self._ns_2g.set_cell_off()

        # Set cell service using CELL_SERVICE parameter
        self._ns_2g.set_cell_service(self._cell_service)

        # Set Traffic Channel Arfcn using TCH_ARFCN parameter
        self._ns_2g.set_tch_arfcn(self._tch_arfcn)

        # Set cell power using CELL_POWER parameter
        self._ns_2g.set_cell_power(self._cell_power)

        # set data uplink channel
        self._data_2g.set_data_channel(self._uplink_channel)

        # configure cellular network burst slots
        self._slots_conf = self.em_core_module.configure_slot()

        # Set main timeslot to 3 and set slot configs
        self._data_2g.set_custom_multislot_config(
            3, self._slots_conf["DL_STATE"],
            self._slots_conf["DL_VALUE"],
            self._slots_conf["UL_STATE"],
            self._slots_conf["UL_VALUE"])

        # init capacity
        msic = self.update_battery_info()

        # get capability targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_OFF_ON_VC_2G", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # init verdict value
        if self._em_targets["MSIC_REGISTER_PLUG.BATTERY.CHARGE_NOW"] is not None:
            EMUtil.update_conf(
                self._em_targets["MSIC_REGISTER_PLUG.BATTERY.CHARGE_NOW"],
                "hi_lim", msic["BATTERY"]["CHARGE_FULL_DESIGN"][0], "=")

        # init verdict value
        if self.tc_module is not None:
            if self._em_targets["THERMAL_MSIC_REGISTER_PLUG.BATTERY.TEMP"] is not None:
                EMUtil.update_conf(
                    self._em_targets["THERMAL_MSIC_REGISTER_PLUG.BATTERY.TEMP"],
                    ["lo_lim", "hi_lim"], self._tct, "*")

        # Charge battery
        self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time,
                              self.__em_meas_tab)

        # set CMU cell phone ON
        self._data_2g.set_data_cell_on()

        # register phone
        self._data_2g.data_register_dut(None,
                                        self._registration_timeout)

        # Check registration status on DUT
        self.modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)

        self._logger.info("Start OFF ON cycle until battery capacity is at %s%%" %
                          self.em_core_module.batt_min_capacity)
        # init failed measurement counter
        measurement_fail = 0

        # OFF ON loop
        while self.batt_capacity > self.em_core_module.batt_min_capacity:

            # reboot board
            self.em_core_module.reboot_board(self._reboot_mode)
            # check board connection
            self.em_core_module.check_board_connection(1, False)

            try:
                # stop to charge through usb
                self.em_api.set_usb_charging("off")
                time.sleep(10)

                # Perform a call in data packet TestMode
                if self._data_call:
                    try:
                        self._data_2g.data_call(self._data_call_mode)
                    except TestEquipmentException as error:
                        self._logger.error(error)
                else:
                    # establish a call
                    self.voicecall_api.dial("OOOO12121121")

                # get msic registers value after booting
                msic_reg = self.update_battery_info()
                self.__em_meas_tab.add_dict_measurement(msic_reg)

                # get thermal info
                thermal_conf = self.em_api.get_thermal_sensor_info()
                self.__em_meas_tab.add_dict_measurement(thermal_conf)

                # compute OCV
                self._ocv_list.append(self.batt_voltage)
                ocv_avg = numpy.average(self._ocv_list)
                ocv_stdev = numpy.std(self._ocv_list)

                # create XML files
                self._em_meas_result.add_measurement(
                    [("BATT_CAPACITY_PERCENT", self.batt_capacity),
                     ("OCV_AVG_V", ocv_avg),
                     ("OCV_STDEV_V", ocv_stdev),
                     ("OCV_V", self.batt_voltage)])

                # store result on xml
                # switch to next meas
                self._em_meas_result.switch_to_next_meas()

                self._meas_list.add_dict("MSIC_REGISTER_PLUG", msic_reg)

                # check thermal capabilities only if thermal chamber is used
                if self.tc_module is not None:
                    # Store various information
                    self.__em_meas_tab.add_measurement(
                        [self.tc_module.feed_meas_report()])
                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_PLUG", msic_reg)
                    self._meas_list.add_dict("THERMAL_CONF_PLUG", thermal_conf)

                # get the call state
                call_state = self._cell_service
                if self._data_call:
                    # Check data call state
                    try:
                        self._data_2g.check_data_call_connected(10)
                        call_state += "_DATA_CALL_CONNECTED"
                    except TestEquipmentException as error:
                        self._logger.error(error)
                        call_state += "_DATA_CALL_DISCONNECTED"
                else:
                    # Check cs call state
                    call_state += "_CS_CALL_" + str(self.voicecall_api.get_state())

                # Store various information
                self.__em_meas_tab.add_measurement(
                    [("REGISTRATION",
                      self.modem_api.get_network_registration_status()),
                     ("VOICE_CALL", call_state)])

                measurement_fail = 0

            except AcsBaseException as e:
                # Just log error , board will be rebooted in next iteration
                self._logger.error("fail to get measurement: %s" % str(e))
                measurement_fail += 1

                # stop the usecase if measurement fail several times.
                if measurement_fail >= self._consecutive_meas_error:
                    tmp_txt = "Measurement failed after %s times, stop usecase" % \
                        self._consecutive_meas_error
                    self._logger.error(tmp_txt)
                    if self.batt_voltage > self.vbatt_mos_shutdown or \
                            self.batt_voltage == -1:
                        raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

                    else:
                        self._logger.info("battery must be empty, stop usecase")
                        break
            finally:
                # generate em verdict
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
                self._em_meas_verdict.judge(ignore_blocked_tc=True)
                self._meas_list.clean()

                # Store various information
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     ("COMMENTS", "OFF ON cycle discharge")])

                # switch to next meas
                self.__em_meas_tab.switch_to_next_meas()

                # reinitialize reboot variable
                self.phone_as_reboot = False

            self._logger.info(
                "waiting board discharge during  %s seconds" %
                self.em_core_module.discharge_time)
            time.sleep(self.em_core_module.discharge_time)

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.save_data_report_file())

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # retrieve measurement from test
        self.__em_meas_tab.generate_global_file()

        # set CMU cell phone OFF
        self._data_2g.set_data_cell_off()

        # Disconnect from the equipment (Network simulator)
        self._ns.release()

        if self.is_board_and_acs_ok():
            self.em_api.set_usb_charging("on")

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
