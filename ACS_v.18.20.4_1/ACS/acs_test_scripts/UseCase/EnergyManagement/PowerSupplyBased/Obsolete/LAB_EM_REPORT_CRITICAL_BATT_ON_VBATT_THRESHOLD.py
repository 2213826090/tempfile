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

:organization: SII on behalf PSI QA & Validation
:summary: EM - test device when capacity reach 0 percent
:author: jortetx
:since: 03/12/2013
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
import time


class LabEmReportCriticalBattOnVbattThreshold(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call the LAB_EM_USE_CASE init method
        EmUsecaseBase.__init__(self, tc_name, global_config)
        # Read load to activate
        self.__load = self._tc_parameters.get_param_value("LOAD")
        # Read the critical treshold
        self.__critical_threshold = self._tc_parameters.get_param_value("CRITICAL_THRESHOLD", default_cast_type=float)
        # Read the accuracy for crtical threshold
        self.__accuracy = self._tc_parameters.get_param_value("ACCURACY", default_cast_type=float)
        # Calculate the real threshold
        self.__calculateThreshold = self.__critical_threshold * (1.0 - self.__accuracy / 100)
        # Activate Loadmodule instance
        self.__load_module = LoadModule()
        # Add list of LOAD to the loadmodule
        self.__load_module.add_load(self.__load)
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.USB_HOST_PC
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # get target and initialise measurements
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_REPORT_CRITICAL_BATT_ON_VBATT_THRESHOLD", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)
        # charger to use
        self.__charger_type = "SDP"
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)
        # Load module to have a high current drawn
        self.__load_module.start_load()
        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------
    def run_test_body(self):
        """
        Execute the test
        """
        # time between two measure of voltage OCV
        timebetweenmeasure = 30
        vocv = 4.0
        capacity = 100
        # After setup board is ALIVE
        board_state = "ALIVE"
        # Read the starting battery voltage
        batteryvoltage = self.em_core_module.vbatt
        # test timeout to avoid infinite loop
        test_timeout = 3600 * 20.0
        time_elapsed = 0.0
        start_time = time.time()
        # waiting still voltage ocv decrease to critical treshold

        while time_elapsed < test_timeout:
            # test if board alive
            devicemode = self._device.get_boot_mode()
            if devicemode == "MOS":
                # get register value
                msic_register = self.em_api.get_msic_registers()
                vocv = float(msic_register["BATTERY"]["VOLTAGE_OCV"][0])
                capacity = int(msic_register["BATTERY"]["CAPACITY"][0])
                self._logger.info("Voltage OCV = %f" % vocv)
                self._logger.info("Capacity = %d" % capacity)

                # if capacity is near 0 the test is FAILED
                if capacity <= 1:
                    board_state = "CAPA_OFF"
                    errormsg = "Device stop due to capacity"
                    self._logger.error(errormsg)
                    raise AcsConfigException(AcsConfigException.OPERATION_FAILED, errormsg)

                # manage decrease of vocv
                if vocv - batteryvoltage <= 0.02:
                    # when ocv voltage reach battery voltage
                    # Battery voltage is decreased of 0.05V
                    batteryvoltage -= 0.05
                    self.em_core_module.set_battery_voltage(batteryvoltage)

                # if vocv is under vcritt the test is FALSE
                vdiff = vocv - self.__calculateThreshold
                if  vdiff < 0:
                    # board_state still initialized with ALIVE no need to overwrite
                    errormsg = "Voltage OCV is below critical threshold"
                    self._logger.error(errormsg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, errormsg)

                elif (vdiff <= 0.05) and (timebetweenmeasure != 1):
                    # the vocv comes near critical threshold value
                    # time between measurement have to be decrease
                    timebetweenmeasure = 1

            elif devicemode == "COS":
                board_state = "VBATT_OFF"
                break

            # wait
            time.sleep(timebetweenmeasure)
            time_elapsed = time.time() - start_time

        if time_elapsed >= test_timeout:
            errormsg = "Test timeout reached"
            self._logger.error(errormsg)
            raise AcsConfigException(AcsConfigException.TIMEOUT_REACHED, errormsg)

        # generate em verdict
        self._meas_list.add("BOARD_STATE", board_state, "")
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge()
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)
        # desactivate load
        self.__load_module.stop_load()

        return (Global.SUCCESS, "No errors")
