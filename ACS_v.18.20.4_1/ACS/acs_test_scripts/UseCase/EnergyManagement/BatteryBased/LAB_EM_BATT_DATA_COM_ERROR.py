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
:summary: EM Host communication error, Check the battery current in case of normal communication and host communication error
:author: jvauchex
:since: 12/02/2013
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmBattDataComError(EmUsecaseBase):

    """
    Live Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read parameters from TC parameters
        self._usb_compliancy = \
            self._tc_parameters.get_param_value("USB_COMPLIANCY", "FALSE")
        self._schedule_time = \
            int(self._tc_parameters.get_param_value("SCHEDULE_TIME", 30))
        self._sample_rate = \
            int(self._tc_parameters.get_param_value("SAMPLE_RATE", 10))
        self._logger.info("Activation of USB COMPLIANCY : %s" % self._usb_compliancy)
        self._logger.info("Time of scheduling : %d" % self._schedule_time)
        self._logger.info("Time of sample rate : %d" % self._sample_rate)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_DATA_COM_ERROR", self._tc_parameters.get_params_as_dict(),
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

        # check the battery capacity
        msic_result = self.em_api.get_msic_registers()
        capacity_level = msic_result["BATTERY"]["CAPACITY"][0]
        capacity_level = int(capacity_level)

        # If capacity is full => discharge to 95%
        if capacity_level > 95:
            self._logger.info("The battery capacity is full !")
            self.em_core_module.monitor_discharging(95, 120)
        elif capacity_level < 5:
            self._logger.info("Charge the battery !")
            self.em_core_module.monitor_charging(10, 120)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Initialize the variables
        cpt_exec = 0.0
        current_average = 0.0

        # set the screen off
        if self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # Check the msic register with a valid communication => Recuperation of CURRENT_AVG
        while cpt_exec < 3:
            msic_result = self.em_api.get_msic_registers()

            # Check the Battery current AVG
            value_valid_com = msic_result["BATTERY"]["CURRENT_AVG"][0]
            self._logger.info("The value of the current is equal to %s" % value_valid_com)

            # MAJ cpt and value
            current_value = float(value_valid_com)
            self._logger.debug("Exec : %f" % current_value)
            current_average += current_value
            self._logger.debug("Average = %f" % current_average)
            cpt_exec += 1
            time.sleep(10)

        # Check the current average
        current_average /= 3.0
        self._logger.info("The current average is equal to %f" % current_average)

        # store value in dict for later comparison
        self._meas_list.add("CURRENT_VALID_COM", current_average, "A")

        # schedule the print of the msic register
        pid_result1 = self.em_api.get_msic_registers("scheduled", self._schedule_time)
        pid_result2 = self.em_api.get_msic_registers("scheduled", self._schedule_time + self._sample_rate)
        pid_result3 = self.em_api.get_msic_registers("scheduled", self._schedule_time + (self._sample_rate * 2))

        # disconnect the board
        self._device.disconnect_board()

        # Check the msic register with an invalid communication
        self._io_card.usb_connector_data(False)
        time.sleep(self._sample_rate * 6)
        self._io_card.usb_connector_data(True)

        # Connect the board
        self._device.connect_board()

        # Check the msic register in order to recup the CURRENT_AVG
        msic_result1 = self.em_api.get_msic_registers("read", pid_result1)
        msic_result2 = self.em_api.get_msic_registers("read", pid_result2)
        msic_result3 = self.em_api.get_msic_registers("read", pid_result3)

        # Check the battery current AVG
        value_valid_com1 = msic_result1["BATTERY"]["CURRENT_AVG"][0]
        value_valid_com2 = msic_result2["BATTERY"]["CURRENT_AVG"][0]
        value_valid_com3 = msic_result3["BATTERY"]["CURRENT_AVG"][0]

        # Convert str to float
        current_value1 = float(value_valid_com1)
        current_value2 = float(value_valid_com2)
        current_value3 = float(value_valid_com3)

        # Calculate the current average
        current_tot_average = current_value1 + current_value2 + current_value3
        self._logger.info("The total current in case of host communication error : %f A" % current_tot_average)
        self._meas_list.add("CURRENT_ERROR_COM", current_tot_average, "A")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
        self._em_meas_verdict.judge()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
