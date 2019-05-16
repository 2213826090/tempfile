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
:summary: EM - Connect a device with the OTG and check if the board stays in low power mode.
:author: jvauchex
:since: 25/02/2013
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import update_conf
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmBattOtgInsertion(EmUsecaseBase):

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
        self._execution_time = \
            int(self._tc_parameters.get_param_value("EXECUTION_TIME", 300))
        self._sleep_residency_passrate = \
            int(self._tc_parameters.get_param_value("SLEEP_RESIDENCY_PASSRATE", 60))
        self._logger.info("Time of sample rate : %d" % self._execution_time)
        self._logger.info("Passrate of sleep residency : %d" % self._sleep_residency_passrate)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_OTG_INSERTION", self._tc_parameters.get_params_as_dict(),
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

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        Utilisation of sleep residency :
        This UEcmd use the kernel register in order to check the status time of the board
        In the run test, the registers are cleared, the test is launched and the registers are read.
        We check the sleep passrate, and we determinate if the passrate is ok or not.
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)
        update_conf(self._em_targets["OTG_SLEEP_RESIDENCY"],
                    "lo_lim", self._sleep_residency_passrate, "=")

        # set the screen off
        if self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # ***** First, check the sleep residency when the USB is disconnected *****
        # disconnect the board
        self._device.disconnect_board()
        self.phonesystem_api.reset_sleep_residency()
        self._io_card.usb_connector(False)
        time.sleep(90)

        # Check the msic register with the USB SDP is disconnected
        time.sleep(self._execution_time)

        # Connect the board
        self._io_card.simulate_insertion("SDP")
        self._device.connect_board()

        # Read the S3 counter in order to find the sleep time of the board
        no_usb_sleep_residency = self.phonesystem_api.get_sleep_residency("s3")
        self._logger.info("The value of idle state for NO USB = %f" % no_usb_sleep_residency)

        # Add the value
        self._meas_list.add("NO_USB_SLEEP_RESIDENCY", no_usb_sleep_residency, "%")

        # ***** Second, check the sleep residency when the OTG is connected *****

        # set the screen off
        if self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # disconnect the board
        self._device.disconnect_board()
        self.phonesystem_api.reset_sleep_residency()
        self._io_card.usb_connector(False)
        time.sleep(90)

        # Check the msic register with the OTG connected
        self._io_card.simulate_insertion("OTG")
        time.sleep(self._execution_time)
        self._io_card.simulate_insertion("SDP")

        # Connect the board
        self._device.connect_board()
        otg_sleep_residency = self.phonesystem_api.get_sleep_residency("s0i3")
        self._logger.info("The value of idle state for OTG = %f" % otg_sleep_residency)

        # Add the value
        self._meas_list.add("OTG_SLEEP_RESIDENCY", otg_sleep_residency, "%")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets,
                                           clean_meas_list=True)
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
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
