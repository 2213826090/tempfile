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
:summary: EM - New NFT Tests based on thermal measurement
Permits to check the phone temperature with a thermal camera
author: jvauchex
:since: 30/07/2013
"""
import time
import os
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.TelephonyModule3G import TelephonyModule3g


class LabEmBattNftThermalMeasurement(EmUsecaseBase):

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
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # Read parameters from TC parameters
        self.__test_process = \
            self._tc_parameters.get_param_value("TEST_PROCESS")
        self.__test_length = \
            int(self._tc_parameters.get_param_value("TEST_LENGTH"))
        self.__shutdown_time = \
            int(self._tc_parameters.get_param_value("SHUTDOWN_TIME"))

        self._logger.info("Process of the test : %s" % self.__test_process)
        self._logger.info("Length of the test : %d" % self.__test_length)
        self._logger.info("Max Capacity of the Battery : %s" % self.em_core_module.batt_max_capacity)
        self._logger.info("Time of Charge : %d" % self.em_core_module.charge_time)

        # toggle param depending of what was put in LOAD var
        if "VIDEO" in self.__test_process:
            self.__multimedia_file = \
                self._tc_parameters.get_param_value("VIDEO_PATH", "")
            self.__volume = \
                int(self._tc_parameters.get_param_value("VOLUME", "100"))
            self._logger.info("Multimedia File : %s" % self.__multimedia_file)
            self._logger.info("Volume of the Video: %d" % self.__volume)

            self._system_api = self._device.get_uecmd("System")
            self._video_api = self._device.get_uecmd("Video")
            self._multimedia_path = self._device.multimedia_path

        if "CALL3G" in self.__test_process:
            self.__telmodule = TelephonyModule3g()

        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

        self.__batt_temp = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # toggle techno ON depending of what was put in LOAD var
        if "CALL3G" in self.__test_process:
            # setup simulator
            self.__telmodule.set_up_eq()
            # wait for registration
            self.__telmodule.register_to_network()

        # Disable lock screen
        self.phonesystem_api.set_phone_screen_lock_on("off")

        # Config the brightness
        # set the screen on and brightness to 100%
        self.phonesystem_api.set_screen_timeout(3600)
        # deactivate set auto brightness
        self.phonesystem_api.set_brightness_mode("manual")
        # set display brightness to max value
        self.phonesystem_api.set_display_brightness(100)
        # Wake up the screen
        self.phonesystem_api.wake_screen()

        # Check the battery capacity
        msic_result = self.update_battery_info()
        self.__batt_temp = msic_result["BATTERY"]["TEMP"][0]
        self._logger.info("Before the test, Battery temperature : %d" % self.__batt_temp)

        # Check the battery capacity
        if ((self.em_core_module.batt_max_capacity != "FULL" and str(self.em_core_module.batt_max_capacity).isdigit() and
                                        self.batt_capacity < int(self.em_core_module.batt_max_capacity)) and
                                        self.em_core_module.batt_max_capacity != "SKIP"):
            self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity,
                                  self.em_core_module.charge_time,
                                  self.__em_meas_tab)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        Check the end of the main battery charge
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Add Measurement
        self._meas_list.add("COLD BATTERY", self.__batt_temp, "Degrees")

        # Launch IFWI
        if "WIFI" in self.__test_process:
            self.networking_api.set_wifi_power("on")

        if "BLUETOOTH" in self.__test_process:
            self.bt_api.set_bt_power(1)

        # Launch Video
        if "VIDEO" in self.__test_process:
            self._system_api.adjust_specified_stream_volume("Media", self.__volume)
            self._video_api.play(self._multimedia_path + self.__multimedia_file, loop=True)

        if "CALL3G" in self.__test_process:
            # establish voice call
            self.__telmodule.establish_call()

        # Plug the Charger
        if "SDP" in self.__test_process:
            self._device.disconnect_board()
            self._io_card.simulate_insertion(self._io_card.SDP)
        elif "DCP" in self.__test_process:
            self._device.disconnect_board()
            self._io_card.simulate_insertion(self._io_card.DCP)
        elif "CDP" in self.__test_process:
            self._device.disconnect_board()
            self._io_card.simulate_insertion(self._io_card.CDP)
        elif "NOCHGR" in self.__test_process:
            self._device.disconnect_board()
            self._io_card.usb_connector(False)
            self._io_card.ac_charger_connector(False)
        elif "ACCHGR" in self.__test_process:
            self._device.disconnect_board()
            self._io_card.simulate_insertion(self._io_card.AC_CHGR)

        # Wait
        self._logger.info("Waiting %d seconds" % self.__test_length)
        time.sleep(self.__test_length)

        # Unplug the Charger
        if "SDP" in self.__test_process:
            self._io_card.usb_host_pc_connector(True)
            self._device.connect_board()
        elif "DCP" in self.__test_process:
            self._io_card.usb_host_pc_connector(True)
            self._device.connect_board()
        elif "CDP" in self.__test_process:
            self._io_card.usb_host_pc_connector(True)
            self._device.connect_board()
        elif "NOCHGR" in self.__test_process:
            self._io_card.usb_host_pc_connector(True)
            self._device.connect_board()
        elif "ACCHGR" in self.__test_process:
            self._io_card.usb_host_pc_connector(True)
            self._device.connect_board()

        # Measure the battery temperature
        msic_result = self.update_battery_info()
        self.__batt_temp = msic_result["BATTERY"]["TEMP"][0]
        self._logger.info("Before the test, Battery temperature : %d" % self.__batt_temp)

        # Add the value
        self._meas_list.add("HOT BATTERY", self.__batt_temp, "Degrees")

        # Shutdown all app
        if "WIFI" in self.__test_process:
            self.networking_api.set_wifi_power("off")

        if "BLUETOOTH" in self.__test_process:
            self.bt_api.set_bt_power(0)

        if "CALL3G" in self.__test_process:
            self.__telmodule.release_eq()

        if "VIDEO" in self.__test_process:
            self._video_api.stop()

        # Shutdown the board
        self._device.disconnect_board()
        self._device.soft_shutdown_cmd()
        self._logger.info("Waiting %d seconds" % self.__shutdown_time)
        time.sleep(self.__shutdown_time)
        self._device.switch_on()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # activate set auto brightness
        self.phonesystem_api.set_brightness_mode("automatic")

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
