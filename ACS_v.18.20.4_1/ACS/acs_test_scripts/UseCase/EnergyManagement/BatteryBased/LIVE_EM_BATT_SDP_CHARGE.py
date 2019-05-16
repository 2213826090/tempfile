"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL AMPS
@summary: Energy Management battery charging.
@author: mmorchex
@since: 13/03/2014
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException


class LiveEmBattSdpCharge(UseCaseBase):

    """
    Live Energy Management Battery Sdp Charge.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read the unplug/plug timeout from testcase
        self._unplug_plug_timeout = int(self._tc_parameters.get_param_value("UNPLUG_PLUG_TIMEOUT"))

        # Read minimum charge variation from testcase
        self._min_capacity_variation = int(self._tc_parameters.get_param_value("CAPACITY_VARIATION"))

        # Read maximum battery capacity from testcase
        self.__max_capacity = int(self._tc_parameters.get_param_value("MAXIMUM_CAPACITY"))

        # Read charging timeout from testcase
        self._charging_timeout = int(self._tc_parameters.get_param_value("CHARGING_TIMEOUT"))

        # Get uecmd api
        self.__networking_api = self._device.get_uecmd("Networking")
        self.__em_api = self._device.get_uecmd("EnergyManagement")
        self.__phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self.__camera_api = self._device.get_uecmd("Camera")

        self.__initial_brightness_value = None
        self.__initial_sleep_timeout_value = None
#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # call the UseCaseBase Setup function
        UseCaseBase.set_up(self)

        # wake the screen
        self.__phonesystem_api.display_on()
        time.sleep(self._wait_btwn_cmd)

        # unlock the screen
        self.__phonesystem_api.set_phone_lock(0)
        time.sleep(self._wait_btwn_cmd)

        # store initial Sleep Timeout value
        self._initial_sleep_timeout_value = self.__phonesystem_api.get_screen_timeout()
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Checking if battery is as requested for the test: capacity < %s%%" %
                          str(self.__max_capacity))

        if int(self.__em_api.get_msic_registers()['BATTERY']['CAPACITY'][0]) > self.__max_capacity:
            # Discharge the DUT
            self._logger.info("Battery capacity is : %s, start discharging for %s seconds"
                     % (str(self.__em_api.get_msic_registers()['BATTERY']['CAPACITY'][0]),
                         str(self._unplug_plug_timeout)))

            # wake the screen
            self.__phonesystem_api.display_on()
            time.sleep(self._wait_btwn_cmd)

            # unlock the screen
            self.__phonesystem_api.set_phone_lock(0)
            time.sleep(self._wait_btwn_cmd)

            # start camera
            self.__camera_api.launch_camera("BACK")
            time.sleep(self._wait_btwn_cmd)

            # set camera flash mode to torch
            self.__camera_api.set_flash_mode("torch")
            time.sleep(self._wait_btwn_cmd)

            # set brightness to it's maximum value
            self.__phonesystem_api.set_display_brightness(100)
            time.sleep(self._wait_btwn_cmd)

            # set sleep timeout to it's maximum value
            self.__phonesystem_api.set_screen_timeout(1800)
            time.sleep(self._wait_btwn_cmd)

            # disconnect the board
            self._device.disconnect_board()
            time.sleep(self._wait_btwn_cmd)

            self._logger.info("Disconnecting the DUT  for %s seconds: UNPLUG" % str(self._unplug_plug_timeout))

            # UNPLUG
            self._io_card.usb_connector(False)

            # Wait for PLUG
            time.sleep(self._unplug_plug_timeout)

            self._logger.info("Connecting the DUT : PLUG")

            # PLUG
            self._io_card.usb_connector(True)
            time.sleep(self._wait_btwn_cmd)

            # connect the board
            self._device.connect_board()
            time.sleep(self._wait_btwn_cmd)

            # stop camera
            self.__camera_api.shutdown()
            time.sleep(self._wait_btwn_cmd)

            # set brightness mode to automatic
            self.__phonesystem_api. set_brightness_mode("automatic")
            time.sleep(self._wait_btwn_cmd)

            # set initial sleep timeout
            self.__phonesystem_api.set_screen_timeout(self._initial_sleep_timeout_value)
            time.sleep(self._wait_btwn_cmd)

            # lock the screen
            self.__phonesystem_api.set_phone_lock(1)
            time.sleep(self._wait_btwn_cmd)

            self._logger.info("Getting battery status after UNPLUG\PLUG")
            if int(self.__em_api.get_msic_registers()['BATTERY']["CAPACITY"][0]) > self.__max_capacity:
                error_text = "Capacity after discharging (%s%%) is higher than the maximum value required " \
                            "for the test (%s%%)" % (str(self.__em_api.get_msic_registers()['BATTERY']['CAPACITY'][0]),
                            str(self.__max_capacity))

                raise DeviceException(DeviceException.OPERATION_FAILED, error_text)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # call UseCaseBase Run function
        UseCaseBase.run_test(self)

        verdict = Global.SUCCESS
        message = "No errors"

        # check if battery state is charging
        if self.__em_api.get_battery_state() != "BATTERY_STATUS_CHARGING":
            verdict = Global.FAILURE
            message = "Battery status is not 'CHARGING'"

        return verdict, message
