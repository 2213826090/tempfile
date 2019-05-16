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

:organization: INTEL AMPS
:summary: Energy Management battery discharge after an unplug/plug.
:author: mmorchex
:since: 19/11/2013
"""
import time
import datetime
from datetime import timedelta
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException


class LiveEmBattSdpDischarge(UseCaseBase):

    """
    Live Energy Management unplug plug.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read the unplug/plug timeout from testcase
        self._unplug_plug_timeout = int(self._tc_parameters.get_param_value("UNPLUG_PLUG_TIMEOUT"))

        # Read maximum charge variantion from testcase
        self._max_charge_variation = int(self._tc_parameters.get_param_value("CHARGE_VARIATION"))

        # Read minimum battery capacity ftom testcase
        self._min_capacity = int(self._tc_parameters.get_param_value("MINIMUM_CAPCITY"))

        # Read charging timeout from testcase
        self._charging_timeout = int(self._tc_parameters.get_param_value("CHARGING_TIMEOUT"))

        # Get uecmd api
        self.networking_api = self._device.get_uecmd("Networking")
        self.em_api = self._device.get_uecmd("EnergyManagement")
        self.localconnectivity_api = self._device.get_uecmd("LocalConnectivity")
        self.phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self.camera_api = self._device.get_uecmd("Camera")

        self._initial_brightness_value = None
        self._initial_seleep_timeout_value = None
        self._charge_duration = 60

        self._verdict = Global.SUCCESS
        self._message = "No errors"

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # call the UseCaseBase Setup function
        UseCaseBase.set_up(self)

        self._logger.info("Checking if battery is as requested for the test: capacity > %s%%"\
                           % str(self._min_capacity))

        # check initial battery's capacity
        duration = datetime.datetime.now() + timedelta(seconds=self._charging_timeout)

        # charging DUT if it's capacity is under the minimum capacity's value
        # required for test and specified in the testcase.
        while int(self.em_api.get_msic_registers()['BATTERY']['CAPACITY'][0]) < self._min_capacity:
            self._logger.info("Charging DUT for %s ms" % str(duration - datetime.datetime.now()))
            time.sleep(self._charge_duration)

            if datetime.datetime.now() > duration:
                error_text = "Capacity after charging (%s%%) is lower than the minimum value required for the test"\
                "(%s%%)" % (str(self.em_api.get_msic_registers()['BATTERY']['CAPACITY'][0]), str(self._min_capacity))
                raise DeviceException(DeviceException.OPERATION_FAILED, error_text)

        # store initial Sleep Timeout value
        self._initial_seleep_timeout_value = self.phonesystem_api.get_screen_timeout()
        time.sleep(self._wait_btwn_cmd)

        # wake the screen
        self.phonesystem_api.display_on()
        time.sleep(self._wait_btwn_cmd)

        # unlock the screen
        self.phonesystem_api.set_phone_lock(0)
        time.sleep(self._wait_btwn_cmd)

        # start camera
        self.camera_api.launch_camera("BACK")
        time.sleep(self._wait_btwn_cmd)

        # set camera flash mode to torch
        self.camera_api.set_flash_mode("torch")
        time.sleep(self._wait_btwn_cmd)

        # set brightness to it's maximum value
        self.phonesystem_api.set_display_brightness(100)
        time.sleep(self._wait_btwn_cmd)

        # set sleep timeout to it's maximum value
        self.phonesystem_api.set_screen_timeout(1800)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # call UseCaseBase Run function
        UseCaseBase.run_test(self)

        self._logger.info("Getting battery status before UNPLUG\PLUG")
        # get initial battery informations
        initial_battery_info = self.em_api.get_msic_registers()
        time.sleep(self._wait_btwn_cmd)

        # get initial battery voltage
        initial_battery_voltage = float(initial_battery_info['BATTERY']['VOLTAGE_OCV'][0])

        # get initial battery capacity
        initial_battery_capacity = int(initial_battery_info['BATTERY']['CAPACITY'][0])

        self._logger.info("Disconnecting the DUT for %s seconds : UNPLUG" % str(self._unplug_plug_timeout))

        # disconnect the board
        self._device.disconnect_board()
        time.sleep(self._wait_btwn_cmd)

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

        self._logger.info("Getting battery status after UNPLUG\PLUG")

        # get battery info after UNPLUG/PLUG
        battery_info = self.em_api.get_msic_registers()
        time.sleep(self._wait_btwn_cmd)

        # get battery voltage after PLUG\UNPLUG
        battery_voltage = float(battery_info['BATTERY']['VOLTAGE_OCV'][0])

        # get battery capacity after PLUG\UNPLUG
        battery_capacity = int(battery_info['BATTERY']['CAPACITY'][0])

        # check voltage variation
        if battery_voltage >= initial_battery_voltage:
            self._verdict = Global.FAILURE
            self._message = "Test Fails : battery voltage did not change"
            return self._verdict, self._message

        # check capacity variation
        if initial_battery_capacity - battery_capacity > self._max_charge_variation:
            self._verdict = Global.FAILURE
            self._message = "Test Fails : charge variation is too high : > %s" % str(self._max_charge_variation)

        return self._verdict, self._message

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        UseCaseBase.tear_down(self)

        # stop camera
        self.camera_api.shutdown()
        time.sleep(self._wait_btwn_cmd)

        # set brightness mode to automatic
        self.phonesystem_api.set_brightness_mode("automatic")
        time.sleep(self._wait_btwn_cmd)

        # set initial sleep timeout
        self.phonesystem_api.set_screen_timeout(self._initial_seleep_timeout_value)
        time.sleep(self._wait_btwn_cmd)

        # lock the screen
        self.phonesystem_api.set_phone_lock(1)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"
