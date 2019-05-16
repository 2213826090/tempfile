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
:summary: This file implements use case to check accelerometer after setting vibrator on
:since: 28/01/2014
:author: mmorchex
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveSystemSensorAccelerometer(UseCaseBase):

    """
    Class Live System accelerometer.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get MINIMUM_VARIATION name from TC parameters
        self.__minimal_variation = self._tc_parameters.get_param_value("MINIMUM_VARIATION", default_cast_type=float)

        # Vibration duration in  milliseconds
        self.__vibration_duration = 15000

        # Checking accelerometer value interval in milliseconds
        self.__checking_interval = 2500

        # Get UECmdLayer
        self.__vibrator_api = self._device.get_uecmd("Vibrator")
        self.__sensor_api = self._device.get_uecmd("Sensor")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check MINIMUM_VARIATION value
        if self.__minimal_variation <= 0:
            msg = "Please update MINIMUM_VARIATION in TC, should not be <= 0"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        verdict = Global.SUCCESS
        msg = "No errors"

        accelerometer_variation = 0

        # Check accelerometer return
        output, x, y, z = self.__sensor_api.check_sensor_info("accelerometer", "data", "reserve")
        time.sleep(self._wait_btwn_cmd)
        initial_accelerometer_value = abs(x) + abs(y) + abs(z)

        # Start the vibrator
        self.__vibrator_api.start_vibrator()
        time.sleep(0.5)

        # Check accelerometer value where vibrator is on
        # Check is done N times following a checking interval
        for iter in range(self.__vibration_duration/self.__checking_interval):
            output, x, y, z = self.__sensor_api.check_sensor_info("accelerometer", "data", "reserve")
            instant_accelerometer_value = abs(x) + abs(y) + abs(z)
            # Store the highest value of accelerometer variation
            if instant_accelerometer_value > accelerometer_variation:
                accelerometer_variation = instant_accelerometer_value
            time.sleep(self._wait_btwn_cmd)

        # Stop the vibrator
        self.__vibrator_api.stop_vibrator()
        time.sleep(0.5)

        # Check if accelerometer variation is as expected
        info_msg = "initial accelerometer value : %s" % str(initial_accelerometer_value)
        self.get_logger().info(info_msg)
        info_msg = "maximum accelerometer value : %s" % str(accelerometer_variation)
        self.get_logger().info(info_msg)

        if float(accelerometer_variation) - float(initial_accelerometer_value) < float(self.__minimal_variation):
            verdict = Global.FAILURE
            msg = "Accelerometer variation is not as expected, initial value : %s, variation : %s, flexibility : %s" \
                % (str(initial_accelerometer_value), str(accelerometer_variation), str(self.__minimal_variation))
        else:
            msg = "Accelerometer variation is as expected, initial value : %s, variation : %s, flexibility : %s" \
                % (str(initial_accelerometer_value), str(accelerometer_variation), str(self.__minimal_variation))

        return (verdict, msg)
