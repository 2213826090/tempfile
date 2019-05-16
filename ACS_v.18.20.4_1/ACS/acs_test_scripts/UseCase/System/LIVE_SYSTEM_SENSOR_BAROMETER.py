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
:summary: This file implements use case to check barometer
:since: 28/01/2014
:author: mmorchex
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveSystemSensorBarometer(UseCaseBase):

    """
    Class Live System sensor barometer.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get EXPECTED_VALUE from TC parameters
        self.__expected_value = self._tc_parameters.get_param_value("EXPECTED_VALUE", default_cast_type=int)

        # get FLEXIBILITY name from TC parameters
        self.__flexibility = self._tc_parameters.get_param_value("FLEXIBILITY", default_cast_type=int)

        # Get UECmdLayer
        self.__sensor_api = self._device.get_uecmd("Sensor")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check EXPECTED_VALUE value
        if not self.__expected_value:
            msg = "Please set EXPECTED_VALUE in TC."
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Check  LEXIBILITY value
        if self.__flexibility <= 0:
            msg = "Please update FLEXIBILITY in TC, should not be <= 0"
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

        # Check  barometer return
        output, x, y, z = self.__sensor_api.check_sensor_info("barometer", "data", "reserve")
        time.sleep(self._wait_btwn_cmd)
        barometer_value = x + y + z

        if barometer_value < self.__expected_value - self.__flexibility or\
                        barometer_value > self.__expected_value + self.__flexibility:
            verdict = Global.FAILURE
            msg = "barometer value does not mach the expected one, value : %s, expected : %s +- %s"\
                % (str(barometer_value), str(self.__expected_value), str(self.__flexibility))

        return (verdict, msg)
