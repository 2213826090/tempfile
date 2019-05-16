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
:summary: This file is the process of LiveSystemSensor
:since: 08/08/2011
:author: wave
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveSystemSensorInfo(UseCaseBase):

    """
    Class live system sensor.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        # Get TC Parameters
        self._sensor = self._tc_parameters.get_param_value("SENSOR")
        self._information = self._tc_parameters.get_param_value("INFORMATION")
        self._value = self._tc_parameters.get_param_value("VALUE")

        # Get UECmdLayer
        self._sensor_api = self._device.get_uecmd("Sensor")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Sensor information test ...")

        result = self._sensor_api.check_sensor_info(self._sensor, self._information, self._value)

        return Global.SUCCESS, result

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Quit sensor information test...")

        return self._error.Code, "No errors"
