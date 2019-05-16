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
:summary: This file implements the Sensor UEcmd for Android device
:since: 08/16/2011
:author: wchen61
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.System.ISensor import ISensor
from ErrorHandling.AcsConfigException import AcsConfigException

class Sensor(BaseV2, ISensor):

    """
    :summary: PhoneSystem UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """

        BaseV2.__init__(self, device)
        ISensor.__init__(self, device)

        self._logger = device.get_logger()

        self._target_class = "acscmd.sensor.SensorInfoModule"

    def check_sensor_info(self, sensor, information, value):
        """
        check the basic information of sensor

        :type sensor: str
        :param sensor: sensor to be test
        :type information: int or str
        :param information: information to be check

        :rtype: list or tuple
        :return: operation status & output log or in case of data check, the value returned by the sensor (x, y , z)
        """
        target_method = "checkSensorInformation"
        cmd = "--es name {0} --es information {1} --es value {2}".format(sensor, information, value)

        result = self._internal_exec_v2(self._target_class, target_method, cmd)

        if information.lower() == "data" and result["output"] is not None:
            return result["output"], float(result['X']), float(result['Y']), float(result['Z'])

        return result["output"] if result["output"] is not None else "No error"

    def get_sensor_data(self, sensor, information, duration=None):
        """
        get data from the specified sensor

        :type sensor: str
        :param sensor: sensor to be test
        :type information: int or str
        :param information: information to be check
        :type duration: int
        :param duration: listening/check data from sensor during specified time

        :rtype: list or tuple
        :return: operation status & output log or in case of data check, the value returned by the sensor (x, y , z)
        """
        target_method = "checkSensorInformation"
        cmd_timeout = None
        cmd = "--es name {0} --es information {1}".format(sensor, information)

        if not information.lower() in ["data", "listen"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "information {0} is not supported".format(information))
        if duration:
            # cmd timeout = sensor listening time + constant
            cmd_timeout = duration + 2
            time_in_millis = duration * 1000
            cmd += " --ei time {0}".format(time_in_millis)

        result = self._internal_exec_v2(target_class=self._target_class,
                                        target_method=target_method,
                                        cmd_args=cmd,
                                        timeout=cmd_timeout)

        return result["output"], float(result.get('X', 0)), float(result.get('Y', 0)), float(result.get('Z', 0))

