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
:summary: This script defines the interface for sensor uecmd.
:since: 08/16/2010
:author: wchen61
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class ISensor():

    """
    Abstract class that defines the interface to be implemented
    by sensor uecmd.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
