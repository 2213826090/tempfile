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

:organization: INTEL NDG SW
:summary: This file implements low speed IO UECmds
:since: 01/08/2014
:author: dpierrex
"""
from ErrorHandling.DeviceException import DeviceException


class II2C():
    """
    Abstract class that defines the interface to be implemented
    by io sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def detect_i2c_devices(self, bus_nbr="1"):
        """
        Return a table of available devices on the given i2c bus

        :type bus_nbr: String
        :param bus_nbr: the bus number to scan

        :rtype String
        :return string representing I2C adresses usage
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_i2c_address(self, bus_nbr="1", address="00", is_present=False, have_driver=False):
        """
        Verify the I2C I{adress} in order to check if a device is present and have a driver.

        :type bus_nbr: String
        :param bus_nbr: the bus number to scan

        :type address: String
        :param address: the address to verify

        :type is_present: bool
        :param is_present: True if a device should be present

        :type have_driver: bool
        :param have_driver: True if the device should be preempted by a driver ( patern /UU/ )

        :rtype bool
        :return True if the conditions I{is_present} and I{have_driver} are verified for the given I{address}
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
