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
:summary: This file implements low speed IO UECmds for GPIOs
:since: 12/09/2014
:author: dpierrex
"""
from ErrorHandling.DeviceException import DeviceException


class IGPIO():
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

    def init_gpio(self, gpio_nbr=0, direction="output"):
        """
        initialise the specified gpio to the given direction and the value

        :type gpio_nbr: Integer
        :param bus_nbr: the gpio to init
        :type direction: String
        :param direction: direction to set as initial status
        :rtype tuple
        :return execution status and output of the command
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_gpio_direction(self, gpio_nbr=0, gpio_direction="output"):
        """
        set the I{direction} for the I{gpio_nbr}

        :type gpio_nbr: Integer
        :param gpio_nbr: the gpio to set
        :type gpio_direction: String
        :param gpio_direction: direction to set

        :rtype tuple
        :return execution status and output of the command
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_gpio_direction(self, gpio_nbr=0):
        """
        get the I{direction} for the I{gpio_nbr}

        :type gpio_nbr: Integer
        :param gpio_nbr: the gpio where get the direction
        :rtype: String
        :return: the direction string
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_gpio_value(self, gpio_nbr=0, gpio_value=0):
        """
        set the I{value} for the I{gpio_nbr}

        :type gpio_nbr: Integer
        :param gpio_nbr: the gpio to set
        :type gpio_value: Integer
        :param gpio_value: value to set

        :rtype tuple
        :return execution status and output of the command
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_gpio_value(self, gpio_nbr=0):
        """
        get the I{value} for the I{gpio_nbr}

        :type gpio_nbr: Integer
        :param gpio_nbr: the gpio where get the value
        :rtype: Integer
        :return: the value of the GPIO
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
