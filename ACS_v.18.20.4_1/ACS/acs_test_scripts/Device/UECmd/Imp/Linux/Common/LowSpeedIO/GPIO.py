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
from acs_test_scripts.Device.UECmd.Interface.LowSpeedIO.IGPIO import IGPIO
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base


class GPIO(Base, IGPIO):
    """
    :summary: GPIO UEcommands operations for Linux platforms
    """

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        IGPIO.__init__(self, device)
        self.__init_cmd = "/home/root/init_gpio.sh -o {0} -d {1}"
        self.__set_cmd = "/home/root/set_gpio.sh -o {0} -v {1}"
        self.__get_cmd = "/home/root/read_gpio.sh -o {0}"

    def init_gpio(self, gpio_nbr=0, direction="output"):
        """
        initialise the specified gpio to the given direction and the value

        :type gpio_nbr: int
        :param bus_nbr: the gpio to init
        :type direction: str
        :param direction: direction to set as initial status
        :rtype tuple
        :return execution status and output of the command
        """
        cmd = self.__init_cmd.format(gpio_nbr, direction)
        return self._internal_exec(cmd)


    def set_gpio_direction(self, gpio_nbr=0, gpio_direction="output"):
        """
        set the I{direction} for the I{gpio_nbr}

        :type gpio_nbr: int
        :param gpio_nbr: the gpio to set
        :type gpio_direction: str
        :param gpio_direction: direction to set

        :rtype tuple
        :return execution status and output of the command
        """
        cmd = self.__init_cmd.format(gpio_nbr, direction)
        return self._internal_exec(cmd)

    def set_gpio_value(self, gpio_nbr=0, gpio_value=0):
        """
        set the I{value} for the I{gpio_nbr}

        :type gpio_nbr: int
        :param gpio_nbr: the gpio to set
        :type gpio_value: int
        :param gpio_value: value to set

        :rtype tuple
        :return execution status and output of the command
        """
        cmd = self.__set_cmd.format(gpio_nbr, gpio_value)
        return self._internal_exec(cmd)

    def get_gpio_value(self, gpio_nbr=0):
        """
        get the I{value} for the I{gpio_nbr}

        :type gpio_nbr: int
        :param gpio_nbr: the gpio where get the value
        :rtype: int
        :return: the value of the GPIO
        """
        cmd = self.__get_cmd.format(gpio_nbr)
        return self._internal_exec(cmd)
