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
from acs_test_scripts.Device.UECmd.Interface.LowSpeedIO.II2C import II2C
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
from ErrorHandling.DeviceException import DeviceException


class I2C(Base, II2C):
    """
    :summary: I2C UEcommands operations for Linux platforms
    """

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        II2C.__init__(self, device)
        self.__i2c_tool = "i2cdetect"

    def detect_i2c_devices(self, bus_nbr="1"):
        """
        Return a table of available devices on the given i2c bus

        :type bus_nbr: String
        :param bus_nbr: the bus number to scan

        :rtype String
        :return string representing I2C adresses usage
        """
        cmd = self.__i2c_tool + " -y -r " + bus_nbr
        status, output = self._internal_exec(cmd)
        if not status:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "Unable to detect i2c bus {0}\n{1}".format(bus_nbr, output))
        return output

    def check_i2c_address(self, bus_nbr="1", address="00", is_present=False, have_driver=False):
        """
        Verify the I2C I{adress} in order to check if a device is present and have a driver.

        :type bus_nbr: String
        :param bus_nbr: the bus number to scan

        :type address: String
        :param address: the address to verify

        :type is_present: bool
        :param is_present: True if a device should be present (patern = I{address})

        :type have_driver: bool
        :param have_driver: True if the device should be preempted by a driver ( patern = /UU/ )

        :rtype bool
        :return True if the conditions I{is_present} and I{have_driver} are verified for the given I{address}
        """
        result = False
        # extract the line and the column
        address_line = int(address[:-1], 16)
        address_column = int(address[-1:], 16)

        status, output = self.detect_i2c_devices(bus_nbr)
        # split the lines and remove the first one (column ID)
        lines = output.split("\n")[1:]
        line = lines[address_line]

        # splits the columns and remove the first one (line ID)
        columns = line.split(" ")[1:]

        i2c_device = columns[address_column]

        if is_present:
            if have_driver and "UU" in i2c_device:
                result = True
            elif not have_driver and address[-2:] in i2c_device:
                result = True
        elif not is_present and "--" in i2c_device:
            result = True

        return result
