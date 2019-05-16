"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW DEV
:summary: This file implements the Energy Management UEcmd for SensorEnabled device
:since: 22/01/2015
:author: jreynaux
"""
from datetime import datetime

from acs_test_scripts.Utilities.EMUtilities import EMConstant, DEGREE_UNITS, VOLTAGE_UNITS, NONE_UNIT
from acs_test_scripts.Device.UECmd.Interface.EnergyManagement.IEnergyManagement import IEnergyManagement
from acs_test_scripts.Device.UECmd.Imp.BareMetal.Common.Base import Base


class EnergyManagement(Base, IEnergyManagement):

    """
    :summary: EnergyManagement UEcommands operations for SensorEnabled device(s).
    """
    def __init__(self, device):
        """
        Constructor
        """
        Base.__init__(self, device)
        IEnergyManagement.__init__(self, device)
        self._logger = device.get_logger()
        self._device = device

    def get_msic_registers(self, behavior=None,
                           behavior_value=None):
        """
        Gets the msic registers.

        :type behavior: str
        :param behavior:
                        - "scheduled" to schedule the operation
                        - "read" to read the output from a scheduled operation
                        - None to act like normaly

        if behavior is equal to "scheduled":
        schedule the method to be launch after x seconds
                :type behavior_value: int
                :param behavior_value: time in second to wait
                                       before executing the method
                :rtype: str
                :return: pid of scheduled operation

        if behavior is equal to "read":
        read the output of a previous scheduled operation

                :type behavior_value: int
                :param behavior_value: pid

                :rtype: dict
                :return: a dictionary that contains 3 registers : [CHARGER], [BATTERY], [INTERRUPT]

        if behavior is equal to None:

                :type behavior_value: None
                :param behavior_value: not used

                :rtype: dict
                :return: a dictionary that contains 3 registers : [CHARGER], [BATTERY], [INTERRUPT]

        the elements of returned dictionary are like below :
             - (value, unit) = [register name][property]

        eg: (10V, unit) = [CHARGER][VOLTAGE]
        if there is no unit , unit value will be set to none.

        """
        max_battery_voltage = 4350
        msic_register = {}
        battery = {}

        time_stamp = datetime.now().strftime('%Y-%m-%d_%Hh%M.%S')
        msic_register[EMConstant.TIME_STAMP] = (time_stamp, NONE_UNIT)

        # command: battery_get_voltage
        # The voltage in millivolts : 3854 (0xf0e)
        #
        # command: system_get_batt_info
        # The current timestamp : 49 (0x31)
        # The periodic battery level in mV : 3854 (0xf0e)
        # The charging cycles number : 0 (0x0)

        # BATTERY
        #   CAPACITY = (int(value), "none")
        #   VOLTAGE = (float(value) / 1000000, "V")
        #   TEMP = (int(value) / 10, "DegreeCelsius")
        #   CHARGE_COUNTER = int(value)
        # CHARGER
        #   VOLTAGE = (float(value) / 1000, "V")
        #   CURRENT = (float(value) / 1000, "A")
        # TIME_STAMP = 2015-01-22_16h18.55

        # sample:
        # {'BATTERY': {'CAPACITY': (89, 'none'), 'VOLTAGE': (3.904, 'V'),
        # 'TEMP': (8, 'DegreeCelsius')}, 'TIME_STAMP': '2015-01-22-18h28.41'}

        batt_voltage = self._internal_exec(cmd="battery_get_voltage", broadcast=False)

        value = int(100 * float(batt_voltage[0]) / float(max_battery_voltage))
        battery[EMConstant.CAPACITY] = (int(value), NONE_UNIT)

        # TODO: Find way to retrieve proper battery status info on device
        # FULL, NOT CHARGING, CHARGING, DISCHARGING
        # battery[EMConstant.STATUS] = (str(value).upper(), NONE_UNIT)

        # TODO: Find way to retrieve proper battery current info on device
        # battery[EMConstant.IBATT_NOW] = (float(0) / 1000000, CURRENT_UNITS[0])

        battery[EMConstant.VOLTAGE] = (float(batt_voltage[0]) / 1000, VOLTAGE_UNITS[0])

        batt_temp = self._internal_exec(cmd="system_get_temperature", broadcast=False)
        battery[EMConstant.TEMP] = (int(batt_temp) / 10, DEGREE_UNITS[0])

        msic_register.update({"BATTERY": battery})
        return msic_register