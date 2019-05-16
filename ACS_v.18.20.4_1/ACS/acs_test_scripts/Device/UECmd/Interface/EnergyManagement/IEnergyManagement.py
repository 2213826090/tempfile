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
:summary: This script implements the interfaces for Energy Management features
:since: 19/10/2010
:author: vgombert
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IEnergyManagement(object):

    """
    Abstract class that defines the interface to be implemented
    by local connectivity handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def get_msic_registers(self, behavior, behavior_value):
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
             - (value,unit) = [register name][property]

        eg: (10V,unit) = [CHARGER][VOLTAGE]
        if there is no unit ,unit value will be set to none.

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_proc_interrupt(self, behavior=None, behavior_value=None):
        """
        Measures the number of interrupt which happen when charger type is plugged.

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "read" to read the output of a scheduled method

        :rtype: str
        :return: pid of scheduled operation

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: int
        :return: number of interrupts caught
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_thermal_sensor_info(self, behavior=None,
                                behavior_value=None):
        """
        Gets the sensor info (value , threshold , state) for each sensor.

        :type behavior: str
        :param behavior:
                        - "scheduled" to schedule the operation
                        - "read" to read the output from a scheduled operation
                        - None to act normalLy

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
                :return: a dictionary that contains thermal conf info : [THERMAL_CONF], [DTS], [BACKSKIN], [FRONTSKIN]

        if behavior is equal to None:

                :type behavior_value: None
                :param behavior_value: not used

                :rtype: dict
                :return: a dictionary that contains thermal conf info : [THERMAL_CONF], [DTS], [BACKSKIN], [FRONTSKIN]

        the elements in returned dict are like below :
             - [register name][property] = (value, unit)

        i.e [DTS] =  (35 , DegreeCelsius)
        if there is no unit, unit value will be set to none.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_fuel_gauging_monitoring_time_result(self):
        """
        run a script which schedule the battery
        fuel Gauging monitoring delta time

        :rtype: tuple
        :return: tuple of the delta time, and the time of the last battery level update
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_usb_charging(self, mode):
        """
        set the usb charging on or off.
        need to be refresh once in a while.

        :type mode: str or int
        :param mode: can be ('on') to enable
                            ('off') to disable
                            ('low', 'medium', 'high') to choose the value of the current

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_charger_level(self):
        """
        Gets the charger level info

        :rtype: int
        :return: The charger level (1=>100mA, 2=>500mA, 3=>1A, 4=>1.5A)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bcu_status(self):
        """
        Get the burst control unit status.

        :rtype: str
        :return: str with 2 digits as hexa like 0xFF
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_autolog_msic_registers(self):
        """
        Gets the msic registers auto logging result.

        :rtype: list of (dict, dict)
        :return: return a list containing tuples of (special stamp, msic dict)
            special stamp contains the following keys ("AUTOLOG_HOURS", "REBOOT", "REBOOT_TIME")
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_autolog_thermal_sensor_info(self):
        """
        Gets the thermal sensor auto logging result.

        :rtype: list of (dict, dict)
        :return: return a list containing tuples of (special stamp, thermal conf dict)
            special stamp contains the following keys ("AUTOLOG_HOURS", "REBOOT", "REBOOT_TIME")
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def build_fake_thermal_conf(self):
        """
        make a fake MID_thermal_em.conf to emule fake temperature
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def restore_thermal_file(self):
        """
        restore the real thermal configuration

        :rtype: boolean
        :return: True if the file has been restored, False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def restore_thermal_captor_temp(self, thermal_captor, temperature, timeout):
        """
        restore the thermal captor temperature

        :type  thermal_captor: str
        :param thermal_captor: captor file path

        :type  temperature: int
        :param temperature: temperature to restore.

        :rtype: Boolean
        :return: return true if the captor temp was succefully restored
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def thermal_test_set_temp(self, zone_id, temp):
        """
        Set the fake temperature to a specific zone

        :type  zone_id: int
        :param zone_id: number of the zone to set. Should be between 0 and 2
        :type  temp: int
        :param temp: Temperature value to set in milli degre Celcius
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bcu_activation_status(self):
        """
        Check if the BCU is running or not.

        :rtype: int
        :return: 1 if it is running, 0 otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bcu_interrupt(self):
        """
        Measures the number of interrupt related to the BCU.

        :rtype: int
        :return: number of interrupts caught
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def poll_multi_msic_registers(self, start_delay,
                                  duration, ack_delay):
        """
        start to poll msic registers for a given duration.

        :type start_delay: int
        :param start_delay: delay before starting to poll in seconds

        :type duration: int
        :param duration: polling duration in seconds

        :type ack_delay: int
        :param ack_delay: time between 2 polling in seconds

        :rtype: str
        :return: task id
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_multi_msic_registers(self, task_id):
        """
        Gets the values of msic registers for a given duration.

        :type task_id: str
        :param task_id: id of the task

        :rtype: list of dict
        :return: list of dict which contains msic info
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bcu_warn_level(self, voltage, warn_type):
        """
        Set the bcu warn A or B voltage level

        :type voltage: float
        :param voltage: the voltage level to set as warn level
        :type warn_type: str
        :param warn_type: the warn type to change; it can be "A" or "B" or "_crit"

        :rtype: none
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bcu_warn_level(self, warn_type):
        """
        get the bcu warn A or B voltage level

        :type warn_type: str
        :param warn_type: the warn type to change; it can be "A" or "B" or "_crit"

        :rtype: float
        :return: the voltage level to set as warn level
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_thermal_management(self, mode):
        """
        set or deactivate the thermal management

        :type mode: boolean
        :param mode: True to activate thermal management ,False to stop it
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def is_digital_battery_valid(self):
        """
        This UECmd permits to verify the battery validity
        Search the "researched_string" in the dmesg logs

        :type  researched_string: string
        :param researched_string: String in the dmesg logs who permits to find the battery information
        :rtype : bool
        :return : True is battery is value, false otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def load_thermal_file(self, load_from_orig=False):
        """
        Load the contents of thermal file from the DUT.
        This will allow you to modify the file several times without interacting with the DUT.

        @type load_from_orig:boolean
        @param load_from_orig: try to load thermal file from the original file.
                               the original file is the .orig generated or by default the normal file.

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def update_thermal_file(self):
        """
        push the thermal file on the DUT
        if no secure copy was done before, a secure copy will be do
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def modify_thermal_threshold(self, value, sensor="all", threshold="all", operation="="):
        """
        modify the sensor temperature threshold to a value depending of the operation you want to do.
        this command does not take care of the consistent of the value you put.
        For example, it will allow you to put a CRITICAL threshold at a value below ALERT threshold
        which does not make any sense.

        :type value: int
        :param value: the temperature value we will use to modify the threshold

        :type operation: str
        :param operation: the type of operation to apply on the threshold, to be chosen among + - / * =
                          leave empty to perform a = operation by default

        :type sensor: str
        :param sensor: the sensor to modify, by default will target all sensors

        :type threshold: str
        :param threshold: the threshold level to modify for a given sensor, by default will target all threshold

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_battery_state(self):
        """
        This URCmd permits to check the battery state
        :rtype: str
        :return: the battery state
        """

        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
