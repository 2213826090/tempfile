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
:summary: virtual interface with power supplies
:since:24/03/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IPowerSupply(object):

    """
    Virtual interface with power supplies
    """

    def init(self):
        """
        Initializes the power supply. After that the power supply is ready
        to use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases the power supply and all resources allocated for its use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_full_preset(self):
        """
        Resets all parameters to their default values
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_max_current(self, max_cur, port):
        """
        This function sets the maximum current allowed of power supply
        :param max_cur: maximum current allowed
        :param port: port number on which the current level has to be set
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_current_voltage(self, voltage, port):
        """
        This function sets the current voltage of power supply
        :param voltage: current voltage to set
        :param port: port number on which the voltage level has to be set
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_current_meas(self, port, meas_type):
        """
        This function measures the current
        :type port: integer
        :param port: the port on which measurement has to be done
        :type meas_type: str
        :param meas_type: the type of measurement to do. Possible values:
            - "DC"    : dc current
            - "ACDC"  : ac+dc rms current
            - "HIGH"  : high current
            - "LOW"   : low current
            - "MAX"   : maximum current
            - "MIN"   : minimum current
        :rtype: float
        :return: the result of the measurement
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_current_meas_average(self, port, meas_type, iteration):
        """
        This function measures the current
        :type port: integer
        :param port: the port on which measurement has to be done
        :type meas_type: str
        :param meas_type: the type of measurement to do. Possible values:
            - "DC"    : dc current
            - "ACDC"  : ac+dc rms current
            - "HIGH"  : high current
            - "LOW"   : low current
            - "MAX"   : maximum current
            - "MIN"   : minimum current
        :type iteration: int
        :param duration: number of iteration for the average computing

        :rtype: float
        :return: the result of the measurement
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def power_on(self):
        """
        This function powers on the power supply
        :raise: raises TestEquipmentException in case of failure
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def power_off(self):
        """
        This function powers off the power supply
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_eq_id(self):
        """
        Gets the ID of the equipment
        :rtype: str
        :return: the identification str of the equipment
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_display_channel(self, channel):
        """
        Sets the displayed channel
        :type channel: integer
        :param channel: the channel to display
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_sense_protect(self, state):
        """
        Sets the open sense lead detection state
        :type state: str
        :param state: the state to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_over_current_protection_state(self, state):
        """
        Sets overcurrent protection state
        :type state: str
        :param state: the state to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_resistance(self, level):
        """
        Sets the output resistance of the power supply
        :type level: float
        :param level: the resistance to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_voltage_protection_level(self, level):
        """
        Sets the overvoltage protection level
        :type level: float
        :param level: the level to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_output_state(self, state, output):
        """
        Sets the output state
        :type state: str
        :param state: the state to set ("ON" | "OFF")
        :type output: integer
        :param output: the output to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_curr_sense_detector(self, detector):
        """
        Sets the sense detector
        :type detector: str
        :param detector: the detector to set ("ACDC" | "DC")
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_coupling_mode(self, mode):
        """
        Sets the coupling mode
        :type mode: str
        :param mode: the mode to set ("ALL" | "NONE")
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_compensation_mode(self, mode):
        """
        Sets the compensation mode
        :type mode: str
        :param mode: the mode to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_basic_parameters(self):
        """
        Configuration of basic parameters of the power supply
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def power_cycle(self):
        """
        Toggles this power supply:
            - sets the power OFF
            - wait a few seconds
            - sets the power ON
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def reset_protection(self):
        """
        write a GPIB Command in order to reset the protection state.

        :rtype: str
        :return: The response from agilent for protection state
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_protection_state(self):
        """
        query a GPIB Command in order to get alim status.

        :rtype: str
        :return: The response from agilent for protection state
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_protection_delay(self, delay):
        """
        write a GPIB Command in order to set the protection delay.

        :type delay: float
        :param delay: the delay to set

        :return: none
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def query_command(self, command):
        """
        query a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: The response from agilent for the command
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_voltage_range(self, rang):
        """
        Sets the voltage range to LOW (8V-5A) or HIGH (20V-2.5A)

        :type rang: string
        :param rang: can be low or LOW or high or High
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)