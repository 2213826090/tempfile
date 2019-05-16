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
:summary: virtual interface with bluetooth network simulator
:since:23/03/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IBTNetSim(object):

    """
    Virtual interface for bluetooth network simulators
    """

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_eqt_id(self):
        """
        Gets equipment identification str
        :rtype: str
        :return: the identification str of the connected equipment.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_full_preset(self):
        """
        Resets all parameters to their default values
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_operating_mode(self, mode):
        """
        Sets the operating mode
        :type mode: str
        :param mode: the connection type to use:
            - "LINK"
            - "RFA"
            - "RFG"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_link_type(self, link_type):
        """
        Sets the link type
        :type link_type: str
        :param link_type: the connection type to use:
            - "ACL"
            - "SCO"
            - "TEST"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_test_mode_link_type(self, mode):
        """
        Sets the test mode link type
        :type mode: str
        :param mode: the test mode link to set:
            - "LOOP"
            - "TRAN"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def select_link_profile(self, profile):
        """
        Selects the link profile
        :type profile: str
        :param profile: the head set profile to set:
            - "NONE"
            - "HSP": head set profile
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_link_profile_state(self, state):
        """
        Sets the link profile state
        :type state: str
        :param profile: the head set profile state to set:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_loss_compensation_state(self, state):
        """
        Sets loss compensation state
        :type state: str
        :param profile: the loss compensation state to set:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_fixed_loss_compensation(self, loss_comp):
        """
        Sets fixed loss compensation
        :type loss_comp: double
        :param loss_comp: loss compensation is in dB. Range -50dB to +40dB.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_eut_power_class(self, power_class):
        """
        Sets equipment under test power class
        :type power_class: str
        :param power_class: the power class to set:
            - "PC1"
            - "PC2"
            - "PC3"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def reset_eut_bd_address(self):
        """
        Resets equipment under test BD addresses
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_link_inquiry_duration(self, duration):
        """
        Sets link inquiry duration
        :type duration: double
        :param duration: duration in seconds: range 1.28 to 61.44. Value is
        rounded to the nearest multiple of 1.28 (example: giving 2.3s sets
        2.56s).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def initiate_inquiry_procedure(self):
        """
        Initiate inquiry procedure
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def list_bd_address(self):
        """
        Lists BD addresses
        :rtype: str
        :return: the addresses of the bluetooth devices that have responded to
        the inquiry procedure.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ste_tx_power_level(self, power_level):
        """
        Sets Tx power level
        :type power_level: double
        :param power_level: the power level in dBm. Range -90.0 to 0.0,
        resolution 0.1.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ste_rx_power_level(self, power_level):
        """
        Sets Rx power level
        :type power_level: double
        :param power_level: the power level in dBm. Range -70.0 to 25,
        resolution 5.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_specific_tx_power_level(self, power_level):
        """
        Sets specific Tx power level
        :type power_level: double
        :param power_level: the power level in dBm. Range -95.0 to 0.0,
        resolution 0.1.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_specific_rx_power_level(self, power_level):
        """
        Sets specific Rx power level
        :type power_level: double
        :param power_level: the power level in dBm. Range -70 to 25,
        resolution 5.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def reset_link_configure_parameters(self):
        """
        Resets link configure parameters
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_frequency_hopping_state(self, state):
        """
        Sets frequency hopping state
        :type state: str
        :param profile: the frequency hopping state to set:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def reset_test_sequence(self):
        """
        Resets test sequence
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def add_test(self, name):
        """
        Adds a test to the test sequence
        :type name: str
        :param name: the test name to set:
            - "CFDR"
            - "ICFT"
            - "MCH"
            - "MIL"
            - "MSEN"
            - "OPOW"
            - "PCON"
            - "SSEN"
            - "BFP"
            - "DPEN"
            - "EMIL"
            - "ESEN"
            - "FSM"
            - "GTIM"
            - "RPOW"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def select_test(self, name, occurrence):
        """
        Selects a test in the test sequence
        :type name: str
        :param name: the test name to set. Possible values
            - {"CFDR"; "ICFT"; "MCH"; "MIL"; "MSEN"; "OPOW"; "PCON"; "SSEN";
               "BFP"; "DPEN"; "EMIL"; "ESEN"; "FSM"; "GTIM"; "RPOW"}
        :type occurrence: integer
        :param occurrence: the occurrence of the test name to select
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_test_sequence_loop_mode(self, mode):
        """
        Sets test sequence loop mode
        :type mode: str
        :param mode: the test name to set:
            - "SING"
            - "CONT"
            - "FIX"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_test_sequence_loop_number(self, nb_loop):
        """
        Sets test sequence loop number
        :type nb_loop: integer
        :param nb_loop: the number of test loops (1 to 99).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def run_test_sequence(self):
        """
        Runs test sequence
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def auto_disconnect_test_set(self, state):
        """
        Auto disconnects from test set
        :type state: str
        :param profile: the auto disconnect state to set:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disconnect_test_set(self):
        """
        Disconnects from test set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_impairment_state(self, state):
        """
        Sets impairment state
        :type state: str
        :param profile: the impairment state to set:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_bits_number(self, nb_bits):
        """
        Sets bits number
        :type nb_bits: unsigned long
        :param nb_bits: the number of returned payload bits to use for the test
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_output_power_average(self, channel="SUMMARY", meas_type="MAX",
                                 occurrence=1):
        """
        Gets the average output power
        :type channel: str
        :param channel: the channel to measure
            - "SUMMARY" => measure all channels
            - "LOW"
            - "MEDIUM"
            - "HIGH"
        :type meas_type: str
        :param meas_type: the type of measurement to operate
            - "MIN"
            - "MAX"
            - "AVERAGE"
        :type occurrence: integer
        :param occurrence: the occurrence of the test in the sequence (1 to 10)
        :rtype: double
        :return: the result of the measurement.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
