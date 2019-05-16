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
:summary: Use Case GSM Voice Call Mobile Originated / Mobile release when
    Flight Mode is ON.
:since: 15/03/2013
:author: asebbanx
"""
# Module name follows ACS conventions
# pylint: disable=C0103

import time
import re

from UtilitiesFWK.Utilities import Global, int_to_bcd
from LAB_GSM_VC_BASE import LabGsmVcBase
from acs_test_scripts.Device.UECmd import UECmdTypes
from UtilitiesFWK.Utilities import str_to_bool
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.CommunicationUtilities import SerialHandler, ATCommandAnalyser
from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.ModemFlashing import ModemFlashing
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.TestEquipmentException import TestEquipmentException


class LabGsmFlightmodeEmVc(LabGsmVcBase):
    """
    Lab GSM Emergency Voice Call MO/MR with Flight Mode on.
    """

    EMERGENCY_NUMBER_PROPERTY_NAME = "ril.ecclist"
    """
    The name of the property corresponding to the
    list of emergency numbers.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call gsm voice call  base Init function
        LabGsmVcBase.__init__(self, tc_name, global_config)

        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

        # Read IS_EMERGENCY_NUMBER from test case xml file
        self._is_emergency_number_param = \
            str(self._tc_parameters.get_param_value("IS_EMERGENCY_NUMBER"))

        # Retrieve testcase parameters for AT proxy usage
        self._launch_mode = self._tc_parameters.get_param_value("LAUNCH_MODE")
        self._command = self._tc_parameters.get_param_value("COMMAND")
        self._index = self._tc_parameters.get_param_value("INDEX")
        self._expected_result = \
            self._tc_parameters.get_param_value("EXPECTED_RESULT")
        self._command_timeout = \
            self._tc_parameters.get_param_value("COMMAND_TIMEOUT")
        self._command_timeout = int(self._command_timeout)

        # Instantiate the PhoneSystem UE Command category
        self._phone_system = self._device.get_uecmd("PhoneSystem")

        # Instantiate the modem UE commands
        self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")

        # Get Serial Handler instance
        self._serial_handler = SerialHandler()

        # Initialize a boolean attribute corresponding to the
        # previously read parameter
        self._is_emergency_number = False

        # The attribute that will store the initial list
        # of emergency numbers.
        self._initial_emergency_numbers = None

        # Win COM port to be used
        self.at_proxy_com_port = None

        # Store the AT proxy connection availability
        self.modem_available = False

        # Store the verdict for AT@NVM interrogation
        self._at_inter_nvm_em_verdict = None

        # Store the response for AT@NVM interrogation
        self._at_inter_nvm_em = None

        # Store the verdict for setting EM in NVM via AT@NVM
        # it will be used in tear down to know if the initial values
        # should be restored or not in NVM:cust.emergency
        self._at_set_nvm_em = None

        # AT command that will be used for initial NVM:cust.emergency
        # values at the tear down.
        self.command_to_rest_em_in_nvm = None

        # AT command that will be used for setting new EM in  NVM:cust.emergency
        self.command_to_add_em_in_nvm = None

        # Store the initial list of RIL emergency number from UECmdTypes
        self._initial_ue_command_em_number = UECmdTypes.EMERGENCY_NUMBERS_LIST

        #Set up the parameters for the AT@NVM:cust.emergency
        # AT@NVM:cust.emergency[<index>]={<P1>,<P2>,<P3>,<P4>,<P5>,<P6>,<P7>,<P8>}

        # < index>, <index1>, <index2> - index of Emergency number to be stored
        self.index = self._index

        # <P1>,<P2>,<P3> BCD encoded ECC in decimal values
        # Make sure the provided Phone Number parameter is not present
        # in the list of emergency numbers
        self.em_no_length = len(self._phone_number)
        if self.em_no_length != 6:
            # Otherwise create an error message
            message = "Provided parameter PHONE_NUMBER [%s] should be 6 digits" % (
                                self._phone_number)
            # Raise an exception
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                    message)

        # <P1> (in BCD representation)
        self.p1_bcd = int_to_bcd(self._phone_number[0:2])

        # <P2> (in BCD representation)
        self.p2_bcd = int_to_bcd(self._phone_number[2:4])

        # <P2> (in BCD representation)
        self.p3_bcd = int_to_bcd(self._phone_number[4:6])

        #<P4> denotes whether ECC is valid when sim present or not
        # 0 - ECC is not valid when SIM present
        # 1 - ECC is valid when SIM is present
        self.p4 = 1

        #<P5> denotes ECC Category
        self.p5 = 1

        #<P6>, <P7>
        # MCC 2 bytes same as 3GPP PLMN format
        self.p6 = self._mcc
        self.p7 = self._mnc

        #<P8> reserved . give 0xff
        self.p8 = "255"
        self.command_to_add_em_in_nvm = "%s[%s]={%s, %s, %s, %s, %s, %s, %s, %s}" % (
                                                                               str(self._command),
                                                                               str(self._index),
                                                                               str(self.p1_bcd),
                                                                               str(self.p2_bcd),
                                                                               str(self.p3_bcd),
                                                                               str(self.p4),
                                                                               str(self.p5),
                                                                               str(self.p6),
                                                                               str(self.p7),
                                                                               str(self.p8))

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call use case base Setup function
        LabGsmVcBase.set_up(self)

        # Update the attribute indicated whether phone number
        # is an emergency number in ril.ecclist
        self._is_emergency_number = \
            str_to_bool(self._is_emergency_number_param)

        # Check the phone number, ensuring that it is not
        # a number corresponding to a "real" emergency number
        # on the LIVE network

        # Retrieve the list of emergency numbers from the DUT as str
        emergency_numbers_string = self._device.get_property_value(
            LabGsmFlightmodeEmVc.EMERGENCY_NUMBER_PROPERTY_NAME)

        # Split the str to a list
        emergency_numbers = emergency_numbers_string.split(",")

        # Make sure the provided Phone Number parameter is not present
        # in the list of emergency numbers
        if self._phone_number in emergency_numbers:
            # Otherwise create an error message
            message = "Provided parameter PHONE_NUMBER [%s] as a value  \
                    that corresponds to a real emergency number on the LIVE  \
                    network [list: (%s)]. The test will not be executed." % (
                    self._phone_number,
                    emergency_numbers_string)
            # Raise an exception
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Store the list of emergency numbers from RIL so that we can
        # restore it later
        self._initial_emergency_numbers = emergency_numbers_string

        # Update the list of RIL Emergency Numbers in UE Command categories
        # Make the 'addition' of 2 tuples by using a single-element tuples
        # using the (<element>,) notation.
        # This needs only to be done if the provided phone number shall
        # be considered to be an Emergency number.
        if self._is_emergency_number:
            UECmdTypes.EMERGENCY_NUMBERS_LIST = \
                self._initial_ue_command_em_number + (self._phone_number,)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable=E1101
        # Disable this pylint error due to Enum class VOICE_CALL_STATE

        # Call GSM VoiceCall base run_test function
        LabGsmVcBase.run_test(self)

        # Update the list of RIL Emergency Numbers on the DUT
        # This imperatively has to be done after settting the Flight Mode
        self._logger.info("Add Emergency Number in ril.ecclist list")
        if self._is_emergency_number:
            # Update the property holding the Emergency Numbers list.
            self._device.set_property_value(
                LabGsmFlightmodeEmVc.EMERGENCY_NUMBER_PROPERTY_NAME,
                self._phone_number)

        self._logger.info("Add Emergency Number in NVM:cust.emergency list "
                           "at index %s" % self._index)
        #Start up the proxy for sending AT commands
        self.at_proxy_com_port = self._modem_flashing_api.start_at_proxy_from_mos(
                                                                        int(self._launch_mode))
        #check serial connection
        self._serial_handler.set_data_analyser(
            ATCommandAnalyser(self._expected_result))
        self._serial_handler.set_default_timeout(self._command_timeout)
        self._logger.info("Connecting to the port " + str(self.at_proxy_com_port))

        # Connect to the at proxy
        self._serial_handler.connect(self.at_proxy_com_port)

        # Check that the modem is available
        modem_status = self._modem_flashing_api.ping_modem_from_serial(
            self._serial_handler.get_serial())

        if modem_status:
            self._logger.info("AT Proxy correctly respond to ping command")
            self.modem_available = True
        else:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "Connection to AT proxy failed")
        #Send AT@NVM to get initial parameters stored at index "self.index" for restore at
        #finalize
        self._logger.info("Read NVM:cust.emergency list at index %s" % self._index)
        inerrogate_at_nvm_command = self._command + "[" + \
                                    str(self.index) + "]" + "?"
        self._logger.info("Send AT command: %s" % inerrogate_at_nvm_command)
        self._at_inter_nvm_em_verdict, at_nvm_params_to_be_rest_raw =\
                self._serial_handler.send_at_command_and_get_result(inerrogate_at_nvm_command, self._command_timeout)
        at_nvm_params_to_be_rest = re.match("{*.*}", at_nvm_params_to_be_rest_raw)
        at_nvm_params_to_be_rest = at_nvm_params_to_be_rest.group(0)
        self._logger.info("Data read from NVM:cust.emergency list at index %s are: %s" % (
                          self._index, at_nvm_params_to_be_rest))
        self.command_to_rest_em_in_nvm = "%s[%s]=%s" % (str(self._command),
                                                        str(self._index),
                                                        str(at_nvm_params_to_be_rest))
        #Send AT@NVM to add an emergency number to NNV emergency number list
        self._logger.info("Add NVM:cust.emergency number \"%s\" "
                           "at index %s" % (self._phone_number, self._index))
        self._at_set_nvm_em, msg = self._serial_handler.send_at_command_and_get_result(
                                            self.command_to_add_em_in_nvm, self._command_timeout)
        if msg == "OK":
            self._logger.info("Response to AT command %s "
                               "is: %s" % (self.command_to_add_em_in_nvm, msg))
        else:
            raise DeviceException(DeviceException.OPERATION_SET_ERROR,
                                  "Response to AT command %s is: %s" % (
                                        self.command_to_add_em_in_nvm, msg))

        self._logger.info("Start the Emergency Call when DUT is in Flight Mode")
        # Release any previous call (Robustness)
        self._voicecall_api.release()

        # Enable flight mode
        self._networking_api.set_flight_mode("on")

        # Wait 30 seconds to be sure that modem is down before dialing
        time.sleep(30)

        # Wake the screen otherwise the Voice Call will not be established
        self._phone_system.wake_screen()
        self._phone_system.set_phone_lock(0)

        # Dial using PHONE_NUMBER parameter
        self._voicecall_api.dial(self._phone_number)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_2g.check_call_connected(self._call_setup_time,
                                                    blocking=False)

        # Wait for state "active" before callSetupTimeout seconds
        self._voicecall_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)

        # Check call is connected for CALL_DURATION seconds
        self._ns_voice_call_2g.is_voice_call_connected(self._call_duration)

        # Mobile Release call
        self._voicecall_api.release()

        # Check voice call state is "IDLE" (8960)
        self._ns_voice_call_2g.check_call_state("IDLE",
                                                self._call_setup_time,
                                                blocking=False)

        # pylint: disable=E1101
        # Disable this pylint error due to Enum class VOICE_CALL_STATE
        # Check voice call state is "no_call" (CDK)
        time.sleep(self._wait_btwn_cmd)
        self._voicecall_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)
        # pylint: enable=E1101

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Disable flight mode in the first place in any case
        self._networking_api.set_flight_mode("off")

        self._logger.info("Restore initial emergency numbers in ril.ecclist list")
        # Restore the list of emergency number in UE Command categories
        # if we changed it in the beginning of the Use Case
        if self._is_emergency_number:
            UECmdTypes.EMERGENCY_NUMBERS_LIST = self._initial_ue_command_em_number

            # Restore the list of emergency numbers
            self._device.set_property_value(
                LabGsmFlightmodeEmVc.EMERGENCY_NUMBER_PROPERTY_NAME,
                self._initial_emergency_numbers)

        # Call power measurement base tear_down function
        LabGsmVcBase.tear_down(self)
        if self.modem_available and (self._at_set_nvm_em is not None):
            self._logger.info("Restore initial emergency numbers in "
                               "NVM:cust.emergency list at index %s" % self._index)
            response, msg = self._serial_handler.send_at_command_and_get_result(self.command_to_rest_em_in_nvm,
                                                           self._command_timeout)
            if msg == "OK":
                self._logger.info("Response to AT command %s is: %s" % (
                                     self.command_to_rest_em_in_nvm, msg))
            else:
                raise DeviceException(DeviceException.OPERATION_SET_ERROR,
                                      "Response to AT command %s is: %s" % (
                                          self.command_to_rest_em_in_nvm, msg))
        #Stop AT Proxy
        self._logger.info("Stop AT Proxy")
        self._modem_flashing_api.stop_at_proxy_from_mos()

        #Close the serial connection
        self._logger.info("Close the serial connection")
        self._logger.info("Disconnecting from the port " +
                            str(self._serial_handler.get_port()))
        self._serial_handler.disconnect()

        return Global.SUCCESS, "No errors"
