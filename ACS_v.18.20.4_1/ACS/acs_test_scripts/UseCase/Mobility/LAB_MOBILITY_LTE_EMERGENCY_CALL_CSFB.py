"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: Test specific cases of emergency calls during a circuit switched fallback from
LTE to UTRAN/2G
:since: 02/07/2014
:author: mariussx
"""

import re
import time
import sys

from UtilitiesFWK.Utilities import Global, int_to_bcd, str_to_bool
from UseCase.Mobility.LAB_MOBILITY_LTE_CSFB import LabMobilityLteCsfb
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.NetworkingUtilities import ping
from acs_test_scripts.Utilities.CommunicationUtilities import SerialHandler, ATCommandAnalyser
from acs_test_scripts.Device.UECmd import UECmdTypes
from ErrorHandling.TestEquipmentException import TestEquipmentException


class LabMobilityLteEmergencyCallCsfb(LabMobilityLteCsfb):
    """
    Test specific cases of calls during a circuit switched fallback from LTE
    to UTRAN:
    - Network rejecting the call
    - Cancel voice call before establishment
    """

    EMERGENCY_NUMBER_PROPERTY_NAME = "ril.ecclist"

    def __init__(self, tc_name, global_config):
        # Call LabMobilityLteCsfb init method
        LabMobilityLteCsfb.__init__(self, tc_name, global_config)

        self.__rejection_cause = self._tc_parameters.get_param_value(
                "REJECTION_CAUSE", "not defined", str)

        self._flight_mode = self._tc_parameters.get_param_value(
                "FLIGHT_MODE", "not defined", str)
        self._flight_mode = str_to_bool(self._flight_mode)

        # Instantiate the PhoneSystem UE Command category
        self._phone_system = self._device.get_uecmd("PhoneSystem")

        # Instantiate the modem UE commands
        self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")

        # Get Serial Handler instance
        self._serial_handler = SerialHandler()

        # Initialize launch mode for AT proxy
        self._launch_mode = 2

        # Initialize the AT command timeout
        self._command_timeout = 10

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
        self._index = 0
        self.index = self._index

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
        self.p6 = self._ns_lte_mcc
        self.p7 = self._ns_lte_mnc

        #<P8> reserved . give 0xff

        #this AT command writes Emergency Number to NVM AT@NVM
        self._command = "AT@NVM:cust.emergency"
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
    def set_up(self):
        """
        Set up the test configuration
        """
        status, msg = LabMobilityLteCsfb.set_up(self)

        # Check REJECTION_CAUSE parameter
        if self._loss_coverage_type == "REJECTION":
            if self.__rejection_cause in ("not defined", "", None):
                return Global.FAILURE, "Rejection cause parameter is not correct (%s). Please update your test-case"\
                       % self.__rejection_cause

        # Cancel call specific setup
        if self._loss_coverage_type == "CANCELLATION":
            if self._ns_3gsm_cell_tech == "2G":
                # Set call setup timeout to 100 seconds
                self._ns_3gsm_vc.set_mt_originate_call_timeout(100)

        # Check the phone number, ensuring that it is not
        # a number corresponding to a "real" emergency number
        # on the LIVE network

        # Retrieve the list of emergency numbers from the DUT as str
        emergency_numbers_string = self._device.get_property_value(
            self.EMERGENCY_NUMBER_PROPERTY_NAME)

        # Split the str to a list
        emergency_numbers = emergency_numbers_string.split(",")

        # Log the initial EM Number List
        self._logger.info("The initial emergency number list is: (%s)" %(emergency_numbers_string))

        # Make sure the provided Phone Number parameter is not present
        # in the list of emergency numbers
        if self._phone_number in emergency_numbers and self._phone_number != "123456":
            # Otherwise create an error message
            message = "Provided parameter PHONE_NUMBER [%s] as a value  \
                    that corresponds to a real emergency number on the LIVE  \
                    network [list: (%s)]. The test will not be executed." % (
                    self._phone_number,
                    emergency_numbers_string)
            # No need to raise an exception
            self._logger.info(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Store the list of emergency numbers from RIL so that we can
        # restore it later
        self._initial_emergency_numbers = emergency_numbers_string

        # Update the list of RIL Emergency Numbers in UE Command categories
        # Make the 'addition' of 2 tuples by using a single-element tuples
        # using the (<element>,) notation.
        # This needs only to be done if the provided phone number shall
        # be considered to be an Emergency number.
        UECmdTypes.EMERGENCY_NUMBERS_LIST = \
            self._initial_ue_command_em_number + (self._phone_number,)

        return status, msg


    def run_test(self):
        """
        Execute the test
        """
        time_to_wait_before_3g_idle = 10
        status = Global.SUCCESS
        status_msg = "No error occurred"

        try:
            # Configure the EM numbers
            self.__configure_cust_em_number()

            self._logger.info("Start the Emergency Call when DUT is in Flight Mode")

            # Release any previous call (Robustness)
            self._voicecall_api.release()
            time.sleep(self._wait_btwn_cmd)

            if self._flight_mode == True:
                # Enable flight mode
                self._networking_api.set_flight_mode("on")
                # Wait 5 seconds
                time.sleep(5)

            if self._loss_coverage_type != "REJECTION":
                # Initiate a LTE RRC Connection Release when performing MO Call to go back to IDLE
                self._ns_lte_data.ue_detach()
            else:
                self._logger.info("The EM call will be rejected with cause '%s' by LTE NW thanks to loaded scenario, but it will be accepted on UTRAN/2G NW " %(self.__rejection_cause))

            # Dial using the phone number given in parameters
            self._logger.info("Calling %s ..." % self._phone_number)

            # Wake the screen otherwise the Voice Call will not be established
            self._phone_system.wake_screen()
            self._phone_system.set_phone_lock(0)

            #Dial
            self._voicecall_api.dial(self._phone_number)

            # Check voice call is incoming on reselected network side
            self._ns_3gsm_vc.check_call_state(
                    "CONN", 3 * self._call_setup_time, blocking = True)

            # Check call is connected for CALL_DURATION seconds
            self._ns_3gsm_vc.is_voice_call_connected(self._call_duration)

            # If flight mode on
            if self._flight_mode == True:
                self._logger.info("Voice call will be release and set the flight mode OFF")
                # Mobile Release call
                self._voicecall_api.release()
                # DISABLE flight mode
                self._networking_api.set_flight_mode("off")

                # Wait 5 seconds
                time.sleep(5)

                # Check Data Connection State => CON before timeout
                self._ns_lte_data.check_data_connection_state("CON",
                                                      self._registration_timeout,
                                                      blocking = True,
                                                      cell_id = self._ns_lte_cell_id)

            # Specific cases for call cancellation
            # (rejection is done automatically)
            if self._loss_coverage_type == "CANCELLATION" or self._loss_coverage_type == "REJECTION":
                #Release normally the call from the DUT
                self.release_csfb_vc(time_to_wait_before_3g_idle)

            # Check current RAT, the MS shall be camped on LTE cell
            if self._modem_api.get_current_rat() != "LTE":
                # Check also the status on 4G cell
                current_state = self._ns_lte_data.get_data_connection_status(cell_id = self._ns_lte_cell_id)
                # If the NW response is not CONNECTED, raise an error
                if current_state != "CON":
                    status_msg = "MS is camped in the WRONG RAT - NOT on LTE !"
                    status = Global.FAILURE
                    self._logger.error(status_msg)
                    return (status, status_msg)
                else:
                    self._logger.info("The MS is camped on LTE - normal behavior!")

            # Wait 10 seconds
            self._logger.info("Wait 10 seconds")
            time.sleep(10)

            # make a ping
            ping(self._networking_api,
                     self._server_ip_address,
                     self._packet_size,
                     self._nb_pings,
                     self._target_ping_packet_loss_rate,
                     self._logger,
                     blocking = True)

        except TestEquipmentException:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "Connection to AT proxy failed")

        except DeviceException:
            raise DeviceException(DeviceException.CONNECTION_LOST, "Connection to DUT Lost")

        except:
            # In case of any exception, return FAILED status
            e = sys.exc_info()[0]
            self._logger.error( "Error: %s" % e)
            status = Global.FAILURE
            status_msg = "Error: %s" % e

        finally:
            # Reset 3G cell
            self._ns_3gsm_cell.set_cell_power(self._ns_3gsm_cell_power)

            #in case of any error
            if self._flight_mode == True:
                # disable flight mode
                self._networking_api.set_flight_mode("off")

            return status, status_msg

    def tear_down(self):
        """
        End and dispose the test
        """

        # Disable flight mode in the first place in any case
        self._networking_api.set_flight_mode("off")

        # Restore the initial Emergency Numbers
        self.__restore_initial_em_numbers()
        # Call tear down
        LabMobilityLteCsfb.tear_down(self)
        # Wait 10 seconds
        time.sleep(10)
        # return result
        return Global.SUCCESS, "No errors"


    def __restore_initial_em_numbers(self):
        """
        Restores the initial emergency number in ril list

        Raise an exception in case of error during AT commands execution
        """

        self._logger.info("Restore initial emergency numbers in ril.ecclist list")
        # Restore the list of emergency number in UE Command categories
        # if we changed it in the beginning of the Use Case
        UECmdTypes.EMERGENCY_NUMBERS_LIST = self._initial_ue_command_em_number

        # Restore the list of emergency numbers
        self._device.set_property_value(
            self.EMERGENCY_NUMBER_PROPERTY_NAME,
            self._initial_emergency_numbers)

        if self.modem_available and (self._at_set_nvm_em is not None):
            self._logger.info("Restore initial emergency numbers in "
                               "NVM:cust.emergency list at index %s" % self._index)
            response, msg_serial = self._serial_handler.send_at_command_and_get_result(self.command_to_rest_em_in_nvm,
                                                           self._command_timeout)
            if msg_serial == "OK":
                self._logger.info("Response to AT command %s is: %s" % (
                                     self.command_to_rest_em_in_nvm, msg_serial))
            else:
                raise DeviceException(DeviceException.OPERATION_SET_ERROR,
                                      "Response to AT command %s is: %s" % (
                                          self.command_to_rest_em_in_nvm, msg_serial))
        #Stop AT Proxy
        self._logger.info("Stop AT Proxy")
        self._modem_flashing_api.stop_at_proxy_from_mos()

        #Close the serial connection
        self._logger.info("Close the serial connection")
        self._logger.info("Disconnecting from the port " +
                            str(self._serial_handler.get_port()))
        self._serial_handler.disconnect()


    def __configure_cust_em_number(self):
        """
        Configure the emergency number in ril list

        Raise an exception in case of error during AT commands execution
        """
        # Update the list of RIL Emergency Numbers on the DUT
        # This imperatively has to be done after setting the Flight Mode
        self._logger.info("Add Emergency Number in ril.ecclist list")
        # Update the property holding the Emergency Numbers list.
        self._device.set_property_value(
            self.EMERGENCY_NUMBER_PROPERTY_NAME,
            self._phone_number)

        self._logger.info("Add Emergency Number in NVM:cust.emergency list "
                           "at index %s" % self._index)
        # Start up the proxy for sending AT commands
        self.at_proxy_com_port = self._modem_flashing_api.start_at_proxy_from_mos(
                                                                        int(self._launch_mode))
        # Check serial connection
        self._expected_result="OK"
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
        interrogate_at_nvm_command = self._command + "[" + \
                                    str(self.index) + "]" + "?"
        self._logger.info("Send AT command: %s" % interrogate_at_nvm_command)

        self._at_inter_nvm_em_verdict, at_nvm_params_to_be_rest_raw =\
                self._serial_handler.send_at_command_and_get_result(interrogate_at_nvm_command, self._command_timeout)
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
