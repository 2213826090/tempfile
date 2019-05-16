"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements the procedure for checking the LTE coexistence procedure
    with WIFI and Bluetooth ON/OFF and +XNRTCWS AT command and URCs
:since: 11/07/2014
:author: mariussX
"""
# Module name follows ACS conventions
# pylint: disable=C0103

import time
import shutil
import os


from UtilitiesFWK.Utilities import Global, str_to_bool
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UseCase.Networking.LAB_LTE_BASE import LabLteBase

from Core.PathManager import Folders, Paths

class LabLteCoexistenceCheck(LabLteBase):

    """
    LTE coexistence check.
    We test:
    - WIFI and BT ON/OF procedures are correctly performed
    - that "+XNRTCWS" AT commands and URCs are successfully sent and received
    """

    STATE_ON = 1
    """
    Static attribute that represents the status I{ON}.
    """

    STATE_OFF = 0
    """
    Static attribute that represents the status I{OFF}.
    """

    """
    Static variables for FDD bands
    """
    BAND_1 = 1
    BAND_7 = 7

    """
    Define the dictionaries based on the procedure
    """
    DICT_REGISTERED = \
        {"msg_seq_band_1" : ["CoexBroadcastReceiver got modem status: 1"], \
         "nb_param_band_1" : 0, \
         "msg_seq_band_7" : ["CoexBroadcastReceiver got modem status: 1", "CoexBroadcastReceiver got modem band: BAND_LTE_7"], \
         "nb_param_band_7" : 0
        }

    DICT_WIFI_ON = \
        {"msg_seq_band_1" : ["AT+XNRTCWS=2,1,0", "+XNRTCWS: 1,1,0,0,-1"], \
         "nb_param_band_1" : 5, \
         "msg_seq_band_7" : ["AT+XNRTCWS=2,1,0",
                           "CSilo_rfcoexistence::ParseCoexURC() - Final Response=[+XNRTCWSI: -1,2400,"], \
         "nb_param_band_7" : 45
        }

    DICT_WIFI_OFF = \
        {"msg_seq_band_1" : ["AT+XNRTCWS=2,0", "+XNRTCWS: 1,1,0,0,-1"], "nb_param_band_1" : 5, \
         "msg_seq_band_7" : ["AT+XNRTCWS=2,0", "+XNRTCWS: 1,1,0,0,-1"], "nb_param_band_7" : 5,
        }

    DICT_BT_ON = \
        {"msg_seq_band_1" : ["AT+XNRTCWS=2,,,1"], \
         "nb_param_band_1" : 0, \
         "msg_seq_band_7" : ["AT+XNRTCWS=2,,,1",
                             "CSilo_rfcoexistence::ParseCoexURC() - Final Response=[+XNRTCWSI: -1,-1,-1,2400,2484,-1,-1,-1,-1,-1,-1,-1,2400,2484,-1,-1,15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15]"], \
         "nb_param_band_7" : 45
        }

    DICT_BT_OFF = \
        {"msg_seq_band_1" : ["TX [AT+XNRTCWS?",
                             "RX [<cr><lf>+XNRTCWS: 1,0,0,1,-1<cr><lf><cr><lf>OK<cr><lf>]",
                             "TX [AT+XNRTCWS=2,,,0",
                             "RX [<cr><lf>OK<cr><lf>]"], \
         "nb_param_band_1" : 5, \
         "msg_seq_band_7" : ["TX [AT+XNRTCWS?",
                             "RX [<cr><lf>+XNRTCWS: 1,0,0,1,-1<cr><lf><cr><lf>OK<cr><lf>]",
                             "TX [AT+XNRTCWS=2,,,0",
                             "RX [<cr><lf>OK<cr><lf>]"], \
         "nb_param_band_7" : 5
        }

    DICT_AIRPLANE_ON = \
        {"msg_seq_band_1" : ["+XNRTCWSI: 0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1"], \
         "nb_param_band_1" : 45, \
         "msg_seq_band_7" : ["+XNRTCWSI: 0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1"], \
         "nb_param_band_7" : 45
        }


    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        LabLteBase.__init__(self, tc_name, global_config)

        # Check the cell band
        if self._cell_band not in range(1,33):
            # Raise an exception
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "'CELL BAND' parameter not in [1,32] range")

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._camp_timeout = self._registration_timeout

        # Read parameters and convert them to their final type
        self._check_bt = \
            str(self._tc_parameters.get_param_value("CHECK_BT"))
        self._check_bt = str_to_bool(self._check_bt)

        self._check_wlan = \
            str(self._tc_parameters.get_param_value("CHECK_WLAN"))
        self._check_wlan = str_to_bool(self._check_wlan)

        # Value for the number of iterations regarding ON-OFF procedures
        number_of_iterations = \
            str(self._tc_parameters.get_param_value("ON_OFF_ITERATION"))
        if number_of_iterations.isdigit() is True:
            self._number_of_iterations = int(number_of_iterations)
            if self._number_of_iterations <= 0:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, \
                                         "'ON_OFF_ITERARION' parameter must be a positive value > 0")
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, \
                                      "'ON_OFF_ITERARION' parameter must be an integer")

        # Time for waiting with BT or WIFI enabled
        time_wait_on = \
            str(self._tc_parameters.get_param_value("WAIT_ON"))
        if time_wait_on.isdigit() is True:
            self._time_wait_on = int(time_wait_on)
            if self._number_of_iterations <= 0:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, \
                                         "'WAIT_ON' parameter must be a positive value > 0")
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, \
                                      "'WAIT_ON' parameter must be an integer")

        # Instantiate UE Command categories
        self._networking_api = self._device.get_uecmd("Networking")
        self._connectivity_api = self._device.get_uecmd("LocalConnectivity")

        # Instantiate the PhoneSystem UE Command category
        self._phone_system = self._device.get_uecmd("PhoneSystem")

        # Instantiate the modem UE commands
        self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")

        # Set time variable
        self._time = 5

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Test setup
        """

        # If WIFI is ON, turn it off
        if self._networking_api.get_wifi_power_status() == self.STATE_ON:
            self._logger.info("Turn off WIFI")
            # Turn off WIFI
            self._networking_api.set_wifi_power(self.STATE_OFF)
            time.sleep(self._wait_btwn_cmd)

        # Turn off Bluetooth
        self._connectivity_api.set_bt_power(self.STATE_OFF)
        time.sleep(self._wait_btwn_cmd)

        # Call LAB_LTE_BASE set_up function
        LabLteBase.set_up(self)

        # Recording initial state before starting the test
        self._initial_flight_mode_state = \
            self._networking_api.get_flight_mode()

        # If DUT is not in airplane mode on, enable it
        if self._initial_flight_mode_state == 0:
            self._logger.info("Now enabling Flight Mode.")
            self._networking_api.set_flight_mode(self.STATE_ON)

        # Set Cell on
        self._cell_4g.set_cell_on(self._mimo)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call LAB_LTE_BASE run_test function
        LabLteBase.run_test(self)

        # Clear the logcat buffer
        self.__clear_logcat()

        # Phone has to see the cell off!
        self._modem_api.check_cdk_state_bfor_timeout("unregistered",
                                                     self._camp_timeout)

        # Deactivate airplane mode
        self._logger.info("Flight Mode was active, deactivating it now.")
        self._networking_api.set_flight_mode(self.STATE_OFF)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          self._wanted_reg_state)

        self._modem_api.check_cdk_state_bfor_timeout(
            self._wanted_reg_state,
            self._registration_timeout)

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, False)

        # Check data connection state is "CON"
        self._data_4g.check_data_connection_state("CON",
                                                  self._camp_timeout,
                                                  blocking = False,
                                                  cell_id = self._cell_id)

        # Get RAT from Equipment
        network_type = self._data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._camp_timeout)

        # Analize in the logcat the CWS response after DUT registration
        self._logger.info("Checking CWS response after DUT registration.")
        result, output = self.__check_message_in_logcat("REGISTERED")
        if result == Global.FAILURE:
            self._logger.error(output)
            return Global.FAILURE, "CWS responses not found in logcat after DUT registration"
        else:
            self._logger.info("CWS responses after DUT registration was found in logcat")

        # Wait between two commands sending
        time.sleep(self._wait_btwn_cmd)

        # ------------------------------------------------------------

        # Perform ON/OFF for configured number of iterations
        self._logger.info("Perform %d ON-OFF iterations for BT or WIFI" % self._number_of_iterations)
        counter = 0
        for counter in xrange(0, self._number_of_iterations):
            self._logger.info("Iteration no. %d" % (counter+1))

            # Check if the user has requested to check LTE coexistence with WIFI ON/OFF
            if self._check_wlan:
                # Check WIFI ON/OFF
                verdict, result_message = self.__check_wlan_procedures()
                if verdict == Global.FAILURE:
                    return verdict, result_message
            else:
                # Evacuate an info message
                self._logger.info("The user has decided to skip the WLAN activation/deactivation procedure")

            # Check if the user has requested to check LTE coexistence with BT ON/OFF
            if self._check_bt:
                # Check BT ON/OFF
                verdict, result_message = self.__check_bt_procedures()
                if verdict == Global.FAILURE:
                    return verdict, result_message
            else:
                # Evacuate an info message
                self._logger.info("The user has decided to skip the BT activation/deactivation procedure")

            # Increment the counter
            counter = counter + 1

        # Wait between two commands sending
        time.sleep(self._wait_btwn_cmd)

        # Get RAT from Equipment
        network_type = self._data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._logger.info("Check that DUT is registered on the good RAT")
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._camp_timeout)

        # Wait between two commands sending
        time.sleep(self._wait_btwn_cmd)

        # Execute PING command, and compute packet loss value
        packet_loss = self._networking_api.\
            ping(self._server_ip_address,
                 self._packet_size,
                 self._nb_pings)

        # Compute verdict depending on % of packet loss
        if packet_loss.value > self._target_ping_packet_loss_rate:
            self._error.Code = Global.FAILURE
        else:
            self._error.Code = Global.SUCCESS

        self._error.Msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
            % (packet_loss.value,
               packet_loss.units,
               self._target_ping_packet_loss_rate,
               packet_loss.units)

        # Wait between two commands sending
        time.sleep(self._wait_btwn_cmd)

        # Enable Flight Mode
        self._logger.info("Now enabling Flight Mode.")
        self._networking_api.set_flight_mode(self.STATE_ON)
        # Analize in the logcat the procedure for "AIRPLANE MODE ON"
        result, output = self.__check_message_in_logcat("AIRPLANE MODE ON")
        if result == Global.FAILURE:
            self._logger.error(output)
            return Global.FAILURE, "AIRPLANE MODE ON procedure was not found in logcat"
        else:
            self._logger.info("\"+XNRTCWS\" URC for 'AIRPLANE MODE ON' was found in logcat")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Tear down function
        """

        # Call the inherited method
        LabLteBase.tear_down(self)

        # In case the WIFI or BT are enabled, disable them
        # Disable WIFI
        self._logger.info("Turn WIFI OFF")
        self._networking_api.set_wifi_power(self.STATE_OFF)
        time.sleep(self._wait_btwn_cmd)

        # Disable BT
        self._logger.info("Turn BT OFF")
        self._connectivity_api.set_bt_power(self.STATE_OFF)
        time.sleep(self._wait_btwn_cmd)

        # Set flight mode back to initial state
        # if flight mode has been correctly retrieved in the beginning
        if self._initial_flight_mode_state not in ("", None):
            self._logger.info("Restoring Flight Mode to its initial state.")
            self._networking_api.set_flight_mode(self._initial_flight_mode_state)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __check_bt_activation(self, expected_state=STATE_ON):
        """
        Checks that the I{Bluetooth} is active
        and raises an exception otherwise.

        :type expected_state: int
        :param expected_state: [optional] the expected state (1:ON / 0:OFF)
            Default value is 1 (ON).

        :raise DeviceException: if the I{Bluetooth}
            is not active.
        """
        # Store the expected state as str
        expected_state_str = self.__get_state_str(expected_state)
        # Retrieve the BT power status and convert it to str
        bt_power_status = self._connectivity_api.get_bt_power_status()
        bt_states_on = (
            "STATE_ON",
            "STATE_TURNING_ON",
            "STATE_CONNECTED",
            "STATE_CONNECTING")
        if str(bt_power_status) in bt_states_on:
            bt_power_status = "ON"
        else:
            bt_power_status = "OFF"
        self._logger.info("Got BT power status: %s" % str(bt_power_status))
        # Compare the actual state with the expected one
        if bt_power_status != expected_state_str:
            # If the retrieved BT status does not match the expected one
            # build an error message.
            message = "Current BT state '%s' does not match the " \
                "expected state '%s'." % (
                    bt_power_status,
                    expected_state_str)
            # Raise an exception
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, message)
        else:
            # Otherwise simply log a message indicating that
            # everything went OK
            self._logger.debug("BT activation status is the expected one.")

#------------------------------------------------------------------------------

    def __check_wlan_activation(self, expected_state=STATE_ON):
        """
        Checks that the I{WLAN} is active
        and raises an exception otherwise.

        :type expected_state: int
        :param expected_state: [optional] the expected state (1:ON / 0:OFF)
            Default value is 1 (ON).

        :raise DeviceException: if the I{WLAN}
            is not active.
        """
        # Retrieve the current WLAN power status
        wlan_power_state = self._networking_api.get_wifi_power_status()
        # Compare the actual state with the expected one
        if wlan_power_state != expected_state:
            # If the retrieved power status does not match the expected one
            # build an error message.
            power_state_str = self.__get_state_str(wlan_power_state)
            expected_state_str = self.__get_state_str(expected_state)
            message = "Current WLAN state '%s' does not match the " \
                "expected state '%s'." % (power_state_str, expected_state_str)
            # Raise an exception
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, message)

        else:
            # Otherwise simply log a message indicating that
            # everything went OK
            self._logger.debug("WLAN power status is the expected one.")

#------------------------------------------------------------------------------

    def __get_state_str(self, state):
        """
        Returns the str representation of the given binary state
        (ON/OFF).

        :type state: int
        :param state: the expected state (1:ON / 0:OFF)

        :rtype: str
        :return: the state as str
        """

        # Initialize the return value
        state_str = "OFF"
        # Check whether the state is ON or not
        if state == self.STATE_ON:
            # Update the return value accordingly
            state_str = "ON"
        # Return the computed value
        return state_str

# -----------------------------------------------------------------------------

    def __check_message_in_logcat(self, procedure):
        """
        Compute the AT command and URC list according with the procedure requested
        and trigger DUT's logcat analysis

        :type procedure: str
        :param procedure:

        :rtype: int, str
        :return: status ("result") as int and an additional message ("output")
        """

        self._logger.info("Check in logcat the specific messages for '%s' procedure" %(procedure))

        # Compute the messages which needs to be checked based on the procedure and cell band
        message_list, urc_parameters = self._compute_strings_to_check(procedure,
                                                                      self._cell_band)

        # Create a file which stores the locgcat from DUT
        self._logcat_extract = os.path.join(Paths.REPORTS,"parserlogcat.log")

        if procedure in ("REGISTERED"):
            # Dump the 'main' log into a file on the local computer
            adb_command = "adb logcat -b main -d > %s" %(self._logcat_extract)
        else:
            # Dump the 'radio' log into a file on the local computer
            adb_command = "adb logcat -b radio -d > %s" %(self._logcat_extract)

        # Wait for 5 seconds
        time.sleep(self._time)

        # Execute adb command
        os.system(adb_command)
        time.sleep(self._wait_btwn_cmd)

        # Parse the logcat
        result, output = self.analyzeLog(self._logcat_extract,
                                         message_list,
                                         urc_parameters)

        # In case of failure, create a backup of the logcat to be analyzed further
        if result == Global.FAILURE:
            try:
                # Create a backup of the file
                self._backuplog = "logcatdut_backup_" + procedure + ".log"
                # Copy the logcat file
                shutil.copy2(self._logcat_extract,self._backuplog)
                self._logger.info("For analysis a backup file was created, please see: " + self._backuplog)
            except Exception as e:
                # Raise an exception
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                         ("Cannot create a backup file - " + str(e)))

        # Delete the log
        os.remove(self._logcat_extract)

        # Clear the logcat buffer
        self.__clear_logcat()

        return result, output

#------------------------------------------------------------------------------

    def analyzeLog(self, logcat_file, message_list, urc_parameters):

        """
        This function will analize DUT logcat against some input expressions
        so deciding a PASS/FAILED status based on expressions presence

        :rtype: int, str
        :return: status ("result") as int and an additional message ("output")
        """

        output = ""

        # Open the logcat file to be read
        logcat = open(logcat_file, 'r')
        file_list = logcat.readlines()
        content = "".join(file_list)

        # Close the logcat file
        logcat.close()

        if not content.strip():
            raise Exception("logcat was not saved")

        i = 0
        procedure_found = True

        while (i<len(message_list) and procedure_found == True):
            procedure_found = False
            # Compare with every line
            for line in file_list:
                if message_list[i] in line:

                    procedure_found = True
                    # Check the number of parameters
                    if (urc_parameters != 0) and \
                        (("+XNRTCWS:" in message_list[i]) or ("+XNRTCWSI:" in message_list[i])):

                        nb_of_parameters = self._check_number_of_parameters(line)
                        # In case the number of parameters is different from the one which is expected,
                        # mark the procedure as failed
                        if nb_of_parameters != urc_parameters:
                            self._logger.error("The number of parameters from URC is different in: " + line)
                            procedure_found = False
                    break
            # Go to next message from the list in order to be searched in the logcat
            i+=1

        # Check if all procedure has been found
        if procedure_found == True:
            result = Global.SUCCESS
            output = "Procedure found, no error"
        else:
            result = Global.FAILURE
            output = "The AT procedure was not found in the logcat"

        # Return results
        return result, output

#------------------------------------------------------------------------------

    def __clear_logcat(self):
        """
        This function will clear the logcat buffers from the DUT
        """

        self._logger.debug("Clear the logcat buffers")
        time.sleep(self._wait_btwn_cmd)

        try:
            adb_clear_comand = "adb logcat -b radio -c"
            adb_clear_comand = "adb logcat -b main -c"
            # Execute adb command
            os.system(adb_clear_comand)
            time.sleep(self._wait_btwn_cmd)
        except Exception as excp:
            return (Global.FAILURE,
                    "Error while deleting the logcat : " + str(excp))

#------------------------------------------------------------------------------

    def _check_number_of_parameters(self, line):
        """
        This function will check the number of parameters from the URCs ("+XNRTCWSI" or "+XNRTCWS")

        :type line: str
        :param line: the line which contains the URC
        :rtype: int
        :return: number of paramters from the respnse
        """

        # Initialize the count value and the string which will contain the URC parameters
        count = 0
        string_params = ""

        # Process the line in order to retrieve only the URC parameters
        try:
            string_params = line.split("+XNRTCWS: ")[1]
        except IndexError:
            string_params = line.split("+XNRTCWSI: ")[1]
        # Eliminate any ']' character
        string_params = string_params.replace("]","")
        if "<cr>" in string_params:
            string_params = string_params.split("<cr>")[0]

        # Count the parameters present in the URC response
        for i in string_params.split(','):
            # Check if it can be converted to integer
            # if yes -> increment the counter
            # if no -> skip, it's not a parameter
            try:
                number = int(i)
                count = count + 1
            except:
                pass

        # Return the number of parameters
        return count

#------------------------------------------------------------------------------

    def _compute_strings_to_check(self, procedure, band):
        """
        This function will compute the strings which needs to be checked in the logcat based on the procedure

        :type procedure: str
        :param procedure: the procedure which has to be checked
        :type band: int
        :param band: the band on which DUT was requested to register

        :rtype : string, int
        :return: the strings which has to be becked in the logcat in "message_list" parameter
            and the number of parameters from URC response in "urc_parameters"
        """

        message_list = []

        # Set the dictionaries related with the procedure
        procedures_dict = {"REGISTERED": self.DICT_REGISTERED,
                          "WIFI ON" : self.DICT_WIFI_ON,
                          "WIFI OFF" : self.DICT_WIFI_OFF,
                          "BT ON" : self.DICT_BT_ON,
                          "BT OFF" : self.DICT_BT_OFF,
                          "AIRPLANE MODE ON" : self.DICT_AIRPLANE_ON}

        # Retrieve the dictionary based on the procedure
        dictionary = procedures_dict[procedure]

        # Get the number of parameters which needs to be checked
        urc_parameters = dictionary["nb_param_band_%d" %(band)]
        # Get the sequences which needs to be checked
        for i in xrange(0, len(dictionary["msg_seq_band_%d" %(band)])):
            message_list.append(dictionary["msg_seq_band_%d" %(band)][i])

        return message_list, urc_parameters

#------------------------------------------------------------------------------

    def __check_wlan_procedures(self):
        """
        This function will perform the WIFI ON/OFF procedures and
        check the specific CWS sequences in the DUT's logcat

        :rtype : int, string
        :return: verdict as integer and a message related with the verdict as string
        """

        self._logger.info("Turn WIFI ON")
        # Activate WLAN
        self._logger.debug("Checking WLAN activation.")
        self._networking_api.set_wifi_power(self.STATE_ON)
        time.sleep(self._wait_btwn_cmd)
        # Check WLAN activation
        self.__check_wlan_activation()

        # Analize in the logcat the procedure for "WIFI ON"
        result, output = self.__check_message_in_logcat("WIFI ON")
        if result == Global.FAILURE:
            self._logger.error(output)
            return Global.FAILURE, "WIFI ON procedure was not found in logcat"
        else:
            self._logger.info("\"+XNRTCWS\" AT command and URC procedures for 'WIFI ON' was found in logcat")

        # Check that DUT is still connected on LTE
        dut_state = self._data_4g.get_data_connection_status(self._cell_id)
        if dut_state not in ("CON","IDLE"):
            self._logger.error("DUT's state it's not 'CON' or 'IDLE'")
            return Global.FAILURE, "DUT's state it's not 'CON' or 'IDLE'"

        self._logger.info("Wait %d seconds with WIFI activated"
                          % self._time_wait_on)
        time.sleep(self._time_wait_on)

        # Disable WLAN
        self._logger.info("Turn WIFI OFF")
        self._networking_api.set_wifi_power(self.STATE_OFF)
        time.sleep(self._wait_btwn_cmd)
        # Check WLAN deactivation
        self._logger.debug("Checking WLAN deactivation.")
        self.__check_wlan_activation(self.STATE_OFF)

        # Analize in the logcat the procedure for "WIFI OFF"
        result, output = self.__check_message_in_logcat("WIFI OFF")
        if result == Global.FAILURE:
            self._logger.error(output)
            return Global.FAILURE, "WIFI OFF procedure was not found in logcat"
        else:
            self._logger.info("\"+XNRTCWS\" AT command and URC procedures for 'WIFI OFF' was found in logcat")

        # Check that DUT is still connected on LTE
        dut_state = self._data_4g.get_data_connection_status(self._cell_id)
        if dut_state not in ("CON","IDLE"):
            self._logger.error("DUT's state it's not 'CON' or 'IDLE'")
            return Global.FAILURE, "DUT's state it's not 'CON' or 'IDLE'"

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __check_bt_procedures(self):
        """
        This function will perform the BT ON/OFF procedures and
        check the specific CWS sequences in the DUT's logcat

        :rtype : int, string
        :return: verdict as integer and a message related with the verdict as string
        """

        self._logger.info("Turn BT ON")
        # Activate bluetooth
        self._logger.debug("Checking BT activation.")
        self._connectivity_api.set_bt_power(self.STATE_ON)
        self._logger.info("Sleeping %d seconds."
                          % self._wait_btwn_cmd)
        time.sleep(self._wait_btwn_cmd)
        # Check bluetooth activation
        self.__check_bt_activation()

        # Analize in the logcat the procedure for "BT ON"
        result, output = self.__check_message_in_logcat("BT ON")
        if result == Global.FAILURE:
            self._logger.error(output)
            return Global.FAILURE, "BT ON procedure was not found in logcat"
        else:
            self._logger.info("\"+XNRTCWS\" AT command and URC procedures for 'BT ON' was found in logcat")

        # Check that DUT is still connected on LTE
        dut_state = self._data_4g.get_data_connection_status(self._cell_id)
        if dut_state not in ("CON","IDLE"):
            self._logger.error("DUT's state it's not 'CON' or 'IDLE'")
            return Global.FAILURE, "DUT's state it's not 'CON' or 'IDLE'"

        self._logger.info("Wait %d seconds with BT activated"
                          % self._time_wait_on)
        time.sleep(self._time_wait_on)

        # Disable BT
        self._logger.info("Turn BT OFF")
        self._connectivity_api.set_bt_power(self.STATE_OFF)
        time.sleep(self._wait_btwn_cmd)

        # Check BT deactivation
        self._logger.debug("Checking BT deactivation.")
        self.__check_bt_activation(self.STATE_OFF)

        # Analize in the logcat the procedure for "BT OFF"
        result, output = self.__check_message_in_logcat("BT OFF")
        if result == Global.FAILURE:
            self._logger.error(output)
            return Global.FAILURE, "BT OFF procedure was not found in logcat"
        else:
            self._logger.info("\"+XNRTCWS\" AT command and URC procedures for 'BT OFF' was found in logcat")

        # Check that DUT is still connected on LTE
        dut_state = self._data_4g.get_data_connection_status(self._cell_id)
        if dut_state not in ("CON","IDLE"):
            self._logger.error("DUT's state it's not 'CON' or 'IDLE'")
            return Global.FAILURE, "DUT's state it's not 'CON' or 'IDLE'"

        return Global.SUCCESS, "No errors"
