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
:summary: This file implements use case to control to CMW500 to interface with its MLAPI

:since: 01/06/2012
:author: ssavrimoutou
"""

import os
import urllib2
import time

from SOAPpy import SOAPProxy  # @UnresolvedImport
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase

from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException


class RsCmw500ProtocolTester(UseCaseBase):

    """
    Usecase that configure equipment(s) thanks to an soap requests.
    The use case waits for messages returned by the equipment
    and execute ue commands in function of the messages.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read parameters from test case parameters :
        #    - EQUIPMENT_IP_ADDRESS
        #    - EQUIPMENT_PORT
        #    - CONFIGURATION_FILE
        #    - TESTCASE_NAME
        self._eqp_ip_address = \
            str(self._tc_parameters.get_param_value("EQUIPMENT_IP_ADDRESS"))
        self._eqp_port = \
            str(self._tc_parameters.get_param_value("EQUIPMENT_PORT"))
        self._conf_filename = \
            str(self._tc_parameters.get_param_value("CONFIGURATION_FILE"))
        self._testcase_name = \
            str(self._tc_parameters.get_param_value("TESTCASE_NAME"))

        # Initialize equipment parameters (MLAPI ip, port)
        self._mlapi_table = {}
        self._xdd_explorer_service = \
            "http://%s:%s/axis2/services/XDDExplorerRemoteControlService" % \
            (self._eqp_ip_address, self._eqp_port)
        self._prj_explorer_service = \
            "http://%s:%s/axis2/services/ProjectExplorerRemoteControlService" % \
            (self._eqp_ip_address, self._eqp_port)

        # Get Computer instance from Bench Config
        self._computer = self._em.get_computer("COMPUTER1")
        self._is_eqp_network_reachable = False

    def _parse_mlapi_messages_table(self):
        """
        Parse the RSCMW500 MLAPI table to get ue command correspondance
        with equipment messages
        """

        # Open and read the file content
        conf_file = os.path.join(self._execution_config_path,
                                 "RSCMW500_MLAPI_MESSAGES.csv")
        fileid = open(conf_file, 'r')
        lines = fileid.readlines()
        fileid.close()

        # Store each messages in dictionary
        for line in lines:
            line = line.replace('\n', '').strip()
            element = line.split(',')
            self._mlapi_table.update({element[0]: element[1]})

    def run_script(self, script_file):
        """

        :type script_file: str
        :param script_file: Script file to be used
        """
        script_file = os.path.normpath(script_file)
        script_file_in_execconfig = os.path.join(self._execution_config_path,
                                                 script_file)
        if os.path.isfile(script_file):
            self._logger.debug("Launching script %s" % str(script_file))
            # Execute the script
            execfile(script_file)

        elif os.path.isfile(script_file_in_execconfig):
            self._logger.debug("Launching script %s from _ExecutionConfig" % str(script_file))
            # Execute the script
            execfile(script_file_in_execconfig)

        else:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Unable to execute script ! - %s" % str(script_file))

    def send_soap_request(self, soap_service, soap_action):
        """
        Send the soap request through url library

        :type soap_service: str
        :param soap_service: Soap service to request

        :type soap_action: str
        :param soap_action: Soap action to request

        :rtype: str
        :return: Raw soap response
        """
        soap_response = ""
        soap_request = soap_service + "/" + soap_action
        self._logger.debug("Network simulator: Sending request : %s" % soap_request)
        try:
            connection = urllib2.urlopen(soap_request)
            soap_response = connection.read()
            connection.close()
        except IOError as excp:
            self._logger.warning(excp)

        # Return an exception in case of SOAP fault
        if "faultstring" in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "SOAP request error : %s" % soap_response)

        self._logger.debug("Network simulator: Received response : %s" % soap_response)

        return soap_response

    def check_equipment_connectivity(self, ip_address, port):
        """
        Check that equipment interface is reachable

        :type ip_address: str
        :param ip_address: Ip address of the equipment
        """
        self._logger.info("Network simulator: Check equipment connectivity")

        # Ping the equipment
        try:
            # Ping
            packet_loss = self._computer.ping(ip_address, 128, 2)
            if packet_loss.value > 0:
                error_msg = "Measured Packet Loss: %.0f%s !" % (packet_loss.value, packet_loss.units)
                error_msg += " Please check Equipment and ACS Host network link"
                self._is_eqp_network_reachable = False
                raise TestEquipmentException(TestEquipmentException.CONNECTION_LOST, error_msg)

        except TestEquipmentException as excp:
            error_msg = str(excp)
            error_msg += " Please check that Equipment and ACS Host are properly configured (ip address ...) "
            self._is_eqp_network_reachable = False
            raise TestEquipmentException(TestEquipmentException.CONNECTION_LOST, error_msg)

        # Check SOAP interface is launched on the equipment
        try:
            self._logger.info("Check that equipment SOAP interface is started")
            soap_interface_url = "http://%s:%s" % (ip_address, port)
            connection = urllib2.urlopen(soap_interface_url, timeout=1)
            connection.close()
        except IOError:
            error_msg = "Equipment SOAP interface is not reachable ! Please check that ProjectExplorer is launched in SOAP server mode"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

        # Update if the status of equipment connectivity
        self._is_eqp_network_reachable = True

    def _load_configuration_file(self, configuration_file):
        """
        Load configuration file on the equipment

        :type configuration_file: str
        :param configuration_file: Project file to load on the equipment
        """
        self._logger.info("Network simulator: Load the configuration file %s" % configuration_file)

        soap_action = "doLoadDocument?NAME=%s" % configuration_file
        soap_response = self.send_soap_request(self._xdd_explorer_service, soap_action)

        if "OK" not in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to load file ! - %s" % soap_response)

    def _unload_configuration_file(self):
        """
        Unload configuration file on the equipment
        """
        self._logger.info("Unload the current configuration file")

        soap_action = "doCloseDocument"
        soap_response = self.send_soap_request(self._xdd_explorer_service, soap_action)

        if "OK" not in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to unload the configuration file ! - %s" % soap_response)

    def _set_terminal_adapter(self, mode):
        """
        The transport service used to communicate with the DUT.
        Note: This option must be set to REMOTE
        to redirect all AT commands and MMI
        messages to the remote client via SOAP.

        :type mode :  str
        :param mode : Automation control to set on the equipment
        """
        self._logger.info("Network simulator: Set the terminal adapter to %s" % mode)

        if mode.upper() not in ["RS232", "TCPIP", "REMOTE"]:
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED,
                                         "Invalid terminal adapter %s !" % mode)

        soap_action = "doSetOption?NAME=TERMINAL_ADAPTER&VALUE=%s" % mode.upper()
        soap_response = self.send_soap_request(self._xdd_explorer_service, soap_action)

        if "OK" not in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to set terminal adapter ! - %s" % soap_response)

    def _set_automation_control(self, mode):
        """
        Set the automation mode

        :type mode :  str in manual or automatic
        :param mode : Automation control to set on the equipment
        """
        self._logger.info("Network simulator: Set the automation control to %s" % mode)

        if mode.upper() not in ["MANUAL", "AUTOMATIC"]:
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED,
                                         "Invalid automation mode %s !" % mode)

        soap_action = "doSetOption?NAME=AUTOMATION_CONTROL&VALUE=%s" % mode.upper()
        soap_response = self.send_soap_request(self._xdd_explorer_service, soap_action)

        if "OK" not in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to set automation control ! - %s" % soap_response)

    def _start_test_case(self, test_case):
        """
        Start the test case on the equipment

        :type test_case :  str
        :param test_case : Test case to start on the equipment
        """
        self._logger.info("Network simulator: Start test case \"%s\"" % test_case)

        # format test case name before sending request
        test_case = test_case.replace(' ', '%20')

        soap_action = "doRunTestCase?TESTCASE_ID=%s" % test_case
        self.send_soap_request(self._prj_explorer_service, soap_action)

    def _stop_test_case(self):
        """
        Stop the test case on the equipment
        """
        self._logger.info("Stop the test case")

        soap_action = "doStopTestCase"
        soap_response = self.send_soap_request(self._prj_explorer_service, soap_action)

        if "OK" not in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to stop the test case ! - %s" % soap_response)

    def _get_test_case_state(self):
        """
        Return the test case state

        :rtype: str
        :return: The test case state
        """
        soap_response = SOAPProxy(self._prj_explorer_service, throw_faults=0).doCheckTestCaseStatus()

        if "faultstring" in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to get test case state ! - %s" % soap_response)

        tc_state = soap_response._asdict().get('TC_STATUS')
        self._logger.debug("Network simulator: Test Case status - %s" % tc_state)

        return tc_state.upper()

    def _get_test_case_indication(self):
        """
        Display the test case indication
        """
        soap_response = SOAPProxy(self._prj_explorer_service, throw_faults=0).doCheckTestCaseStatus()

        if "faultstring" in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to get test case indication ! - %s" % soap_response)

        tc_indication_number = int(soap_response._asdict().get('AVAIL_TC_INDS'))
        indication_msg = ""

        for _i in range(tc_indication_number):
            soap_response = SOAPProxy(self._prj_explorer_service, throw_faults=0).doGetNextTCIndication()
            if "faultstring" not in soap_response:
                tc_indication = soap_response._asdict()
                if 'MESSAGE' in tc_indication.keys():
                    # Format str by removing all strip spaces, \n
                    tc_indication = tc_indication.get('MESSAGE').strip()
                    tc_indication = tc_indication.replace('\n', '')

                    # Update the full indication message
                    indication_msg += tc_indication
                    indication_msg += '\n'

        if indication_msg != "":
            self._logger.info("\n" + str(indication_msg))

    def _wait_for_test_case_state(self, state, timeout):
        """
        Wait for test case state on the equipment

        :type state : str
        :param state : Test Case state to check

        :type timeout :  float
        :param timeout : Timeout before raising exception
        """
        self._logger.info("Network simulator: Wait for test case %s state before %s s" % (str(state), str(timeout)))

        is_reached = False
        start_time = time.time()
        while ((time.time() - start_time) < timeout) and (is_reached is False):
            tc_state = self._get_test_case_state()
            if tc_state == state:
                is_reached = True
            time.sleep(1)

        if is_reached:
            self._logger.info("Network simulator: Test case is %s" % str(state))
        else:
            msg = "Network simulator: Test case failed to reach %s state before timeout !" % str(state)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

    def _get_test_case_messages(self):
        """
        Get if received MMI, AT messages

        :rtype: tuple
        :return: A tuple of value containing (message_type, message)
        message_type = MMI, AT
        """

        message_type = "NONE"
        message = "No message available"

        # Get MMi Messages first
        mmi_message = SOAPProxy(self._prj_explorer_service, throw_faults=0).doGetNextMmiMessage()

        if "No MMI message available" in mmi_message:
            # Get AT comment if no MMi messages
            soap_response = SOAPProxy(self._prj_explorer_service, throw_faults=0).doGetNextAtCommand()

            if "No AT command available" in soap_response:
                self._logger.debug("Network simulator: No message available")
                message_type = "NONE"
                message = "No message available"

            else:
                at_command = soap_response._asdict()
                self._logger.debug("Network simulator: Received AT Command - %s" % at_command)
                message_type = "AT"
                message = at_command.get('COMMAND')

        else:
            self._logger.debug("Network simulator: Received MMI Message - %s" % mmi_message)
            message_type = "MMI"
            message = mmi_message

        message = message.replace('\r', '')
        return message_type, message

    def _get_test_case_verdict(self):
        """
        Get The final verdict which can be one of the following:
        - "PASS" meaning the testcase passed
        - "FAIL" meaning the testcase failed
        - "INCONC" meaning the testcase result is inconclusive
        - "NONE" meaning no verdict does exist

        :rtype: tuple
        :return: The final verdict of the test case
        """
        self._logger.info("Get the test case final verdict")
        soap_response = SOAPProxy(self._prj_explorer_service, throw_faults=0).doGetTestCaseVerdict()

        if "faultstring" in soap_response:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to start test case ! - %s" % soap_response)

        if soap_response != "PASS":
            return Global.FAILURE, soap_response
        else:
            return Global.SUCCESS, "No Errors"

    def _confirm_mmi_messages(self, response, result_flag):
        """
        Confirm the MMI message dialog

        :param response :str
        :param response : The response of the MMI/EMMI message.

        :param result_flag:str
        :param result_flag: The flag indicating the success of the MMI/EMMI message.
        """

        soap_action = "doConfirmMmiMessage?RESPONSE=%s&RESULT_FLAG=%s" % (response, result_flag)
        self.send_soap_request(self._prj_explorer_service, soap_action)

    def _confirm_at_commands(self, response, result_flag):
        """
        Confirm the AT commands dialog

        :param response :str
        :param response : The response of the MMI/EMMI message.

        :param result_flag:str
        :param result_flag: The flag indicating the success of the MMI/EMMI message.
        """

        soap_action = "doConfirmAtCommand?RESPONSE=%s&RESULT_FLAG=%s" % (response, result_flag)
        self.send_soap_request(self._prj_explorer_service, soap_action)

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call UseCase base set_up function
        UseCaseBase.set_up(self)

        # Check equipment connectivity
        self.check_equipment_connectivity(self._eqp_ip_address, self._eqp_port)

        # Read and store MLAPI_DIALOG_MESSAGES table to perform dut commands
        self._parse_mlapi_messages_table()

        # Load the CONFIGURATION_FILE on the equipment
        time.sleep(self._wait_btwn_cmd)
        self._load_configuration_file(self._conf_filename)

        # Set the terminal adapter to remote
        time.sleep(self._wait_btwn_cmd)
        self._set_terminal_adapter("REMOTE")

        # Set equipment to automatic mode
        time.sleep(self._wait_btwn_cmd)
        self._set_automation_control("AUTOMATIC")

        return Global.SUCCESS, "No Errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base run_test function
        UseCaseBase.run_test(self)

        # Launch the TESTCASE_NAME  on the equipment
        time.sleep(self._wait_btwn_cmd)
        self._start_test_case(self._testcase_name)

        # Wait for TESTCASE_NAME is "running" before timeout
        self._wait_for_test_case_state("RUNNING", 30)

        # WHILE test case status not "finished"
        tc_state = self._get_test_case_state()

        while tc_state == "RUNNING":

            # Get test case MMI,AT messages
            time.sleep(self._wait_btwn_cmd)
            (message_type, message) = self._get_test_case_messages()

            # IF indication in MLAPI_DIALOG_MESSAGES table
            if message_type in ["MMI", "AT"] and \
                    message in self._mlapi_table.keys():
                # Retrieve commands to execute from table
                script_to_execute = self._mlapi_table.get(message)
                # Execute the command
                # As we execute a python script we can directly confirm the message
                # in the script
                self.run_script(script_to_execute)
                time.sleep(self._wait_btwn_cmd)

            # Get test case state
            time.sleep(self._wait_btwn_cmd)
            tc_state = self._get_test_case_state()
            self._get_test_case_indication()

            # END WHILE

        # Get the final verdict from equipment
        return self._get_test_case_verdict()

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        if self._is_eqp_network_reachable:
            # Stop the test case on equipment
            tc_state = self._get_test_case_state()
            if tc_state == "RUNNING":
                self._stop_test_case()

            # Close the CONFIGURATION_FILE on the equipment
            self._unload_configuration_file()

        return Global.SUCCESS, "No Errors"
