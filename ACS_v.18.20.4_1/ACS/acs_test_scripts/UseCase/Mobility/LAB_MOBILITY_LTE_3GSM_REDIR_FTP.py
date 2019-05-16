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
:summary:  This file implements usecase that do Inter-RAT redirection from
LTE network to UTRAN/GSM during FPT data transfer
:since: 03/09/2014
:author: mariussX
"""

import os
import datetime
import time
from LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.CommunicationUtilities import SerialHandler, ATCommandAnalyser
from ErrorHandling.TestEquipmentException import TestEquipmentException


class LabMobilityLte3gsmRedirFtp(LabMobilityLte3gsmBase):

    """
    Lab I-RAT LTE - 3G/2G FTP
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LabMobilityLte3gsmBase Init function
        LabMobilityLte3gsmBase.__init__(self, tc_name, global_config)

        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_LTE_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Define PING parameters
        self._packet_size = 32
        self._nb_pings = 10

        # Read the the direction file name from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION")

        # Checking if the entered direction is correct.
        if self._direction not in ("UL", "DL", "BOTH"):
            self._error.Msg = "%s is not a known xfer direction" % \
                self._direction
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        # Read the ftp file name from UseCase xml Parameter
        if self._direction == "DL":
            self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILENAME"))
            self._ftp_filename = self._ftp_filename.replace('\\', '/')
        elif self._direction == "UL":
            # Read the UL_FILE value from UseCase xml Parameter//
            self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("UL_FILENAME"))
            self._ftp_filename = self._ftp_filename.replace('\\', '/')
        elif self._direction == "BOTH":
            self._dl_ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILENAME"))
            self._dl_ftp_filename = self._dl_ftp_filename.replace('\\', '/')
            # Read the UL_FILE value from UseCase xml Parameter
            self._ul_ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("UL_FILENAME"))
            self._ul_ftp_filename = self._ul_ftp_filename.replace('\\', '/')

        # Read CH_BANDWIDTH from test case xml file
        self._bandwidth = \
            self._tc_parameters.get_param_value("CELL_CHANNEL_BANDWIDTH", "20")

        # Read ANTENNAS NUMBER from test case xml file
        self._antennas_number = \
            self._tc_parameters.get_param_value("ANTENNAS_NUMBER", '1')

        # Read TRANSMISSION_MODE from test case xml file
        self._transmission_mode = \
            self._tc_parameters.get_param_value("TRANSMISSION_MODE","TM1")
        # Read BANDWIDTH from test case xml file
        self._type0_bitmap = \
            str(self._tc_parameters.get_param_value("TYPE0_BITMAP","63"))
        # Read DL_RB_SIZE from test case xml file
        self._dl_nb_rb = \
            self._tc_parameters.get_param_value("DL_RB_SIZE","50")
        # Read UL_RB_SIZE from test case xml file
        self._ul_nb_rb = \
            self._tc_parameters.get_param_value("UL_RB_SIZE","10")
        # Read I_MCS from test case xml file
        self._dl_i_mcs = \
            self._tc_parameters.get_param_value("DL_I_MCS","23")
        # Read I_MCS from test case xml file
        self._ul_i_mcs = \
            self._tc_parameters.get_param_value("UL_I_MCS","3")
            # Read UL Grant Mode from test case xml file
        self._ul_grant_mode = \
            self._tc_parameters.get_param_value("UL_GRANT_MODE", "FIXEDMAC")

        # Read the the ip _version file name from UseCase xml Parameter
        self._ip_version = self._tc_parameters.get_param_value("IP_VERSION", "IPV4")

        # Get the server ipv6 address from the BenchConfig, if the key
        # is present in the file.
        if self._server.has_parameter("IPV6"):
            self._server_ip_v6_address = self._server.get_param_value("IPV6")

        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = self._tc_parameters.get_param_value("XFER_TIMEOUT")
        if self._xfer_timeout is not None and str(self._xfer_timeout).isdigit():
            self._xfer_timeout = int(self._xfer_timeout)
        else:
            self._xfer_timeout = None

        if self._xfer_timeout is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "XFER_TIMEOUT should be int")

        self._two_eps_bearers = self._tc_parameters.get_param_value("TWO_EPS_BEARER")
        self._two_eps_bearers = \
            str_to_bool(self._two_eps_bearers)

        # In case of 2 PDPs, define the APN and IP address for the 2nd one
        if self._two_eps_bearers is True:
            self._second_apn = "agilent2"
            self._second_dut_ip_version = "IPV4"
            self._second_dut_ip_address = "10.102.243.25"

        # Read Inter-Rat redirection type
        self._irat_type = self._tc_parameters.get_param_value("I-RAT_TYPE")
        if self._irat_type is None:
            self._irat_type = "NORMAL"
        elif self._irat_type not in ("NORMAL", "BLIND"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "I-RAT_TYP should be 'NORMAL' or 'BLIND'")

        # Default timeout for DUT camping
        self._camp_timeout = 180

        # Define timeout for AT command
        self._command_timeout = 10

        # Define AT PROXY launch mode
        self._launch_mode = 2

        # Initializing the variable which will contain the IP address to use.
        self._ip_address = None

        self._phone_system = self._device.get_uecmd("PhoneSystem")

        # if 2 EPS bearers were requested
        if self._two_eps_bearers == True:
            # Instantiate the modem UE commands
            self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")

            # Get Serial Handler instance
            self._serial_handler = SerialHandler()

            # Win COM port to be used
            self.at_proxy_com_port = None

            # Store the AT proxy connection availability
            self.modem_available = False

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        # Stay on if plugged
        self._phone_system.set_stay_on_while_plugged_in(3)

        # Remove screen off timeout
        self._phone_system.set_screen_timeout(3600)

        # Call LabMobilityLte3gsmBase set_up function
        LabMobilityLte3gsmBase.set_up(self)

        # Flight mode activation
        self._networking_api.set_flight_mode("on")

        # Set Network Simulator 3GSM cell on
        self._ns_3gsm_cell.set_cell_on()

        # Set the same DUT IP address also for Agilent
        self._ns_3gsm_data.set_dut_ip_address(1, self._ns1_ip_dut)

        if self._two_eps_bearers is True:
            # Configure secondary PDP/EPS bearer on the NW simulators
            self._ns_lte.stop_scenario()
            self._configure_for_second_pdp()
            self._ns_lte.start_scenario()

        time.sleep(self._wait_btwn_cmd)

        if self._ns_3gsm_cell_tech == "3G":
            # Sets RAU FOP on 3G cell to manual:
            # FOProceed:CONTrol:AUTO 0
            # FOProceed:MANual 1
            self._ns_3gsm_cell.set_rau_fop_control(0,1)
            time.sleep(self._wait_btwn_cmd)

        #  Set External EPC connection
        self._ns_3gsm_cell.set_external_epc_connection(self._ns_lte_ip_lan1,
                                                       self._ns_lte_dl_earfcn)

        # Set LTE Cell on
        self._ns_lte_cell.set_cell_on(self._ns_lte_mimo)

        # Set cell power to -115 dBm
        self._ns_3gsm_cell.set_cell_power(-115)

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, False)

        # Check data connection state is "CON"
        self._ns_lte_data.check_data_connection_state("CON",
                                                  self._camp_timeout,
                                                  blocking = False,
                                                  cell_id = self._ns_lte_cell_id)

        # Get RAT from Equipment
        network_type = self._ns_lte_data.get_network_type()
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Wait 10 seconds
        time.sleep(10)

        if self._irat_type == "NORMAL":
            # Set cell power to -70 dBm
            self._ns_3gsm_cell.set_cell_power(-70)

        # Start FTP service on equipment side
        self._ns_lte_data.start_ftp_service()

        # Selecting the  IPV6 address of the FTP server, according to
        # the TC parameter value.
        if self._ip_version == "IPV6":
            if self._server_ip_v6_address is not None:
                # If Protocol is IPV6 use IPV6 address.
                log_msg = "Using IPV6 address to connect to the FTP server."
                self._logger.info(log_msg)
                self._ip_address = self._server_ip_v6_address
            else:
                # If IPV6 address is not present in the BenchConfig.
                msg = "The IPV6 parameter is missing from the Bench Config!"
                raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)
        else:
            self._ip_address = self._server_ip_address

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        Test steps:
            Launch FTP.
            'Normal' or 'blind' Inter-RAT redirection from LTE to UTRAN/GSM
            Wait for the transfer to finish.
            Measure FTP duration
            Get transferred file size on phone
            Compute throughput
            We are using here ftpput and ftpget methods
        """

        # -------------------- 2 EPS bearers procedure -----------------------

        # If 2 EPS bearer are requested to be activated
        if self._two_eps_bearers == True:
            #Start up the proxy for sending AT commands
            self.at_proxy_com_port = self._modem_flashing_api.start_at_proxy_from_mos(
                                                                            int(self._launch_mode))
            #check serial connection
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

            # Activate the 2nd EPS bearer
            self._activate_2nd_eps_bearer()

            # Query for the number of active PDPs
            number_active_pdp = self._get_number_active_pdp()
            if number_active_pdp != 2:
                self._error.Code = Global.FAILURE
                self._error.Msg = "The number of the active EPS bearers is not the expected one (2). Currently active: %d" \
                                    % number_active_pdp
                return self._error.Code, self._error.Msg

        # Ping the FTp server first
        packet_loss = self._networking_api.\
            ping(self._server_ip_address,
                 self._packet_size,
                 self._nb_pings)

        # Check packet loss
        if packet_loss.value > 80:
            self._error.Code = Global.FAILURE
            self._error.Msg = "Pinging the FTP server has failed"
            return self._error.Code, self._error.Msg


        # -------------------- FTP procedure -----------------------

        if self._direction == "UL":
            ftp_file_path = os.path.join(self._device.multimedia_path,
                                       self._tc_parameters.get_param_value("UL_FILENAME"))
            d1 = datetime.datetime.now()
            (process, _q) = self._networking_api.ftpput(self._ip_address,
                                                      self._username,
                                                      self._password,
                                                      self._ftp_filename,
                                                      ftp_file_path)
        elif self._direction == "DL":
            # the FTP download file will write to /data/ to get max throughput
            # because of emmc sequential write limitation
            ftp_file_path = os.path.join(self._device.binaries_path,
                                       self._tc_parameters.get_param_value("DL_FILENAME"))
            d1 = datetime.datetime.now()

            (process, _q) = self._networking_api.ftpget(self._ip_address,
                                                      self._username,
                                                      self._password,
                                                      ftp_file_path,
                                                      self._ftp_filename)

        elif self._direction == "BOTH":
            # Launch FTP download
            # the FTP download file will write to /data/ to get max throughput
            # because of emmc sequential write limitation
            dl_ftp_file_path = os.path.join(self._device.binaries_path,
                                       self._tc_parameters.get_param_value("DL_FILENAME"))
            d1_dl = datetime.datetime.now()
            (process_dl, _q_dl) = self._networking_api.ftpget(self._ip_address,
                                                      self._username,
                                                      self._password,
                                                      dl_ftp_file_path,
                                                      self._dl_ftp_filename)
            # Launch FTP upload
            ul_ftp_file_path = os.path.join(self._device.multimedia_path,
                                       self._tc_parameters.get_param_value("UL_FILENAME"))
            d1_ul = datetime.datetime.now()
            (process_ul, _q_ul) = self._networking_api.ftpput(self._ip_address,
                                                      self._username,
                                                      self._password,
                                                      self._ul_ftp_filename,
                                                      ul_ftp_file_path)

            while process_ul.poll() is None and process_dl.poll() is None:
                pass
            if process_dl.poll() is not None:
                # Compute end of transfer time
                d2_dl = datetime.datetime.now()
                while process_ul.poll() is None:
                    pass
                d2_ul = datetime.datetime.now()
            elif process_ul.poll() is not None:
                # Compute end of transfer time
                d2_ul = datetime.datetime.now()
                while process_dl.poll() is None:
                    pass
                d2_dl = datetime.datetime.now()

        # Wait 30 seconds during FTP data transfer and then redirect to UTRAN/GSM cell
        while process.poll() is None:
            d2 = datetime.datetime.now()
            d3 = d2 - d1
            # Wait 30 seconds and send DUT release
            if d3.seconds > 30:
                # Check if the file exists
                if (self._direction == "DL"):
                    self._phone_system.check_file_exist(ftp_file_path)

                if self._irat_type == "BLIND":
                    # Set cell power to -70 dBm
                    self._ns_3gsm_cell.set_cell_power(-70)
                    #time.sleep(self._wait_btwn_cmd)

                # Initiate a LTE RRC Connection Release
                self._logger.info("Send LTE Detach")
                self._ns_lte_data.ue_detach()
                # Exit the loop
                break
            # Sleep 1 second
            time.sleep(1)

        # Compute end of transfer time
        #d2 = datetime.datetime.now()
        timeout = 30
        if self._ns_3gsm_cell_tech == "3G":
            # Check Data Connection State on 3GSM cell => ATTACHED before timeout
            result = self._ns_3gsm_data.check_data_connection_state("ATTACHED",
                                                               timeout,
                                                               blocking=True)
        if process.poll() is not None:
            self._logger.info("Transfer ended after DUT is attached on 3G cell - %s"\
                              %(str(process.poll())))

        # Wait 20 seconds
        self._logger.info("Wait 20 seconds")
        time.sleep(20)

        # Gets the connection status
        connection_status = self._ns_3gsm_data.get_data_connection_status()
        t1 = datetime.datetime.now()

        # Wait 60 seconds during FTP data transfer
        self._logger.info("Check if FTP data transfer remains active for at least 60 seconds after IRAT redirection")
        while process.poll() is None:

            t2 = datetime.datetime.now()
            t3 = t2 - t1
            # Gets the connection status
            connection_status = self._ns_3gsm_data.get_data_connection_status()

            # Exit the loop if PDP has been released
            if (self._ns_3gsm_cell_tech == "3G") and \
                (connection_status not in ("PDP_ACTIVATING","PDP_ACTIVE")):
                break

            # Exit the loop if PDP has been released
            if (self._ns_3gsm_cell_tech == "2G") and \
                (connection_status not in ("TRANSFERRING","PDP_ACTIVE")):
                break

            # Wait 60 seconds and exit the loop
            if t3.seconds > 60:
                break
            # Sleep 1 second
            time.sleep(1)

        # Compute end of transfer time
        d2 = datetime.datetime.now()
        tput = self._compute_ftp_tput(d1, d2, ftp_file_path)

        try:
            if (tput == -1.0) or \
                ((t3 is not None) and (t3.seconds < 60)):
                self._error.Msg = "FTP data transfer has been interrupted"
                return Global.FAILURE, self._error.Msg

        except UnboundLocalError:
            self._error.Msg = "FTP data transfer has been interrupted"
            return Global.FAILURE, self._error.Msg

        # If 2 EPS bearer were requested to be activated
        if self._two_eps_bearers == True:
            # Query for the number of active PDPs
            number_active_pdp = self._get_number_active_pdp()

        self._error.Msg = "Throughput is %.2f kbits/s" % (tput)
        if tput > 0:
            self._error.Code = Global.SUCCESS
        else:
            self._error.Code = Global.FAILURE

        # If 2 EPS bearer were requested to be activated check how many are active after IRAT redirection
        if self._two_eps_bearers == True:
            # Query for the number of active PDPs
            number_active_pdp = self._get_number_active_pdp()

            if number_active_pdp != 1:
                self._error.Code = Global.FAILURE
                self._error.Msg = "The number of the active PDPs is not the expected one (1). Currently active: %d" \
                                    % number_active_pdp
                return self._error.Code, self._error.Msg

        return self._error.Code, self._error.Msg

#-----------------------------------------------------------------------------

    def tear_down(self):
        """
        Finishing the test.
        Stopping the FTP service and releasing the equipment.
        """
        # Stopping the FTP service before releasing the equipment.
        self._ns_lte_data.stop_ftp_service()

        # Disconnect from external EPC
        self._ns_3gsm.disconnect_from_external_epc()

        if self._ns_3gsm_cell_tech == "3G":
            # Sets RAU FOP on 3G cell to auto:
            # FOProceed:CONTrol:AUTO 1
            # FOProceed:MANual 0
            self._ns_3gsm_cell.set_rau_fop_control(1,0)

        # Reset the stay on setting if plugged
        self._phone_system.set_stay_on_while_plugged_in(0)

        # Set the initial screen off timeout
        self._phone_system.set_screen_timeout(60)

        # If 2 EPS bearers were requested
        if self._two_eps_bearers == True:
            #Stop AT Proxy
            self._logger.info("Stop AT Proxy")
            self._modem_flashing_api.stop_at_proxy_from_mos()

            #Close the serial connection
            self._logger.info("Close the serial connection")
            self._logger.info("Disconnecting from the port " +
                                str(self._serial_handler.get_port()))
            self._serial_handler.disconnect()

        LabMobilityLte3gsmBase.tear_down(self)

        return Global.SUCCESS, "No errors"

#-----------------------------------------------------------------------------

    def _compute_ftp_tput(self, d1, d2, ftp_file_path):
        """
        compute FTP data throughput
        :type d1: datetime
        :param d1: FTP start time
        :type d2: datetime
        :param d2: FTP end time
        :type ftp_file_path: string
        :param ftp_file_path: path to ftp transferred file on phone

        :rtype: float
        :return: throughput in kbps or -1 if an error occured
        """

        d3 = d2 - d1
        # Remove 1 second to total duration for FTP connection establishment time
        d = (d3.seconds - 1) * 1000000 + d3.microseconds
        self._logger.info("FTP finished or interrupted on purpose in %d seconds "
                          % (d3.seconds))

        # If transfer time is too short it means that FTP connection
        # was lost (ftpput and get do not return error)
        if d3.seconds < 3:
            self._error.Msg = "The FTP transfer is too short (less than 3 seconds), FTP connection is lost"
            return -1.0

        # Check if the file exists
        if (self._direction == "DL"):
            self._phone_system.check_file_exist(ftp_file_path)

        size = self._phone_system.get_file_size(ftp_file_path)
        self._logger.info("file size %d" % size)
        # compute throughput = file_size in kbits/duration in s
        #                    = (size * 8/1000) / (d/1000000)
        #                    = (size * 8 * 1000) / d
        tput = float((size * 1000 * 8) / d)
        self._logger.info("Measured Throughput  %.2f kbits/s" % tput)

        return tput

#------------------------------------------------------------------------------

    def _get_lte_max_tput_params(self):
        """
        Compute and return lte parameters to reach the phone maximum throughput.
        Read max lteCategory supported from DeviceCatalog.xml
        get antennas number and bandwidth for maximum throughput available for this category

        :rtype: str
        :return: lte_category, the maximum lte category supported by the phone
        :rtype: str
        :return: mimo, mimo support by phone
        :rtype: str
        :return: bw, the maximum bandwidth supported by phone
        :rtype: str
        :return: antennas_number, the maximum number of antenna supported by phone
        """

        mimo = "False"
        antennas_number = "1"
        bw = "10"
        lte_category = str(self._dut_config.get("maxSupportedLteCategory", "3"))
        if int(lte_category) > 1:
            mimo = "True"
            antennas_number = "2"
        if int(lte_category) > 2:
            bw = "20"

        return mimo, lte_category, bw, antennas_number

#------------------------------------------------------------------------------

    def _activate_2nd_eps_bearer(self):
        """
        Activate the 2nd EPS bearer on LTE cell
        Raise errors if some AT commands dones't return the expected message for a succesfull activation

        """

        self._logger.info("Start the procedure for activating the 2nd EPS bearer")

        # Define the 2nd EPS bearer APN on CID 2
        at_command_config_2nd_eps = "at+cgdcont=2,\"IP\",\"%s\"" %(self._second_apn)
        expected_at_command_response = "OK"

        # Execute AT command
        at_command_verdict, at_command_response_raw =\
                self._serial_handler.send_at_command_and_get_result(at_command_config_2nd_eps,
                                                                    self._command_timeout)

        if at_command_response_raw != expected_at_command_response:
            self._logger.debug("at_command_response_raw = " + at_command_response_raw)
            self._error.Msg = "Failed to configure the 2nd EPS bearer"
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     self._error.Msg)

        # Wait 5 seconds
        time.sleep(5)

        # Activate 2nd EPS bearer
        at_command_activate_2nd_eps_bearer = "at+cgact=1,2"
        expected_at_command_response = "OK"
        at_command_verdict, at_command_response_raw =\
                self._serial_handler.send_at_command_and_get_result(at_command_activate_2nd_eps_bearer,
                                                                    self._command_timeout)

        if at_command_response_raw != expected_at_command_response:
            self._logger.debug("at_command_response_raw = " + at_command_response_raw)
            self._error.Msg = "Failed to configure the 2nd EPS bearer"
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, self._error.Msg)

        # Wait 10 seconds
        time.sleep(10)

        at_comand_interogate_status_eps_bearers = "at+cgact?"
        expected_at_command_response = "+CGACT: 1,1+CGACT: 2,1OK"
        at_command_verdict, at_command_response_raw =\
                self._serial_handler.send_at_command_and_get_result(at_comand_interogate_status_eps_bearers,
                                                                    self._command_timeout)

        if at_command_response_raw != expected_at_command_response:
            self._logger.debug("at_command_response_raw = " + at_command_response_raw)
            self._error.Msg = "Failed to configure the 2nd EPS bearer"
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, self._error.Msg)

        # Wait 5 seconds
        time.sleep(5)

#------------------------------------------------------------------------------

    def _get_number_active_pdp(self):
        """
        Return the number of the activated PDPs

        :rtype: int
        :return: number_pdp_active, the number of active PDPs
        """

        # Initialize the number of active PDPs
        number_pdp_active = 0

        # Execute "AT+CGACT?" AT command to see the status of active PDPs
        at_comand_interogate_status_eps_bearers = "at+cgact?"
        at_command_verdict, at_command_response =\
                self._serial_handler.send_at_command_and_get_result(at_comand_interogate_status_eps_bearers,
                                                                    self._command_timeout)
        self._logger.debug("at_command_response = " + at_command_response)

        # Check how many PDPs/EPS bearers are active
        i = 1
        for i in xrange(1,8):
            pdp_active_check_seq = "+CGACT: %d,1" %(i)
            if pdp_active_check_seq in at_command_response:
                number_pdp_active = number_pdp_active + 1

        # Return the number of active PDPs/EPS bearers
        return number_pdp_active

#------------------------------------------------------------------------------

    def _configure_for_second_pdp(self):
        """
        Sets the IP which will be configured for the 2nd EPS bearer (on PXT)
        and 2nd PDP context (on Agilent8960)
        """

        # Set the 2nd IP address on Agilent8960
        self._ns_3gsm_data.set_dut_ip_address(2, self._second_dut_ip_address)

        time.sleep(self._wait_btwn_cmd)

        # Configure the EPS bearer config 2 on PXT
        self._ns_lte_data.configure_eps_bearer_config_2(self._second_apn,
                                                        self._second_dut_ip_version,
                                                        self._second_dut_ip_address)
