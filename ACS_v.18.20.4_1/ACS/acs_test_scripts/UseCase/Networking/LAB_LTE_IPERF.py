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
:summary:  This file implements the LTE Iperf testCase.
:since: 06/04/2013
:author: rbertolx
"""
import time
from LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.CommunicationUtilities \
    import KPIUtil, BlerMeasurements, throughput_targets_string
from UtilitiesFWK.Utilities import format_exception_info
from acs_test_scripts.Utilities.IPerfUtilities \
    import compute_iperf_verdict, compute_iperf_bw, retrieve_parameters_from_tc_params
from acs_test_scripts.Utilities.NetworkingUtilities import wait_for_dut_ipv4_address


class LabLteIperf(LabLteBase):

    """
    Lab LTE Iperf test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        LabLteBase.__init__(self, tc_name, global_config)
        # Get the IP of the Iperf LTE server
        self._server = \
            global_config.benchConfig.get_parameters("LAB_LTE_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        # Read Iperf PORT
        self._port = int(self._tc_parameters.get_param_value("PORT"))
        # Read Iperf measurement duration
        self._duration = \
            int(self._tc_parameters.get_param_value("DURATION", "120"))
        # Read direction (Upload/Download/BOTH)
        self._direction = self._tc_parameters.get_param_value("DIRECTION", "UL")

        self._rrc_state = self._tc_parameters.get_param_value("RRC_STATE", "RRC_CONNECTED")

        ######################################################
        #                   IPERF SETTINGS                   #
        ######################################################

        # Read Iperf protocol
        self._iperf_protocol = \
            str(self._tc_parameters.get_param_value("IPERF_PROTOCOL", "UDP"))

        self._iperf_bandwidth = \
            str(self._tc_parameters.get_param_value("IPERF_BANDWIDTH", None)).upper()

        self._computer = None
        if self._ns_model == "AGILENT_E6621A":
            # Get computer type
            self._computer = \
                self._tc_parameters.get_param_value("COMPUTER",
                                                    "IPERF_SERVER")
            self._computer_type = \
                global_config.benchConfig.get_parameters("IPERF_SERVER")

        # Get TCP window size and number of iperf client threads to run concurrently
        # TCP Window size shall be greater than:
        # - Expected throughput * LTE_RTT (round trip time)
        # Max LTE throughput is 150Mbps and LTE_RTT < 20ms
        # Iperf compute automatically the windows size.
        # It is better to not set a window size as result with iperf default size are pretty good
        self._iperf_uldl_parameters = retrieve_parameters_from_tc_params(self._tc_parameters)

        self._buffer_length = \
            int(self._tc_parameters.get_param_value("BUFFER_LENGTH", 0))
        self._dut_ip_address = None

        # Detect if it is a KPI test or not
        self._kpi_test = self._tc_parameters.get_param_value("KPI_TEST", False, "str_to_bool")
        if self._kpi_test:
            self._logger.info("It is a KPI test")
            self._kpi_data = None
            self._current_iteration = 0

        # Set Iperf direction
        if self._direction in ("UL", "up", None):
            self._iperf_direction = "up"
        elif self._direction in ("DL", "down"):
            self._iperf_direction = "down"
        else:
            self._iperf_direction = "both"

        # Detect if a BLER measurement is needed. It performs BLER measurement the following way:
        # Wait a quarter of IPERF duration then perform BLER measurement during half of IPERF duration
        self._perform_bler = self._tc_parameters.get_param_value("BLER", False, "str_to_bool")

        self._iperf_settings = {}

        ######################################################
        #             LTE THROUGHPUT SETTINGS                #
        ######################################################
        self._set_lte_throughput_settings()

        # Update the failure targets
        self._throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                    self._failure_targets,
                                                                    self._kpi_test,
                                                                    tc_name._name)

        # Log Throughput targets for LTE category
        self._logger.info(throughput_targets_string(self._throughput_targets))

    def set_up(self):
        """
        Initialize the test.
        """
        # configure RRC state after cell attach process (will be used on LabLteBase set_up)
        if "IDLE" in str(self._rrc_state):
            self._ns_cell_4g.keep_rrc_connection(False)
        else:
            self._ns_cell_4g.keep_rrc_connection(True)

        # Call LAB_LTE_BASE set_up function
        LabLteBase.set_up(self)

        # Reset KPI data needed for computing KPI test result
        if self._kpi_test:
            self._kpi_data = KPIUtil()
            self._current_iteration = 0

        if self._iperf_protocol is None:
            return Global.FAILURE, "The IPERF protocol parameter should be present."

        self._iperf_settings["direction"] = self._iperf_direction
        if self._iperf_protocol.lower() == "udp":
            # Compute IPERF bandwidth from target throughput and iperf bandwidth
            compute_iperf_bw(self._iperf_settings, self._throughput_targets, self._iperf_bandwidth)

        # Set Cell on.
        self._ns_cell_4g.set_cell_on(self._mimo)

        # Activate 4G
        self._connect_dut_to_4g_cell()

        # Configuration for different LTE equipments:
        # Configuration for Rohde & Schwarz CMW500
        if "RS_CMW500" in self._ns_model:
            # Add CMW500 equipment api to perform settings on internal IPERF server/client.
            self._iperf_settings.update({"server_ip_address": self._server_ip_address,
                                         "equipment_api": self._ns_data_4g,
                                         "computer": None})
            # Be sure to use equipment api for IPERF and not Computer equipment
            self._computer = None
        else:
            # LTE equipment not supported
            return Global.FAILURE, "LTE equipment %s not supported (AGILENT_E6621A, RS_CMW500)." \
                                   % self._ns_model

        # Trying to get the DUT IP address as the CMW500 does not provide the
        # IP described in the Bench Config. The two checks are done because
        # platform can have different interface names.
        try:
            self._dut_ip_address = self._ns_dut_ip_Address
            if self._ip_version == "IPV4":
                self._dut_ip_address = wait_for_dut_ipv4_address(self._registration_timeout, self._networking_api, self._device.get_cellular_network_interface())
            if self._ip_version == "IPV6":
                # Transform IPV6 DUT Prefix (xxxx:xxxx:xxxx:xxxx::/64) => DUT IP Adress (xxxx:xxxx:xxxx:xxxx::1)
                self._dut_ip_address = self._dut_ip_address.split("/")[0] + "1"
        except (KeyboardInterrupt, SystemExit):
            raise

        # Setup the IPERF test.
        self._iperf_settings.update({"port_number": self._port,
                                     "duration": self._duration,
                                     "protocol": self._iperf_protocol.lower(),
                                     "dut_ip_address": self._dut_ip_address})
        if self._iperf_uldl_parameters != {}:
            # If it's a KPI test we should disable the setting of the window size on the CMW500
#             if self._kpi_test:
#                 self._ns_data_4g.config_window_size_off()
            self._iperf_settings.update(self._iperf_uldl_parameters)

        if self._buffer_length != 0:
            self._iperf_settings.update({"buffer_length": self._buffer_length})
        if self._ip_version == "IPV6":
            self._iperf_settings["ipv6_domain"] = True

        # Setup the IPERF server/client, if IPERF embedded on equipment (CMW500 equipment)
        if self._computer is None:
            # Stop FTP server on CMW
            self._ns_data_4g.stop_ftp_service()
            # Configure internal IPERF server/client of the equipment.
            if self._iperf_direction.upper() in ("UP"):
                self._logger.info("Enable CMW500 IPERF server.")
                self._ns_data_4g.disable_iperf_client()
                self._ns_data_4g.configure_iperf_server(self._iperf_settings)
            elif self._iperf_direction.upper() in ("BOTH"):
                # Configure first iperf server as iperf duration is set by server and client but the desired value is client duration
                self._logger.info("Enable CMW500 IPERF server.")
                self._ns_data_4g.configure_iperf_server(self._iperf_settings)
                self._logger.info("Enable CMW500 IPERF client.")
                # Add 30 more seconds to iperf service duration on CMW to allow concurrency transfer
                self._iperf_settings.update({"duration": self._duration + 15})
                # Change port for DL iperf
                self._iperf_settings.update({"port_number": self._port + 1})
                self._ns_data_4g.configure_iperf_client(self._iperf_settings)

                # Reset iperf duration and port for DUT iperf client
                self._iperf_settings.update({"duration": self._duration, "port_number": self._port})
            else:
                self._logger.info("Enable CMW500 IPERF client.")
                self._ns_data_4g.disable_iperf_server()
                self._ns_data_4g.configure_iperf_client(self._iperf_settings)

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test.
        Configuring the internal IPERF server of the equipment.
        Starting this IPERF server.
        Launching the IPERF client
        Computing the throughput to get a verdict.
        """
        result_code = Global.SUCCESS
        result_msg = ""
        result_msg_tmp = ""
        throughput = None

        # Call LAB_LTE_BASE Run function
        LabLteBase.run_test(self)
        time.sleep(self._wait_btwn_cmd)

        # Launch the IPERF test and get throughput
        try:
            if self._ns_cell_4g.get_cell_status() == "OFF":
                self._logger.info("4G cell is OFF, restarting it")
                self._ns_cell_4g.set_cell_on()
                if self._ns_cell_4g.get_cell_status() == "OFF":
                    self._logger.error("4G cell is OFF, cannot run test iteration")
                    # raise Exception in order to get proper exit handling
                    raise Exception
                self._connect_dut_to_4g_cell()
            # If transfer starts from IDLE, reactivate PDP context for windows platform
            elif self._rrc_state == "RRC_IDLE":
                self._networking_api.reactivate_pdp_context(self._apn)
            if self._perform_bler:
                # IN case of LTE TDD and UDP data transfer, perform BLER measurement the following way:
                # Wait a quarter of iperf duration (+10 seconds to handle iperf start delays) then perform bler measurement during half of iperf duration
                bler_measure = BlerMeasurements(float(self._duration) / 2, float(self._duration) / 4 + 10, self._iperf_direction, self._ns_data_4g)
                bler_measure.start()
            # clean iperf environment on DUT side
            self._networking_api.clean_iperf_env()
            throughput = self._networking_api.iperf(self._iperf_settings)

            # Retrieve measured block error rate
            bler = 0.0
            bler_msg = ""
            if self._perform_bler:
                bler_measure.join()
                if bler_measure.bler_status:
                    bler = float(bler_measure.bler_measure)
                    self._logger.info("Measured BLER: %s %%" % bler)
                else:
                    bler = 100.0
                    bler_msg = "Error during BLER measurement!! "
                    self._logger.error(bler_msg)

            (result_code, result_msg_tmp) = \
                compute_iperf_verdict(throughput,
                                      self._throughput_targets,
                                      self._iperf_direction,
                                      bler)

            if self._kpi_test:
                # In case of KPI test, store measured throughput
                # for final verdict (it will depend of the median value of all measured values)
                self._kpi_data.append(throughput, bler)
                result_msg = "KPI test Iteration: %d " % (self._current_iteration + 1) + result_msg_tmp + bler_msg
                self._logger.info(result_msg)
            else:
                result_msg = result_msg_tmp + bler_msg

        except Exception:
            result_code = Global.FAILURE
            result_msg = "!!!! WARNING Exception occurred during iperf test!!! "
            exception_text = format_exception_info()
            self._logger.debug("Exception during iperf test: %s ", exception_text)
            self._logger.error("!!!! Exception occurred during iperf test!!! ")
            # Wait end of BLER measurement
            if self._perform_bler:
                bler_measure.join()
        finally:
            if self._kpi_test:
                self._current_iteration += 1
                # For KPI test verdict is computed only on median throughput computed on last iteration ,
                # other reasons(exception, iteration not run, not last iteration ...) does not alter the verdict
                result_code = Global.SUCCESS
                # If we are in the latest iteration of a KPI test
                # compute throughput median value and compute verdict on this median value
                if self._current_iteration == self.get_b2b_iteration():
                    median_throughput = self._kpi_data.get_median_throughput()
                    median_bler = self._kpi_data.get_median_bler()
                    self._logger.info("Median Throughput : DL: %s UL %s - Median BLER : %.2f" %
                                      (str(median_throughput.dl_throughput), str(median_throughput.ul_throughput),
                                       median_bler))
                    (result_code, result_msg_tmp) = compute_iperf_verdict(median_throughput,
                                                                          self._throughput_targets,
                                                                          self._iperf_direction,
                                                                          median_bler)

                    result_msg = "KPI median throughput: " + result_msg_tmp
        # Return result
        return result_code, result_msg

# ------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finishing the test.
        Stopping the IPERF server and releasing the equipment.
        """
        # Stop and disable CMW500 IPERF server/client
        if self._computer is None:
            self._ns_data_4g.stop_iperf_service()
            self._ns_data_4g.disable_iperf_client()
            self._ns_data_4g.disable_iperf_server()

        LabLteBase.tear_down(self)
        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def _connect_dut_to_4g_cell(self):
        """
        Grouping of functions to connect the dut to the 4g cell :
        Initialization, registration, data connection, checking of RRC state.
        """
        # Deactivate the flight mode.
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          self._wanted_reg_state)

        self._modem_api.check_cdk_state_bfor_timeout(
            self._wanted_reg_state,
            self._registration_timeout)

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # Activate PDP
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.activate_pdp_context(check=False)

        # Check data connection state is "CON"
        self._check_data_connection_state("CON")

        # Get RAT from Equipment
        network_type = self._ns_data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.\
            check_network_type_before_timeout(network_type,
                                              self._registration_timeout)

        # Check if Radio Resource Control state is RRC_IDLE
        current_rrc_state = self._ns_cell_4g.get_rrc_state()
        if "RRC_IDLE" in self._rrc_state:
            # Disable data traffic
            self._networking_api.disable_output_traffic()

            self._ns_cell_4g.check_rrc_state_before_timeout("IDLE", 60)
            # Update RRC state (should be: IDLE)
            current_rrc_state = self._ns_cell_4g.get_rrc_state()
            # Re-enable data traffic
            self._networking_api.enable_output_traffic()
        # Log current RRC state (RRC_IDLE or RRC_CONNECTED)
        self._logger.info("The IPERF transfer will start from %s state" % current_rrc_state)
