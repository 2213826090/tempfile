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
:summary: This file implements the USB TETHERING UC
:since: 01/07/2013
:author: jduran4x
"""

import time
import os

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.ADBUtilities import ADBSocket
from acs_test_scripts.UseCase.Networking.LAB_HSPA_BASE import LabHspaBase
from acs_test_scripts.UseCase.Networking.LAB_EGPRS_BASE import LabEgprsBase
from acs_test_scripts.UseCase.Networking.LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Utilities.DataUtilities import FtpClient
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.CommunicationUtilities import KPIUtil, throughput_targets_string
from acs_test_scripts.Utilities.IPerfUtilities import compute_iperf_verdict
from UtilitiesFWK.Utilities import format_exception_info
from acs_test_scripts.Utilities.ThroughputMeasure import DuplexThroughputMeasure, ThroughputMeasure


class AnyUsbTether(object):
    """
    provides the test of USB tethering connection
    once tethering connection is established over USB
    the connection quality is tested using a FTP transfer
    """

    def __new__(cls, tc_name, global_config):
        rat = \
            tc_name.get_params().get_param_value("RAT", "").upper()
        if "2G" in rat:
            obj = super(AnyUsbTether, cls).__new__(LabEgprsUsbTether)
        elif "3G" in rat:
            obj = super(AnyUsbTether, cls).__new__(LabHspaUsbTether)
        elif "4G" in rat:
            obj = super(AnyUsbTether, cls).__new__(LabLteUsbTether)
        elif "LIVE" in rat:
            obj = super(AnyUsbTether, cls).__new__(LiveUsbTether)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_TEST_CASE_FILE,
                                     "RAT parameter not defined!")
        obj._rat = rat
        return obj

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """
        # Get Test Case Parameters
        self._ping = self._tc_parameters.get_param_value("PING_NUMBER", 0, int)

        self._ftp_file = self._tc_parameters.get_param_value("FTP_FILE")

        self._timeout = \
            float(self._tc_parameters.get_param_value("TIMEOUT"))

        self._direction = self._tc_parameters.get_param_value("DIRECTION")

        self._computer = \
            self._em.get_computer("COMPUTER1")

        self._ftp = FtpClient()

        self._adb = ADBSocket()

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

        # Failure targets
        self._failure_targets = self._tc_parameters.get_param_value("FAILURE_TARGETS")

        # Get FTP server IP address
        lab_server = None
#         if "4G" in self._rat:
#             # Try to get LAB_LTE_SERVER from bench config
#             try:
#                 self._logger.info("LAB_LTE_SERVER will be used for this TC")
#                 lab_server = global_config.benchConfig.get_parameters("LAB_LTE_SERVER")
#             except:
#                 self._logger.warning("LAB_LTE_SERVER not present in config file, searching for LAB_SERVER")
        # if lab_server still not defined...
        if lab_server is None:
            # try to get LAB_SERVER from bench config
            try:
                self._logger.info("LAB_SERVER will be used for this TC")
                lab_server = global_config.benchConfig.get_parameters("LAB_SERVER")
            except:
                msg = "No server identified in bench config to execute this TC"
                self._logger.error(msg)
                # No server is found in Bench Config: TC aborted.
                return Global.FAILURE, msg
        self._ftp_server = lab_server.get_param_value("IP")

        # username and password should have already set
        self._username = lab_server.get_param_value("username")
        if self._username in ("None", "", None):
            self._username = "anonymous"
        self._password = lab_server.get_param_value("password")
        if self._password in ("None", "", None):
            self._password = ""

        self._perform_ftp = (self._ftp_server is not None and self._direction is not None
                             and self._ftp_file is not None)
        # Detect if it is a KPI test or not
        self._kpi_test = self._tc_parameters.get_param_value("KPI_TEST", False, "str_to_bool")
        if self._kpi_test:
            self._logger.info("It is a KPI test")
            self._kpi_data = None
            self._current_iteration = 0
        if self._direction.upper() == "UL":
            self._verdict_direction = "up"
        if self._direction.upper() == "DL":
            self._verdict_direction = "down"

# ------------------------------------------------------------------------------

    def set_up(self):
        """
        Initializes this tests.
        """
        # Reset KPI data needed for computing KPI test result
        if self._kpi_test:
            self._kpi_data = KPIUtil()
            self._current_iteration = 0

        # if FTP transfer will be perform
        if self._perform_ftp:

            # check FTP_FILE or local path exist
            if self._direction == "UL":
                # should check local file to upload exist
                local_file_or_path = os.path.join(os.getcwd(), "_Embedded", "USERDATA", self._ftp_file)
                msg = "FTP_FILE parameter: local file (%s) not exist" % str(local_file_or_path)
            else:
                # should check local path use to store FTP file to download exist
                local_file_or_path = os.path.join(os.getcwd(), "_Embedded", "USERDATA")
                msg = "LOCAL_FILE parameter: local path (%s) not exist" % str(local_file_or_path)
            if os.path.exists(local_file_or_path):
                if self._direction == "UL":
                    self._ftp_file = os.path.join(os.getcwd(), "_Embedded", "USERDATA", self._ftp_file)
            else:
                if self._direction == "UL":
                    # In case of UL file to transfer does not exist, create it from its name
                    self._logger.info("File %s does not exist, generate a file with a size based in its name" % self._ftp_file)
                    self._ftp.retrieve_size_from_filename_and_create_it(local_file_or_path)
                    self._ftp_file = os.path.join(os.getcwd(), "_Embedded", "USERDATA", self._ftp_file)
                else:
                    raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, msg)

        # In case USB tethering is already activated
        self._disable_tethering()

        # Enable USB tethering connection for KPI tests
        # (no need to activate tethering before each FTP transfer)
        if self._kpi_test:
            try:
                self._enable_tethering()
            except:
                exception_text = format_exception_info()
                self._logger.error("Failed to enable USB tethering! ")
                self._logger.debug("%s ", exception_text)
                return Global.FAILURE, "Failed to enable USB tethering"

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Executes the test
        """
        return_status = Global.FAILURE
        return_message = ""
        return_message_tmp = ""
        throughput_value = 0

        # Step 1 - Enable USB Tethering for non-KPI test
        # -----------------------------
        if not self._kpi_test:
            try:
                self._enable_tethering()
            except:
                exception_text = format_exception_info()
                self._logger.error("Failed to enable USB tethering! ")
                self._logger.debug("%s ", exception_text)
                return Global.FAILURE, "Failed to enable USB tethering"

        # Step 2 (optional) - Perform Successful Ping
        # -------------------------------------------
        if self._ping > 0:
            self._logger.info("Try to perform %s successful ping" % self._ping)
            try:
                packet = self._computer.ping(self._ftp_server, 32, self._ping, unreach_loss=True)
                self._logger.debug("Ping returned loss: %d%s" % (packet.value, packet.units))
                return_message += "Ping: %d packets send and receive successfully and loss is %d%s" \
                                  % (self._ping, packet.value, packet.units)
                if packet.value == 0:
                    return_status = Global.SUCCESS
                else:
                    return Global.FAILURE, return_message
            except TestEquipmentException as error:
                self._logger.error("Successful ping test Fail.")
                return_message += "Error: Failed to perform %d ping (%s)." % (self._ping, str(error))
                return Global.FAILURE, return_message
            self._logger.info("Successful ping test Pass.")

        # Step 3 (Optional) - Perform FTP upload or download
        if self._perform_ftp:
            msg = "Perform FTP transmission (%s) of file: %s" % (self._direction, self._ftp_file)
            self._logger.info(msg)
            # Start FTP transmission
            try:
                self._ftp.start_ftp_xfer(self._direction,
                                         self._ftp_server,
                                         self._username,
                                         self._password,
                                         self._ftp_file,
                                         self._ftp_path)
                # Throughput is returned in B/s, transform it to Mbits/s
                # for comparison with targets
                throughput_value = 8 * self._ftp.get_data_throughput() / 1e6
                throughput = DuplexThroughputMeasure()
                if self._direction.upper() == "UL":
                    throughput.ul_throughput.set(throughput_value, ThroughputMeasure.MBPS_UNIT)
                if self._direction.upper() == "DL":
                    throughput.dl_throughput.set(throughput_value, ThroughputMeasure.MBPS_UNIT)
                self._logger.info("FTP transmission Pass")
                return_message += "- FTP transmission Pass - "
                # In case of KPI test, store measured throughput
                # for final verdict (it will depend of the median value of all measured values)
                (return_status, return_message_tmp) = compute_iperf_verdict(throughput,
                                                                            self._throughput_targets,
                                                                            self._verdict_direction)
                return_message += return_message_tmp
                if self._kpi_test:
                    return_message = "KPI test Iteration: %d " % (self._current_iteration + 1) + return_message_tmp
                    self._kpi_data.append(throughput)
            except Exception as e:
                return_message += "- FTP transmission Failed"
                return_status = Global.FAILURE
                exception_text = format_exception_info()
                self._logger.error("FTP transmission Failed: %s ", exception_text)
                # Restart tethering on KPI test
                if self._kpi_test:
                    self._disable_tethering()
                    self._enable_tethering()
            finally:
                if self._kpi_test:
                    (return_status, return_message) = self._perform_kpi_throughput_computation(return_message)

        # Step 4 - Disable USB tethering for non-KPI tests
        if not self._kpi_test:
            self._disable_tethering()

        # Step 5 (optional) - Perform Unsuccessful Ping
        if self._ping > 0:
            self._logger.info("Try to perform %s unsuccessful ping" % self._ping)
            try:
                packet = self._computer.ping(self._ftp_server, 32, self._ping, unreach_loss=True)
                self._logger.debug("Ping loss is: %d%%" % packet.value)
                return_message += "- Ping: %d packets send and receive and loss is %d%%" \
                                  % (self._ping, packet.value)
                if packet.value >= 100:
                    return_status = Global.SUCCESS
                else:
                    return Global.FAILURE, return_message
            except:
                self._logger.error("Unsuccessful ping test Fail.")
                return_message += "- Failed to perform %d ping." % self._ping
                return_status = Global.SUCCESS
            self._logger.error("Unsuccessful ping test Pass.")

        # Final return value must be Global.SUCCESS if all steps are passed
        return return_status, return_message

# ------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        # Abort FTP upload or download
        if self._perform_ftp:
            self._logger.info("FTP disconnect")
            try:
                self._ftp.ftp_disconnect()
            except:
                self._logger.warning("Failed to perform FTP disconnect")
        # Disable USB tethering connection
        # (To let the phone in a clean state)
        self._disable_tethering()

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------

    def _enable_tethering(self):
        self._logger.info("Kill adb server")
        self._adb.stop_adb_server()
        self._logger.info("Enabling tethering connection over USB")
        self._networking_api.start_usb_tethering()
        # Get the USB Tethering connection info
        usb_if_number, usb_ip_addr, usb_gateway_addr = self._computer.search_for_interface()

        # Add the route with all the correct parameters
        if usb_ip_addr is not None:
            self._logger.info("Tethering interface detected, set IP route now")
            # Replace last digit of ftp IP address with "0"
            route_ip_addr = self._computer.get_ip_mask(self._ftp_server)
            # This mask is always the same
            route_ip_Mask = "255.255.255.0"
            self._computer.add_route(route_ip_addr, route_ip_Mask, usb_gateway_addr, usb_if_number)
        else:
            self._logger.error("Failed to find IP address for USB tethering")
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Failed to find IP address for USB tethering")

        if self._rat == "3G":
            # Set the initial PS Data RRC State to DCH
            self._logger.info("Set Test equipment initial RRC state to DCH")
            self._ns_data.set_initial_ps_data_rrc_state("DCH")

        # don't continue unless the connection is established
        end_time = time.time() + self._timeout
        timeout = False
        self._logger.info("Waiting until tethering connection is established")
        while not timeout:
            try:
                packet = self._computer.ping(self._ftp_server, 32, 1)
                self._logger.debug("Ping returned loss: %d%s" % (packet.value, packet.units))
                if packet.value == 0:
                    self._logger.info("Tethering Connection established")
                    break
            except TestEquipmentException:
                # the ping function raises an error if the pinged address is not
                # reachable
                pass
            time.sleep(1)
            timeout = time.time() > end_time
        else:
            self._logger.error("Timeout during establishing tethering connection!")
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Fail establishing connection")

    def _disable_tethering(self):
        self._logger.info("Stopping tethering connection over USB")
        self._networking_api.stop_usb_tethering()

        # Delete route to tethering connection over USB
        self._logger.info("Deleting USB tethering connection route")
        self._computer.delete_route(self._computer.get_ip_mask(self._ftp_server))

        # Give time to DUT to clean IP route
        time.sleep(10)

        self._logger.info("Restart adb server")
        self._adb.adb_start()

    def _perform_kpi_throughput_computation(self, msg):
        """
        Perform KPI median throughput computation

        :type msg: str
        :param msg: msg to append to result

        :return: status result
        :rtype: integer

        :return: message result
        :rtype: str

        """
        self._current_iteration += 1
        # For KPI test verdict is computed only on median throughput computed on last iteration ,
        # other reasons(exception, iteration not run, not last iteration ...) does not alter the verdict
        return_status = Global.SUCCESS
        return_message = msg

        # If we are in the latest iteration of a KPI test
        # compute throughput median value and compute verdict on this median value
        if self._current_iteration == self.get_b2b_iteration():
            median_throughput = self._kpi_data.get_median_throughput()
            (return_status, return_message_tmp) = \
                compute_iperf_verdict(median_throughput,
                                      self._throughput_targets,
                                      self._verdict_direction)
            return_message = "KPI median throughput: " + return_message_tmp
        return return_status, return_message

# ------------------------------------------------------------------------------


class LabEgprsUsbTether(AnyUsbTether, LabEgprsBase):
    """
    USB tethering connection test specific for EGPRS lab network
    """
    def __init__(self, tc_name, global_config):
        LabEgprsBase.__init__(self, tc_name, global_config)
        AnyUsbTether.__init__(self, tc_name, global_config)
        # if FTP transfer will be perform
        if self._perform_ftp:
            # Update the failure targets
            self._throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                        self._failure_targets,
                                                                        self._kpi_test,
                                                                        tc_name._name)
            # Log Throughput targets for EGPRS
            self._logger.info(throughput_targets_string(self._throughput_targets))

        self._ns_data = self._ns_data_2g
        self._ns_cell = self._ns_cell_2g

    def set_up(self):
        (code, message) = LabEgprsBase.set_up(self)
        if code == Global.SUCCESS:
            (code, message) = AnyUsbTether.set_up(self)
        return code, message

    def run_test(self):
        (code, message) = LabEgprsBase.run_test(self)
        if code == Global.SUCCESS:
            (code, message) = AnyUsbTether.run_test(self)
        return code, message

    def tear_down(self):
        AnyUsbTether.tear_down(self)
        return LabEgprsBase.tear_down(self)


class LabHspaUsbTether(AnyUsbTether, LabHspaBase):
    """
    USB tethering connection test specific for HSPA lab network
    """
    def __init__(self, tc_name, global_config):
        LabHspaBase.__init__(self, tc_name, global_config)
        AnyUsbTether.__init__(self, tc_name, global_config)
        # if FTP transfer will be perform
        if self._perform_ftp:
            # Update the failure targets
            self._throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                        self._failure_targets,
                                                                        self._kpi_test,
                                                                        tc_name._name)
            # Log Throughput targets for HSPA
            self._logger.info(throughput_targets_string(self._throughput_targets))

        self._ns_data = self._ns_data_3g
        self._ns_cell = self._ns_cell_3g

    def set_up(self):
        (code, message) = LabHspaBase.set_up(self)
        if code == Global.SUCCESS:
            (code, message) = AnyUsbTether.set_up(self)
        return code, message

    def run_test(self):
        (code, message) = LabHspaBase.run_test(self)
        if code == Global.SUCCESS:
            (code, message) = AnyUsbTether.run_test(self)
        return code, message

    def tear_down(self):
        AnyUsbTether.tear_down(self)
        return LabHspaBase.tear_down(self)


class LabLteUsbTether(AnyUsbTether, LabLteBase):
    """
    USB tethering connection test specific for Lte lab network
    """
    def __init__(self, tc_name, global_config):
        LabLteBase.__init__(self, tc_name, global_config)
        AnyUsbTether.__init__(self, tc_name, global_config)
        if self._perform_ftp:
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

            # Initializing the variable which will contain the IP address to use.
            self._ip_address = None
        self._rrc_state = self._tc_parameters.get_param_value("RRC_STATE", "RRC_CONNECTED")

    def set_up(self):
        # Setup FTP and LTE
        (code, msg) = self._setup_ftp_for_lte()

        if code == Global.SUCCESS:
            # The device and network setup are ready
            (code, msg) = AnyUsbTether.set_up(self)

        return code, msg

    def run_test(self):
        (code, message) = LabLteBase.run_test(self)
        if code == Global.SUCCESS:
            (code, message) = AnyUsbTether.run_test(self)
        return code, message

    def tear_down(self):
        AnyUsbTether.tear_down(self)
        # Stopping the FTP service before releasing the equipment.
        self._ns_data_4g.stop_ftp_service()
        return LabLteBase.tear_down(self)


class LiveUsbTether(AnyUsbTether, UseCaseBase):
    """
    USB tethering connection test specific for live network
    """
    def __init__(self, tc_name, global_config):
        self._ftp_path = None
        UseCaseBase.__init__(self, tc_name, global_config)
        AnyUsbTether.__init__(self, tc_name, global_config)
        self._throughput_targets = TelephonyConfigsParser("Throughput_Targets").\
            parse_live_wcdma_targets()

    def set_up(self):
        (code, message) = UseCaseBase.set_up(self)
        if code == Global.SUCCESS:
            (code, message) = AnyUsbTether.set_up(self)
        return code, message

    def run_test(self):
        (code, message) = UseCaseBase.run_test(self)
        if code == Global.SUCCESS:
            (code, message) = AnyUsbTether.run_test(self)
        return code, message

    def tear_down(self):
        AnyUsbTether.tear_down(self)
        return UseCaseBase.tear_down(self)
