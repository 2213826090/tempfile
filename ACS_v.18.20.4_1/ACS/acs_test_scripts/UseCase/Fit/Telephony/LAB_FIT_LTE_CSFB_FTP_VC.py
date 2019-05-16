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
:summary: This file implements the test of a FTP transfer on LTE followed by a
CSFB and resuming the transfer on LTE.
:since: 10/12/2013
"""

from acs_test_scripts.UseCase.Mobility.LAB_MOBILITY_LTE_CSFB import LabMobilityLteCsfb
from acs_test_scripts.UseCase.Mobility.LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
from acs_test_scripts.Device.UECmd import UECmdTypes
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global

import os
import time


class LabFitLteCsfbFtpVc(LabMobilityLteCsfb):
    """
    Class implementing the test in which you perform an CSFB on a 2G cell while
    making a FTP transfer.
    .. warning:: Only supports CSFB with 2G cells for now.
    """

    def __init__(self, tc_name, global_config):
        """
        Retrieves all the necessary parameters of the test case.
        """
        LabMobilityLteCsfb.__init__(self, tc_name, global_config)
        # Read voice call specific parameters.
        # Read the number of VC to do during the FTP transfer.
        self.nb_of_vc = self._tc_parameters.get_param_value("NB_OF_VC")
        # Get FTP parameters.
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        # Read the FTP file name from UseCase XML Parameter
        self._ftp_filename_param = self._tc_parameters.\
            get_param_value("FTP_FILENAME")
        # Read the XFER_TIMEOUT from UseCase XML Parameter
        self._xfer_timeout = self._tc_parameters.\
            get_param_value("XFER_TIMEOUT")
        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_LTE_SERVER")
        self._ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""
        self._rrc_state = self._tc_parameters.get_param_value("RRC_STATE", "RRC_CONNECTED")
        self._process = None
        self._q = None
        # Store the initial list of emergency number from UECmdTypes
        self._initial_ue_command_em_number = UECmdTypes.EMERGENCY_NUMBERS_LIST
        self._initial_emergency_numbers = None
        self._interface = None

    def set_up(self):
        """
        Checking the format of some of the input parameters and configure the
        equipments and DUT.
        """
        # Checking the format of the transfer timeout parameter.
        if str(self._xfer_timeout).isdigit():
            self._xfer_timeout = int(self._xfer_timeout)
        else:
            self._xfer_timeout = None
        # Checking the format of the number of CSFB to be made on one FTP
        # transfer.
        if not self.nb_of_vc.isdigit() or self.nb_of_vc is None:
            self.nb_of_vc = 3
        # Checking the RRC_STATE parameter is in the expected range.
        if self._rrc_state not in ("RRC_IDLE", "RRC_CONNECTED"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The RRC_STATUS should be either RRC_IDLE"
                                     " or RRC_CONNECTED not: %s"
                                     % self._rrc_state)
        # Building the path to the file used for the FTP transfer.
        self._ftp_filename = os.path.join(self._ftp_path,
                                          self._ftp_filename_param)
        # Setting the DUT APN.
        self._networking_api.set_apn(interface=self._ssid, apn=self._apn)
        # Activate the data on the DUT.
        self._networking_api.activate_pdp_context(check=False)
        # Get the name of the data interface of DUT.
        self._interface = self._device.get_cellular_network_interface()
        # Execute the setup of the base class.
        (verdict, msg) = LabMobilityLteCsfb.set_up(self)
        if verdict == Global.FAILURE:
            return verdict, msg
        #  Set External EPC connection
        self._ns_3gsm_cell.set_external_epc_connection(
                self._ns_lte_ip_lan1,
                self._ns_lte_dl_earfcn)
        # Set DTM to OFF
        self._ns_3gsm_cell.set_dtm_state("OFF")
        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test:
        1 - Checks the registration on an LTE network.
        2 - Initiate a FTP transfer.
        3 - Perform a voice call (MO or MT).
        4 - Check the CSFB as succeeded.
        5 - Release the call
        6 - check the DUT camps back on LTE
        8 - Perform 3 to 6  several times.
        7 - Finish the FTP transfer.
        """
        wait_time = 15
        iteration = 0
        # Call LabMobilityLte3gsmBase run_test function
        (verdict, msg) = LabMobilityLte3gsmBase.run_test(self)
        if verdict == Global.FAILURE:
            return verdict, msg
        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")
        self.wait_proper_lte_camp()
        # Set RRC state to the value defined in the TC
        current_rrc_state = self._ns_lte_cell.get_rrc_state()
        if self._rrc_state == "RRC_IDLE":
            self._networking_api.disable_output_traffic()
            self._ns_lte_data.ue_detach()
            self._networking_api.enable_output_traffic()
            current_rrc_state = self._ns_lte_cell.get_rrc_state()
        self._logger.info("The test will start from %s state"
                          % current_rrc_state)
        # Start the FTP transfer
        self.start_ftp()
        # Wait for the FTP to settle.
        self._logger.info("Waiting %s seconds for the FTP to start."
                          % wait_time)
        # Set Network Simulator 3GSM cell on
        self._ns_3gsm_cell.set_cell_on()
        start = time.time()
        time.sleep(wait_time)
        # Check the FTP successfully started
        if self._process.poll() is not None:
            self.log_ftp_output()
            self._ns_3gsm_cell.set_cell_off()
            raise AcsBaseException(AcsBaseException.PROHIBITIVE_BEHAVIOR,
                                   "The FTP transfer stopped before the test "
                                   "could begin.")
        while iteration <= int(self.nb_of_vc):
            self._logger.info("Iteration N%s on %s" % (iteration + 1,
                                                       self.nb_of_vc))
            # Establish MO or MT voice call
            self._logger.info("Originate a %s voice call." % self._vc_type)
            self.originate_csfb_vc()
            # Check call state "CONNECTED" on NW and DUT
            self.check_call_connection()
            # Release MO or MT voice call
            self._logger.info("Making a %s voice call release."
                              % self.release_csfb_vc_type)
            self.release_csfb_vc()
            # Checking the current RAT of the DUT.
            rat = self._modem_api.get_network_type()
            if self._ns_3gsm_cell_tech == "2G" \
                    and rat not in ("EGPRS", "GPRS"):
                self._logger.warning("DUT not camped on 2G: current RAT: %s"
                                     % rat)
            self._logger.info("Waiting for the DUT to camp back on LTE.")
            self.wait_proper_lte_camp()
            self._logger.info("Waiting %s seconds to let the transfer to "
                              "resume on LTE" % wait_time)
            time.sleep(wait_time)
            self._logger.info("Checking if the FTP is still ongoing after the"
                              " DUT camped back on LTE.")
            if self._process.poll() is not None:
                self.log_ftp_output()
                self._ns_3gsm_cell.set_cell_off()
                raise AcsBaseException(AcsBaseException.PROHIBITIVE_BEHAVIOR,
                                       "The FTP transfer stopped after "
                                       "resume.")
            iteration += 1
        self._logger.info("Waiting for the FTP transfer to finish...")
        while self._process.poll is None:
            if time.time() > start + self._xfer_timeout:
                self.log_ftp_output()
                self._ns_3gsm_cell.set_cell_off()
                raise AcsBaseException(AcsBaseException.OPERATION_FAILED,
                                       "The FTP transfer did not finished "
                                       "before the timeout of %s"
                                       % self._xfer_timeout)
            time.sleep(1)
        if self._process is not None and self._process.poll is None:
            self._process.terminate()
        self.log_ftp_output()
        self._ns_3gsm_cell.set_cell_off()
        return self._error.Code, self._error.Msg

    def tear_down(self):
        """
        Terminate the test and return the DUT to its initial state.
        """
        # Release all CSFB on going voice call.
        self.release_csfb_vc()
        # Release all ongoing voice call on the DUT.
        self._voicecall_api.release()
        # Terminate the FTP transfer if still ongoing.
        if self._process is not None and self._process.poll is None:
            self._process.terminate()

        self.log_ftp_output()
        return LabMobilityLteCsfb.tear_down(self)

    def start_ftp(self):
        """
        Starts the FTP transfer.
        """
        # Start the FTP transfer.
        if self._direction == "UL":
            (self._process, self._q) = self._networking_api.ftpput(
                self._ip_address,
                self._username,
                self._password,
                self._ftp_path,
                self._ftp_filename)
        else:
            # the FTP download file will write to /dev/null to get max
            # throughput because of EMMC sequential write limitation
            (self._process, self._q) = self._networking_api.ftpget(
                self._ip_address,
                self._username,
                self._password,
                "/dev/null",
                self._ftp_filename)

    def release_csfb_vc(self):
        """
        Release Voice Call
        """
        if self._ns_3gsm_cell is not None and self.release_csfb_vc_type == "NR":
            # Release previous call from the network
            self._ns_3gsm_vc.voice_call_network_release()
        else:
            # Release previous call from the DUT
            self._voicecall_api.release()

    def log_ftp_output(self):
        """
        Format the output message of the FTP queue to the ACS format
        """
        msg = ""
        if self._q is not None:
            while not self._q.empty():
                msg = msg + self._q.get(False)
            self._logger.error(msg)

    def wait_proper_lte_camp(self):
        """
        Wait for the DUT to camp on the LTE cell, and checks it gets a correct
        IP address.
        """
        # Check Data Connection State => CON before timeout
        self._ns_lte_data.check_data_connection_state("CON",
                self._registration_timeout,
                blocking=True,
                cell_id=self._ns_lte_cell_id)
        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(
                self._registration_timeout)
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(
                self._ns_lte_data.get_network_type(),
                self._registration_timeout)
        ip_address = self._networking_api.get_interface_ipv4_address(self._interface)
        if ip_address != self._ns_lte_ip_dut:
            raise AcsBaseException(AcsBaseException.PROHIBITIVE_BEHAVIOR,
                                   "Wrong IP address: %s should be %s"
                                   % (ip_address, self._ns_lte_ip_dut))
