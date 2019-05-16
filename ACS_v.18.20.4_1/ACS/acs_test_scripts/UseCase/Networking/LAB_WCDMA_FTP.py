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
:summary: This file implements WCDMA FTP UC
:author: ccontreras
:since:16/10/2010
"""

import time
import os
from UtilitiesFWK.Utilities import Global
from LAB_WCDMA_BASE import LabWcdmaBase
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.CommunicationUtilities import throughput_targets_string
from acs_test_scripts.Utilities.FtpUtilities import perform_ftp_transfer


class LabWcdmaFtp(LabWcdmaBase):

    """
    Lab WCDMA ftp test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_WCDMA_BASE Init function
        LabWcdmaBase.__init__(self, tc_name, global_config)

        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("DL_FILENAME", ""))

        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("UL_FILENAME", ""))

        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = \
            int(self._tc_parameters.get_param_value("XFER_TIMEOUT"))

        # Get customized failure Targets
        self._failure_targets = \
            str(self._tc_parameters.get_param_value("FAILURE_TARGETS", "FUTE"))

        # Update the failure targets
        self._throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                    self._failure_targets)

        # Log Throughput targets for WCDMA
        self._logger.info(throughput_targets_string(self._throughput_targets))

        # Initializing the variable which will contain the IP address to use.
        self._ip_address_list = []

        self._ftp_api = self._device.get_uecmd("Ftp")

# ------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWcdmaBase.set_up(self)

        if self._wanted_reg_state == "roaming":
            # Activate PDP context
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Active PDP Context...")
            self._networking_api.activate_pdp_context(self._ssid)

            # Check Data Connection State => PDP Active before timeout
            self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                         self._registration_timeout,
                                                         blocking=False)

            # Get RAT from Equipment
            network_type = self._ns_data_3g.get_network_type()

            # Check that DUT is registered on the good RAT
            self._modem_api.check_network_type_before_timeout(network_type,
                                                              self._registration_timeout)

        # Selecting the IPV4, IPV6 or both addresses(IPV4V6) of the FTP
        # server, according to the TC parameter value.
        if self._ip_version in ("IPV6", "IPV4V6"):
            if self._server_ip_v6_address is not None:
                # If IP version is IPV6 or IPV4V6 append IPV6 address to the
                # IP list.
                log_msg = "Using IPV6 address: %s to connect to the FTP" \
                    " server." % self._server_ip_v6_address
                self._logger.info(log_msg)
                self._ip_address_list.append(self._server_ip_v6_address)
            else:
                # If IPV6 address is not present in the BenchConfig.
                msg = "The IPV6 parameter is missing from the Bench Config!"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._ip_version in ("IPV4", "IPV4V6", None):
            # If the IP version is IPV4, IPV4V6 or the parameter is not present
            # in the TestCase.
            log_msg = "Using IPV4 address: %s to connect to the FTP" \
                " server." % self._server_ip_address
            self._logger.info(log_msg)
            self._ip_address_list.append(self._server_ip_address)
        if self._ip_version not in ("IPV4", "IPV6", "IPV4V6", None):
            # If the IP version parameter is present in the TestCase but not
            # IPV6 or IPV4.
            msg = "The IP_VERSION parameter from the TestCase should be IPV6" + \
                ", IPV4 or IPV4V6. Not %s" % self._ip_version
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Initializing local variables.
        iteration = 0
        current_result = (Global.SUCCESS, "")
        message = ""
        # Calling run_test from base class
        LabWcdmaBase.run_test(self)

        # Run FTP transfer using FTP parameters :
        # - LAB_SERVER parameters (ip, username, password)
        # - DIRECTION
        # - DL_FILE or UL_FILE
        # - XFER_TIMEOUT
        while iteration < len(self._ip_address_list) and \
                current_result[0] == Global.SUCCESS:
            # Make as many FTP transfer as there is IP in the IP list.
            time.sleep(self._wait_btwn_cmd)
            current_result = perform_ftp_transfer(self._direction,
                                                  self._ip_address_list[iteration],
                                                  self._username,
                                                  self._password,
                                                  self._ulfilename,
                                                  self._xfer_timeout,
                                                  self._device.multimedia_path,
                                                  self._ns_DUT_IP_Address,
                                                  self._ftp_api,
                                                  self._throughput_targets.ul_failure.value,
                                                  self._logger,
                                                  self._dlfilename,
                                                  self._throughput_targets.dl_failure.value,
                                                  self._device.binaries_path)

            iteration += 1
            # Append messages of the different FTP transfer.
            message = message + current_result[1]
        # Store result.
        result = (current_result[0], message)
        if self._wanted_reg_state == "roaming":
            # Deactivate PDP context
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Deactivate PDP Context...")
            self._networking_api.deactivate_pdp_context(self._ssid)

            # Check Data Connection State => ATTAched before timeout
            self._ns_data_3g.\
                check_data_connection_state("ATTACHED",
                                            self._registration_timeout,
                                            blocking=False)

            # Check that DUT is registered on the good RAT
            state = self._modem_api.get_network_registration_status()
            self._logger.info("the network registration is in %s state"
                              % state)

        return result

# ------------------------------------------------------------------------------
