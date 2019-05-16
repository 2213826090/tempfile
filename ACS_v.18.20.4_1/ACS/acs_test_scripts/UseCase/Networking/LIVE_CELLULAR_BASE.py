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
:summary: This file implements the LIVE WCDMA BASE
:since: 30/03/2010
:author: cbresoli
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveCellularBase(UseCaseBase):

    """
    Live Wcdma Data Transfer base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC parameters
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Allow possibility to choose specific server on BenchConfig (e.g LIVE_SERVER )
        self._server_name = \
            self._tc_parameters.get_param_value("SERVER_NAME",
                                                "LAB_SERVER")

        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters(self._server_name)
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        self._network_pref = \
            self._tc_parameters.get_param_value("PREFERRED_NETWORK_TYPE",
                                                None)
        if self._network_pref:
            self._network_pref = self._network_pref.upper()

        # Store current device state
        self._initial_pdp_context_status = None
        self._stored_pref_network = None

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        self._initial_pdp_context_status = self._networking_api._get_pdp_context_status()

        # Clear all data connections
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.clean_all_data_connections()

        time.sleep(self._wait_btwn_cmd)
     
     
        #Added Airplane mode on/off as cht_hr fails attch to cellular network after cleaning the PDP connection 
        self._networking_api.set_flight_mode("on")
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        time.sleep(self._wait_btwn_cmd)

        # Configure Preferred Network if set
        self._stored_pref_network = \
            self._dut_config.get("defaultPreferredNetwork")
        if self._network_pref is None:
            # If there is no Network preference in the testcase.
            self._logger.warning("No preferred network set in the testcase,"
                                 " will use the one currently set on the"
                                 " phone: %s" % str(self._stored_pref_network))
        elif self._networking_api.is_preferred_network_type_valid(self._network_pref):
            # Setting the DUT preferred network type to the one specified
            # in the TC.
            self._networking_api.\
                set_preferred_network_type(self._network_pref)
            # Check the DUT is camped on a compatible network with the selected
            # preferred network.
            self._modem_api.\
                check_rat_with_pref_network(self._network_pref,
                                            self._registration_timeout)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "Unknown network type: %s"
                                   % self._network_pref)

        self._logger.info("Activate PDP Context")
        self._networking_api.activate_pdp_context(check=False)

        return self._error.Code, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)

        # Deactivate only if was initially deactivated
        if self._initial_pdp_context_status == "2":
            self._logger.info("Deactivate PDP Context")
            self._networking_api.deactivate_pdp_context()

        time.sleep(self._wait_btwn_cmd)

        # Restore initial preferred network if set
        if self._stored_pref_network is not None:
            self._networking_api.\
                set_preferred_network_type(self._stored_pref_network)

        return self._error.Code, "No errors"
