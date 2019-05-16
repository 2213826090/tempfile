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
:summary: This file implements the LAB WIFI BASE UC with DHCP added
:since: Friday, July 06 2012
:author: rneu
"""

import time

from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global


class LabDHCPServerBase(LabWifiBase):

    """
    Lab Wifi Base Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Get configurable AP dhcp settings
        self._dhcp_enabled = str(self._configurable_ap.
                                 get_param_value("DHCP_ENABLED"))
        if self._dhcp_enabled == "None" or self._dhcp_enabled == "":
            self._dhcp_enabled = "False"
        self._low_excluded_addr = str(self._configurable_ap.
                                      get_param_value("LOW_EXCLUDED_IP"))
        self._high_excluded_addr = str(self._configurable_ap.
                                       get_param_value("HIGH_EXCLUDED_IP"))
        self._dhcp_subnet = str(self._configurable_ap.
                                get_param_value("DHCP_SUBNET"))
        self._dhcp_subnet_mask = str(self._configurable_ap.
                                     get_param_value("DHCP_SUBNET_MASK"))
        self._dhcp_lease = str(self._configurable_ap.
                               get_param_value("DHCP_LEASE"))
        self._dhcp_gateway_address = str(self._configurable_ap.
                                         get_param_value("DHCP_GATEWAY"))

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        # disconnect wifi as LAB_WIFI_BASE connects it
        self._networking_api.wifi_disconnect_all()
        time.sleep(self._wait_btwn_cmd)

        # Initiate connection to the equipment
        self._ns.init()

        # turn off dhcp and clean params in any case
        self._ns.set_dhcp("off")
        # then check if it has to be turned on
        if self._dhcp_enabled in ("True", "TRUE"):
            self._ns.set_dhcp("on",
                              self._low_excluded_addr,
                              self._high_excluded_addr,
                              self._dhcp_subnet,
                              self._dhcp_subnet_mask,
                              self._dhcp_lease,
                              self._dhcp_gateway_address)

        # Close the connection to AP
        self._ns.release()

        # Connect the DUT on the Wifi network
        self._networking_api.wifi_connect(self._ssid, True)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiBase.tear_down(self)

        # Initiate connection to the equipment
        self._ns.init()

        # Put back the dhcp parameters from the bench configuration if needed
        self._ns.set_dhcp("off")
        if self._dhcp_enabled in ("True", "TRUE"):
            self._ns.set_dhcp("on",
                              self._low_excluded_addr,
                              self._high_excluded_addr,
                              self._dhcp_subnet,
                              self._dhcp_subnet_mask,
                              self._dhcp_lease,
                              self._dhcp_gateway_address)

        # Close the connection to AP
        self._ns.release()

        return Global.SUCCESS, "No errors"
