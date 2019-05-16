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

:organization: UMG PSI
:summary: This file implements the Lab WIFI PING BROWSE UC
:since: 21/01/2013
:author: emarchan
"""
import time
from acs_test_scripts.UseCase.Networking.LAB_WIFI_PING import LabWifiPing
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class LabWifiPingBrowse(LabWifiPing):

    """
    Lab Wifi Ping test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiPing.__init__(self, tc_name, global_config)

        # Get BROWSE TC Parameters
        self._browser_type = \
            str(self._tc_parameters.get_param_value("BROWSER_TYPE")).lower()
        self._website_url = str(self._tc_parameters.get_param_value("WEBSITE_URL"))
        self._webpage_loading_timeout = \
            int(self._tc_parameters.get_param_value("TIMEOUT"))

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the PING test
        """
        LabWifiPing.set_up(self)

        """
        Initialize the BROWSE test
        """

        time.sleep(self._wait_btwn_cmd)
        self._phone_system_api.display_on()

        # Wakes up the phone
        time.sleep(self._wait_btwn_cmd)
        self._phone_system_api.wake_screen()

        # Starts Berkeley Internet Name Domain to resolve the web server.
        self._computer.start_stop_service("bind9", "start")

        if self._website_url == "HOST":
            if self._use_ipv6:
                host_ip = self._computer.get_ipv6_addresses()[0]
                self._website_url = 'http://[%s]' % host_ip
            else:
                msg = "IPV4 not supported for host resolution."
                raise AcsConfigException(AcsConfigException.FEATURE_NOT_IMPLEMENTED, msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the PING test
        """
        LabWifiPing.run_test(self)

        """
        Execute the BROWSE test
        """
        if self._use_ipv6:
            if "rdnssd" in self._ipv6_configuration_mode.lower():
                self._networking_api.copy_dhcp6_dns(self._dut_wlan_iface, '1',
                                                    is_rdnssd=True)
            else:
                self._networking_api.copy_dhcp6_dns(self._dut_wlan_iface, '1',
                                                    is_rdnssd=False)

        # Open the browser and load the url before timeout
        (self._error.Code, self._error.Msg) = \
            self._networking_api.open_web_browser(self._website_url,
                                                  self._browser_type,
                                                  self._webpage_loading_timeout)

        # Close the browser
        time.sleep(self._webpage_loading_timeout)
        self._networking_api.close_web_browser(self._browser_type)

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiPing.tear_down(self)

        self._phone_system_api.display_off()
        self._computer.start_stop_service("bind9", "stop")

        return Global.SUCCESS, "No errors"
