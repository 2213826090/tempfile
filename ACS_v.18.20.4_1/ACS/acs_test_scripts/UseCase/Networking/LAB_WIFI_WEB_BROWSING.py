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
:summary: This file implements the Lab WIFI WEB BROWSING UC
:since: 17/12/2012 BZ3076
:author: apairex
"""
import time
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabWifiWebBrowsing(LabWifiBase):

    """
    Lab Wifi web browsing test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._browser_type = \
            str(self._tc_parameters.get_param_value("BROWSER_TYPE")).lower()
        self._website_url = \
            str(self._tc_parameters.get_param_value("WEBSITE_URL"))
        self._webpage_loading_timeout = \
            int(self._tc_parameters.get_param_value("TIMEOUT"))

        if self._website_url.upper() in ["", "NONE"]:
            # In case of empty WEBSITE_URL TC parameter, use the IP server in the bench config
            self._website_url = self._wifi_server_ip_address

        # Store current screen timeout (default to 15s in case of failure)
        self._current_screen_timeout = 15

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        # Recall current screen timeout
        self._current_screen_timeout = self._phone_system_api.get_screen_timeout()

        # Set the screen timeout to be sure that the screen will on
        # *15 because the minimum timeout is 1: 1*15 = 15s which is the minimum screen timeout
        time.sleep(self._wait_btwn_cmd)
        if self._webpage_loading_timeout > 0:
            self._phone_system_api.set_screen_timeout(self._webpage_loading_timeout * 15)

        # Wakes up the phone
        time.sleep(self._wait_btwn_cmd)
        self._phone_system_api.wake_screen()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Open the browser and load the url before timeout
        self._test_web_browsing(self._website_url, self._browser_type, self._webpage_loading_timeout)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiBase.tear_down(self)

        # Set the screen timeout to default
        time.sleep(self._wait_btwn_cmd)
        self._phone_system_api.set_screen_timeout(self._current_screen_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _test_web_browsing(self, website_url, browser_type, timeout):
        """
        Open the WEB browser

        :type website_url: str
        :param website_url: URL to open
        :type browser_type: str
        :param browser_type: "native" will open the default browser,
                             "acs_agent" will use the browser of the acs agent
        :type timeout: int
        :param timeout: timeout to open the page
        """
        try:
            (code, msg) = self._networking_api.open_web_browser(website_url, browser_type, timeout)

            if code != Global.SUCCESS:
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # Close the browser
            time.sleep(timeout)

        finally:
            self._networking_api.close_web_browser(browser_type)
