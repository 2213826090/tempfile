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
:summary: This file implements the Live Cellular WEB BROWSING UC
:since: 30/07/2012
:author: cbresoli
"""
import time
from acs_test_scripts.UseCase.Networking.LIVE_CELLULAR_BASE import LiveCellularBase
from acs_test_scripts.Utilities.WebBrowsingUtilities import WebBrowsing
from UtilitiesFWK.Utilities import Global


class LiveCellularWebBrowsing(LiveCellularBase):

    """
    Live Wifi web browsing test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveCellularBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._browser_type = str(self._tc_parameters.
                                 get_param_value("BROWSER_TYPE")).lower()
        self._website_url = str(self._tc_parameters.
                                get_param_value("WEBSITE_URL"))
        self._webpage_loading_timeout = int(self._tc_parameters.
                                            get_param_value("TIMEOUT"))

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._modem_api = self._device.get_uecmd("Modem")

        # Initialize the list that will be populated by all the urls to load.
        self._urls_list = None

        # Initialize the variable storing the total number of pages to load.
        self._nb_of_pages = 0

        # Store current screen timeout (default to 15s in case of failure)
        self._current_screen_timeout = 15

        # Instantiate WebBrowsing class
        self.browsing = WebBrowsing(self._device,
                                    self._browser_type,
                                    self._webpage_loading_timeout)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveCellularBase.set_up(self)

        # Activate the PDP context if it's deactivated/not connected
        pdp_context_status = self._networking_api._get_pdp_context_status()
        if pdp_context_status in ("0", "2"):
            self._networking_api.activate_pdp_context()

        # Create URLs List that will be browsed
        self._urls_list = self.browsing.create_urls_list(self._website_url)

        # Set the number of pages to browse using the number
        # of URL present in the list of url
        self._nb_of_pages = len(self._urls_list)

        # Recall current screen timeout
        self._current_screen_timeout = self._phonesystem_api.\
            get_screen_timeout()

        # Set the screen timeout to be sure that the screen will on
        # *15 because the minimum timeout is 1: 1*15 = 15s which is
        # the minimum screen timeout
        time.sleep(self._wait_btwn_cmd)
        if self._webpage_loading_timeout > 0:
            global_time_out = self._nb_of_pages * self._webpage_loading_timeout
            self._phonesystem_api.set_screen_timeout(global_time_out * 15)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LiveCellularBase.run_test(self)

        # Browse the list of web URLs
        browsing_success = self.browsing.browse_and_check(
                        self._urls_list, self._webpage_loading_timeout)

        return browsing_success

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LiveCellularBase.tear_down(self)

        # Set the screen timeout to default
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.set_screen_timeout(self._current_screen_timeout)

        return Global.SUCCESS, "No errors"
