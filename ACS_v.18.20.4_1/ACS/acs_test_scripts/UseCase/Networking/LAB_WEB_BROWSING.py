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
:summary: Use Case to perform Web Browsing over Network simulator (for 2G / 3G)
:since: 08/07/2013
:author: lvacheyx
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.WebBrowsingUtilities import WebBrowsing
from LAB_WEB_BROWSING_BASE import LabWebBrowsingBase


class LabWebBrowsing(LabWebBrowsingBase):
    """
    Constructor
    """
    def __init__(self, tc_name, global_config):
        """
        Call base class initialization
        """
        # Call LabWebBrowsingBase __init__ function
        LabWebBrowsingBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._browser_type = str(self._tc_parameters.
                                 get_param_value("BROWSER_TYPE")).lower()
        self._website_url = str(self._tc_parameters.
                                get_param_value("WEBSITE_URL"))
        self._webpage_loading_timeout = int(self._tc_parameters.
                                            get_param_value("TIMEOUT"))

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        # Initialize the list that will be populated by all the urls to load.
        self._urls_list = None

        # Recall current screen timeout
        self._current_screen_timeout = self._phonesystem_api.\
            get_screen_timeout()

        # Instantiate WebBrowsing class
        self.browsing = WebBrowsing(self._device,
                                    self._browser_type,
                                    self._webpage_loading_timeout)

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Setting up the test
        """
        # Call LabWebBrowsingBase set_up function
        LabWebBrowsingBase.set_up(self)

        # Create URLs List that will be browsed
        self._urls_list = self.browsing.create_urls_list(self._website_url)

        # Set the screen timeout to be sure that the screen will on
        # *15 because the minimum timeout is 1: 1*15 = 15s which is
        # the minimum screen timeout
        time.sleep(self._wait_btwn_cmd)
        if self._webpage_loading_timeout > 0:
            self._global_time_out = 1 * self._webpage_loading_timeout
            self._phonesystem_api.set_screen_timeout(self._global_time_out * 15)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call LabWebBrowsingBase set_up function
        LabWebBrowsingBase.run_test(self)

        total_failed_loads = 0
        total_succeeded_loads = 0
        run_test_msg = ""
        iteration = 0

        self._logger.info("Start loading the %s web page(s)" % str(len(self._urls_list)))

        # Open the browser and load the URLs before timeout
        for web_page in self._urls_list:
            iteration += 1

            msg = "Loading page number " + str(iteration) + " of " + str(len(self._urls_list)) + " : " + str(web_page) + "; "
            self._logger.info(msg)

            # Browse the list of web URLs
            (result_code, result_msg) = \
                    self.browsing.start_browsing(web_page, self._webpage_loading_timeout)

            if result_code == Global.SUCCESS:
                total_succeeded_loads += 1
            else:
                total_failed_loads += 1

            # Close the browser
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.close_web_browser(self._browser_type)
            time.sleep(self._wait_btwn_cmd)

            # Building the result message of the test.
            run_test_msg += "Page nb " + str(iteration) + ", result message: " \
                + str(result_msg) + "; "

        # Compute the result of the test
        (self._error.Code, self._error.Msg) = self.browsing.compute_test_result(total_failed_loads,
                                                                                total_succeeded_loads,
                                                                                run_test_msg,
                                                                                len(self._urls_list))

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        Tear down
        """
        # Call LabWebBrowsingBase tear_down function
        LabWebBrowsingBase.tear_down(self)
        self._phonesystem_api.set_screen_timeout(self._current_screen_timeout)

        return Global.SUCCESS, "No errors"
