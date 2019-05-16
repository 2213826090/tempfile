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
:summary: Utilities class for Web Browsing implementation
:since: 10/07/2011
:author: lvacheyx
"""
import time

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class WebBrowsing:

    """
    Structure that represent MMS message for UECmd
    """

    def __init__(self, device, browser_type, timeout):

        # Instantiate the logger
        self._logger = LOGGER_TEST_SCRIPT

        # Type of browser to use to be browsed
        self.browser_type = browser_type.lower()

        # Timeout of web page display
        self.timeout = timeout

        # Instantiate Phone System API for UEcmd
        self._phonesystem_api = device.get_uecmd("PhoneSystem")
        self._networking_api = device.get_uecmd("Networking")

#------------------------------------------------------------------------------
    def create_urls_list(self, website_url):
        """
        Create the URLs list that will be browsed

        :type website_url: str
        :param website_url: list of Website URL separated with ";" (ex : www.intel.com;www.youtube.com)

        :rtype: list
        :return: table of url list to browse

        """
        # Create a url list using ";" as separator between
        # the different url to browse
        website_url = website_url.replace(" ", "")
        temp_urls_list = website_url.split(";")

        # Add all none empty urls to the url list to load.
        urls_list = [x for x in temp_urls_list if x]

        return urls_list

#------------------------------------------------------------------------------
    def browse_and_check(self, urls_list, timeout):
        """
        For each web pages to test this method do:
        - unlock device screen
        - wake up screen
        - open web browser and load a page
        - store verdict
        - close web browser

        At the end, a global verdict is computed.

        :type urls_list: list
        :param urls_list: table of url list to browse

        :type timeout: int
        :param timeout: timeout to open each web pages
        """
        total_failed_loads = 0
        total_succeeded_loads = 0
        run_test_msg = ""
        wait_btwn_cmd = 2
        iteration = 0
        nb_pages = len(urls_list)
        local_urls_list = urls_list
        # Check input parameter urls_list
        if type(urls_list) is str:
            local_urls_list = self.create_urls_list(urls_list)
            nb_pages = len(local_urls_list)
        elif not hasattr(urls_list, '__iter__'):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "URLs must be a list")
        self._logger.info("Start loading the %s web page(s)" % str(nb_pages))

        # Open the browser and load the URLs before timeout
        for web_page in local_urls_list:
            iteration += 1

            msg = "Loading page number " + str(iteration) + " of " \
                    + str(nb_pages) + " : " + web_page + "; "
            self._logger.info(msg)

            # Wakes up the phone
            time.sleep(wait_btwn_cmd)
            self._phonesystem_api.set_phone_lock("off")
            time.sleep(wait_btwn_cmd)
            self._phonesystem_api.wake_screen()
            time.sleep(10)

            (result_code, result_msg) = \
                self._networking_api.open_web_browser(web_page,
                                                  self.browser_type,
                                                  timeout)

            if result_code == Global.SUCCESS:
                total_succeeded_loads += 1
            else:
                total_failed_loads += 1

            # Close the browser
            time.sleep(wait_btwn_cmd)
            self._networking_api.close_web_browser(self.browser_type)

            # Building the result message of the test.
            run_test_msg += "Page nb " + str(iteration) + ", result message: "\
                + str(result_msg) + "; "

        # Compute the result of the test
        return self.compute_test_result(total_failed_loads,
                                        total_succeeded_loads,
                                        run_test_msg,
                                        nb_pages)

#------------------------------------------------------------------------------
    def start_browsing(self, web_page, timeout):
        """
        Browse the URLs list of web pages

        :type web_page: str
        :param web_page: Web page address to browse

        :type timeout: int
        :param timeout: timeout to open the page
        """
        wait_btwn_cmd = 2

        # Wakes up the phone
        time.sleep(wait_btwn_cmd)
        self._phonesystem_api.set_phone_lock("off")
        time.sleep(wait_btwn_cmd)
        self._phonesystem_api.wake_screen()
        time.sleep(10)

        return self._networking_api.open_web_browser(web_page,
                                                     self.browser_type,
                                                     timeout)

#------------------------------------------------------------------------------
    def compute_test_result(self,
                            total_failed_loads,
                            total_succeeded_loads,
                            run_test_msg,
                            nb_of_pages):
        """
        Compute the test result

        :type total_failed_loads: int
        :param total_failed_loads: Total of failed web page loads

        :type total_succeeded_loads: int
        :param total_succeeded_loads: Total of successful web page loads

        :type run_test_msg: str
        :param run_test_msg: result message of the test.

        :type nb_of_pages: int
        :param nb_of_pages: number of web pages displayed on the DUT's screen
        """
        # Compute the result of the test.
        error_code = Global.SUCCESS
        if total_failed_loads > 0:
            error_code = Global.FAILURE

        # Compute the result message of the test.
        msg = "Total pages loaded = " + str(nb_of_pages) + \
            "; Total successes = " + str(total_succeeded_loads) + \
            "; Total failures = " + str(total_failed_loads) + \
            "; " + run_test_msg

        return error_code, msg
