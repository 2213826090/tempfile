"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements the Base test Step for Web Broswsing
:since 26/03/2014
:author: jreynaux
"""
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase

from acs_test_scripts.Utilities.WebBrowsingUtilities import WebBrowsing
from UtilitiesFWK.Utilities import Global


class WebBrowseAndCheck(DeviceTestStepBase):
    """
    Implements the base test step for Browser
    Attributes:
        DEVICE (string): @see DeviceTestStepBase
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        browser_type = str(self._pars.browser_type).lower()
        website_url = str(self._pars.website_url)
        webpage_loading_timeout = int(self._pars.timeout)

        # Configure optional parameters
        if self._pars.loop_timer is not None:
            if self._pars.loop_timer < 0:
                loop_timer = 0
                self._logger.warning("Invalid LOOP_TIMER value, use 0 as default value")
            else:
                loop_timer = int(self._pars.loop_timer)
        else:
            loop_timer = 0
        if self._pars.expected_verdict is not None:
            assert self._pars.expected_verdict in ["PASS","FAIL"], \
                "passed value (%s) is invalid at this stage" % self._pars.expected_verdict
            expected_verdict = str(self._pars.expected_verdict)
        else:
            expected_verdict = "PASS"

        # Instantiate Browser class
        browsing = WebBrowsing(self._device, browser_type, webpage_loading_timeout)

        urls_list = browsing.create_urls_list(website_url)
        initial_time = time.time()
        first_loop = False

        while loop_timer > (time.time() - initial_time) or first_loop is False:
            first_loop = True
            status, output = browsing.browse_and_check(urls_list, webpage_loading_timeout)

            if status != Global.SUCCESS and expected_verdict == "PASS":
                self._raise_device_exception("Unable to load page correctly ({0})".format(output))
            elif status == Global.SUCCESS and expected_verdict == "FAIL":
                self._raise_device_exception(
                    "Expected verdict FAIL - Capable to load page correctly ({0})".format(output))
