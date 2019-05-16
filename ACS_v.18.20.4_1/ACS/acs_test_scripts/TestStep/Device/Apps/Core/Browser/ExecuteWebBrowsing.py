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
:since 03/12/2014
:author: kturban
"""

import time
from Core.PathManager import Paths
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException


class WebBrowsingBase(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._networking_api = self._device.get_uecmd("Networking")
        self._system_api = self._device.get_uecmd("System")


class StartBrowsing(WebBrowsingBase):
    """
    Implements the step to start browsing
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

        self._networking_api.close_web_browser(self._pars.browser_type.lower())
        self._networking_api.open_web_browser(website_url=self._pars.website_url,
                                              browser_type=self._pars.browser_type.lower(),
                                              skip_eula=True)
        # check process is running when app_name is specified
        if self._pars.app_name:
            self._system_api.wait_process_loaded(app_name=self._pars.app_name, timeout=self._pars.timeout, refresh=3)


class StopBrowsing(WebBrowsingBase):
    """
    Implements the step to stop browsing
    """

    def run(self, context):
        DeviceTestStepBase.run(self, context)
        self._networking_api = self._device.get_uecmd("Networking")
        self._networking_api.close_web_browser(self._pars.browser_type.lower())

