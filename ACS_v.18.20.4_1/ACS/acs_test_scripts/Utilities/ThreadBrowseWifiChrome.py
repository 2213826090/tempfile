"""

:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA

:since: 2/9/15
:author: mihaela
"""
import time
import threading
from acs_test_scripts.Utilities.NetworkingUtilities import ThreadWebBrowsing
from UtilitiesFWK.Utilities import Error, Global
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class ThreadBrowseWifiChrome(ThreadWebBrowsing):
    """
    This class inherits the ThreadWebBrowsing and allows for manipulations locally,
    in order to fit the instability of the Bluetooth component, without affecting
    the already in-place classes for LocalConnectivity and NetworkConnectivity
    """
    WAIT_BETWEEN_PAGES = 10.0
    def __init__(self, exceptions_queue, phone_local_wifi, duration, retries, *website_url):
        """
        Constructor
        """
        ThreadWebBrowsing.__init__(self, exceptions_queue, phone_local_wifi, duration, *website_url)
        threading.Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._phone_local_wifi = phone_local_wifi
        self._duration = duration
        self._website_url = website_url
        self._retries = retries
        self._error = Error()


    def run(self):
        """
        Execute the thread.
        Try to open a page for a number of times, even if it hits failure the first time,
        as we are trying to bypass any test limitations
        """
        try:
            start = time.time()
            while(start + self._duration) > time.time():
                for element in self._website_url:
                    time.sleep(self.WAIT_BETWEEN_PAGES)
                    (self._error.Code, self._error.Msg) = \
                        self._phone_local_wifi.open_web_browser(element, "chrome", 30)
                    if self._error.Code != Global.SUCCESS:
                        self._phone_local_wifi.close_web_browser("chrome")
                        msg = str.format("Web browsing fail on {0} - {1}", str(element), self._error.Msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            self._phone_local_wifi.close_web_browser("chrome")

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)


    def join(self, timeout=180):
        """
        Wait the thread finish.

        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadBrowseWifiChrome, self).join()
        return self._error.Code, self._error.Msg