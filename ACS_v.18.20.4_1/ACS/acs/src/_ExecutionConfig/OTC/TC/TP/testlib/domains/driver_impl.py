"""
@summary: module for keep application
@since: 10/2/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""

from testlib.util.common import g_common_obj
import time
import os

class DriverImpl:
    """
    Driver functions.
    """

    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_search(self):
            """ UI button search """
            return self.d(
                resourceId="com.google.android.apps.docs:id/menu_search")

        @property
        def btn_search_text(self):
            """ UI button search text """
            return self.d(
                resourceId="com.google.android.apps.docs:id/search_text")
        @property
        def btn_cancel_search(self):
            """UI button to cancel search"""
            return self.d(
                description="Cancel search")

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = DriverImpl.Locator(self.d)

    def search_in_driver(self, input_key):
        """ search in keep """
        print "[INFO] Test input in Drive"
        g_common_obj.launch_app_from_home_sc("Drive")
        #Skip set up screen while launch the music app in the first time
        time.sleep(5)
        self._locator.btn_search.click()
        self._locator.btn_search_text.clear_text()
        self._locator.btn_search_text.set_text(input_key)
        assert self.d(text=input_key).exists
        self._locator.btn_cancel_search.click()
        g_common_obj.back_home()
