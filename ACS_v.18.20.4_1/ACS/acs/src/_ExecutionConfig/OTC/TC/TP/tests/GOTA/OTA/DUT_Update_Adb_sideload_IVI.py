'''
@summary: Test OTA update
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.ota_impl import otaImpl
from testlib.multimedia.multimedia_log import MultimediaLogger
logger = MultimediaLogger.instance()

class OTASideloadTest(UIATestBase):
    """
    @summary: Test it can phone flash DUT successfully
    """
    def setUp(self):
        super(OTASideloadTest, self).setUp()
        self._test_class = __name__
        logger.info("[Setup]:Class  %s" % self._test_class)
        self.ota = otaImpl()
        self.ota.incremental_ota_url

    def tearDown(self):
        logger.info("[Teardown]:Class  %s" % self._test_class)
        super(OTASideloadTest, self).tearDown()



    def DeviceUpdateUsingFullOTApackageThroughRecoveryOSoperations_adbSideload(self):
        """
        Test Case Step:
        1. Flash specified old version
        2. Enter sideload mode
        3. Flash OTA package, 'adb sideload [full build]'
        4. Reboot and check build no.
        """
        self.ota.ota_update_adbsideload(self.ota.full_ota_url)

    def DeviceUpdateUsingIncrementalOTApackageThroughRecoveryOSoperations_adbSideload(self):

        """
        Test Case Step:
        1. Flash specified old version
        2. Enter sideload mode
        3. Flash OTA package, 'adb sideload [Incremental build]'
        4. Reboot and check build no.
        """
        self.ota.ota_update_adbsideload(self.ota.incremental_ota_url)
