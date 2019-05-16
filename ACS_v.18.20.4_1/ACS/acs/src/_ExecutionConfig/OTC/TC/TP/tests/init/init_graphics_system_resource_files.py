import os
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.push_system_resource_files import PushSystemResourceFiles
from testlib.util.common import g_common_obj

class PushGraphicsResourceFiles(UIATestBase):

    def setUp(self):
        super(PushGraphicsResourceFiles, self).setUp()
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("push_graphics_resources_files")
        if self.retry_num is None:
            self.retry_num = 3
        self.retry_num = int(self.retry_num)
        self.push_resource_files = PushSystemResourceFiles()

    def tearDown(self):
        super(PushGraphicsResourceFiles, self).tearDown()

    def testPushGraphicsResourceFiles(self):
        """
        test_Push_graphics_source_files
        """
        message = "Push Graphics files failed"
        for i in range(self.retry_num):
            try:
                output = self.push_resource_files.testPushSytemResourceFiles()
                if output == True:
                    break
            except Exception as e:
                print output
                message += "%d:%s\n" % (i, e)
        assert output, message