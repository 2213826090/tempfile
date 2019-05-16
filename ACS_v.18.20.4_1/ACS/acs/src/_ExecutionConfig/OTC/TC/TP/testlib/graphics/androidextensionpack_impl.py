# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for ChromeCast operation
@since: 05/06/2015
@author: Yingjun Jin
'''


import time
import os
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class AndroidExtensionPackImpl:

    '''
    classdocs`
    '''

    def init_environment(self):
        """ Init the test environment
        """
        self.config = TestConfig()
        self.cfg_file = 'tests.tablet.androidextensionpack.conf'
        cfg_arti = self.config.read(self.cfg_file, 'artifactory')
        cfg = self.config.read(self.cfg_file, 'binary')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("name")
        file_path = arti.get(binary_name)
        print "%s" % file_path
        g_common_obj.adb_cmd_common('push ' + file_path + ' /data/app/')
        g_common_obj.adb_cmd('chmod 777 /data/app/oglconform_and_x86')

    def run_case(self, case):
        """ run the binary file and check the result
        """
        cfg = self.config.read(self.cfg_file, case)
        cmd = str(cfg.get('cmd'))
        key = str(cfg.get('key'))
        cmd = '/data/app/oglconform_and_x86 %s | grep -A2 %s > /tmp/log.txt' % (
            cmd, key)
        g_common_obj.adb_cmd(cmd)
        assert os.system(
            "cat /tmp/log.txt| grep reported| awk '{print $1}'") == 0, "The test failed"
        assert os.system(
            "cat /tmp/log.txt| grep supported|awk '{print $1}'") == 0, "The test failed"

    def rm_log(self):
        """ remove log file
        """
        os.system('rm -rf log.txt')
