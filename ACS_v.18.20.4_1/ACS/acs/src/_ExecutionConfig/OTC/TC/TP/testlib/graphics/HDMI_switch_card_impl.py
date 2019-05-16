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
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.
# Any license under such intellectual property rights must be express
#    and approved by Intel in writing.
"""
@summary: HDMIswitchcardimpl class
@since: 10/26/2015
@author: Zhao, XiangyiX
"""

import time

from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.process import shell_command_ext

class HDMISwitchCardImpl(object):

    """HDMISwitchCardImpl"""

    config_file = 'tests.common.display_cable_switch.conf'

    def __init__(self):
        configer = TestConfig()
        self.config = configer.read(self.config_file, "DisplayCableSwitch")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))
        self.excuter_path = ''

    def setup(self):
        self.excuter_path = self.arti.get(self.config.get("bin_linkconvert"))
        assert self.excuter_path, \
            "Download resource failed!"
        cmd = 'chmod 777 %s' % self.excuter_path
        code, msg = self.shell_command(cmd)
        assert code == 0, \
            "[FAILURE] %s %s" % (cmd, msg)

    def switch_on(self):
        cmd = "%s -l 0100" % (self.excuter_path)
        code, msg = self.shell_command(cmd)
        assert code == 0, \
            "[FAILURE] %s %s" % (cmd, msg)
        time.sleep(5)

    def switch_off(self):
        cmd = "%s -l 0000" % (self.excuter_path)
        code, msg = self.shell_command(cmd)
        assert code == 0, \
            "[FAILURE] %s %s" % (cmd, msg)
        time.sleep(5)

    def shell_command(self, cmd):
        print 'Execute command: %s' % cmd
        exit_code, stdout_log, stderr_log = shell_command_ext(cmd)
        message = stdout_log + stderr_log
        print 'Result exit:%s\n%s' % (exit_code, message)
        return exit_code, message

