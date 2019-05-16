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
@summary: HDMIcapturedecodeimpl class
@since: 10/26/2015
@author: Zhao, XiangyiX
"""

import os
from shutil import copyfile
from testlib.util.config import TestConfig
from testlib.util.process import shell_command_ext
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.common.common import reportSemiAutoVerdict
# import sys
# sys.path.append("../../../../..")
# from acs_test_script.UseCase.Misc.PY_UNIT import *


class HDMICaptureDecodeImpl(object):

    """HDMI_capture_decode_impl"""

    config_file = 'tests.common.display_cable_switch.conf'

    def __init__(self):
        configer = TestConfig()
        self.config = configer.read(self.config_file, "DisplayCableSwitch")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))
        self.excuter_path = ''

    def setup(self):
        self.excuter_path = self.arti.get(self.config.get("bin_qrdecode"))
        assert self.excuter_path, \
            "Download resource failed!"
        cmd = 'chmod 777 %s' % self.excuter_path
        code, msg = self.shell_command(cmd)
        assert code == 0, \
            "[FAILURE] %s %s" % (cmd, msg)

    def capture_decode(self, QRstring):
        """Excute binary and camera will capture and decode photos."""
        cmd = "%s Command Camera" % (self.excuter_path)
        _, msg = self.shell_command(cmd)
#         assert code == 0, \
#             "[FAILURE] %s %s" % (cmd, msg)
        if not msg.find('QR string:') >= 0:
            return False, "capture failed"
        if not msg.find(QRstring) >= 0:
            return False, "decode failed"
        else:
            return True, ""

    def shell_command(self, cmd):
        print 'Execute command: %s' % cmd
        exit_code, stdout_log, stderr_log = shell_command_ext(cmd)
        message = stdout_log + stderr_log
        print 'Result exit:%s\n%s' % (exit_code, message)
        return exit_code, message

    def get_path(self):
        """To get the binary path"""
        self.binary_path = self.excuter_path.replace("QRdecode", "")
        print 'binary_path: %s' % (self.binary_path)
        return self.binary_path

    def capture_image_and_upload(self, casename):
        """Excute binary and camera will capture and upload photos."""
        cmd = "%s Command Camera" % (self.excuter_path)
        _, msg = self.shell_command(cmd)
#         assert code == 0, \
#             "[FAILURE] %s %s" % (cmd, msg)
        if not msg.find('SaveAs:fail_0_0.png') >= 0:
            assert False, "capture failed"
        pathlist = [self.get_path(), "saveimages/fail_0_0.png"]
        image_path = os.sep.join(pathlist)
        print image_path
        if not os.path.exists(image_path):
            assert False, "No images found"
        else:
#             copyfile(image_path, self._acs_params["report_path"] + "/PHONE1/DEBUG_LOGS/Capture.png")
#             copyfile(image_path, self._acs_params["report_path"] + "/PHONE1/DEBUG_LOGS/" + casename + "/Capture.png")
            live_report_log_path = g_common_obj.get_user_log_dir()
            copyfile(image_path, live_report_log_path + "/Capture.png")
            reportSemiAutoVerdict()
