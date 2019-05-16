# Copyright (C) 2014
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
# and approved by Intel in writing.

'''
@summary: push system file resources
@since: 09/21/2015
@author: Zhang,RongX Z
'''
import os
from testlib.graphics.oglconform_impl import OglconformImpl
from testlib.graphics.GLCTS_impl import GLCTSImpl
from testlib.graphics.conform_impl import ConformImpl
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.dut_init.dut_init_impl import Function
from testlib.androidframework.common import EnvironmentUtils


class PushSystemResourceFiles:

    def __init__ (self):
        self.device = g_common_obj.get_device()
        g_common_obj.root_on_device()

    def testPushSytemResourceFiles(self):
        """
        Push system resources files to devices
        """
        self.flag = []
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_oglconform')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("name")
        file_path = arti.get(binary_name)
        output = g_common_obj.push_file(file_path, "/data/app/oglconform")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/oglconform')
        self.flag.append(output)
        egl_config = cfg.get("egl_config")
        egl_config_file_path = arti.get(egl_config)
        output = g_common_obj.push_file(egl_config_file_path, "/data/app/egl-config")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/egl-config')
        self.flag.append(output)
        egl_config = cfg.get("binary_es3")
        egl_config_file_path = arti.get(egl_config)
        output = g_common_obj.push_file(egl_config_file_path, "/data/app/get-program-binary_es3")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/get-program-binary_es3')
        self.flag.append(output)
        cfg = config.read(cfg_file, 'content_gits')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("busybox")
        file_path = arti.get(binary_name)
        output = g_common_obj.push_file(file_path, "/data/busybox")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/busybox')
        self.flag.append(output)
        cfg = config.read(cfg_file, 'content_GLCTS')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("name")
        file_path = arti.get(binary_name)

        print "push android_standalone_x86/ to DUT"
        output = g_common_obj.push_file(file_path, "/data/app/")
        g_common_obj.adb_cmd_common("shell /data/busybox tar xzvf /data/app/android_standalone_x86.tgz -C /data/app", 10)
        g_common_obj.shell_cmd("rm -rf android_standalone_x86*")
        g_common_obj.adb_cmd_common('shell rm /data/app/android_standalone_x86.tgz')
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/android_standalone_x86/glcts')
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/android_standalone_x86/test.sh')
        self.flag.append(output)
        cfg = config.read(cfg_file, 'content_conform')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("name")
        file_path = arti.get(binary_name)
        output = g_common_obj.push_file(file_path, "/data/app/conform")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/conform')
        self.flag.append(output)
        binary_name = cfg.get("covegl")
        file_path = arti.get(binary_name)
        output = g_common_obj.push_file(file_path, "/data/app/covegl")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/covegl')
        self.flag.append(output)
        binary_name = cfg.get("covgl")
        file_path = arti.get(binary_name)
        output = g_common_obj.push_file(file_path, "/data/app/covgl")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/covgl')
        self.flag.append(output)
        binary_name = cfg.get("primtest")
        file_path = arti.get(binary_name)
        output = g_common_obj.push_file(file_path, "/data/app/primtest")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/primtest')
        self.flag.append(output)
        cfg = config.read(cfg_file, 'content_android_extension_pack')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("name")
        file_path = arti.get(binary_name)
        output = g_common_obj.push_file(file_path, "/data/app/oglconform_x86")
        g_common_obj.adb_cmd_common('shell chmod 777 /data/app/oglconform_x86')
        self.flag.append(output)


        for i in range(0, len(self.flag)):
            if self.flag[i] is False:
                return False
        return True
