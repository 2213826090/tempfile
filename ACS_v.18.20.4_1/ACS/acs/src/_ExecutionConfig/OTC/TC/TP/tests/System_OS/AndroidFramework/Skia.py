# -*- coding:utf-8 -*-

'''
@summary: Check if skia library exist and is a dependency of /system/bin/animation.
@since: 07/07/2016
@author: Lijin Xiong
'''

import os
from testlib.util.common import shell_command
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.common import AdbUtils,ShellUtils,Environment
from testlib.androidframework.fetch_resources import resource

class Skia_Lib(UIATestBase):

    bootanimation_bin = "bootanimation"
    ndk_depends_bin = "ndk-depends"

    def setUp(self):
        super(Skia_Lib, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self._bin_path = _path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "BIN", "ndk_depends")
        shell_command("chmod 755 %s" % self._bin_path)

    def test_skia_lib_existance(self):
        try:
            check_so_lib_cmd = "ls /system/lib | grep skia"
            output = AdbUtils._run_adb_cmd(check_so_lib_cmd, add_ticks=False)
            self.assertTrue("libskia.so" in output, "could not find libskia.so in /system/lib")
            AdbUtils.pull("/system/bin/bootanimation", os.path.split(self._bin_path)[0])
#             ndk_depends_abs_path = os.path.join(Environment.tmp_dir_path, Skia_Lib.ndk_depends_bin)
            ndk_depends_abs_path = self._bin_path
            bootanimation_bin_path = os.path.join(os.path.split(self._bin_path)[0], Skia_Lib.bootanimation_bin)
            output = ShellUtils.run_shell_cmd(ndk_depends_abs_path + " " + bootanimation_bin_path)
            self.assertTrue("libskia.so" in output, "could not find libskia.so in bootanimation dependecies")
        finally:
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)