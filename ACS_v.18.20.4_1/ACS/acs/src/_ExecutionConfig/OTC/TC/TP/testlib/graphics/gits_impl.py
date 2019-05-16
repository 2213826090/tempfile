# Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
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

"""
@summary: This file implements for binary command
@since: 07/14/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import os
import os.path
import time
import tempfile
from testlib.util.common import g_common_obj
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.config import TestConfig
from testlib.common.base import getTmpDir
from testlib.util.otc_image import OTCImage

class GitsImpl:
    """
    Binary Test Impl Class
    """
    def __init__ (self):
        self.tmpdir = getTmpDir()
        self.tmp = tempfile.gettempdir()
        self.basic_screenshot = None
        self.screenshot = None

    @staticmethod
    def setup_enviroment(cfg):
        """
        @summary: set up gits enviroment
        """
        if g_common_obj.adb_cmd_capture_msg("ps | grep adbd")[0:4] != "root":
            g_common_obj.root_on_device()
#         g_common_obj.remount_device()

        config_handle = ConfigHandle()
        cfg["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        arti = Artifactory(cfg.get('artifactory_location'))
        renderscript_player_path = arti.get(cfg.get("renderscript_player"))
        busybox_path = arti.get(cfg.get("busybox"))
        gits_path = arti.get(cfg.get("gits"))
        busybox_tar_path = cfg.get("busybox_path")
        gits_tar_path = cfg.get("gits_path")
        g_common_obj.adb_cmd_capture_msg(
            "mkdir -p " + os.path.dirname(gits_tar_path))
        g_common_obj.push_file(busybox_path, busybox_tar_path)
        g_common_obj.push_file(gits_path, gits_tar_path)
        g_common_obj.push_file(renderscript_player_path, os.path.dirname(gits_tar_path))
        print g_common_obj.adb_cmd_capture_msg("chmod 777 " + busybox_tar_path)
        print g_common_obj.adb_cmd_capture_msg("chmod 777 " + gits_tar_path)
        print g_common_obj.adb_cmd_capture_msg("chmod 777 " + os.path.dirname(gits_tar_path) + "/" + os.path.basename(renderscript_player_path))

        # install busybox to /data/bb
        print g_common_obj.adb_cmd_capture_msg("mkdir -p /data/bb")
        print g_common_obj.adb_cmd_capture_msg(busybox_tar_path + " --install -s /data/bb")
#         print g_common_obj.adb_cmd_capture_msg(busybox_tar_path)
        # install gits
        print g_common_obj.adb_cmd_capture_msg("\"cd " + \
            os.path.dirname(gits_tar_path) + ";" + "export PATH=$PATH:/data/bb" + ";" + "sh " + gits_tar_path + "\"")

    def setup_stream(self):
        """
        @summary: set up stream and checked screenshot
        """
        self.basic_folder = self.cfg.get("stream_name")
        self.folder_name = self.cfg.get("screenshot_folder")
        os.system("mkdir -p %s/%s" % (self.tmp, self.basic_folder))
        g_common_obj.root_on_device()
        config_handle = ConfigHandle()
        self.cfg["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        arti = Artifactory(self.cfg.get('artifactory_location'))
        stream_path = arti.get(self.cfg.get("stream"))
        print "stream path", stream_path
        tar_path = self.cfg.get("tar_path")
        self.stream_folder = stream_path.replace(".tar.gz", "")
#         self.stream_folder = "%s/%s" % (self.tmpdir, self.basic_folder)
        print self.stream_folder
        # unzip
        print os.system("cd " + os.path.dirname(stream_path) + 
            ";tar xzf " + stream_path)
        for path, dirs, _ in os.walk(self.stream_folder):
            print path
            print "dirs:"
            print(dirs)
            for self.screenshot_folder in dirs:
                if self.folder_name in self.screenshot_folder:
                    print "screenshot folder:"
                    print self.screenshot_folder
                    print os.system("mv " + os.path.join(self.stream_folder,
                        self.screenshot_folder) + " %s/%s" % (self.tmp, self.basic_folder))
        # push stream to tar path
        print g_common_obj.adb_cmd_common("push " + 
            self.stream_folder + " " + tar_path, time_out=900)

    def diff(self, file_path, check_path):
        """
        compare folder:
        diff -r file_path check_path;echo $?
        """
        #cmd = "timeout 900 diff -r " + file_path + " " + check_path + ";echo $?"
        #print cmd
        #result = os.popen(cmd).read().strip()
        same = True
        old_pic_dir = os.path.join(file_path,  os.popen("ls " + file_path).read())
        new_pic_dir = os.path.join(check_path,  os.popen("ls " + file_path).read())
        for pic in os.popen("ls " + old_pic_dir):
            pic = pic.strip("\n")
            old_pic = os.path.join(old_pic_dir.strip("\n"), pic)
            new_pic = os.path.join(new_pic_dir.strip("\n"), pic)
            if OTCImage().calc_similar_by_path(old_pic, new_pic) != '1.0000':
                same = False
                break
        print "snapshot campare result: ", same
        return same
        #print "diff result:", result
        #if result[-1] == '0':
        #    print "[remove tmp screenshot files]"
        #    self.remove_screenshot_in_logdir()
        #    return True
        #else:
        #    return False

    def replay_stream(self, count):
        """
        replay the stream
        """
        replay_cmd = self.cfg.get("replay_cmd") + " " + \
        self.cfg.get("tar_path") + " " + \
        self.cfg.get("capture_cmd") + str(count)
        assert g_common_obj.adb_cmd_capture_msg(replay_cmd, time_out=900)
        screen_shot = self.cfg.get("screenshot_folder")
        result_shot = os.path.join(
            os.path.dirname(self.cfg.get("tar_path")), screen_shot)
        self.screenshot = self.cfg.get("result_screenshot")
        cmd = "pull " + self.screenshot + " " + os.path.join(
                self.tmpdir, self.cfg.get("screenshot_folder"))
        g_common_obj.adb_cmd_common(cmd, time_out=900)
        print "Pull command:", cmd
        g_common_obj.adb_cmd_capture_msg("rm -rf " + result_shot)

    def replay_renderscript_stream(self):
        """
        replay the renderscript stream
        """
        replay_cmd = self.cfg.get("replay_cmd") + " " + \
        self.cfg.get("tar_path")
        print replay_cmd
        cmd_output = g_common_obj.adb_cmd_capture_msg(replay_cmd)
        assert cmd_output.find("PASSED test") >= 0

    def remove_screenshot_in_logdir(self):
        """
        remove screenshot in log dir
        """
        print "rm -rf %s" % (self.tmpdir)
        os.system("rm -rf " + self.tmpdir)

    def remove_file(self):
        """
        remove the screenshot file
        """
        os.system("rm -rf %s/%s" % (self.tmp, self.basic_folder))
        os.system("rm -rf " + self.stream_folder)
        g_common_obj.adb_cmd_capture_msg("rm -rf " + self.cfg.get("tar_path"))

    def handle_test(self, tc_conf, tc_sect):
        """ run the test case
        """
        self.cfg = TestConfig().read(tc_conf, tc_sect)
        self.setup_stream()
        self.replay_stream(self.cfg.get("count"))
        assert self.check_screenshot(), "The screenshot is different!"

    def handle_test_renderscript(self, tc_conf, tc_sect):
        """ Run renderscript test case
        """
        self.cfg = TestConfig().read(tc_conf, tc_sect)
        self.setup_stream()
        self.replay_renderscript_stream()

    def remove_cache_rs(self):
        """ remove the cache
        """
        cmd = "rm -rf /data/data/com.android.rs.image/"
        g_common_obj.adb_cmd(cmd)

    def check_screenshot(self):
        """ check screenshot
        """
        check_folder = "%s/%s" % (self.tmp, self.basic_folder)
        for path, dirs, _ in os.walk(check_folder):
            print(path)
            print "dirs:"
            print(dirs)
            for self.screenshot_folder in dirs:
                if self.folder_name in self.screenshot_folder:
                    print "screenshot folder:"
                    print self.screenshot_folder
                    if self.diff(os.path.join(check_folder, self.screenshot_folder), os.path.join(
                        self.tmpdir, self.cfg.get("screenshot_folder"))):
                        return True
        print "[remove tmp screenshot files]"
        self.remove_screenshot_in_logdir()
        return False

