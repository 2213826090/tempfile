#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: Gits test template class
@since: 01/29/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""
import os
import re
import time
from collections import deque
from testlib.util.config import TestConfig
from testlib.util.common import g_common_obj
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.gits_impl import GitsImpl


class GitsTestTemplate(UIATestBase):

    @classmethod
    def setUpClass(self):
        print "[setUpClass]: %s" % self.__name__
        super(GitsTestTemplate, self).setUpClass()

        cfg_gits = 'tests.tablet.artifactory.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_gits, "content_gits"))

        cfg_file = 'tests.tablet.gits_test_template.conf'
        self.conf = self.config.read(cfg_file, "gits_test_template")

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(GitsTestTemplate, self).setUp()

    def commonTest(self, config, section=None):
        print "[RunTest]: %s" % self.__str__()
        if not section:
            section = self.id().split('.')[-1]

        case_cfg = TestConfig()
        cfg = case_cfg.read(config, section)
        config_handle = ConfigHandle()
        cfg["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        arti = Artifactory(cfg.get('artifactory_location'))
        stream_fullpath = arti.get(cfg.get("stream"))
        stream_folder, stream_file = os.path.split(stream_fullpath)
        stream_name = os.path.basename(stream_file).split('.')[0]

        local_temp = '%s/%s' % (self.conf.get("local_temp"), stream_name)
        base_screenshots_path = '%s/base' % (local_temp)
        down_screenshots_path = '%s/remote' % (local_temp)
        remote_path = self.conf.get("remote_temp")
        os.system("rm -rf %s; mkdir -p %s" % (local_temp, local_temp))
        os.system("mkdir -p %s" % (base_screenshots_path))
        os.system("mkdir -p %s" % (down_screenshots_path))

        os.system("rm -rf %s/%s" % (stream_folder, stream_name))
        os.system("tar -C %s -zxvf %s" % (stream_folder, stream_fullpath))
        os.system("mv -v %s/%s/%s/* %s" \
            % (stream_folder, stream_name,\
               self.conf.get("screenshots_subpath"), base_screenshots_path))

        check_points = []
        for path, _, files in os.walk(base_screenshots_path):
            for _name in files:
                _point = int(re.findall('\d+', _name )[0])
                _full_path = os.path.join(path, _name)
                _remote_path = '%s/%s/%s/%s'\
                % (remote_path, stream_name,\
                   self.conf.get("screenshots_subpath"), _name)
                check_points.append((_point, _name, _full_path, _remote_path))
        assert check_points, "check_points empty!"
        check_points = deque(sorted(check_points, key=lambda t:t[0]))
        print "Check Points\n%s" % ([p[0] for p in check_points])

        g_common_obj.adb_cmd_capture_msg("rm -rf %s/%s; sync"\
                                         % (remote_path, stream_name))

        ret = g_common_obj.push_file("%s/%s" % (stream_folder, stream_name),\
                                     "%s/%s" % (remote_path, stream_name))
        assert ret, 'Failed push %s' % (stream_name)

        gits_player = self.conf.get("gits_player")
        while len(check_points) > 0:
            point1, name1, fullpath1, _remote_path1 = check_points.popleft()
            point2, name2, fullpath2, _remote_path2 =\
                                    point1, name1, fullpath1, _remote_path1
            if len(check_points) > 0:
                point2, name2, fullpath2, _remote_path2 =\
                                                    check_points.popleft()

            lenth = point2-point1 if point2-point1 > 0 else 1
            cmd_play = '%s %s/%s --captureFrames %s-%s:%s'\
                    % (gits_player, remote_path, stream_name,\
                                point1, point2, lenth)
            g_common_obj.adb_cmd_capture_msg(cmd_play)

            pull_path1 = '%s/%s' % (down_screenshots_path, name1)
            ret = g_common_obj.pull_file(pull_path1, _remote_path1)
            assert ret, 'Failed get %s' % (_remote_path1)
            self._diff_imag_file(stream_name, fullpath1, pull_path1)

            if point1 == point2: continue

            pull_path2 = '%s/%s' % (down_screenshots_path, name2)
            ret = g_common_obj.pull_file(pull_path2, _remote_path2)
            assert ret, 'Failed get %s' % (_remote_path1)
            self._diff_imag_file(stream_name, fullpath2, pull_path2)

        g_common_obj.adb_cmd_capture_msg("rm -rf %s/%s; sync"\
                                         % (remote_path, stream_name))

    def _diff_imag_file(self, stream_name, fullpath, pull_path):

        diff_img = "timeout 900 diff %s %s;echo $?"\
                        % (fullpath, pull_path)
        result = os.popen(diff_img).read().strip()

        if result != "0":
            result_dir = '%s/%s'\
                        % (self.conf.get("result_path"), stream_name)
            result_base = '%s/base' % (result_dir)
            result_remote = '%s/remote' % (result_dir)
            os.system("mkdir -p %s" % (result_base))
            os.system("mkdir -p %s" % (result_remote))
            os.system("cp -v %s %s" % (fullpath, result_base))
            os.system("cp -v %s %s" % (pull_path, result_remote))
            assert False, diff_img

        print diff_img, result
    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(GitsTestTemplate, self).tearDown()

    @classmethod
    def tearDownClass(self):
        print "[tearDownClass]: %s" % self.__name__
        super(GitsTestTemplate, self).tearDownClass()
