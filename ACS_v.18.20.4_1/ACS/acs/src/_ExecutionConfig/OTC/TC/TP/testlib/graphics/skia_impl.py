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
@summary: SkiaTestImpl class
@since: 04/07/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import re

from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import logcat


class SkiaImpl(object):

    """SkiaImpl"""

    config_file = 'tests.common.skia.conf'

    def __init__(self):
        self.configer = TestConfig()
        self.config = self.configer.read(self.config_file, "SkiaImpl")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))
        self.dut_skia_test = self.config.get('dut_skia_test')
        self.dut_skia_bench = self.config.get('dut_skia_bench')

    def setup(self):
        local_skia_test = self.arti.get(self.config.get("skia_test"))
        assert local_skia_test,\
            "Download resource failed!"
        ret = g_common_obj.push_file(local_skia_test, self.dut_skia_test)
        assert ret, 'Failed push %s' % (local_skia_test)

        local_skia_bench = self.arti.get(self.config.get("skia_bench"))
        assert local_skia_bench,\
            "Download resource failed!"
        ret = g_common_obj.push_file(local_skia_bench, self.dut_skia_bench)
        assert ret, 'Failed push %s' % (local_skia_bench)

        cmd = 'chmod 777 %s; chmod 777 %s'\
            % (self.dut_skia_test, self.dut_skia_bench)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

    def clean(self):
        cmd = 'rm -f %s; rm -f %s; sync'\
            % (self.dut_skia_test, self.dut_skia_bench)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

    def run_skia_test(self):
        time_mark = logcat.get_device_time_mark()
        cmd = "%s -v" % (self.dut_skia_test)
        g_common_obj.adb_cmd_capture_msg(repr(cmd), time_out=60 * 5)
        self.parse_skia_test_log(time_mark)

    def run_skia_bench(self):
        time_mark = logcat.get_device_time_mark()
        cmd = "%s" % (self.dut_skia_bench)
        g_common_obj.adb_cmd_capture_msg(repr(cmd), time_out=60 * 60)
        self.parse_skia_bench_log(time_mark)

    def parse_skia_test_log(self, time_mark):
        logs = logcat.get_device_log(time_mark, "skia:D *:S")
        # Finished XX tests, XX failures, XX skipped.
        pattern = r"Finished (?P<RET_TOTAL>\d+) tests, "\
            r"(?P<RET_FAIL>\d+) failures, (?P<RET_SKIP>\d+) skipped."
        match = re.search(pattern, logs)
        assert match,\
            "[FAILURE] can't match result summary log"
        failed_results = re.findall(".+FAILED.*", logs, re.IGNORECASE)

        total_cnt = match.group('RET_TOTAL')
        fail_cnt = match.group('RET_FAIL')
        skip_cnt = match.group('RET_SKIP')
        summary = "Finished %s tests, %s failures, %s skipped."\
            % (total_cnt, fail_cnt, skip_cnt)
        print "[Debug] %s" % (summary)
        assert total_cnt and fail_cnt == '0' and skip_cnt == '0',\
            "[FAILURE] %s\n%s" % (summary, ''.join(failed_results))

    def parse_skia_bench_log(self, time_mark):
        logs = logcat.get_device_log(time_mark, "skia:D *:S")
        assert len(logs), "[FAILURE] No log caught!"
        tested_cases = re.findall(r'running.+', logs, re.IGNORECASE)
        print "[Debug] skia bench tested count:%s" % (len(tested_cases))
        assert len(tested_cases) > 0,\
            "[FAILURE] not found! skia bench test log"
        errors = re.findall(r'.*Can\'t get.+|.*fail.*', logs, re.IGNORECASE)
        assert len(errors) == 0,\
            "[FAILURE] skia bench test results:\nTested %s Failed %s \n%s"\
            % (len(tested_cases), len(errors), ''.join(errors))
