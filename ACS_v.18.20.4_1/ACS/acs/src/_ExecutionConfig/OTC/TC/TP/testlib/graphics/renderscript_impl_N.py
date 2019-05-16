# -*- coding: utf-8 -*-
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
@summary: CtsRenderscriptTestCases.apk command test Impl class
@since: 01/07/2016
@author: Zhao, Xiangyi
"""

import time
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class RenderScriptImpl(object):

    '''
        CtsRenderscriptTestCases.apk command test
        run adb shell am instrument command to test
    '''
    
    #pkg_name = "com.android.cts.renderscript"
    #for N
    pkg_name = "android.renderscript.cts"
    class_name = "android.support.test.runner.AndroidJUnitRunner"

    def __init__(self):
        self._device = g_common_obj.get_device()

    def setup(self):
        '''
            install from Artifactory
        '''
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_renderscript_N')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + str(file_path), 300)

    def run_case(self, case_name):
        """ Run given test case
        """
        cmd = "shell am instrument -w  -r -e class android.renderscript.cts.%s %s/%s | grep -w 'OK'" % (case_name, self.pkg_name, self.class_name)
        _result=[]
        n=2
        while n > 0:
            diag_output = g_common_obj.adb_cmd_common(cmd, 3000)
            _result.append(diag_output[0:2])
            if _result[0] != "OK":
                n = n - 1
                time.sleep(3)
            else:
                n = 0
        assert 'OK' in _result, "%s failed" % (case_name)

    def run_case_list(self, list_name):
        """ Run given test case in list
        """
        cfg_file = 'tests.tablet.artifactory.conf'
        config = TestConfig()
        cfg_instrument = config.read(cfg_file, 'renderscript_cmd_list')
        instrument_str = cfg_instrument.get(list_name)
        print instrument_str
        instrument_list = list(instrument_str.split(","))
        print instrument_list
        for i in range(0, len(instrument_list)):
            print instrument_list[i]
            self.run_case(instrument_list[i])

    def run_case_sublist(self, case_name, list_name):
        """ Run given test case in list
        """
        cfg_file = 'tests.tablet.artifactory.conf'
        config = TestConfig()
        cfg_instrument = config.read(cfg_file, 'renderscript_cmd_list')
        instrument_str = cfg_instrument.get(list_name)
        print instrument_str
        instrument_list = list(instrument_str.split(","))
        print instrument_list
        for i in range(0, len(instrument_list)):
            subcasename = case_name + "#" + instrument_list[i]
            print subcasename
            self.run_case(subcasename)

