#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
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
@summary: This file implements for binary command
@since: 07/14/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import os
import os.path
import time
from testlib.util.common import g_common_obj

class BinaryImpl:
    """
    Binary Test Impl Class
    """
    def __init__ (self, cfg={}):
        self.cfg = cfg

    @staticmethod
    def push_binary(file_path, tar_path, timeout=60):
        """
        @summary: push binary to device and link to system binary
        @paramater:
            file_path: binary file path in host
            tar_path: target path in device
            timeout: push timeout
        @return: None
        """
        g_common_obj.push_file(file_path, tar_path)
        chromd_cmd = "chmod 777 " + tar_path
        g_common_obj.adb_cmd_capture_msg(chromd_cmd)

    @staticmethod
    def get_key_in_logcat(key):
        """
        @summary: get key message in the logcat
        """
        cmd = "logcat -d|grep \"%s\"" % key
        message = g_common_obj.adb_cmd_common(cmd)
        return message

    @staticmethod
    def run_cmd_and_collect_output(cmd, file_path):
        """
        run binary command and collect output
        """
        os.system("rm -rf " + file_path)
        run_cmd = "\"" + cmd + "\"" + " > " + file_path
        g_common_obj.adb_cmd_capture_msg(run_cmd)
        return file_path

    @staticmethod
    def get_result_paragraph_from_log(key, file_path, result_path, line=None):
        """
        get key word in log file and output the paragraph
        """
        os.system("rm -rf " + result_path)
        f_result = open(file_path, "r")
        results = f_result.readlines()
        f_result.close()
        f_log = file(result_path, "a+")
        res_find = False
        if line is not None:
            line = int(line)
        for result in results:
            if key in result:
                res_find = True
            if res_find:
                print result
                f_log.write(result)
                if line is not None:
                    line -= 1
                if line <= 0:
                    break
        f_log.close()
        if not res_find:
            print "[INFO] Does not find %s in output" % key
        else:
            print "[INFO]Find %s in output" % key
        return res_find

    @staticmethod
    def get_result_line_from_log(key, file_path, result_path, line=None):
        """
        get key word in log file and output the line
        """
        os.system("rm -rf " + result_path)
        f_result = open(file_path, "r")
        results = f_result.readlines()
        f_result.close()
        f_log = file(result_path, "a+")
        res_find = False
        for result in results:
            if key in result:
                print result
                res_find = True
                f_log.write(result)
                if line is not None:
                    line = int(line)
                    line -= 1
                if line <= 0:
                    break

        f_log.close()
        return res_find

    @staticmethod
    def check_key_in_file(key, file_path):
        """
        check if key in file
        """
        f_result = open(file_path, "r")
        results = f_result.readlines()
        f_result.close()
        res_find = False
        for result in results:
            if key in result:
                print result
                res_find = True
        return res_find








