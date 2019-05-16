"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL OTC ANDROID QA
:description: #TODO
:since: 10/15/14
:author: mihaela
"""

from OGLConform import Auxiliary
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.GFXUtilities.opengles_log_parser import StatusCode
import csv
import os
import Core.PathManager as PathManager


class KhronosBin(object):
    """
    #TODO: enter description
    """

    def __init__(self, device, report_path, logs_folder):
        """
        Constructor
        """
        self._device = device
        self._dut_location = "data/app/"
        # self._log_folder = 'gles1_logs'
        self._log_folder = logs_folder
        # self._logs_location = PathManager.absjoin(self._dut_location, self._log_folder)
        self._logs_location = self._dut_location + self._log_folder
        self._report_path = report_path
        self._logs_in_report = PathManager.absjoin(self._report_path, self._log_folder)
        self._aux = Auxiliary()

    def run_all(self, test_file, conform_option_list):
        testlist = 'TESTLIST'
        f_option = ' -f ' + self._dut_location + testlist
        random_seed = 32555
        # conform_option_list = ['mustpass'
        #                         # 'p_0'
        #                        # 'p_1',
        #                        # 'p_2',
        #                        # 'p_3'
        #  ]
        log_extension = '_log.txt'
        run_cmd = "'" + self._dut_location + test_file + ' -l ' + self._logs_location + '/'
        result, output = Global.BLOCKED, "Not run"
        if test_file in ['covgl', 'covegl', 'primtest']:
            run_cmd += test_file + log_extension + "'"
            result, output = self._aux.send_cmd(run_cmd, 1500)
        if test_file == 'conform':
            if 'mustpass' in conform_option_list:
                run_cmd2 = run_cmd + test_file + '_mustpass' + log_extension + ' -r ' + str(
                    random_seed) + ' -1 mustpass.c' + "'"
                result, output = self._aux.send_cmd(run_cmd2, 1500)
            if 'p_0' in conform_option_list:
                run_cmd2 = run_cmd + test_file + testlist + "_p_0" + log_extension + ' -r ' + str(
                    random_seed) + f_option + "'"
                result, output = self._aux.send_cmd(run_cmd2, 1500)
            if 'p_1' in conform_option_list:
                run_cmd2 = run_cmd + test_file + testlist + "_p_1" + log_extension + ' -r ' + str(
                    random_seed + 1) + f_option + ' -p 1' + "'"
                result, output = self._aux.send_cmd(run_cmd2, 1500)
            if 'p_2' in conform_option_list:
                run_cmd2 = run_cmd + test_file + testlist + "_p_2" + log_extension + ' -r ' + str(
                    random_seed + 2) + f_option + ' -p 2' + "'"
                result, output = self._aux.send_cmd(run_cmd2, 1500)
            if 'p_3' in conform_option_list:
                run_cmd2 = run_cmd + test_file + testlist + "_p_3" + log_extension + ' -r ' + str(
                    random_seed + 3) + f_option + ' -p 3' + "'"
                result, output = self._aux.send_cmd(run_cmd2, 1500)
        return result, output


    def run_one(self, binary, test):
        testlist = 'TESTLIST'
        f_option = ' -f ' + self._dut_location + testlist
        random_seed = 32555

        rmd_cmd = "adb shell rm -rf %s" % self._logs_location
        rmd_res, rmd_out = self._device.run_cmd(rmd_cmd, 600)
        makedir_cmd = "adb shell mkdir %s" % self._logs_location
        mkd_res, mkd_out = self._device.run_cmd(makedir_cmd, 600)
        result, output = Global.BLOCKED, "Not run"
        log_extension = '_log.txt'
        run_cmd = "'" + self._dut_location + binary + ' -l ' + self._logs_location + '/'
        result, output = Global.BLOCKED, "Not run"
        if binary in ['covgl', 'covegl', 'primtest']:
            run_cmd += binary + log_extension + "'"
            result, output = self._aux.send_cmd(run_cmd, 1500)
        if binary == 'conform':
            run_cmd2 = run_cmd + binary + '_' + str(test).replace(".c", "") + log_extension + ' -r ' + str(
                random_seed) + ' -1 ' + test + "'"
            result, output = self._aux.send_cmd(run_cmd2, 1500)
            run_cmd3 = run_cmd + binary + '_' + str(test).replace(".c", "") + '_p1' + log_extension + ' -r ' + str(
                random_seed) + ' -1 ' + test + " -p 1" + "'"
            result, output = self._aux.send_cmd(run_cmd3, 1500)
            run_cmd4 = run_cmd + binary + '_' + str(test).replace(".c", "") + '_p2' + log_extension + ' -r ' + str(
                random_seed) + ' -1 ' + test + " -p 2" + "'"
            result, output = self._aux.send_cmd(run_cmd4, 1500)
            run_cmd5 = run_cmd + binary + '_' + str(test).replace(".c", "") + '_p3' + log_extension + ' -r ' + str(
                random_seed) + ' -1 ' + test + " -p 3" + "'"
            result, output = self._aux.send_cmd(run_cmd5, 1500)
        return result, output

    def run_conformance(self):

        test_list = [
            'covgl',
            'covegl',
            'primtest',
            'conform'
        ]
        conform_option_list = ['mustpass',
                               'p_0',
                               'p_1',
                               'p_2',
                               'p_3'
        ]
        rmd_cmd = "adb shell rm -rf %s" % self._logs_location
        rmd_res, rmd_out = self._device.run_cmd(rmd_cmd, 60)
        makedir_cmd = "adb shell mkdir %s" % self._logs_location
        mkd_res, mkd_out = self._device.run_cmd(makedir_cmd, 60)
        result, output = Global.BLOCKED, "Not run"
        for test in test_list:
            result, output = self.run_all(test, conform_option_list)
        return result, output

    def run_single(self, test, conformance):
        """
        This might just be redundant
        """
        rmd_cmd = "adb shell rm -rf %s" % self._logs_location
        rmd_res, rmd_out = self._device.run_cmd(rmd_cmd, 600)
        makedir_cmd = "adb shell mkdir %s" % self._logs_location
        mkd_res, mkd_out = self._device.run_cmd(makedir_cmd, 600)
        result, output = self.run_all(test, conformance)
        return result, output

    def pull_results_gles1(self, local_path, timeout):
        """

        """
        pull_path = PathManager.absjoin(local_path, self._log_folder)
        result, output = self._device.pull(self._logs_location, pull_path, timeout)
        return result, output

    def parse_file_gles1(self, filename):
        results_list = []
        with open(PathManager.absjoin(self._logs_in_report, filename)) as result_file:
            all_lines = [line.strip().replace(".", "") for line in result_file]
        if 'conform' not in filename:
            test_prefix = ''
        else:
            test_prefix = 'conform: '
        for line in all_lines:
            if 'conform' in filename:
                test_name = test_prefix + filename.replace("_log.txt", "")
                k = 0
                if "SUMMARY:" in line:
                    k += 1
                if 'NO tests failed' in line:
                    results_list.append(test_name + '$' + 'passed')
                elif k == 3 and "test failed" in line:
                    results_list.append(test_name + '$' + 'failed')
                elif "TESTLIST" in filename:
                    if 'test' in line:
                        test_name = test_prefix + line.split('test')[0]
                        results_list.append(test_name + '$' + line.split('test')[1])
            else:
                if 'passed' in line or 'failed' in line:
                    if 'primtest' not in line and 'test' in line:
                        test_name = test_prefix + line.split('test')[0]
                        results_list.append(test_name + '$' + line.split('test')[1])
                    elif 'primtest' in line:
                        test_name = test_prefix + line.split(' (single precision) ')[0]
                        results_list.append(test_name + '$' + line.split(' (single precision) ')[1])
                    else:
                        test_name = test_prefix + line.split(' ')[0]
                        results_list.append(test_name + '$' + line.split(' ')[1])
        return results_list

    def res_to_csv(self, filename, results_file):
        all_results = self.parse_file_gles1(filename)
        csv_file_writer = csv.writer(results_file, delimiter=',', quoting=csv.QUOTE_NONE)
        for result in all_results:
            split_result = result.split('$')
            if split_result[1].replace(" ", "") == StatusCode.PASS_GLES:
                csv_file_writer.writerow([split_result[0], StatusCode.PASSED])
            elif split_result[1].replace(" ", "") == StatusCode.FAIL_GLES:
                csv_file_writer.writerow([split_result[0], StatusCode.FAILED])
            else:
                csv_file_writer.writerow([split_result[0], split_result[1]])
        return results_file

    def parse_all(self, result_filename):
        results_file = open(result_filename, 'wb')
        for root, dirs, files in os.walk(r'%s' % self._logs_in_report):
            for fileu in files:
                results_file = self.res_to_csv(fileu, results_file)
        results_file.close()
        return result_filename
