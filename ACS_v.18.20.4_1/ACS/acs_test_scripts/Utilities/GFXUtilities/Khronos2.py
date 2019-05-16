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
:since: 10/7/14
:author: mmaraci
"""

from UtilitiesFWK.Utilities import Global
import os
import csv

from acs_test_scripts.Utilities.GFXUtilities.opengles_log_parser import BatchResultParser
from acs_test_scripts.Utilities.GFXUtilities.opengles_log_parser import StatusCode

import Core.PathManager as PathManager

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT

from UtilitiesFWK.AcsSubprocess.AcsSubprocess import AcsSubprocess

from Device.DeviceLogger.LogCatLogger.LogCatReaderThread import LogCatReaderThread


class Khronos2(object):
    TEST_SUITES = 'acs_test_suites'
    TEST_SUITES_PATH = PathManager.absjoin(PathManager.ACS_TEST_SCRIPTS_DIR, TEST_SUITES)

    def __init__(self, device, device_logdir, report_path, activity_name):
        """
        Constructor parameters
        """
        self._device = device
        self._device_logdir = device_logdir
        self._report_path = report_path
        self._default_logging_dir = '/sdcard/'  #this is the log directory that the APK has as default and cannot be overwritten in Native mode
        self._activity_name = activity_name

        self._logger = LOGGER_TEST_SCRIPT

        logcat_cmd_line = self._device.get_config("logcatCmdLine", "adb shell logcat -v threadtime")

        self._logcat_analyzer = LogCatReaderThread(self._device, logger=self._logger, enable_writer=False,
                                                   logcat_cmd_line=logcat_cmd_line)

        # self._trigger_message = "Alive"
        # self._trigger_message= 'am_finish_activity'
        self._trigger_message = 'org.khronos.gl_cts/org.khronos.cts.%sActivity,proc died without state saved' % self._activity_name
        self._helper_scripts_path = 'OTC/TC/ACS/Graphics/Graphics/scripts'
        self._helper_scripts_gfx = PathManager.absjoin(Khronos2.TEST_SUITES_PATH,
                                                       self._helper_scripts_path)
        if self._device_logdir is not None:
            self._pull_path = self._default_logging_dir + self._device_logdir

    def _execute_command(self, cmd):
        """
        This private method offers a way of executing a command during the ACS run by calling the AcsSubprocess method of execute_sync(timeout)
        We use this in order to watch the result execution in a separate synchronous subprocess
        """
        my_process, stdout = AcsSubprocess(cmd, LOGGER_TEST_SCRIPT, max_empty_log_time=1, silent_mode=False).execute_sync(200)
        return my_process, stdout

    def _check_process_alive(self, process_name, timeout):
        """
        A method that greps for our activity to check if it is still alive. In addition, it also searches for the
        SIGSEGV message in the logcat, in order to identify a process that has been terminated before it's normal
        finish
        """
        start = 0
        cmd_header = "adb shell ps | grep "
        self._logcat_analyzer.add_trigger_message(self._trigger_message)
        self._logcat_analyzer.start()

        result, msg = Global.BLOCKED, "Did not start watching the process' life-cycle"
        send_cmd = cmd_header + process_name
        while start < timeout:
            my_proc, stdout = self._execute_command(send_cmd)
            if 'khronos' not in stdout:
                if self._logcat_analyzer.get_message_triggered_status(self._trigger_message):
                    self._logger.debug("Caught the activity being terminated")
                    result, msg = Global.FAILURE, "The activity did not finish before the process died"
                else:
                    result, msg = Global.SUCCESS, "The activity finished its execution and properly stopped"
                return result, msg
            else:
                result, msg = Global.SUCCESS, "The %s process is alive" % process_name
                start += 0.1
        return result, msg

    def _pull_results(self, remote_path, local_path, special_string, timeout):
        """
        Pull the results, whatever the metohds generate and need
        Extending(not in a pretty way) _device.pull()
        """
        if special_string != '':
            full_remote_path = remote_path + "/" + special_string
            mock_dir = "mock"
            full_mock_path = remote_path + "/" + mock_dir
            adb_mkdir_mock = "adb shell mkdir %s/%s" % (self._pull_path, mock_dir)
            status_mkdir, msg_mkdir = self._device.run_cmd(adb_mkdir_mock, timeout)
            adb_cmd_mv = "adb shell mv %s %s" % (full_remote_path, full_mock_path)

            status_mv, msg_mv = self._device.run_cmd(adb_cmd_mv, timeout)
            status, err_msg = self._device.pull(full_mock_path, local_path, timeout)
        elif special_string == '':
            status, err_msg = self._device.pull(remote_path, local_path, timeout)
        return status, err_msg

    def _res_to_csv(self, result_filename):
        """
        Taken from the sources sent by the Graphics devs
        This method will read the QPA results and write them into a CSV file
        """
        parser = BatchResultParser()
        file_to_parse = PathManager.absjoin(self._report_path, result_filename)
        results = parser.parseFile(file_to_parse)
        write_in_file = file_to_parse.strip('.qpa') + '.csv'
        result_file_csv = open(write_in_file, 'wb')
        result_writer = csv.writer(result_file_csv, delimiter=',', quoting=csv.QUOTE_NONE)
        for result in results:
            result_writer.writerow([result.name, result.statusCode])
        result_file_csv.close()
        return write_in_file

    def _parse_folder_content(self):
        results_list = []
        parser = BatchResultParser()
        for root, dirs, files in os.walk(r'%s' % self._report_path):
            for fileu in files:
                file2 = PathManager.absjoin(self._report_path, fileu)
                if fileu.endswith('.qpa'):
                    results = parser.parseFile(file2)
                    for result in results:
                        results_list.append(result.name + ',' + result.statusCode)
                else:
                    pass
        return results_list

    def write_to_csv(self, result_list, csv_file):
        """

        """
        result_file_csv = open(csv_file, 'wb')
        result_writer = csv.writer(result_file_csv, delimiter=',', quoting=csv.QUOTE_NONE)
        for result in result_list:
            element = result.split(",")
            result_writer.writerow([element[0], element[1]])
        result_file_csv.close()
        return csv_file

    def extract_results(self, tc_name, tc_order, tc_report, result_filename_val):
        """
        compute results from gfx csv file test results
        - add each test result to a secondary report

        :param result_filename: gfx log output and it's location
        :type file_path: str

        :rtype: str
        :return: output message
        """
        test_executed = 0
        result = Global.FAILURE
        output = ""
        result_filename = result_filename_val
        if result_filename is None:
            self._device.get_logger().error("There is no specified csv file from GFX fwk !")
            output = "GFX result not detected in the log, No result"
            result = Global.FAILURE
        elif not os.path.isfile(result_filename):
            self._device.get_logger().error(
                "Specified GFX result file %s does not exist, cannot compute properly results !" % (result_filename,))
            output = "GFX result is missing, No result"
            result = Global.FAILURE
        else:
            with open(result_filename, "r") as gfx_report:
                csv_content = csv.reader(gfx_report, delimiter=',', quotechar='|')
                result = Global.SUCCESS
                for row in csv_content:
                    if len(row) >= 2:
                        if StatusCode.PASS in row[1] or StatusCode.PASSED in row[1]:
                            test_executed += 1
                            tc_report.add_result(row[0], tc_report.verdict.PASS, "Test PASS", tc_name, tc_order)
                        elif StatusCode.SUPPORTED_REPORTED in row[1]:
                            test_executed += 1
                            tc_report.add_result(row[0], tc_report.verdict.PASS, "Test status is %s" % row[1],
                                                 tc_name, tc_order)
                        elif StatusCode.NOT_SUPPORTED in row[1]:
                            continue
                        else:
                            result = Global.FAILURE
                            output = "Some tests have the result: FAIL"
                            test_executed += 1
                            tc_report.add_result(row[0], tc_report.verdict.FAIL,
                                                 "Test status is %s, see log output for more details" % row[1], tc_name,
                                                 tc_order)
                    else:
                        result = Global.FAILURE
                        output = "CSV seems corrupted, there are less than 2 rows!"

                if result != Global.FAILURE:
                    output = "All tests have the result: PASS"
                if not test_executed:
                    result = Global.FAILURE
                    output = "No tests have been executed"
        return result, output

    def extract_results_from_list(self, tc_name, tc_order, tc_report, results_list):
        """
        compute results from gfx csv file test results
        - add each test result to a secondary report

        :param result_filename: gfx log output and it's location
        :type file_path: str

        :rtype: str
        :return: output message
        """
        test_executed = 0

        if results_list:
            result = Global.SUCCESS
            for element in results_list:
                if len(element) >= 2:
                    row = element.split(",")
                    if StatusCode.PASS in row[1] or StatusCode.PASSED in row[1]:
                        test_executed += 1
                        tc_report.add_result(row[0], tc_report.verdict.PASS, "Test PASS", tc_name, tc_order)
                    elif StatusCode.SUPPORTED_REPORTED in row[1]:
                        test_executed += 1
                        tc_report.add_result(row[0], tc_report.verdict.PASS, "Test status is %s" % row[1],
                                             tc_name, tc_order)
                    elif StatusCode.NOT_SUPPORTED in row[1]:
                        continue
                    else:
                        result = Global.FAILURE
                        output = "Some tests have the result: FAIL"
                        test_executed += 1
                        tc_report.add_result(row[0], tc_report.verdict.FAIL,
                                             "Test status is %s, see log output for more details" % row[1], tc_name,
                                             tc_order)
                else:
                    result = Global.FAILURE
                    output = "The results list seems to be corrupted"

            if result != Global.FAILURE:
                output = "All tests have the result: PASS"
            if not test_executed:
                result = Global.FAILURE
                output = "No tests have been executed"
        else:
            result, output = Global.FAILURE, "The list with the results is empty"
        return result, output

    def extract_no_secondary_report(self, results_list):
        """

        """
        test_executed = 0
        test_passed = 0
        test_failed = 0
        test_not_supported = 0
        test_sigsegv = 0

        if results_list:
            result = Global.SUCCESS
            for element in results_list:
                if len(element) >= 2:
                    row = element.split(",")
                    if StatusCode.PASS in row[1] or StatusCode.PASSED in row[1]:
                        test_executed += 1
                        test_passed += 1
                    elif StatusCode.NOT_SUPPORTED in row[1]:
                        test_executed += 1
                        test_not_supported += 1

                    elif "SIGSEGV" in row[1]:
                        test_executed += 1
                        test_sigsegv += 1
                    else:
                        result = Global.FAILURE
                        output = "Some tests have the result: FAIL"
                        test_executed += 1
                        test_failed += 1
                else:
                    result = Global.FAILURE
                    output = "The results list seems to be corrupted"

            if result != Global.FAILURE:
                output = "All tests have the result: PASS"
            if not test_executed:
                result = Global.FAILURE
                output = "No tests have been executed"
        else:
            result, output = Global.FAILURE, "The list with the results is empty"
        output = "The results look like this: \n %s tests were executed;\n %s tests have passed;\n" \
                 " %s tests are not supported;\n %s tests have a SIGSEGV result;\n %s tests have failed. " \
                 "\n Please check with the .csv and .qpa file logs for more details." % (test_executed, test_passed,
        test_not_supported, test_sigsegv, test_failed)
        return result, output


    def pull_and_extract(self, tc_name, tc_order, tc_report):
        pull_res, pull_out = self._pull_results(self._pull_path, self._report_path, special_string='*.qpa', timeout=500)
        result, output = Global.BLOCKED, "Did not yet start extracting"
        for root, dirs, files in os.walk(r'%s' % self._report_path):
            for file in files:
                if file.endswith('.qpa'):
                    preliminary_file = self._res_to_csv(file)
                    result, output = self.extract_results(tc_name, tc_order, tc_report,
                                                          result_filename_val=preliminary_file)
                else:
                    pass
        return result, output

    def pull_and_extract_with_lists(self, tc_name, tc_order, tc_report, csv_report):
        pull_res, pull_out = self._pull_results(self._pull_path, self._report_path, special_string='*.qpa', timeout=500)
        result, output = Global.BLOCKED, "Did not yet start extracting"
        results_list = self._parse_folder_content()
        csv_file = self.write_to_csv(results_list, csv_report)
        result, output = self.extract_results_from_list(tc_name, tc_order, tc_report, results_list)
        return result, output

    def pull_and_extract_no_secondary(self, csv_report):
        """

        """
        pull_res, pull_out = self._pull_results(self._pull_path, self._report_path, special_string='*.qpa', timeout=500)
        result, output = Global.BLOCKED, "Did not yet start extracting"
        results_list = self._parse_folder_content()
        csv_file = self.write_to_csv(results_list, csv_report)
        result, output = self.extract_no_secondary_report(results_list)
        return result, output

    def generate_groups(self):
        """
            A simple method for generating the test groups
            """
        mock_dir = "mock"
        full_mock_path = self._default_logging_dir + "/" + mock_dir

        adb_mkdir_mock = "adb shell mkdir %s" % full_mock_path
        status_mkdir, msg_mkdir = self._device.run_cmd(adb_mkdir_mock, 20)
        groups_cmd = "adb shell am start -n org.khronos.gl_cts/android.app.NativeActivity \
            -e cmdLine 'cts --deqp-runmode=txt-caselist'"
        pull_sequence = "ES*"
        result, output = self._device.run_cmd(groups_cmd, 1500)
        group_files = ["ES2-CTS-groups.txt", "ES3-CTS-groups.txt", "ES31-CTS-groups.txt"]
        if result == Global.SUCCESS:
            pull_result, pull_msg = self._pull_results(self._default_logging_dir, self._helper_scripts_gfx,
                                                       pull_sequence, 1500)
            if pull_result == Global.FAILURE:
                result, output = Global.FAILURE, "Could not pull the groups files from the DUT, please check logs"
            else:
                for root, dirs, files in os.walk(r'%s/' % self._helper_scripts_gfx):
                    for file in files:
                        for gf in group_files:
                            tf_d = open(self._helper_scripts_gfx + "/" + file, 'r')
                            gf_d = open(self._helper_scripts_gfx + "/" + gf, 'w')
                            group_run = None
                            for line in tf_d:
                                if "GROUP" in line:
                                    group_run = line.split(" ")[1]

                                if ("TEST" in line) and group_run:
                                    gf_d.write(group_run.strip('\n') + '.*' + '\n')
                                    group_run = None
                            tf_d.close()
                            gf_d.close()
                            result, output = Global.SUCCESS, "Files were generated in the %s folder" \
                                                             % self._helper_scripts_gfx
        return result, output


    def _run_certification_mode_full(self):
        """
            Method for running the ES31Activity
            cam hardcodata partea cu activity name, merge momentan
            """
        adb_run_all = "adb shell am start -n org.khronos.gl_cts/org.khronos.cts.%sActivity -e logdir '%s/%s'" % (
            self._activity_name, self._default_logging_dir, self._device_logdir)
        result, output = self._device.run_cmd(adb_run_all, 1500)

        return result, output


    def single_run(self, group_name, window_size, configID, timeout=300):
        """
            This method runs a single test or a group of tests using the NativeActivity from the APK
            Parameters:
            group_name: the name of the test or group to be ran.
                        Can either be a full name test or one that includes more tests.
                        Usage example:
                        - 1 test: ES3-CTS.info.vendor
                        - 1 group: ES3-CTS.info.*
            window_size: this parameter is the possible window size. The possible values map with the
                         win_cmd dictionary.
                         Possible values:
                         - ws_64_64
                         - ws_113_47
                         - fbo1
                         - fbo2
            configID: parameter for the config-id of the test. Possible values are in the range 1-11
            timeout: integer value, given in seconds
                    the timeout parameter for methods that interact via adb with the DUT
            """
        # dictionary to keep formatted options for different windows sizes/ fbo
        win_cmd = {"ws_64_64": "\"cts \
                                    --deqp-surface-width=64 \
                                    --deqp-surface-height=64 \
                                    --deqp-base-seed=1 \
                                    --deqp-surface-type=window ",
                   "ws_113_47": "\"cts \
                                    --deqp-surface-width=113 \
                                    --deqp-surface-height=47 \
                                    --deqp-base-seed=2 \
                                    --deqp-surface-type=window ",
                   "fbo1": "\"cts \
                                    --deqp-surface-width=64 \
                                    --deqp-surface-height=64 \
                                    --deqp-base-seed=3 \
                                    --deqp-surface-type=window \
                                    --deqp-use-fbo=GL_RGBA8,GL_DEPTH24_STENCIL8,64,max ",
                   "fbo2": "\"cts \
                                    --deqp-surface-width=64 \
                                    --deqp-surface-height=64 \
                                    --deqp-base-seed=3 \
                                    --deqp-surface-type=window \
                                    --deqp-use-fbo=GL_RGBA8,GL_DEPTH24_STENCIL8,max,64 "}

        adb_am_start = "adb shell am start -n org.khronos.gl_cts/android.app.NativeActivity -e cmdLine "
        # log name for the test result

        if "*" in group_name:
            log_name = "%s%s/%s_%s_cfgID_%s.qpa" % (
                self._default_logging_dir, self._device_logdir, group_name[0:-2], window_size, configID)
        else:
            log_name = "%s%s/%s_%s_cfgID_%s.qpa" % (
                self._default_logging_dir, self._device_logdir, group_name, window_size, configID)
        # group to test and log file part of the command
        cmd_line_arg = "--deqp-case=%s --deqp-log-filename=%s " % (group_name, log_name)
        cmd_ID_arg = "--deqp-gl-config-id=%s\"" % str(configID)
        adb_single_run = adb_am_start + win_cmd[window_size] + cmd_line_arg + cmd_ID_arg
        run_result, run_output = self._device.run_cmd(adb_single_run, timeout)

        return run_result, run_output


    def _run_all_configs(self, window_sizes, group_list, timeout):
        """
            This method runs all the possible config-ids for the tests that provide the following
            parameters:
            window_sizes: a vector with the possible values for window size
            group_list: parameter for a test or a group that has to be ran.
            This method is called later into the _run_non_certification_mode() method.
            This method uses the previous method single_run
            timeout: integer value, given in seconds
                    the timeout parameter for methods that interact via adb with the DUT
            """
        #~ # run all groups  ws = 64 x 64 config-id 1
        result, output = self.single_run(group_list, window_sizes[0], 1, timeout)
        # run all CTS.gtf.*  ws = 113 x 47 config-id 1
        if "CTS.gtf" in group_list:
            result, output = self.single_run(group_list, window_sizes[1], 1, timeout)
        # run CTS.gtf.* fbo1
        if "CTS.gtf" in group_list:
            result, output = self.single_run(group_list, window_sizes[2], 1, timeout)
        # run CTS.gtf.* fbo2
        if "CTS.gtf" in group_list:
            result, output = self.single_run(group_list, window_sizes[3], 1, timeout)
        # run ws = 64 x 64 all configIDs 2-7, 9-11
        if "CTS.gtf" in group_list or "ES31" in group_list:
            for cfgID in [2, 3, 4, 5, 6, 7, 9, 10, 11]:
                result, output = self.single_run(group_list, window_sizes[1], cfgID, timeout)
        # run ws = 113 x 47 all configIDs 2-7, 9-11
        if "CTS.gtf" in group_list:
            for cfgID in [2, 3, 4, 5, 6, 7, 9, 10, 11]:
                result, output = self.single_run(group_list, "ws_113_47", cfgID, timeout)

        return result, output


    def _run_non_certification_mode(self, group_list_file, timeout):
        """
            Method for running a list of tests from a group on all config IDs and all Window Sizes
            ConfigID is in the range (1,11) without 8
            This method takes 2 parameters:
            group_list_file: either a file or a string for the tests to be ran
            timeout: integer value, given in seconds
                    the timeout parameter for methods that interact via adb with the DUT
            """
        window_sizes = ['ws_64_64', 'ws_113_47', 'fbo1', 'fbo2']
        result, output = Global.BLOCKED, "The run for group lists has not started, blocked at the %s sequence" % group_list_file
        if os.path.isfile(group_list_file):
            group_list_file2 = PathManager.absjoin(self._helper_scripts_gfx, group_list_file)

            read_groups = open(group_list_file2, 'rt')
            for group_file in read_groups:
                result, output = self._run_all_configs(window_sizes, group_file, timeout)
            read_groups.close()
        else:
            group_list_file = os.path.basename(group_list_file)
            result, output = self._run_all_configs(window_sizes, group_list_file, timeout)
        return result, output


    def _basic_run_method(self, cert_mode, group_list, timeout):
        """
            This method asserts which mode will be ran. The reason for it's existence is that, even though now there is only
            one method (full_run) that will be called in the UseCase, a second one is in plan that also takes into account
            the possible crashes encountered by the APK.
            Parameters:
            cert_mode: boolean
                       Whether the certification or partial(non-certification) mode will be used.
            group_list: string/file for the non-certificated method.
            timeout: integer value, given in seconds
                    the timeout parameter for methods that interact via adb with the DUT
            """
        result, output = Global.BLOCKED, "Execution not start based on cert mode"
        if cert_mode == 'True':
            result, output = self._run_certification_mode_full()
        elif cert_mode == 'False':
            result, output = self._run_non_certification_mode(group_list, timeout)

        return result, output


    def full_run(self, process_name, cert_mode, group_list, run_type, timeout):
        """
            The method at hand calls for the _basic_run_method described above. After the parameters are evaluated and the
            activity has started, it launches an ACS subprocess that checks whether the process is still alive. If the
            process is no longer alive, the method for pulling the reports is called.
            The final result and output are those of the pull_and_extract method.
            Parameters:
            process_name: string
                         the name of the process that is being watched.
            cert_mode: boolean
            group_list:
            tc_name:
            tc_order:
            timeout: integer value, given in seconds
                    the timeout parameter for methods that interact via adb with the DUT
            """
        mock_dir = "mock"
        full_mock_path = self._default_logging_dir + mock_dir
        adb_mkdir_mock = "adb shell mkdir %s" % full_mock_path
        status_mkdir, msg_mkdir = self._device.run_cmd(adb_mkdir_mock, 20)
        result, output = self._basic_run_method(cert_mode, group_list, timeout)
        alive_bool, alive_msg = self._check_process_alive(process_name, timeout)

        if run_type is not None:
            if run_type.lower() is 'dry':
                return alive_bool, alive_msg
            else:
                self._device.run_cmd("adb shell rm -rf %s" % self._default_logging_dir, 20)
                self._device.run_cmd("adb shell rm -rf %s" % full_mock_path, 20)
                status_mkdir, msg_mkdir = self._device.run_cmd(adb_mkdir_mock, 20)
                result, output = self._run_non_certification_mode(group_list, timeout)
                return result, output
        else:
            return alive_bool, alive_msg
