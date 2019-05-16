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

:since: 9/12/14
:author: mihaela
"""

from UtilitiesFWK.Utilities import Global
import os
from Device.DeviceLogger.LogCatLogger.LogCatReaderThread import LogCatReaderThread
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.AcsSubprocess.AcsSubprocess import AcsSubprocess
from acs_test_scripts.Utilities.GFXUtilities.opengles_log_parser import BatchResultParser
from acs_test_scripts.Utilities.GFXUtilities.opengles_log_parser import StatusCode
import csv
import time
from Core.PathManager import Paths
import Core.PathManager as PathManager

class KhronosApk(object):
    def __init__(self, tc_name, attr_override, tc_order, device, trigger_message, logcat_cmd_line, process_name, secondary_test_report,
                 helper_scripts_path, device_logdir, activity_name, report_path, cert_mode):
        """
        Constructor parameters
        """
        # BaseConf.__init__(self)
        # TestStepEngine.__init__(self, tc_conf, global_config)
        self._device = device
        self._logger = LOGGER_TEST_SCRIPT
        self._trigger_message = trigger_message
        self._logcat_cmd_line = logcat_cmd_line
        self._process_name = process_name

        self._tc_report = secondary_test_report

        self._logcat_analyzer = LogCatReaderThread(self._device, logger=self._logger, enable_writer=False,
                                                   logcat_cmd_line=logcat_cmd_line)

        self._default_logging_dir = '/sdcard/'  #this is the log directory that the APK has as default and cannot be overwritten in Native mode
        self._helper_scripts_gfx = PathManager.absjoin(Paths.TEST_SUITES,
                                                       helper_scripts_path)  #this path should be paken out of the Paths...
        self._device_logdir = device_logdir
        self._activity_name = activity_name
        self._report_path = report_path
        self._cert_mode = cert_mode

        self._tc_name = tc_name
        self._tc_order = tc_order

    def _execute_command(self, cmd):
        """
        This private method offers a way of executing a command during the ACS run by calling the AcsSubprocess method of execute_sync(timeout)
        We use this in order to watch the result execution in a separate synchronous subprocess
        """
        my_process, stdout = AcsSubprocess(cmd, LOGGER_TEST_SCRIPT, max_empty_log_time=1, silent_mode=False).execute_sync(2)
        return my_process, stdout


    def _dismiss_notification(self, timeout):
        """
        This method is called in the moment that there is a hanging notification left on the screen
        and we want it removed.
        What it does as of right now is to send a keyevent for pressing "Enter"
        """
        press_enter = "adb shell input keyevent 66"
        result, output = self._device.run_cmd(press_enter, timeout)
        return result, output

    def _check_process_alive(self, process_name, timeout):
        """
        A method that greps for our activity to check if it is still alive. In addition, it also searches for the
        SIGSEGV message in the logcat, in order to identify a process that has been terminated before it's normal
        finish
        """
        start = 0
        cmd_header = "adb shell ps | grep "
        #there are a couple of lines here, the self_logger() ones, that have the purpose of
        # debugging in the development stage
        self._logger.debug("Pornim activitate Logcat Analyzer")
        self._logger.debug("Am trimis mesajul SIGSEGV teoretic")
        self._logcat_analyzer.add_trigger_message(self._trigger_message)
        self._logger.debug(self._logcat_analyzer.add_trigger_message(self._trigger_message))
        self._logcat_analyzer.start()

        result, msg = Global.FAILURE, "The process %s has died" % process_name
        send_cmd = cmd_header + process_name
        while start < timeout:
            my_proc, stdout = self._execute_command(send_cmd)
            if 'khronos' not in stdout:
                result, msg = Global.FAILURE, "The process %s has died" % process_name

                self._logger.debug(self._logcat_analyzer.is_message_received(self._trigger_message, 2))
                #this is where we remove the popup from the screen - MUSAI AICI??
                # self._destroy_notification()
                return result, msg
            else:
                result, msg = Global.SUCCESS, "The %s process is alive" % process_name
                start += 0.1
        return result, msg


    #taken from gl_cts script and adapted for now:


    def _pull_results(self, remote_path, local_path, special_string, timeout):
        """
        Pull the results, whatever the metohds generate and need
        Extending(not in a pretty way) _device.pull()
        """
        if special_string != '':
            full_remote_path = remote_path + "/" + special_string
            mock_dir = "mock"
            full_mock_path = remote_path + "/" + mock_dir
            adb_mkdir_mock = "adb shell mkdir %s/%s" % (remote_path, mock_dir)
            print "\n\n am facut fisierul %s/%s" % (remote_path, mock_dir)
            status_mkdir, msg_mkdir = self._device.run_cmd(adb_mkdir_mock, timeout)
            adb_cmd_mv = "adb shell mv %s %s" % (full_remote_path, full_mock_path)
            print "\n\n am mutat asta: %s TO %s " % (full_remote_path, full_mock_path)
            status_mv, msg_mv = self._device.run_cmd(adb_cmd_mv, timeout)
            status, err_msg = self._device.pull(full_mock_path, local_path, timeout)
        elif special_string == '':
            status, err_msg = self._device.pull(remote_path, local_path, timeout)
        return status, err_msg


    def _generate_groups(self):
        """
        A simple method for generating the test groups
        """
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

                # for root, dirs, files in os.walk(r'%s' % self._local_pull_path):
            else:
                print "\n\n sunt in ELSE pt parcurs fisierele"
                for root, dirs, files in os.walk(r'%s/' % self._helper_scripts_gfx):
                    for file in files:
                        for gf in group_files:
                            print "\n\n parcurg fisierele minunate"
                            tf_d = open(self._helper_scripts_gfx + "/" + file, 'r')
                            gf_d = open(self._helper_scripts_gfx + "/" + gf, 'w')
                            group_run = None
                            for line in tf_d:
                                if "GROUP" in line:
                                    group_run = line.split(" ")[1]
                                    #print group_run

                                if ("TEST" in line) and group_run:
                                    gf_d.write(group_run.strip('\n') + '.*' + '\n')
                                    group_run = None
                            tf_d.close()
                            gf_d.close()
                            # result, tests, groups = Global.SUCCESS, tf_d, gf_d
                            result, output = Global.SUCCESS, "Files were generated in the %s folder" \
                                                             % self._helper_scripts_gfx
        # return tf_d, gf_d
        return result, output

    def _run_certification_mode_full(self):
        """
        Method for running the ES31Activity
        cam hardcodata partea cu activity name, merge momentan
        """

        # result, output = Global.FAILURE, "The ES31Activity was not started"
        #TODO: temp mkdir, gotta fix this
        # activity_name = self._activity_name
        self._device.run_cmd("adb shell mkdir /sdcard/%s" % self._device_logdir, timeout=200)
        adb_run_all = "adb shell am start -n org.khronos.gl_cts/org.khronos.cts.%sActivity -e logdir '%s%s'" % (
            self._activity_name, self._default_logging_dir, self._device_logdir)
        result, output = self._device.run_cmd(adb_run_all, 1500)

        return result, output

    def _single_run(self, group_name, window_size, configID, timeout=300):
        """
        This method runs a single test using the NativeActivity from the APK
        """
        print "\n\n parametrul pe care il trimit ::%s==" % group_name
        # dictionary to keep formated options for different windows sizes/ fbo
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
        print "\n\n\n\n\n ce afiseaza group_name[0:-2] :::%s==" % group_name[0:-2]
        log_name = "%s%s/%s_%s_cfgID_%s.qpa" % (
            self._default_logging_dir, self._device_logdir, group_name[0:-2], window_size, configID)
        # group to test and log file part of the command
        cmd_line_arg = "--deqp-case=%s --deqp-log-filename=%s " % (group_name, log_name)
        cmd_ID_arg = "--deqp-gl-config-id=%s " % str(configID)
        cmd_rot_arg = "--deqp-screen-rotation=90\""

        adb_single_run = adb_am_start + win_cmd[window_size] + cmd_line_arg + cmd_ID_arg + cmd_rot_arg
        run_result, run_output = self._device.run_cmd(adb_single_run, timeout)

        return run_result, run_output


    def _run_non_certification_mode(self, group_list_file, timeout):
        """
        Method for running a list of tests from a group on all config IDs and all Window Sizes
        ConfigID is in the range (1,11) without 8
        """
        print "I am suppose to treat al the other situations in which the cert_modes fail miserably cuz of the APK"
        window_sizes = ['ws_64_64', 'ws_113_47', 'fbo1', 'fbo2']
        group_list_file2 = PathManager.absjoin(self._helper_scripts_gfx, group_list_file)
        read_groups = open(group_list_file2, 'rt')
        print "==============="
        print "Pathul fisierului din care execut grupuri de teste: %s " % read_groups
        result, output = Global.BLOCKED, "The run for group lists has not started, blocked at the %s sequence" % group_list_file
        #walk through all the tests in a groups' file
        for group_list in read_groups:
            print "\n\n Grupul meu: ++%s~~\n" % group_list
            #~ # run all groups  ws = 64 x 64 config-id 1
            result, output = self._single_run(group_list, window_sizes[0], 1, timeout)
            # run all CTS.gtf.*  ws = 113 x 47 config-id 1
            if "CTS.gtf" in group_list:
                result, output = self._single_run(group_list, window_sizes[1], 1, timeout)
            # run CTS.gtf.* fbo1
            elif "CTS.gtf" in group_list:
                result, output = self._single_run(group_list, window_sizes[2], 1, timeout)
            # run CTS.gtf.* fbo2
            elif "CTS.gtf" in group_list:
                result, output = self._single_run(group_list, window_sizes[3], 1, timeout)
            # run ws = 64 x 64 all configIDs 2-7, 9-11
            elif "CTS.gtf" in group_list or "ES31" in group_list:
                for cfgID in [2, 3, 4, 5, 6, 7, 9, 10, 11]:
                    result, output = self._single_run(group_list, window_sizes[1], cfgID, timeout)
            # run ws = 113 x 47 all configIDs 2-7, 9-11
            elif "CTS.gtf" in group_list:
                for cfgID in [2, 3, 4, 5, 6, 7, 9, 10, 11]:
                    result, output = self._single_run(group_list, "ws_113_47", cfgID, timeout)

        read_groups.close()

        return result, output


    def _basic_run_method(self, group_list, timeout):
        """
        """
        result, output = Global.BLOCKED, "Did not yet start the run"
        if self._cert_mode == "True":
            result, output = self._run_certification_mode_full()
            # alive_bool, alive_msg = self._check_process_alive(process_name, timeout)
        elif self._cert_mode == "False":
            result, output = self._run_non_certification_mode(group_list, timeout)

        return result, output


    def _res_to_csv(self, result_filename):
        """
        Taken from the sources sent by the Graphics devs
        This method will read the QPA results and write them into a CSV file
        """
        parser = BatchResultParser()
        file_to_parse = PathManager.absjoin(self._report_path, result_filename)
        results = parser.parseFile(file_to_parse)
        write_in_file = file_to_parse + '.csv'
        result_file_csv = open(write_in_file, 'wb')
        result_writer = csv.writer(result_file_csv, delimiter=',', quoting=csv.QUOTE_NONE)
        for result in results:
            print '\n\n Asta are trebui sa scriu in csv:'
            print result.name, result.statusCode
            result_writer.writerow([result.name, result.statusCode])
        result_file_csv.close()
        # print "\n\nIn _res_to_csv"
        # print write_in_file
        # print os.path.dirname(write_in_file.name)
        return write_in_file


    def extract_results(self, result_filename_val):
        """
        #TODO: this method should really be taken out of a usecase, in an utilities file...
        compute results from gfx csv file test results
        - add each test result to a secondary report

        :param result_filename: gfx log output and it's location
        :type file_path: str

        :rtype: str
        :return: output message
        """
        print "==============="
        print "Ce prinde get_name() si dupa tc_order:"
        print str(self._tc_name)
        print self._tc_order
        # print self._tc_name.get_name()
        print "==============="

        test_executed = 0
        result = Global.FAILURE
        output = ""
        result_filename = PathManager.absjoin(self._report_path, result_filename_val)
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
                        if StatusCode.PASS in row[1]:
                            test_executed += 1
                            self._tc_report.add_result(row[0], self._tc_report.verdict.PASS, "Test PASS",
                                                       self._tc_name, self._tc_order)
                        elif StatusCode.NOT_SUPPORTED in row[1]:
                            # TEST SHOULD NOT BE RUN FOR THIS PLATFORM, so pass to another one
                            continue
                        else:
                            result = Global.FAILURE
                            output = "Some tests are FAIL"
                            test_executed += 1
                            self._tc_report.add_result(row[0], self._tc_report.verdict.FAIL,
                                                       "Test status is %s, see log output for more details" % row[1],
                                                       self._tc_name, self._tc_order)
                    else:
                        result = Global.FAILURE
                        output = "CSV seems corrupted, there are less than 2 rows!"

                if result != Global.FAILURE:
                    output = "All tests are PASS"
                if not test_executed:
                    result = Global.FAILURE
                    output = "No tests have been executed"
        return result, output

    def pull_and_extract(self):
        pull_path = self._default_logging_dir + self._device_logdir
        pull_res, pull_out = self._pull_results(pull_path, self._report_path, special_string='*.qpa', timeout=500)

        for root, dirs, files in os.walk(r'%s' % self._report_path):
            for file in files:
                if file.endswith('.qpa'):
                    preliminary_file = self._res_to_csv(file)
                    print "\n\n ******** \n Procesez ACUM fisierul %s \n ******\n" % file
                    result, output = self.extract_results(preliminary_file)
                else:
                    pass
        return result, output


    def dry_run(self, process_name, group_list, timeout):
        """
        """
        result, output = self._basic_run_method(group_list, timeout)
        time.sleep(5)
        alive_bool, alive_msg = self._check_process_alive(process_name, timeout)
        if alive_bool == Global.FAILURE:
            # return
            result, output = Global.FAILURE, "The application has crashed and the run has been marked as a FAILURE"
        self.pull_and_extract()
        return result, output


    def full_run(self, process_name, group_list, timeout):
        """
        """
        result, output = self._basic_run_method(group_list, timeout)
        alive_bool, alive_msg = self._check_process_alive(process_name, timeout)
        while alive_bool == Global.SUCCESS:
            # self._pull_and_extract()
            pass
            if alive_bool == Global.FAILURE:
                print "\n++++ \n Am intrat in bucla aici \n ***********\n"
                notif_res, notif_out = self._dismiss_notification(timeout)
                #TODO: check if the pop-up has been dismissed. Petty at this state of the code
                # self._pull_and_extract() <---------------- asta aici e proaspat comentat, suspectat de bulca infinita
                # pull_path = self._default_logging_dir + self._device_logdir
                # pull_res, pull_out = self._pull_results(pull_path, self._report_path, special_string='*.qpa', timeout=500)
                # for root, dirs, files in os.walk(r'%s' % self._report_path):
                #     for file in files:
                #         if "qpa" in file:
                #             preliminary_file = self._res_to_csv(file)
                #             result, output = self.extract_results(preliminary_file)
                #         else:
                #             pass
                #return result, output
                # self._full_run(process_name, group_list, timeout=timeout)
                continue
        result, output = self.pull_and_extract()
        return result, output
