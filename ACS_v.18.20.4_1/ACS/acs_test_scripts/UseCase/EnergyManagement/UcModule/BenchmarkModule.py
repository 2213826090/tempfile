"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
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

:organization: INTEL MCG PSI
:summary: This module control benchmark execution by using droidbot
:author: vgomberx
:since: 18/03/2015
"""
import time
import calendar
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.OverMind import OverMind
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class BenchmarkModule():
    """
    init.
    """
    __LOG_TAG = "[BENCHMARK_MODULE]\t"
    ELE_MAIN_TEST = "MAIN_TEST"
    ELE_SCENARIO = "SCENARIO"
    ELE_CRASH = "CRASH"
    ELE_DELAY = "DELAY"
    ACTION_START = "START"
    ACTION_BEGIN = "BEGIN"
    ACTION_GET_SCORE = "SCORE"
    ACTION_STOP = "STOP"
    ACTION_EXIT = "EXIT"
    FILE_EXEC_LOG = "/sdcard/current_exec.log"
    DROID_BOT_OUTPUT_LOG = "/sdcard/emdrobot_output.log"
    CST_ENTRY_SEP = "[NEW_ENTRY]"
    CST_ELE_SEP = "[ELE_SEP]"
    CST_SCORE_SEP = "[ELE_SCORE]"
    PARAM_STARTDELAY = "startDelay"
    PARAM_MINWAITIME = "minWaitTime"
    PARAM_MAXWAITIME = "maxWaitTime"
    BENCHMARK_START_TAG = "BENCHMARK_START"

    DDATE = "DATE"
    DELE = "ELEMENT"
    DVALUE = "VALUE"
    DEXTRA = "EXTRA"
    DITER = "ITERATION"

    def __init__(self):
        """
        parameter to initialize this module.
        currently consider that you cant have video and music playing at the same time
        that why testcase parameters for both are the same.
        """
        # DECLARE OPTIONAL GLOBAL PARAMETER HERE
        overmind = OverMind()
        self.__logger = LOGGER_TEST_SCRIPT
        self.__logger.info(self.__LOG_TAG + "INIT")
        self.__device = overmind.get_instance(overmind.DEVICE)
        bench_config = overmind.get_instance(overmind.BENCH_CONFIG)
        tc_parameters = overmind.get_instance(overmind.TC_PARAMETERS)

        #-----------------------------------------------------------------------
        self.__bk_test_name = tc_parameters.get_param_value("BENCHMARK_TEST_NAME", default_cast_type=str)
        # you have the choice to base the test on a number of iteration
        self.__bk_run_number = tc_parameters.get_param_value("BENCHMARK_NB_OF_RUN", 1, default_cast_type=int)
        self.__bk_duration_for_one_exec = tc_parameters.get_param_value("BENCHMARK_DURATION_OF_ONE_EXEC", default_cast_type=int)
        # or a duration
        bk_duration = tc_parameters.get_param_value("BENCHMARK_DURATION", None, default_cast_type=int)

        if bk_duration in [None, ""]:
            self.__bk_iteration_mode = True
            self.__bk_whole_duration = self.__bk_duration_for_one_exec * self.__bk_run_number
        else:
            self.__bk_iteration_mode = False
            self.__bk_run_number = 100
            self.__bk_whole_duration = bk_duration
        # custom option
        self.__bk_min_wait_time = tc_parameters.get_param_value("BENCHMARK_MIN_WAIT_TIME", default_cast_type=int)
        self.__bk_max_wait_time = tc_parameters.get_param_value("BENCHMARK_MAX_WAIT_TIME", default_cast_type=int)
        self.__bk_custom_op = tc_parameters.get_param_value("BENCHMARK_CUSTOM_OPTION", "")

        if bench_config.has_parameter("DROIDBOT"):
            droid_bot_conf = bench_config.get_parameters("DROIDBOT")
            self.__installation_folder = droid_bot_conf.get_param_value("InstallationFolder", "/data/local/tmp/com.intel.droidbot/")
        else:
            self.__installation_folder = "/data/local/tmp/com.intel.droidbot/"

        self.__core_lib_path = self.__installation_folder + "droidbot-emcorelib.jar"
        self.__pnp_test_path = self.__installation_folder + "droidbot-emtests.jar"
        self.__file_api = self.__device.get_uecmd("File", True)

    def setup(self):
        """
        Check that all the mandatory jar are installed
        if it is not the case , raise an error
        """
        txt = ""

        f_list = [self.__core_lib_path, self.__pnp_test_path ]
        for f in f_list:
            if not self.__file_api.exist(f)[0]:
                txt += "mandatory file %s does not exist! \n" % f

        if txt != "":
            self.__logger.error(txt)
            raise DeviceException(DeviceException.FILE_SYSTEM_ERROR, txt)

        if self.__bk_run_number < 1:
            txt = "BENCHMARK_NB_OF_RUN cannot be < 1"
            self.__logger.error(txt)
            raise DeviceException(AcsBaseException.INVALID_PARAMETER, txt)

        # stop any previous run
        self.clean()

    def execute_test(self, background=False, delay=0, max_wait_time=None, min_wait_time=None):
        """
        run the given benchmark test.
        For em domain , we use benchmark as load and dont compute their results

        :type background: boolean
        :param background: True if you want to run the benchmark as a background task
        """
        if background:
            cmd = "adb shell nohup"
        else:
            cmd = "adb shell"

        cmd += " uiautomator runtest {0} {1} -s -c {2} > {3}".format(self.__core_lib_path,
                                                                self.__pnp_test_path,
                                                                self.__bk_test_name, self.DROID_BOT_OUTPUT_LOG)

        cmd += " -e numRun {0}".format(self.__bk_run_number)
        if delay > 0:
            self.__logger.info(self.__LOG_TAG + "benchmark execution will start after {0}s".format(delay))
            cmd += " -e startDelay {0}".format(delay)

        # load value from testcase
        if self.__bk_max_wait_time is not None and max_wait_time is None:
            max_wait_time = self.__bk_max_wait_time
        if max_wait_time > 0:
            cmd += " -e maxWaitTime {0}".format(max_wait_time)

        # load value from testcase
        if self.__bk_min_wait_time is not None and min_wait_time is None:
            min_wait_time = self.__bk_min_wait_time
        if min_wait_time > 0:
            cmd += " -e minWaitTime {0}".format(min_wait_time)

        # add custom option depending of the benchmark params like passing an url to browsing benchmark
        if not self.__bk_custom_op is None and str(self.__bk_custom_op).strip() != "":
            cmd += " " + self.__bk_custom_op.strip()

        self.__device.run_cmd(cmd, 60, force_execution=True, wait_for_response=False)

    def get_output_log(self):
        """
        Return the log generated by the embedded agent.

        :rtype: str
        :return: the log generated as string
        """
        output = "file %s was not generated" % self.DROID_BOT_OUTPUT_LOG
        if self.__file_api.exist(self.DROID_BOT_OUTPUT_LOG)[0]:
            cmd = 'adb shell cat %s' % self.DROID_BOT_OUTPUT_LOG
            _, output = self.__device.run_cmd(cmd, 60, force_execution=True)
        return output

    def is_running(self):
        """
        return True if benchmark execution is on going
        """
        result = False
        output = self.read_exec_log()


        if  len(output) > 0 :
            last_ele = output[-1].get(self.DELE)
            last_value = output[-1].get(self.DVALUE)
            if (last_ele in [self.ELE_DELAY,
                            self.ELE_MAIN_TEST,
                            self.ELE_SCENARIO] and (last_value in [self.ACTION_START,
                                                                self.ACTION_BEGIN,
                                                                self.ACTION_STOP] or str(last_value).isdigit())):
                result = True

        return result

    def is_finished(self):
        """
        return True if benchmark execution is finished or not ongoing (never launch)
        """
        result = False
        output = self.read_exec_log()
        if (len(output) == 0) or (output[-1].get(self.DELE) == self.ELE_MAIN_TEST and
                                   output[-1].get(self.DVALUE) == self.ACTION_EXIT):
            result = True

        return result

    def stop_execution(self):
        """
        stop any benchmark execution and return to main
        """
        self.__logger.info(self.__LOG_TAG + "stop benchmark execution")
        cmds = ["adb shell pkill uiautomator", "adb shell am start -c android.intent.category.HOME -a android.intent.action.MAIN", "adb shell am kill-all"]
        for cmd in cmds:
            self.__device.run_cmd(cmd, 60, force_execution=True)
            time.sleep(1)

    def clean(self):
        """
        stop every thing and clean generated file
        """
        self.__logger.info(self.__LOG_TAG + "clean")
        self.stop_execution()
        file_to_del = [self.FILE_EXEC_LOG, self.DROID_BOT_OUTPUT_LOG]

        for f in file_to_del:
            cmd = 'adb shell rm -f %s' % f
            self.__device.run_cmd(cmd, 60, force_execution=True)

    def read_exec_log(self):
        """
        read current exec log
        """
        result = []
        if self.__file_api.exist(self.FILE_EXEC_LOG)[0]:
            cmd = 'adb shell cat %s' % self.FILE_EXEC_LOG
            _, output = self.__device.run_cmd(cmd, 60, force_execution=True)
            result = self.__parse_exec_log(output)

        return result

    def get_score(self):
        """
        get the scores from benchmark and return it as a list of dictionary with following info:
        score name
        score value
        iteration at which the score was done
        date of the score

        all key will be cast in upper cap

        :rtype: list of dict
        :return:  return a list of dictionary
        """
        result = []
        self.__logger.info(self.__LOG_TAG + "get score info")
        output = self.read_exec_log()
        for line in output:
            dico = {}
            # get only score line
            if line.get(self.DVALUE) == self.ACTION_GET_SCORE:
                score_info = line.get(self.DEXTRA)
                # if there is score element get them
                if score_info is not None:
                    score_info = filter(None, score_info.split(self.CST_SCORE_SEP))
                    if len(score_info) > 0:
                        # parse every score element and make a dictionary
                        for element in score_info:
                            element = element.split("=", 1)
                            if element > 1:
                                dico[element[0].strip().upper()] = element[1].strip()

                        # add some extra info and store the data
                        if len(dico) > 0:
                            dico[self.DITER] = line.get(self.DITER)
                            dico[self.DDATE] = line.get(self.DDATE)
                            result.append(dico)

        return result

    def get_crash(self):
        """
        get the crash info
        there should be only 0 or 1 crash in the file
        """
        self.__logger.info(self.__LOG_TAG + "get crash info")
        result = self.read_exec_log()
        crash_info = ""
        crash_date = None
        for line in result:
            if line.get(self.DELE) == self.ELE_CRASH:
                crash_info += line.get(self.DVALUE) + ".\n"
                if crash_date is None and  line.get(self.DDATE, None) is not None:
                    crash_date = self.__text_to_time(line.get(self.DDATE))

        return crash_date, crash_info

    def get_start_date(self):
        """
        get the exact date at which the benchmark was started
        if a delay is in progress then we wont be able to return the date.
        """
        self.__logger.info(self.__LOG_TAG + "get start date")
        result = self.read_exec_log()
        start_date = None
        for line in result:
            # we want the date of the first benchmark start
            if  line.get(self.DVALUE) == self.ACTION_START:
                start_date = self.__text_to_time(line.get(self.DDATE))
                break

        return start_date

    def check_crash(self):
        """
        check if a crash has been raised and raise an error if it is the case
        """
        self.__logger.info(self.__LOG_TAG + "check if a crash has been raised")
        result = self.read_exec_log()
        crash_info = ""
        for line in result:
            if line.get(self.DELE) == self.ELE_CRASH:
                crash_info += line.get(self.DVALUE) + ".\n"

        if crash_info != "":
            crash_info = " CRASH Happen during benchmark schedule execution : " + crash_info
            self.__logger.error(crash_info)
            raise DeviceException(DeviceException.OPERATION_FAILED, crash_info)

    def get_last_complete_iter(self):
        """
        Return the last completed iteration number and date,
        return 0 if no benchmark iteration is on going or not started
        a complete iteration is one that have reach stop state
        """
        self.__logger.info(self.__LOG_TAG + "get last completed iteration info")
        iteration = 0
        iter_date = 0
        output = self.read_exec_log()
        for line in reversed(output):
            if (line.get(self.DELE) == self.ELE_SCENARIO) and (line.get(self.DVALUE) == self.ACTION_STOP):
                # cast to int to force a python crash in case of non int
                iteration = int(line.get(self.DITER))
                iter_date = self.__text_to_time(line.get(self.DDATE))
                break

        return iteration, iter_date

    def get_expected_iter(self):
        """
        get iteration from setup
        """
        return self.__bk_run_number

    def get_complete_iter_duration(self):
        """
        get the duration from the first start iteration to the last completed
        this value can be returned only if at least 1 iteration has been finished
        it cannot return a duration for an ongoing
        """
        self.__logger.info(self.__LOG_TAG + "get the duration from the start to the last iteration")
        duration = 0
        end_time = 0
        output = self.read_exec_log()
        for line in reversed(output):
            if (line.get(self.DELE) == self.ELE_SCENARIO) and (line.get(self.DVALUE) == self.ACTION_STOP):
                # cast to int to force a python crash in case of non int
                end_time = self.__text_to_time(line.get(self.DDATE))
                break

        if end_time > 0:
            for line in output:
                if (line.get(self.DELE) == self.ELE_SCENARIO) and (line.get(self.DVALUE) == self.ACTION_START):
                    # cast to int to force a python crash in case of non int
                    duration = end_time - self.__text_to_time(line.get(self.DDATE))
                    break

        return duration

    def get_exec_whole_duration(self):
        """
        get the whole execution duration
        """
        return self.__bk_whole_duration

    def __text_to_time(self, text):
        """
        convert the date text to gmt time
        """
        # ex : 2015-03-31_13h31.51
        from_year_to_sec = text
        time_structure = time.strptime(from_year_to_sec, "%Y-%m-%d_%Hh%M.%S")
        return float(calendar.timegm(time_structure))

    def __parse_exec_log(self, text):
        """
        compute exec log to appear as a list
        """
        # split by new entries
        result = []
        entries = filter(None, text.split(self.CST_ENTRY_SEP))

        for entry in entries:
            dico_ele = {}
            info = entry.split(self.CST_ELE_SEP)
            info = map(str.strip, info)

            # first element is always the date, the second is the element name and the last the action
            if len(info) > 2:
                dico_ele[self.DDATE] = info[0]

                element = info[1]
                dico_ele[self.DELE] = info[1]
                # [NEW_ENTRY]2015-03-31_12h48.51[ELE_SEP]SCENARIO[ELE_SEP]1[ELE_SEP]START
                if len(info) > 3 and element == self.ELE_SCENARIO:
                    dico_ele[self.DITER] = info[2]
                    dico_ele[self.DVALUE] = info[3]
                    # [NEW_ENTRY]2015-11-03_10h28.48[ELE_SEP]SCENARIO[ELE_SEP]1[ELE_SEP]SCORE[ELE_SEP]Fps=33.0[ELE_SCORE]dps=100
                    if info[3] == self.ACTION_GET_SCORE:
                        dico_ele[self.DEXTRA] = info[4]
                else:
                    dico_ele[self.DVALUE] = info[2]

                result.append(dico_ele)
        return result

    def is_based_on_iteration(self):
        """
        return true if iteration mode is base on a number of run
        otherwise return False if it is based on a duration
        """
        return self.__bk_iteration_mode
