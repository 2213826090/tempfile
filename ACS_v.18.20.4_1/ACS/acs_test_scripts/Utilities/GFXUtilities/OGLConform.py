__author__ = 'mihaela'

import os
import pickle
import time
import subprocess
from GFXFunctions import *

from Common import *

SHELL_DELAY = 2
EXIT_SUCCESS = 0x0
from UtilitiesFWK.Utilities import Global
import Core.PathManager as PathManager
from Khronos2 import Khronos2 as KhronosObj
from acs_test_scripts.Utilities.GFXUtilities.GFXFunctions import *


class Auxiliary(object):
    """

    """

    def exit(self, msg, value=None, comment=None):
        if msg == EXIT_SUCCESS:
            result = '{{"STATUS":"PASSED","VALUE":"%s"' % (value or "") + \
                     ',"COMMENT":"%s"}}' % (comment or "")
        else:
            result = '{{"STATUS":"FAILED","COMMENT":"%s"}}' % msg
            failure = 'Test script ended in failure. ' + msg

        return result

    def exec_command(self, args):
        output = subprocess.Popen(args, stdout=subprocess.PIPE, shell=True)
        stdout, stderr = output.communicate()
        return stdout

    # function that executes a command in shell
    def execute_command(self, cmd, times=1):
        i = 1
        output = self.exec_command(cmd)
        while output is None and i < times:
            i += 1
            time.sleep(SHELL_DELAY)
            output = self.exec_command(cmd)
        if output is None and i == times:
            self.exit("Could not run command %s" % cmd)
            result, output = Global.FAILURE, "The command could not be executed"
        else:
            result, output_ret = Global.SUCCESS, output
        return result, output

    def send_cmd(self, param_cmd, param_timeout=0):
        """
        This function returns the stdout for adb shell commands and allows timeout
        """
        cmd_header = 'adb shell '
        if param_timeout != 0:
            timeout_string = 'timeout -k 1s -s 9 %s ' % str(param_timeout)
            cmd_header = timeout_string + cmd_header

        out_result, out_cmd = self.execute_command(cmd_header + param_cmd)
        return out_result, out_cmd


class OGLConform(object):
    gfx_scripts_path = 'OTC/TC/ACS/Graphics/Graphics/scripts'
    helper_scripts_gfx = PathManager.absjoin(KhronosObj.TEST_SUITES_PATH,
                                             gfx_scripts_path)

    def __init__(self, device):
        self._device = device
        self._dut_location = "data/app/"
        self._adb_bin = self._dut_location + 'oglconform '
        self._minFmt = '-v 4 -minFmt '

        self._aux = Auxiliary()

    def feature_supported(self, *param_element_list):
        """
        This function checks if the given feature is supported
        """

        supported_bool = True
        for element in param_element_list:
            if 'not' in element.lower():
                supported_bool = False
        return supported_bool

    def treat_diag(self, param_diag, feature_name):
        """
        This function treats the conditions containing the -diag option for validating a feature

        param_diag: shell -diag option to run and switch. -diag -es1 or -es2. Given by TCParameters
        feature_name: feature name to be tested. Given by TCParameters
        """
        diag_cmd = self._adb_bin + self._minFmt + param_diag + " | grep '" + feature_name + ":' -A 2"
        out_result, out_file = self._aux.send_cmd(diag_cmd, 60)
        diag_list = out_file.strip().split('\n')
        result = "SUPPORTED AND REPORTED"
        if not self.feature_supported(diag_list[1].strip(), diag_list[2]):
            self._aux.exit('feature is not shown as supported by oglconform -diag')
            result = "NOT SUPPORTED AND REPORTED"
        return result

    def treat_testcase(self, param_test):
        """
        This function treats the condition containing the -test option
        param_test: test parameter to be given for the -test option
        """
        result = ''
        list_outcomes = ['Total Passed', 'Total Failed', 'Total Not run']
        test_cmd = self._adb_bin + self._minFmt + param_test + ' | tail -n5'
        out_result, out_file = self._aux.send_cmd(test_cmd, 60)
        if not any(outcome in out_file for outcome in list_outcomes):
            loc_splitter = param_test.split('-test ')
            tc_name = max(loc_splitter, key=len)
            self._aux.exit(tc_name + ' failed because of subcase timeout ' + "\nERROR is: " + str(out_file))
        err_str = 'Error encountered'
        if err_str in out_file:
            if 'does not contain testcase/group' in err_str:
                self._aux.exit('Error encountered: %s . Skipping execution to next testcase.' % err_str)
            result = "BLOCKED DEPENDENCY"
        test_list = out_file.strip().split('\n')
        for l_line in test_list:
            if 'Total Failed :' in l_line:
                result = "PASSED"
                if not ('0' in l_line):
                    self._aux.exit('%s failed' % str(param_test))
                    result = "FAILED"
        return result

    def _populate_dict_from_file(self, tests_filepath):
        """
        This is a method that we use in the generate_groups method in order to move our tests in a dictionary
        """
        if not os.path.isfile(tests_filepath):
            # if not os.stat(tests_filepath).st_size > 0:
            return Global.FAILURE, "The path does not point to a file"
        else:
            with open(tests_filepath, 'r') as test_instance:
                test_dict = [line.strip() for line in test_instance if "egl-" in line]
            test_instance.close()
            return Global.SUCCESS, test_dict

    def generate_groups(self, output_filename="egl_lists.txt", pull_location=helper_scripts_gfx):
        """
        This method generates the groups that are to be ran for the EGL 1.4 API level validation
        The list is being generated in the '/scripts/'path and it contains all of the tests contained
         in suits for the binary.
         Of this list, a dictionary will be created containing only the egl lists.
         Parameters:
         output_filename: the name of the file that will contain all the tests.
                        Since this more of an auxiliary method, a mock name is already provided. If a user
                        wishes to make use of this method, the user can override the parameter when calling it.
         pull_location: the full path where the list file will be pulled.
                        Analogue to the previous parameter description, this too can be overriden by the user.
                        The default is in the '/scripts/' subfolder of the TestCases in ACS.
        """
        #first, we generate the lists
        generate_cmd = self._adb_bin + "-generateTestList " + self._dut_location + output_filename
        generate_result, generate_output = self._aux.send_cmd(generate_cmd, 60)
        #now, we pull the full lists on the host
        if generate_result == Global.FAILURE:
            return generate_result, generate_output
        else:
            filepath = PathManager.absjoin(pull_location, output_filename)
            pull_result, pull_output = self._device.pull(self._dut_location + output_filename, pull_location, 60)
            dict_result, test_dict = self._populate_dict_from_file(filepath)
            return dict_result, test_dict

    def execute_conformance(self, result_filename):
        """
        This method executes the EGL 1.4 conformance, based on all the egl suites generated on the spot
        with the generate_groups method.
        """
        groups_result, groups_output = self.generate_groups()
        for test in groups_output:
            if "es1" in test:
                test_string = "-es1 -test " + test
                return self.treat_testcase("-es1 -test " + test), test
            else:
                test_string = "-es2 -test " + test
            test_result = self.treat_testcase(test_string)
            write_report(test, test_result, result_filename)

