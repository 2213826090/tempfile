#!/usr/bin/env python
'''
    Copyright 2012 Android Open Source Project

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
'''

import pickle
import os
import sys
import subprocess
import time

from Globals import *

class Common(object):
    def __init__(self):
        '''Constructor'''
        self.t_parm = None
        self.t_parm = os.environ['ATF_PARAMS']
        self.t_parm = pickle.loads(self.t_parm)

    #function that returns the value from specified argument of ATF_PARAMS
    def get_parameter(self, param_name, location='test_conf'):
        try:
            out = self.t_parm[location][param_name]
        except:
            raise Exception('Requested parameter was not found in %s' %location)
        return out

    def exec_command(self, args):
        output = subprocess.Popen(args, stdout = subprocess.PIPE, shell = True)
        stdout, stderr = output.communicate()
        return stdout

    # function that executes a command in shell
    def execute_command(self, cmd, times = 1):
        i = 1
        output = self.exec_command(cmd)
        while (output == None and i < times):
            i = i + 1
            time.sleep(SHELL_DELAY)
            output = self.exec_command(cmd)

        if (output == None and i == times):
            self.exit("Could not run command %s" %cmd)
        else:
            return output

    def debug(self, msg):
        print msg
        sys.stdout.flush()

    # exit function for single testcase
    def exit(self, msg, value = None, comment = None):
        if (msg == EXIT_SUCCESS):
            print '{{"STATUS":"PASSED","VALUE":"%s"' %(value or "") + \
                                           ',"COMMENT":"%s"}}'  %(comment or "")
        else:
            print '{{"STATUS":"FAILED","COMMENT":"%s"}}' %msg
            raise Exception('Test script ended in failure')

    # exit function for multiple testcases
    # dictionary format:
    # dict = {'testcase':{'status', 'value', 'comment', ...}}
    def exit_multiple(self, msg, tests = None):
        if (msg == EXIT_MULTIPLE):
            print_str = "{"
            for test_name,test_results in tests.items():
                print_str = print_str + '{"TESTCASE": "%s", "RESULT":{' %test_name

                for key, value in test_results.items():
                    print_str += '"%s": "%s",' %(key.upper(), value)
                print_str = print_str[0:-1] + "}},"
            print_str = print_str[0:-1] + "}"
            print print_str
        else:
            print '{{"STATUS":"FAILED","COMMENT":"%s"}}' %msg
            raise Exception('Test script ended in failure')