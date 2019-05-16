# coding: UTF-8
'''
Created on July 12, 2017

@author: Li Zixi
'''
import re
import subprocess

from testlib.util.log import Logger
logger = Logger.getlogger()

class MultiMediaLogcatHelper:
    def __init__(self, cmd):
        self.fdp = ""
        self.exec_cmd = cmd

    def __execute_command_with_popen(self, cmd, t_shell=False):
        logger.debug("cmd=%s" % cmd)
        if not t_shell:
            cmd = cmd.split()
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=t_shell)

    def clear_logcat_data(self):
        self.__execute_command_with_popen("adb logcat -c")

    def get_logcat_data_start(self, cmd=""):
        self.clear_logcat_data()
        if cmd == "":
            cmd = self.exec_cmd
        logger.debug("get_logcat_data_start cmd : %s" % cmd)
        self.fdp = self.__execute_command_with_popen(cmd)
        return self.fdp

    def get_logcat_data_end(self, search_string):
        logger.debug("get_logcat_data_end search_string=%s" % search_string)
        search_parttern = re.compile(search_string)
        self.fdp.kill()
        result_list = []
        for line in self.fdp.stdout.readlines():
            result = search_parttern.findall(line)
            for num in range(len(result)):
                result_list.append(result[num])
        logger.debug("get_logcat_data_end result_list=%s" % result_list)
        return result_list