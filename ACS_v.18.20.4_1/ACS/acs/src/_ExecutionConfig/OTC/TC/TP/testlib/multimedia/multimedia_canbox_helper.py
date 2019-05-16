# coding: UTF-8
'''
Created on Aug 5, 2017

@author: Li Zixi
'''
import time
import subprocess
import threading

from testlib.util.log import Logger
logger = Logger.getlogger()

class MultiMediaCanboxHelper:
    def __init__(self):
        self.fdp = ""
        self.t_candump_result = ""
        self.candump_cmd = "adb shell candump slcan0"
        self.cansend_cmd = "adb shell cansend slcan0 %s"

    def __execute_command_with_popen(self, cmd, t_shell=False):
        logger.debug("__execute_command_with_popen cmd=%s" % cmd)
        if not t_shell:
            cmd = cmd.split()
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=t_shell)

    def get_candump_result(self):
        self.t_candump_result = ""
        while not self.fdp.poll():
            t_str = self.fdp.stdout.readline()
            self.t_candump_result += t_str
        self.fdp = ""

    def candump_start(self):
        logger.debug("candump_start")
        assert self.fdp == "", "candump already start!"
        self.fdp = self.__execute_command_with_popen(self.candump_cmd, True)
        thread_1 = threading.Thread(target=self.get_candump_result, args=())
        thread_1.start()
        return self.t_candump_result

    def candump_end(self):
        logger.debug("candump_end")
        assert self.fdp != "", "candump not start!"
        self.fdp.terminate()
        wait_time = 10
        for i in range(wait_time):
            if self.fdp == "":
                break
            time.sleep(1)
        assert i < wait_time - 1, "read candump result timeout!"
        return self.t_candump_result

    def cansend(self, t_param):
        cmd = self.cansend_cmd % t_param
        self.fdp = self.__execute_command_with_popen(cmd, True)
        logger.debug("cansend msg=%s" % self.fdp.stdout.read())