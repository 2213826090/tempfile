# coding: UTF-8
'''
Created on May 5, 2017

@author: Li Zixi
'''
import os
import time
import subprocess

from testlib.util.log import Logger
logger = Logger.getlogger()

class MultiMediaMplayerHelper:
    def __init__(self):
        self.fdp = ""
        self.play_cmd = "mplayer -slave -quiet %s"
        os.environ["DISPLAY"] = ":0.0"

        self.check_mplayer_package()

    def __execute_command_with_popen(self, cmd):
        logger.debug("cmd=%s" % cmd)
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)

    def check_mplayer_package(self):
        fdp = self.__execute_command_with_popen("dpkg -l | grep mplayer")
        assert "mplayer" in fdp.stdout.read(), "Can't find mplayer package in Host!"

    def play_video(self, file_path):
        assert self.fdp == "", "Record already start!"
        cmd = self.play_cmd % (file_path)
        logger.debug("play_video cmd : %s" % cmd)
        self.fdp = self.__execute_command_with_popen(cmd)
        return self.fdp

    def close_video(self):
        logger.debug("close_video")
        if self.fdp == "":
            return
        self.fdp.stdin.write("quit\n")
        self.fdp.kill()
        self.fdp = ""

    def control_video(self, command_list):
        ''' use cmd "mplayer -input cmdlist" to find support command list '''
        logger.debug("control_video command_list=%s" % command_list)
        for command in command_list:
            logger.debug("control_video command=%s" % command)
            if "sleep" in command:
                t_time = int(command.replace("sleep ", ""))
                time.sleep(t_time)
                continue
            t_str = command + "\n"
            self.fdp.stdin.write(t_str)
            time.sleep(2)