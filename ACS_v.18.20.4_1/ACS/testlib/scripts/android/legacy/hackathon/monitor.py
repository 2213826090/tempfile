#! /usr/bin/env python

from testlib.base.base_utils import get_time_string
from testlib.base.base_step import Resolution, FailedError
import os
from testlib.base import base_utils

class monitor():

    __metaclass__ = base_utils.SingletonType
    adb_connection = None
    current_apk = None
    last_pid = 1
    current_pids = []
    p_logcat = None
    step = 0
    
    logs_path = None

    def __init__(self, **kwargs):
        self.adb_connection = kwargs.pop('adb_connection')
        self.current_apk = kwargs.pop('apk')
        self.logs_path = kwargs.pop('results_path')
        self.last_pid = self.actualize_pid_list(initial = True)
        self.logcat_file = self.start_logcat_monitor()
        
        self.current_pids = []

    def __call__(self):
        self.actualize_pid_list()
        self.step += 1
        if self.inspect_logcat(self.logcat_file) is not Resolution.PASS:
            self.get_dmesg()
            print "[FAILED]".format(self.step)
        else:
            print "[PASSED]".format(self.step) 

    def __del__(self):
            print "Stopping monitor"
            self.get_dmesg()
            self.stop_logcat_monitor()
            self.get_dmesg()
            # self.p.terminate()

    def actualize_pid_list(self, initial = False):
        pids = self.adb_connection.pgrep()
        max_pid = 1
        for pid in pids:
            if int(pid) > self.last_pid:
                if not initial:
                    self.current_pids.append(int(pid))
                self.last_pid = int(pid)

    def start_logcat_monitor(self):
        print "Start monitor"
        self.adb_connection.run_cmd("logcat -c")
        log_file = os.path.join(self.logs_path, self.current_apk + '.logcat')
        self.p_logcat = self.adb_connection.run_cmd("logcat -v threadtime",
                        mode = 'async', soutfile = log_file)
        return log_file
        
    def stop_logcat_monitor(self):
        if self.p_logcat.poll() is None:
            self.p_logcat.terminate()
            try:
                self.p_logcat.kill()
            except: pass

    def get_dmesg(self):
        log_file = os.path.join(self.logs_path, self.current_apk + '.dmesg')
        self.p_logcat = self.adb_connection.run_cmd("dmesg",
                        mode = 'sync', soutfile = log_file)
        return log_file

    def inspect_logcat(self, logcat_file):
        with open(logcat_file, "r") as f:
            lines = f.readlines()
            for line in lines:
                for pid in self.current_pids:
                    #TODO - more strings
                    if "{0} E ".format(pid) in line:
                        #print "Error in logcat: \n"
                        #print line
                        return Resolution.FAIL
        return Resolution.PASS

