#! /usr/bin/env python

from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args, TimeoutError
from testlib.base.base_step import Resolution
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.android_utils import get_app_label_from_apk, get_package_name_from_apk
from testlib.base.base_utils import get_time_string

import os
import sys
import subprocess
import multiprocessing
import time

# from settings import tests
import atexit

from monitor import monitor


APKS_PATH = os.path.join('/home/oane/Work/automation/testlib/resources', 'apks')
RESULTS_PATH = os.path.join('/home/oane/Work/automation/testlib/', 'results')
apks = ["com.halfbrick.fruitninjafree_109005.apk", 'com.google.android.apps.translate_30000060.apk']

DEFAULT_TIMEOUT = 300

# each test is executed in a separate process p
p = None
# but linux command is executed using subprocess subp
subp = None
log_out_err_file = None

def on_exit():
        try:
            p.terminate()
            subp.terminate()
            log_out_err_file.close()
        except: pass

class log_stdout_stderr(object):
    '''
    Helper class to log STDOUT and STDERR
    to both STDOUT and to a file
    '''
    def __init__(self, file_name):
        self.file = open(file_name, 'w')
        self.stdout = sys.stdout
        self.stderr = sys.stderr
        sys.stdout = self
        sys.stderr = self

    def close(self):
        if self.stdout:
            sys.stdout = self.stdout
            self.stdout = None
        if self.stderr:
            sys.stderr = self.stderr
            self.stderr = None
        if self.file:
            self.file.close()
            self.file = None

    def write(self, data):
        self.file.write(data)
        self.stdout.write(data)

    def flush(self):
        self.file.flush()
        self.stdout.flush()

    def __del__(self):
        self.close()


atexit.register(on_exit)

def add_argument(command, test, arg):
    if test.has_key(arg) and test[arg]:
        return command + " --{0} {1}".format(arg, test[arg])

def get_output(subp):
    # print the output
    while subp.poll() is None:
        out, err = subp.stdout.read(), subp.stderr.read()
        if out:
            print "STDOUT      :", out
        if err :
            print "STDERR      :", err


def run_test(apk, serial, port, log_dir, log_file):

    # we need STDOUT ant STDERR to both STDOUT and to a file
    log_out_err_file = log_stdout_stderr(log_file)

    adb_connection = connection_adb(serial = serial, port = port)

    # start monitoring
    mon = monitor(adb_connection = adb_connection, apk = apk, results_path = log_dir)

    time.sleep(2)

    # install apk
    apk_path = os.path.join(APKS_PATH, apk)
    adb_steps.install_apk(apk_path = apk_path, install_time = 120)()

    # monitor
    mon()

    print "TC-1"
    print "Start App - Home Screen - App"
    apk_path = os.path.join(APKS_PATH, apk)
    apk_label = get_app_label_from_apk(apk_path)
    apk_package = get_package_name_from_apk(apk_path)
    print apk_path
    print apk_label
    print apk_package

    command = "python homescreen_and_back.py --serial {0} --script-args app-to-test=\'{1}\' wait-time=2000 host-path={2}".format(serial, apk_label, log_dir)
    print command
    subp = subprocess.Popen(command.split(), stdout = subprocess.PIPE, stderr = subprocess.PIPE)
########### T OD SPLIT ###################
    get_output(subp)
    # monitor
    mon()
    time.sleep(2)

    print "TC-2"
    print "Start App - Start Another App - App"
    apk_path = os.path.join(APKS_PATH, apk)
    apk_label = get_app_label_from_apk(apk_path)
    command = "python app_and_back.py --serial {0} --script-args app-to-test={1} wait-time=2000 app-to-open=Calculator host-path={2}".format(serial, apk_label, log_dir)
    subp = subprocess.Popen(command.split(), stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    get_output(subp)
    # monitor
    mon()
    time.sleep(2)

    print "TC-3"
    print "Start App - Rotate"
    apk_path = os.path.join(APKS_PATH, apk)
    apk_label = get_app_label_from_apk(apk_path)
    command = "python rotate_and_back.py --serial {0} --script-args app-to-test={1} wait-time=2000 host-path={2}".format(serial, apk_label, log_dir)
    subp = subprocess.Popen(command.split(), stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    get_output(subp)
    # monitor
    mon()
    time.sleep(2)


    print "TC-5"
    print "Start App - Go through the objects tree"
    apk_path = os.path.join(APKS_PATH, apk)
    apk_label = get_app_label_from_apk(apk_path)

    # record
    remote_path =  "/data/local/tmp/record.mp4"
    cmd = 'screenrecord --time-limit {0} {1}'.format(30, remote_path)
    record_p = adb_connection.run_cmd(cmd, mode = 'async')

    command = "python view_tree.py --serial {0} --script-args app-to-test={1} wait-time=2000".format(serial, apk_label)
    subp = subprocess.Popen(command.split(), stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    get_output(subp)

    time.sleep(1)
    while record_p.poll() is None:
        pass
    adb_steps.pull_file(log_dir, remote_path)
    # monitor
    mon()

    print "TC-6"
    print "Assisted Monkey"
    apk_path = os.path.join(APKS_PATH, apk)
    apk_label = get_app_label_from_apk(apk_path)
    command = "python assisted_monkey.py --serial {0} --script-args app-to-test={1} wait-time=2000".format(serial, apk_label)
    subp = subprocess.Popen(command.split(), stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    get_output(subp)
    # monitor
    mon()
    time.sleep(2)

    print "TC-7"
    print "Monkey"
    cmd = "monkey -p {0} --throttle 1000 --pct-motion 60 --pct-touch 40 300".format(apk_package)
    p_monkey = adb_connection.run_cmd(cmd, timeout = 240)
    # monitor
    mon()
    time.sleep(2)

    print "TC-4"
    print "Start App - Sleep"
    apk_path = os.path.join(APKS_PATH, apk)
    apk_label = get_app_label_from_apk(apk_path)
    command = "python sleep_and_back.py --serial {0} --script-args app-to-test={1} wait-time=2000 sleep-time=10000 host-path={2}".format(serial, apk_label, log_dir)
    subp = subprocess.Popen(command.split(), stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    get_output(subp)
    # monitor
    mon()
    time.sleep(2)

    del mon
    log_out_err_file.close()


def create_or_update_csv(csv_file, test):
    create = not os.path.isfile(csv_file)
    with open(csv_file, "a") as f:
        if create:
            f.write(CSV_HEADER)
        f.write(CSV_LINE_FORMAT.format(test['component'], test['name'],
                                           test['resolution']))

def parse_file_for_resolution(file_name):
    with open(file_name, "r") as f:
        output = f.read()
        if "raise TimeoutError(" in output:
            return Resolution.TIMEOUT
        if "raise BlockingError(" in output:
            return Resolution.BLOCKED
        if "[FAILED]" in output or 'raise FailedError(' in output:
            return Resolution.FAIL
        if "STDERR" in output:
            return Resolution.ERROR
    return Resolution.PASS

args = get_args(sys.argv)

adb_steps.connect_device(serial = args.serial, port = args.adb_server_port)()

for apk in apks:
    # TODO:
    #   add suite logic
    #   add test completion logic
    #   add DUT check
    #   add deps check (another DUT, AP)
    # END

    # test resolution
    test = {}
    test["resolution"] = Resolution.PASS
    log_dir = os.path.join(RESULTS_PATH, apk + get_time_string())
    local_steps.command(command = 'mkdir {0}'.format(log_dir),
                        stdout_grep = "",
                        critical = False,
                        blocking = False)()
    log_file = os.path.join(log_dir, apk + ".log")
    #dmesg_file = os.path.join(RESULTS_PATH, test['name'] + "timestamp.dmesg")
    #logcat_file = os.path.join(RESULTS_PATH, test['name'] + "timestamp.logcat")

    print ""
    print "============================================================"
    print 'Running test for apk: "{0}"'.format(apk)

    # create process for each test
    p = multiprocessing.Process(target=run_test,
                                args=(apk, args.serial,
                                      args.adb_server_port, log_dir, log_file))
    # start the process
    p.start()
    timeout = test['timeout'] if test.has_key('timeout') and test['timeout'] \
                else DEFAULT_TIMEOUT
    p.join(timeout)
    if p.is_alive():
        try:
            subp.terminate()
            subp.kill()
        except: pass
        p.terminate()
        test["resolution"] = Resolution.TIMEOUT
    else:
        # parse the log file to determine script resolution
        test["resolution"] = parse_file_for_resolution(log_file)

    print 'Test for apk: "{0}" is {1}'.format(apk, test['resolution'])
    # create_or_update_csv(CSV_FILE, test)

    print "============================================================"
    print ""


