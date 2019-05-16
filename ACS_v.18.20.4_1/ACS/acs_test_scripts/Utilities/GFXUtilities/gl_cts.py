#!/usr/bin/python
import argparse
import commands
import csv
import itertools
import os
import subprocess
import sys
import textwrap
import time
from subprocess import Popen
from time import gmtime, strftime
from opengles_log_parser import BatchResultParser
import collections 
#from collections import defaultdict
#from collections import OrderedDict

parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, 
        description = textwrap.dedent('''\
        Run GL-CTS groups individually in:
            - ES groups certification mode run for selected levels ES2 / ES3 / ES31 \
(defaults to ES2 + ES3)
            - single group certification mode run for group in levels ES2 / ES3 / ES31 \
(defaults to ES2 + ES3)
            - single group run in window size 64x64 ConfigID=1 mode
        Examples: 
            - python gl_cts.py --cert_mode y --cert_level ES2 --cert_level ES3 (run all groups all window sizes and ConfigIDs)
            - python gl_cts.py --cert_mode y --group_to_run ES2-CTS.* (run single group all window sizes and ConfigIDs)
            - python gl_cts.py --cert_mode n --cert_level ES3 --cert_level ES31 (aka quick run all groups using single window size 64x64 ConfigID=1)
            - python gl_cts.py --cert_mode n --group_to_run ES2-CTS.* (aka quick run group using single window size 64x64 ConfigID=1)
            
            
            '''))
parser.add_argument('--cert_mode', type = str, choices = ['y', 'n'], 
                    dest='cert_mode', required=True)
parser.add_argument('--cert_level', action='append', 
                    choices = ['ES2', 'ES3', 'ES31'], dest='cert_level')
parser.add_argument('--group_to_run', type=str, dest='group_to_run')

args = parser.parse_args()
#print args

SCRIPT_PATH = os.path.abspath( __file__ )
list = SCRIPT_PATH.split(os.sep)
ROOT_PATH = os.sep.join(list[0:-2]) + os.sep
RESULTS_PATH = ROOT_PATH + "results" + os.sep
RESULTS_FOLDER =  strftime("%Y_%m_%d_%H_%M_%S", gmtime())
if not os.path.exists(RESULTS_PATH):
    os.mkdir(os.path.join(ROOT_PATH, "results"))

os.mkdir(os.path.join(RESULTS_PATH, RESULTS_FOLDER))
RESULTS_FOLDER_PATH = RESULTS_PATH + os.sep + RESULTS_FOLDER + os.sep

os.mkdir(os.path.join(RESULTS_PATH, RESULTS_FOLDER, "ES2"))
os.mkdir(os.path.join(RESULTS_PATH, RESULTS_FOLDER, "ES3"))
os.mkdir(os.path.join(RESULTS_PATH, RESULTS_FOLDER, "ES31"))
RESULTS_ES_PATH = {}
RESULTS_ES_PATH["ES2"] = RESULTS_PATH + RESULTS_FOLDER + os.sep + "ES2" + os.sep
RESULTS_ES_PATH["ES3"] = RESULTS_PATH + RESULTS_FOLDER + os.sep + "ES3" + os.sep
RESULTS_ES_PATH["ES31"] = RESULTS_PATH + RESULTS_FOLDER + os.sep + "ES31" + os.sep
RESULTS_FILE = os.path.join(RESULTS_PATH, RESULTS_FOLDER, "Results.csv")
test_files = ["ES2-CTS-cases.txt", "ES3-CTS-cases.txt", "ES31-CTS-cases.txt"]
group_files = ["ES2-CTS-groups.txt", "ES3-CTS-groups.txt", "ES31-CTS-groups.txt"]



# ordered bidimensional dictionary to keep each test with results for
# each run

# the runs used for each group in certification mode, entries can be optained using window_size + "_" + str(configID)
run_list = ["ws_64_64_1",
            "ws_64_64_2",
            "ws_64_64_3",
            "ws_64_64_4",
            "ws_64_64_5",
            "ws_64_64_6",
            "ws_64_64_7",
            "ws_64_64_9",
            "ws_64_64_10",
            "ws_64_64_11",
            "ws_113_47_1",
            "ws_113_47_2",
            "ws_113_47_3",
            "ws_113_47_4",
            "ws_113_47_5",
            "ws_113_47_6",
            "ws_113_47_7",
            "ws_113_47_9",
            "ws_113_47_10",
            "ws_113_47_11",
            "fbo1_1",
            "fbo2_1",
            ]

#~ print SCRIPT_PATH
#~ print ROOT_PATH
print RESULTS_PATH
#~ print RESULTS_ES_PATH["ES2"]
#~ print RESULTS_ES_PATH["ES3"]
#~ print RESULTS_ES_PATH["ES31"]
#~ print RESULTS_FILE

class OrderedDefaultdict(collections.OrderedDict):
    def __init__(self, *args, **kwargs):
        if not args:
            self.default_factory = None
        else:
            if not (args[0] is None or callable(args[0])):
                raise TypeError('first argument must be callable or None')
            self.default_factory = args[0]
            args = args[1:]
        super(OrderedDefaultdict, self).__init__(*args, **kwargs)

    def __missing__ (self, key):
        if self.default_factory is None:
            raise KeyError(key)
        self[key] = default = self.default_factory()
        return default

    def __reduce__(self):  # optional, for pickle support
        args = (self.default_factory,) if self.default_factory else ()
        return self.__class__, args, None, None, self.iteritems()

result_dict = OrderedDefaultdict(dict)

def execute_command(cmd, remote=True, timeout = 60):
    if remote:
        cmd = "timeout " + str(timeout) + " adb shell " + cmd
    #print cmd
    my_process = Popen(cmd, 
                                        shell = True,
                                        stdin = subprocess.PIPE,
                                        stdout = subprocess.PIPE, 
                                        stderr = subprocess.PIPE
                                        )
    return my_process
    
def batchResultToCsv (qpa_file, csv_file, window_size, configID):
    """
    parse the results *.qpa file for each group and:
    - write the test name/ status to proper *.csv file
    - store the test name as key in a dictionary of dictionaries
    - the 2nd level dictionaries are storing results for same test but different run( window size/configID)
    """
    parser = BatchResultParser()
    results = parser.parseFile(qpa_file)
    csv_d = open(csv_file, 'a')
    run_case = window_size + "_" + str(configID)
    for result in results:
        #print "%s,%s" % (result.name, result.statusCode)
        csv_d.write(result.name + "," + result.statusCode + '\n')
        result_dict[result.name][run_case] = result.statusCode

    csv_d.close()

# get the files with the list of all the available tests
# parse the files to extract the groups and create separate files for them
def get_groups():
    cmd = '''am start -n org.khronos.gl_cts/android.app.NativeActivity \
    -e cmdLine "cts --deqp-runmode=txt-caselist --deqp-screen-rotation=90"'''
    proc = execute_command(cmd)
    (stdout, stderr) = proc.communicate(input=None)
    #~ print stdout
    #~ print stderr
    for f in test_files:
        get_result("/sdcard/" + f, RESULTS_PATH)
    for tf, gf in itertools.izip(test_files, group_files):
        tf_d = open(RESULTS_PATH + tf, 'r')
        gf_d = open(RESULTS_PATH + gf, 'w')
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

# check that the gl_cts *.apk finished or crashed to start results pull after   
def wait_for_result(timeout):
    frozen = False
    start = 0
    while (start < timeout):
        proc = execute_command("ps | grep org.khronos.gl_cts")
        (stdout, stderr) = proc.communicate(input=None)
        time.sleep(0.1)
        #~ print stdout
        #~ print stderr
        if "khronos" in stdout:
            time.sleep(0.1)
            start = start + 0.1
        elif "error" in stdout + stderr:
            print "Device disconnected"
            time.sleep(30)
            proc = execute_command("adb kill-server; adb connect 192.168.42.1", remote = False)
            (sout, serr) = proc.communicate(input=None)
            proc = execute_command("adb devices", remote=False)
            (sout, serr) = proc.communicate(input=None)
            if "5555" in sout:
                print "Reconnected to device"
                time.sleep(10)
                proc = execute_command("input keyevent 82")
                (sout, serr) = proc.communicate(input = None)
                proc = execute_command("adb root", remote = False)
                (sout, serr) = proc.communicate(input = None)
                proc = execute_command("adb kill-server; adb connect 192.168.42.1", remote = False)
                (sout, serr) = proc.communicate(input = None)
                break
            else:
                print "Cannot reconnect to device"
                frozen = True
        else:
            break
    if start  >= timeout:
        frozen = True
    return frozen
# runs a single group with a choice of window sizes, fbo or ConfigIDs
#
def single_run(group, window_size, configID, timeout = 300):
    am_start ="am start -n org.khronos.gl_cts/android.app.NativeActivity -e cmdLine "
    # dictionary to keep formated options for different windows sizes/ fbo
    win_cmd = {}
    win_cmd["ws_64_64"] = "\"cts \
--deqp-surface-width=64 \
--deqp-surface-height=64 \
--deqp-base-seed=1 \
--deqp-surface-type=window "
    win_cmd["ws_113_47"] = "\"cts \
--deqp-surface-width=113 \
--deqp-surface-height=47 \
--deqp-base-seed=2 \
--deqp-surface-type=window "
    win_cmd["fbo1"] = "\"cts \
--deqp-surface-width=64 \
--deqp-surface-height=64 \
--deqp-base-seed=3 \
--deqp-surface-type=window \
--deqp-use-fbo=GL_RGBA8,GL_DEPTH24_STENCIL8,64,max "
    win_cmd["fbo2"] = "\"cts \
--deqp-surface-width=64 \
--deqp-surface-height=64 \
--deqp-base-seed=3 \
--deqp-surface-type=window \
--deqp-use-fbo=GL_RGBA8,GL_DEPTH24_STENCIL8,max,64 "
    # log name for the test result
    log_name = "/sdcard/%s_%s_cfgID_%s.qpa" % (group[0:-2], window_size, configID)
    # group to test and log file part of the command    
    cmdLine = "--deqp-case=%s --deqp-log-filename=%s " % (group, log_name)
    cmdID = "--deqp-gl-config-id=%s " % str(configID)
    cmdRot = "--deqp-screen-rotation=90\""
    cmd = am_start + win_cmd[window_size] + cmdLine + cmdID + cmdRot
    print cmd
    # clear the logcat
    proc = execute_command("adb logcat -c", remote = False)
    (stdout, stderr) = proc.communicate(input = None)
    # run the gl_cts command
    proc = execute_command(cmd)
    (stdout, stderr) = proc.communicate(input = None)
    print stdout
    print stderr
   
    if wait_for_result(timeout):
        print "Execution freezed"
        sys.exit(0)
    else:
        print "Group execution finished"
        # prevent crash dialogue on screen
        press_enter = "input keyevent 66"
        proc = execute_command(press_enter)
        (stdout, stderr) = proc.communicate(input = None)
        print stdout
        print stderr
        #press escape 2 times
        press_escape = "input keyevent 111"
        for i in range(0,2):
            proc = execute_command(press_escape)
            (stdout, stderr) = proc.communicate(input = None)
        time.sleep(1)
        
    # create absulute path for results file (in the proper ES folder)
    result_file = RESULTS_ES_PATH[group.split("-")[0]] + log_name.strip(os.sep).split(os.sep)[1]
    get_result(log_name, RESULTS_ES_PATH[group.split("-")[0]])
    # dump the logcat to file
    proc = execute_command("adb logcat -d > " + result_file + ".log", remote = False)
    (stdout, stderr) = proc.communicate(input = None)
    #delete log file from device
    rm_cmd = "rm " + log_name
    proc = execute_command(rm_cmd)
    (stdout, stderr) = proc.communicate(input = None)
    
    try:
        batchResultToCsv (result_file, os.path.join(RESULTS_PATH, RESULTS_FOLDER, window_size + "_" + str(configID) + ".csv"), window_size, configID)
    except IOError:
        run_case = window_size + "_" + str(configID)
        result_dict[group][run_case] = "RunERROR"
    
# run as certification mode (all the required window sizes and configIDs and FBO)
def cert_run(group):
    print "1"
    #~ # run all groups  ws = 64 x 64 config-id 1
    single_run(group, "ws_64_64", 1)
    # run all CTS.gtf.*  ws = 113 x 47 config-id 1
    if "CTS.gtf" in group:
        single_run(group, "ws_113_47", 1)
    # run CTS.gtf.* fbo1
    if "CTS.gtf" in group:
        single_run(group, "fbo1", 1)
    # run CTS.gtf.* fbo2
    if "CTS.gtf" in group:
        single_run(group, "fbo2", 1)
    # run ws = 64 x 64 all configIDs 2-7, 9-11
    if "CTS.gtf" in group or "ES31" in group:
        for cfgID in [2,3,4,5,6,7,9,10,11]:
            single_run(group, "ws_64_64", cfgID)
    # run ws = 113 x 47 all configIDs 2-7, 9-11
    if "CTS.gtf" in group:
        for cfgID in [2,3,4,5,6,7,9,10,11]:
            single_run(group, "ws_113_47", cfgID)

# gets the *.qpa result file from the device    
def get_result(rmt_src, loc_dest):
    cmd = "adb pull %s %s" % (rmt_src , loc_dest)
    proc = execute_command(cmd, remote=False)
    (stdout, stderr) = proc.communicate(input = None)
    print stdout
    print stderr

# writes the final results for the test list
def write_dict(dictionary, run_list, output_file):
    f = open(output_file, 'wt')
    try:
        writer = csv.writer(f)
        header = []
        header.append("Test name")
        writer.writerow(["cert_mode", args.cert_mode, "cert_level", args.cert_level, "group_to_run", args.group_to_run])
        writer.writerow( header + run_list)
        for key in dictionary:
            line_to_write =[]
            line_to_write.append(key)
            for run in run_list:
                try:
                    line_to_write.append(dictionary[key][run])
                except KeyError:
                    line_to_write.append('NotRun')
            writer.writerow(line_to_write)
    finally:
        f.close()
    
def main():
    ## comment the line below to use previously obtained groups files
    ## you can use # to comment group lines in these files
    #get_groups()
    
    if args.cert_mode == 'y':
        if args.group_to_run:
            cert_run(args.group_to_run)
        else:
            group_files_to_run = []
            if "ES2" in args.cert_level:
                group_files_to_run.append(group_files[0])
            if "ES3" in args.cert_level:
                group_files_to_run.append(group_files[1])
            if "ES31" in args.cert_level:
                group_files_to_run.append(group_files[2])
            # the default run
            if len(group_files_to_run) == 0:
                group_files_to_run.append(group_files[0])
                group_files_to_run.append(group_files[1])
            
            for gf in group_files_to_run:
                #~ print gf
                #~ continue
                gf_d = open(RESULTS_PATH + gf, 'r')
                for group in gf_d:
                    if group.startswith('#'):
                        continue
                    else:
                        cert_run(group.strip('\n'))
                        time.sleep(5)

    elif args.cert_mode == 'n':
        #~ print args.group_to_run
        #~ sys.exit()
        if args.group_to_run:
            single_run(args.group_to_run, "ws_64_64", 1)
        else:
            group_files_to_run = []
            if "ES2" in args.cert_level:
                group_files_to_run.append(group_files[0])
            if "ES3" in args.cert_level:
                group_files_to_run.append(group_files[1])
            if "ES31" in args.cert_level:
                group_files_to_run.append(group_files[2])
            # the default run
            if len(group_files_to_run) == 0:
                group_files_to_run.append(group_files[0])
                group_files_to_run.append(group_files[1])
            
            for gf in group_files_to_run:
                #~ print gf
                #~ continue
                gf_d = open(RESULTS_PATH + gf, 'r')
                for group in gf_d:
                    if group.startswith('#'):
                        continue
                    else:
                        single_run(group.strip('\n'), "ws_64_64", 1)
                        time.sleep(3)
        
        
        
    
    write_dict(result_dict, run_list, RESULTS_FILE)
    
if __name__ == "__main__":
    main()
