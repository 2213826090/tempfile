from testlib.base import base_utils
from testlib.scripts.connections.local import local_steps
from testlib.utils.statics.android import statics

import os
import re
import time

SUITE_STATICS = {
    'gts': {
        'runner': 'xts',
        'default_params': ' --max-log-size 320 --max-tmp-logcat-file 335544320'
    },
    'cts': {
        'runner': 'cts',
        'default_params': ''
    }
}

def create_suite_run_command(serials = None,
                             suite = "cts",
                             suite_run = {},
                             runner_type = None,
                             default_params = " --max-log-size 320 --max-tmp-logcat-file 335544320"):
    commandIsCorrect = True
    error_message = ''
    suite_command = "run {0}{1}".format(SUITE_STATICS[suite]['runner'], SUITE_STATICS[suite]['default_params'])

    if runner_type == "module_based":
        if suite_run["{0}_module".format(suite)] != '':
            suite_command += " --module {0}".format(suite_run["{0}_module".format(suite)])
            if suite_run["{0}_test".format(suite)]:
                suite_command += " --test {0}".format(suite_run["{0}_test".format(suite)])
        elif suite_run["{0}_retry_session".format(suite)] != '':
            suite_command += " --retry {0}".format(suite_run["{0}_retry_session".format(suite)])
    elif runner_type == "plan_based":
        if suite_run["{0}_plan".format(suite)]:
            suite_command += " --plan {0}".format(suite_run["{0}_plan".format(suite)])
        elif suite_run["{0}_class".format(suite)]:
            suite_command += " --class {0}".format(suite_run["{0}_class".format(suite)])
            if suite_run["{0}_method".format(suite)]:
                suite_command += " --method {0}".format(suite_run["{0}_method".format(suite)])
        elif suite_run["{0}_continue_session".format(suite)]:
            suite_command += " --continue-session {0}".format(suite_run["{0}_session_id".format(suite)])
    else:
        commandIsCorrect = False
        error_message += "Unable to create the run command! Possible cause: The dessert configured in the settings is not supported by ACAS yet"
    ##If there is only one device available, the script will set the parameter --shards 1 for both CTS and GTS
    if len(serials) != 0:
        suite_command += " --shards {0}".format(len(serials))
    else:
        commandIsCorrect = False
        error_message += "Unable to create the run command! Possible cause: The number of available devices is " + str(len(serials))
    for serial in serials:
        suite_command += " -s {0}".format(serial)
    if "{0}_disable_reboot".format(suite) in suite_run:
        if suite_run["{0}_disable_reboot".format(suite)]:
            suite_command += " --disable-reboot"
    if "{0}_skip_preconditions".format(suite) in suite_run:
        if suite_run["{0}_skip_preconditions".format(suite)]:
            suite_command += " --skip-preconditions"

    if suite_run["{0}_derived_plan".format(suite)]:
        suite_command = "add derivedplan --plan {0}".format(suite_run["{0}_plan".format(suite)])
        suite_command += " --session {0}".format(suite_run["{0}_session_id".format(suite)])
        suite_command += " --result {0}".format(suite_run["{0}_result_type".format(suite)])
    if commandIsCorrect:
        return suite_command
    else:
        return error_message

def format_timestamp(run_timestamp,
                    runner_type):
    if runner_type == "module_based":
        timestamp_split = run_timestamp.split("_")
        timestamp_split[0] = timestamp_split[0].replace(".", "-")
        timestamp_split[1] = timestamp_split[1].replace(".", ":")
        run_timestamp = timestamp_split[0] + " " + timestamp_split[1]
    return run_timestamp

def get_session(timestamp = None,
                     suite = "cts",
                     list_results_command = "list results",
                     screen_name = "irda_cts",
                     screen_log = "screenlog.0"):

    with open(screen_log, "w"):
        pass
    time.sleep(2)
    command = list_results_command
    local_steps.screen_command(screen_name = screen_name,
                               screen_command = command,
                               screen_log = screen_log,
                               with_log = True,
                               grep_for = "{0}-tf >".format(SUITE_STATICS[suite]['runner']),
                               rows = 1)()
    time.sleep(10)
    out = open(screen_log, "r").read()
    out_lines = out.strip().split("\n")
    result_line_re = '[0-9]+\s+[0-9]+\s+[0-9]+\s+[0-9]+\s+.+$'
    res = None
    composite_timestamp = False
    for line in reversed(out_lines):
        match_result = re.match(result_line_re, line)
        if match_result:
            res = match_result.group(0).split()
            if timestamp is not None:
                if res[4] == timestamp:
                    break
                elif res[4] + " " + res[5] == timestamp:
                    composite_timestamp = True
                    break
            else:
                break
    print "Results are: {0}".format(res)
    if res:
        if timestamp is not None:
            return res
        else:
            return int(res[0])
    else:
        return -1

def get_results_by_timestamp(timestamp,
                             suite = "cts",
                             list_results_command = "list results",
                             screen_name = "irda_cts",
                             screen_log = "screenlog.0"):

    res = get_session(timestamp = timestamp,
                      suite = suite,
                      screen_name = screen_name,
                      screen_log = screen_log,
                      list_results_command = list_results_command)
    print "Getting results for session {0}: {1}".format(timestamp, res)
    if res == -1:
        print "Did not find session ", str(timestamp)
        return None
    results = {}
    results["pass"] = res[1]
    results["fail"] = res[2]
    results["notExecuted"] = res[3]
    results["session"] = res[0]
    return results

def get_session_id_by_timestamp(timestamp,
                                suite = "cts",
                                list_results_command = "list results",
                                screen_name = "irda_cts",
                                screen_log = "screenlog.0"):

    res = get_session(timestamp = timestamp,
                      suite = suite,
                      screen_name = screen_name,
                      screen_log = screen_log,
                      list_results_command = list_results_command)
    print "Getting sesion_id for timestamp {0}: {1}".format(timestamp, res)
    if res == -1:
        print "Did not find timestamp ", str(timestamp)
        return None
    results = {}
    results["session_id"] = res[0]
    return results

def get_run_timestamp(screen_log = "screenlog.0",
                      runner_type = None):

    time.sleep(2)
    out = open(screen_log, "r").read()
    out_lines = out.strip().split("\n")
    if runner_type == "plan_based":
        search_pattern_front = "XML test result file generated at "
        search_pattern_back = ". Passed"
    elif runner_type == "module_based":
        search_pattern_front = "android-cts/results/"
        search_pattern_back = "/test_result.xml"

    format_pattern = re.compile(search_pattern_front + "(.*)" + search_pattern_back)

    for line in reversed(out_lines):
        search_results = format_pattern.search(line)
        if search_results:
            discovered_timestamp = search_results.group(1)
            break

    return discovered_timestamp

def get_latest_result_folder(list_results_command = "list results",
                            suite = "cts",
                            screen_name = "irda_cts",
                            screen_log = "screenlog.0"):

    ## This is now obsolete for the CTS/GTS run, replaced with the timestamp

    time.sleep(2)
    command = list_results_command
    local_steps.screen_command(screen_name = screen_name,
                               screen_command = command,
                               screen_log = screen_log,
                               with_log = True,
                               grep_for = "{0}-tf >".format(SUITE_STATICS[suite]['runner']),
                               rows = 1)()
    time.sleep(10)
    out = open(screen_log, "r").read()
    out_lines = out.strip().split("\n")
    result_line_re = '[0-9]+\s+[0-9]+\s+[0-9]+\s+[0-9]+\s+.+$'
    res = None
    for line in reversed(out_lines):
        match_result = re.match(result_line_re, line)
        if match_result:
            res = match_result.group(0).split()
            break

    return res[4]

def create_continue_session(suite_run = {},
                            suite = "cts",
                            list_results_command = "list results",
                            screen_name = "irda_cts",
                            screen_log = "screenlog.0"):
    new_suite_run = {}
    new_suite_run["{0}_plan".format(suite)] = None
    new_suite_run["{0}_class".format(suite)] =  None
    new_suite_run["{0}_method".format(suite)] = None
    new_suite_run["{0}_run_timestamp".format(suite)] = suite_run["{0}_run_timestamp".format(suite)]
    new_suite_run["{0}_session_id".format(suite)] = get_session_id_by_timestamp(suite_run["{0}_run_timestamp".format(suite)],
                                                                                suite,
                                                                                list_results_command,
                                                                                screen_name,
                                                                                screen_log)["session_id"]
    new_suite_run["{0}_result_type".format(suite)] = None
    new_suite_run["{0}_continue_session".format(suite)] = True
    new_suite_run["{0}_derived_plan".format(suite)] = False
    new_suite_run["{0}_disable_reboot".format(suite)] = True
    new_suite_run["loop_no"] = suite_run["loop_no"]
    return new_suite_run

def create_derived_plan(suite_run,
                        suite,
                        suite_base_path,
                        suite_plans_dir,
                        plan_suffix = None,
                        list_results_command = "list results",
                        screen_name = "irda_cts",
                        screen_log = "screenlog.0"):

    if plan_suffix:
        file_name = "{0}_Loop_{1}_{2}.xml".format(os.path.join(suite_base_path,
                                                               suite_plans_dir,
                                                               suite_run["{0}_plan".format(suite)].split("_Loop")[0]),
                                                  suite_run["loop_no"],
                                                  plan_suffix)
    else:
        file_name = "{0}_Loop_{1}.xml".format(os.path.join(suite_base_path,
                                                           suite_plans_dir,
                                                           suite_run["{0}_plan".format(suite)].split("_Loop")[0]),
                                              suite_run["loop_no"])

    local_steps.delete_file(file_name = file_name)()
    new_suite_run = {}
    if plan_suffix:
        new_suite_run["{0}_plan".format(suite)] = "{0}_Loop_{1}_{2}".format(suite_run["{0}_plan".format(suite)].split("_Loop")[0], suite_run["loop_no"], plan_suffix)
    else:
        new_suite_run["{0}_plan".format(suite)] = "{0}_Loop_{1}".format(suite_run["{0}_plan".format(suite)].split("_Loop")[0], suite_run["loop_no"])
    new_suite_run["{0}_class".format(suite)] =  None
    new_suite_run["{0}_method".format(suite)] = None
    new_suite_run["{0}_session_id".format(suite)] = get_session_id_by_timestamp(suite_run["{0}_run_timestamp".format(suite)],
                                                                                suite,
                                                                                list_results_command,
                                                                                screen_name,
                                                                                screen_log)["session_id"]
    new_suite_run["{0}_result_type".format(suite)] = "fail"
    new_suite_run["{0}_continue_session".format(suite)] = False
    new_suite_run["{0}_derived_plan".format(suite)] = True
    new_suite_run["{0}_disable_reboot".format(suite)] = False
    new_suite_run["loop_no"] = suite_run["loop_no"] + 1
    return new_suite_run

def is_plan_created(plan,
                    suite_base_path,
                    suite_plans_dir):

    return os.path.isfile("{0}.xml".format(os.path.join(suite_base_path, suite_plans_dir, plan)))


def get_fails_from_xml_file(xml_file):

    import xml.etree.ElementTree as ET

    tree = ET.parse(xml_file)
    root = tree.getroot()
    fails = {}
    for package in root.iter('TestPackage'):
        suite_package = package.attrib["appPackageName"]
        suite_class_prefix = ""
        for suite in package.iter('TestSuite'):
            suite_class_prefix += suite.attrib["name"] + "."

        for testcase in package.iter('TestCase'):
            suite_class = suite_class_prefix + testcase.attrib["name"]
            for test in testcase.iter('Test'):
                if test.attrib["result"] == "fail":
                    suite_test = test.attrib["name"]
                    if suite_package not in fails:
                        fails[suite_package] = []
                    entry = {}
                    entry["test"] = "{0}#{1}".format(suite_class, suite_test)
                    fails[suite_package].append(entry)
    return fails

def get_fails_from_xml_file_module_based(xml_file):

    import xml.etree.ElementTree as ET

    tree = ET.parse(xml_file)
    root = tree.getroot()
    fails = {}
    for module in root.iter('Module'):
        suite_module = module.attrib["name"]
        for case in module.iter('TestCase'):
            suite_case = case.attrib["name"]

            for test in case.iter('Test'):
                if test.attrib["result"] == "fail":
                    suite_test = test.attrib["name"]
                    if suite_module not in fails:
                        fails[suite_module] = []
                    entry = {}
                    entry["test"] = "{0}#{1}".format(suite_case, suite_test)
                    fails[suite_module].append(entry)
    return fails