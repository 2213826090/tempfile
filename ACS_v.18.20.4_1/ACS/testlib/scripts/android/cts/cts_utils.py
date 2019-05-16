from testlib.base import base_utils
from testlib.scripts.connections.local import local_steps

import os
import re
import time


def create_cts_run_command(serials, cts_run):

    cts_command = "run cts"
    if cts_run["cts_plan"]:
        cts_command += " --plan {0}".format(cts_run["cts_plan"])
    elif cts_run["cts_class"]:
        cts_command += " --class {0}".format(cts_run["cts_class"])
        if cts_run["cts_method"]:
            cts_command += " --method {0}".format(cts_run["cts_method"])
    elif cts_run["cts_continue_session"]:
        cts_command += " --continue-session {0}".format(cts_run["cts_session_id"])
    cts_command += " --shards {0}".format(len(serials))
    for serial in serials:
        cts_command += " -s {0}".format(serial)
    if cts_run["cts_disable_reboot"]:
        cts_command += " --disable-reboot"
    if cts_run["cts_derived_plan"]:
        cts_command = "add derivedplan --plan {0}".format(cts_run["cts_plan"])
        cts_command += " --session {0}".format(cts_run["cts_session_id"])
        cts_command += " --result {0}".format(cts_run["cts_result_type"])
    return cts_command



def get_last_session(timestamp = None,
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
                               grep_for = "cts-tf >",
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
            if timestamp is not None:
                if res[4] == timestamp:
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
                              list_results_command = "list results",
                              screen_name = "irda_cts",
                              screen_log = "screenlog.0"):

    res = get_last_session(timestamp = timestamp,
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
    return results

def get_session_id_by_timestamp(timestamp,
                              list_results_command = "list results",
                              screen_name = "irda_cts",
                              screen_log = "screenlog.0"):

    res = get_last_session(timestamp = timestamp,
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

def get_run_timestamp(screen_log = "screenlog.0"):

    time.sleep(2)
    out = open(screen_log, "r").read()
    out_lines = out.strip().split("\n")
    search_pattern_front = "XML test result file generated at "
    search_pattern_back = ". Passed"
    format_pattern = re.compile(search_pattern_front + "(.*)" + search_pattern_back)

    for line in reversed(out_lines):
        search_results = format_pattern.search(line)
        if search_results:
            discovered_timestamp = search_results.group(1)
            break

    return discovered_timestamp

def get_latest_result_folder(list_results_command = "list results",
                            screen_name = "irda_cts",
                            screen_log = "screenlog.0"):

    ## This is now obsolete for the CTS run, replaced with the timestamp

    time.sleep(2)
    command = list_results_command
    local_steps.screen_command(screen_name = screen_name,
                               screen_command = command,
                               screen_log = screen_log,
                               with_log = True,
                               grep_for = "cts-tf >",
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


def create_continue_session(cts_run,
                            list_results_command = "list results",
                            screen_name = "irda_cts",
                            screen_log = "screenlog.0"):
    new_cts_run = {}
    new_cts_run["cts_plan"] = None
    new_cts_run["cts_class"] =  None
    new_cts_run["cts_method"] = None
    new_cts_run["cts_run_timestamp"] = cts_run["cts_run_timestamp"]
    new_cts_run["cts_session_id"] = get_session_id_by_timestamp(cts_run["cts_run_timestamp"],
                                                                list_results_command,
                                                                screen_name,
                                                                screen_log)["session_id"]
    new_cts_run["cts_result_type"] = None
    new_cts_run["cts_continue_session"] = True
    new_cts_run["cts_derived_plan"] = False
    new_cts_run["cts_disable_reboot"] = True
    new_cts_run["loop_no"] = cts_run["loop_no"]
    return new_cts_run


def create_derived_plan(cts_run,
                        cts_base_path,
                        cts_plans_dir,
                        plan_suffix = None,
                        list_results_command = "list results",
                        screen_name = "irda_cts",
                        screen_log = "screenlog.0"):

    if plan_suffix:
        file_name = "{0}_Loop_{1}_{2}.xml".format(os.path.join(cts_base_path,
                                                               cts_plans_dir,
                                                               cts_run["cts_plan"].split("_Loop")[0]),
                                                  cts_run["loop_no"],
                                                  plan_suffix)
    else:
        file_name = "{0}_Loop_{1}.xml".format(os.path.join(cts_base_path,
                                                           cts_plans_dir,
                                                           cts_run["cts_plan"].split("_Loop")[0]),
                                              cts_run["loop_no"])
    local_steps.delete_file(file_name = file_name)()
    new_cts_run = {}
    if plan_suffix:
        new_cts_run["cts_plan"] = "{0}_Loop_{1}_{2}".format(cts_run["cts_plan"].split("_Loop")[0], cts_run["loop_no"], plan_suffix)
    else:
        new_cts_run["cts_plan"] = "{0}_Loop_{1}".format(cts_run["cts_plan"].split("_Loop")[0], cts_run["loop_no"])
    new_cts_run["cts_class"] =  None
    new_cts_run["cts_method"] = None
    new_cts_run["cts_session_id"] = get_session_id_by_timestamp(cts_run["cts_run_timestamp"],
                                                                list_results_command,
                                                                screen_name,
                                                                screen_log)["session_id"]
    new_cts_run["cts_result_type"] = "fail"
    new_cts_run["cts_continue_session"] = False
    new_cts_run["cts_derived_plan"] = True
    new_cts_run["cts_disable_reboot"] = False
    new_cts_run["loop_no"] = cts_run["loop_no"] + 1
    return new_cts_run


def is_plan_created(plan,
                    cts_base_path,
                    cts_plans_dir):

    return os.path.isfile("{0}.xml".format(os.path.join(cts_base_path, cts_plans_dir, plan)))


def get_fails_from_xml_file(xml_file):

    import xml.etree.ElementTree as ET

    tree = ET.parse(xml_file)
    root = tree.getroot()
    fails = {}
    for package in root.iter('TestPackage'):
        cts_package = package.attrib["appPackageName"]
        cts_class_prefix = ""
        for suite in package.iter('TestSuite'):
            cts_class_prefix += suite.attrib["name"] + "."

        for testcase in package.iter('TestCase'):
            cts_class = cts_class_prefix + testcase.attrib["name"]
            for test in testcase.iter('Test'):
                if test.attrib["result"] == "fail":
                    cts_test = test.attrib["name"]
                    if cts_package not in fails:
                        fails[cts_package] = []
                    entry = {}
                    entry["test"] = "{0}#{1}".format(cts_class, cts_test)
                    fails[cts_package].append(entry)
    return fails
