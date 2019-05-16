from testlib.base import base_utils
from testlib.scripts.connections.local import local_steps

import os
import re
import time


def create_gts_run_command(serials, gts_run, default_params = " --max-log-size 320 --max-tmp-logcat-file 335544320"):

    gts_command = "run xts{0}".format(default_params)
    if gts_run["gts_plan"]:
        gts_command += " --plan {0}".format(gts_run["gts_plan"])
    elif gts_run["gts_class"]:
        gts_command += " --class {0}".format(gts_run["gts_class"])
        if gts_run["gts_method"]:
            gts_command += " --method {0}".format(gts_run["gts_method"])
    elif gts_run["gts_continue_session"]:
        gts_command += " --continue-session {0}".format(gts_run["gts_session_id"])
    if len(serials) > 1:
        gts_command += " --shards {0}".format(len(serials))
    for serial in serials:
        gts_command += " -s {0}".format(serial)
    if gts_run["gts_derived_plan"]:
        gts_command = "add derivedplan --plan {0}".format(gts_run["gts_plan"])
        gts_command += " --session {0}".format(gts_run["gts_session_id"])
        gts_command += " --result {0}".format(gts_run["gts_result_type"])
    return gts_command


def get_last_session(screen_name = "irda_gts",
                     list_results_command = "list results",
                     screen_log = "screenlog.0",
                     session_id = None):

    with open(screen_log, "w"):
        pass
    time.sleep(2)
    command = list_results_command
    local_steps.screen_command(screen_name = screen_name,
                               screen_command = command,
                               screen_log = screen_log,
                               with_log = True,
                               grep_for = "xts-tf >",
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
            if session_id:
                if int(res[0]) == session_id:
                    break
            else:
                break
    print "Results are: {0}".format(res)
    if res:
        if session_id is not None:
            return res
        else:
            return int(res[0])
    else:
        return -1


def get_results_by_session_id(session_id,
                              list_results_command = "list results",
                              screen_name = "irda_gts",
                              screen_log = "screenlog.0"):

    res = get_last_session(session_id = session_id,
                           screen_name = screen_name,
                           screen_log = screen_log,
                           list_results_command = list_results_command)
    print "Getting results for session {0}: {1}".format(session_id, res)
    if res == -1:
        print "Did not find session {0}".format(session_id)
        return None
    results = {}
    results["pass"] = res[1]
    results["fail"] = res[2]
    results["notExecuted"] = res[3]
    return results


def get_latest_result_folder(list_results_command = "list results",
                            screen_name = "irda_gts",
                            screen_log = "screenlog.0"):

    time.sleep(2)
    command = list_results_command
    local_steps.screen_command(screen_name = screen_name,
                               screen_command = command,
                               screen_log = screen_log,
                               with_log = True,
                               grep_for = "xts-tf >",
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


def create_continue_session(gts_run):
    new_gts_run = {}
    new_gts_run["gts_plan"] = None
    new_gts_run["gts_class"] =  None
    new_gts_run["gts_method"] = None
    new_gts_run["gts_session_id"] = gts_run["gts_session_id"]
    new_gts_run["gts_result_type"] = None
    new_gts_run["gts_continue_session"] = True
    new_gts_run["gts_derived_plan"] = False
    new_gts_run["loop_no"] = gts_run["loop_no"]
    return new_gts_run


def create_derived_plan(gts_run,
                        gts_base_path,
                        gts_plans_dir,
                        plan_suffix = None):

    if plan_suffix:
        file_name = "{0}_Loop_{1}_{2}.xml".format(os.path.join(gts_base_path,
                                                               gts_plans_dir,
                                                               gts_run["gts_plan"].split("_Loop")[0]),
                                                  gts_run["loop_no"],
                                                  plan_suffix)
    else:
        file_name = "{0}_Loop_{1}.xml".format(os.path.join(gts_base_path,
                                                           gts_plans_dir,
                                                           gts_run["gts_plan"].split("_Loop")[0]),
                                              gts_run["loop_no"])
    local_steps.delete_file(file_name = file_name)()
    new_gts_run = {}
    if plan_suffix:
        new_gts_run["gts_plan"] = "{0}_Loop_{1}_{2}".format(gts_run["gts_plan"].split("_Loop")[0], gts_run["loop_no"], plan_suffix)
    else:
        new_gts_run["gts_plan"] = "{0}_Loop_{1}".format(gts_run["gts_plan"].split("_Loop")[0], gts_run["loop_no"])
    new_gts_run["gts_class"] =  None
    new_gts_run["gts_method"] = None
    new_gts_run["gts_session_id"] = gts_run["gts_session_id"]
    new_gts_run["gts_result_type"] = "fail"
    new_gts_run["gts_continue_session"] = False
    new_gts_run["gts_derived_plan"] = True
    new_gts_run["loop_no"] = gts_run["loop_no"] + 1
    return new_gts_run

