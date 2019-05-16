#!/usr/bin/env python
# -*- coding: utf-8; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil -*-

"""
RUN EXEC_SCRIPT to test Android boot on VP Simics
"""

from os import getcwd, getenv
from os.path import isfile, isdir, join, basename
from re import compile as re_compile
from shutil import copyfile

VERDICT = FAILURE

# TODO redundancy, vars already defined in setup script
simics_dir = join(getenv('HOME'), 'simics-project/simics-4.8.71/bin')
simics_exec = join(simics_dir, 'simics')
simics_script_path = join(getcwd(), 'bxt-android.simics')
simics_timeout = 180
simics_log_tty_path = join(getcwd(), 'debug-log-console.txt')
simics_log_path = join(getcwd(), 'debug-log.txt')

simics_options = '-no-gui -no-win -batch-mode'
simics_cmd = '{0} {1} {2} &'.format(simics_exec, simics_options,
                                    simics_script_path)

# start VP Pure Simics device
status, output = LOCAL_EXEC(simics_cmd, simics_timeout + 60)

if status == SUCCESS:
    PRINT_INFO('VP Simics execution success:\n{0}'.format(output))

elif status == FAILURE:
    VERDICT = BLOCKED
    PRINT_ERROR('VP Simics execution failure:\n{0}'.format(output))

else:
    VERDICT = BLOCKED
    PRINT_DEBUG('VP Simics unknown execution status:\n{0}'.format(output))

# check VP Simics TTY log file exist
if not isfile(simics_log_tty_path):
    VERDICT = BLOCKED
    PRINT_ERROR('VP Simics log file don\'t exist: {0}'
                .format(simics_log_tty_path))
else:
    # read VP Simics TTY log file to check that system started well
    with open(simics_log_tty_path, 'r') as log_file:
        re_boot = re_compile(r'#1#2#3 kernel')
        for line in log_file:
            if re_boot.search(line):
                VERDICT = SUCCESS
                break

# copy startscript and log files in REPORT_PATH to save under Artifactory
if isdir(REPORT_PATH):
    copyfile(simics_log_path, join(REPORT_PATH, basename(simics_log_path)))
    copyfile(simics_log_tty_path,
             join(REPORT_PATH, basename(simics_log_tty_path)))
    copyfile(simics_script_path,
             join(REPORT_PATH, basename(simics_script_path)))

else:
    PRINT_DEBUG('Invalid REPORT_PATH: {0}'.format(REPORT_PATH))
