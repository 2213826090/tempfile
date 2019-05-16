#!/usr/bin/env python
# -*- coding: utf-8; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil -*-

"""
SETUP EXEC_SCRIPT to test Android boot on VP Simics
"""

from os import remove, getcwd, getenv, listdir
from os.path import isfile, join, basename
from fnmatch import fnmatch
from re import compile as re_compile
from zipfile import ZipFile

from Core.PathManager import Paths

VERDICT = SUCCESS

# TODO redundancy, vars also defined in run script
simics_dir = join(getenv('HOME'), 'simics-project/simics-4.8.71/bin')
simics_exec = join(simics_dir, 'simics')
simics_script_path = join(getcwd(), 'bxt-android.simics')
simics_timeout = 180
simics_log_tty_path = join(getcwd(), 'debug-log-console.txt')
simics_log_path = join(getcwd(), 'debug-log.txt')

simics_craff = join(simics_dir, 'craff')

# TODO search the path of the latest simics revision installed and update the
# simics var directory in exec_script and in the template file with value found

# check if simics executable is available
if not isfile(simics_exec):
    VERDICT = BLOCKED
    PRINT_ERROR('simics executable not found: {0}'.format(simics_exec))

# check if craff executable is available
if not isfile(simics_craff):
    VERDICT = BLOCKED
    PRINT_ERROR('craff executable not found: {0}'.format(simics_craff))

# TODO check VP Simics is well installed, check all configuration file need
# exist

# TODO if kernel modules used by simics aren't already mounted then execute
# simics script to mount kernel module

# TODO check crypto-engine kernel module (cryptsetup) is installed and mounted

# check if VP Simics start script template is  available
template_path = join(getcwd(),
                     '_ExecutionConfig/SI/BBot_EAT_PACT/TC/POST_presi',
                     'BOOT', 'bxt-android.simics.template')
if not isfile(template_path):
    VERDICT = BLOCKED
    PRINT_ERROR('Simics start script template not found: {0}'
                .format(template_path))

# check if FLASH_FILES is available
if not isfile(Paths.FLASH_FILES):
    VERDICT = BLOCKED
    PRINT_ERROR('Invalid FLASH_FILES: {0}'.format(Paths.FLASH_FILES))

# if log files already exist then they are deleted
if isfile(simics_log_tty_path):
    remove(simics_log_tty_path)

if isfile(simics_log_path):
    remove(simics_log_path)

# if a os image already exist then it's deleted
for filename in listdir(getcwd()):
    if fnmatch(filename, '*.img'):
        remove(join(getcwd(), filename))

# if a craff image already exist then it's deleled
for filename in listdir(getcwd()):
    if fnmatch(filename, '*.craff'):
        remove(join(getcwd(), filename))


# if VP Simics start script already exist then it's deleted
if isfile(simics_script_path):
    remove(simics_script_path)

# extract android image from FLASH_FILES with zip
zip_file = ZipFile(Paths.FLASH_FILES)
os_img_path = join(getcwd(), 'bxt_simics.img')
zip_file.extract(basename(os_img_path))

# check image well extracted
if not isfile(os_img_path):
    VERDICT = BLOCKED
    PRINT_ERROR('Faillure of the OS image extraction: {0}'.format(os_img_path))

# generate a craff file from extrated android image
craff_img_path = os_img_path.replace('img', 'craff')
craff_cmd = '{0} -c zlib -o {1} {2}'.format(simics_craff,
                                            craff_img_path, os_img_path)
status, output = LOCAL_EXEC(craff_cmd, 90)

if status == SUCCESS:
    # check craff_img well create
    if not isfile(craff_img_path):
        status = BLOCKED
        PRINT_ERROR('craff image can\'t exist: {0}'.format(output))
else:
    status = BLOCKED
    PRINT_ERROR('craff image creation fail: {0}'.format(output))

# complete the VP Simics start script with the path of android image
re_os_image = re_compile(r'<os_image>')
re_timeout = re_compile(r'<timeout>')
with open(template_path, 'r') as template_file:
    with open(simics_script_path, 'w+') as simics_script_file:
        for line in template_file:
            if re_os_image.search(line):
                # replace <os_image> balise by this value
                newline = line.replace('<os_image>', craff_img_path)
                simics_script_file.write(newline)

            elif re_timeout.search(line):
                # replace <timeout> balise by this value
                newline = line.replace('<timeout>', str(simics_timeout))
                simics_script_file.write(newline)

            else:
                # copy the line read in template file
                simics_script_file.write(line)
