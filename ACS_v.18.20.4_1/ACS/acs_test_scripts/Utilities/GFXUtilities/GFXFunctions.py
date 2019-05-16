"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL OTC ANDROID QA

provided: GATE Framework partial integration of testing features

:since: 5/26/14
:author: mmaracix
"""

import os
import sys
from xml.etree.ElementTree import ElementTree as ET
import test_constant as tc
import collections
import commands
import shutil
import glob
import time
import datetime
import signal
import logging
import argparse

import fru_constant as fc
from test_config import input_keyevent, screen_off_timeout_times, screen_off_timeout_times_adjust_offset, port_relay_0

# Abort Test item flag
ABORT_TEST_ITEM_FLAG = False

"""
from util_lib
"""


class switch(object):
    """
    Class: switch
    Description: This class provides the functionality of a switch statement.
    """

    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        yield self.match
        raise StopIteration

    def match(self, *args):
        if self.fall or not args:
            return True
        elif self.value in args:
            self.fall = True
            return True
        else:
            return False


class TestCaseStatus:
    def __init__(self):
        pass

    passed, failed, blocked_dependency = range(3)
    __toString = {passed: "PASSED", failed: "FAILED", blocked_dependency: "BLOCKED DEPENDENCY"}

    @classmethod
    def tostring(cls, val):
        return cls.__toString.get(val)


# Defines a tuple for the test case status
# Two fields:
# status - test case status of type TestCaseStatus
# is_test_case_cleanup - if test_step is test_case_cleanup, True, else False
# info - informational string

TEST_CASE_STATUS = collections.namedtuple('test_case_status',
                                          'status is_test_case_cleanup info')

test_case_status = TEST_CASE_STATUS(TestCaseStatus.passed, False, '')


def get_test_case_status():
    """
    Function: get_test_case_status
    Returns test_case_status
    Parameters: None
    Output: None
    Return: returns test_case_status
    """

    global test_case_status
    return test_case_status


def reset_test_case_status():
    """
    Function: reset_test_case_status
    resets test_case_status to passed with a blank informational message
    Parameters: None
    Output: test_case_status set to passed, information message blank
    Return: None
    """
    global test_case_status
    test_case_status = TEST_CASE_STATUS(TestCaseStatus.passed, False, '')


def set_test_case_cleanup(is_test_case_cleanup):
    """
    Function: set_test_case_cleanup
    if test_step is test_case_cleanup, True, else False
    Parameters: None
    Output: test_case_status is assigned
    Return: None
    """

    global test_case_status
    test_case_status = TEST_CASE_STATUS(test_case_status.status,
                                        is_test_case_cleanup, test_case_status.info)


def find_root_dir():
    return os.path.join(os.path.dirname(os.path.realpath(__file__)))


class TimeoutException(Exception):
    """
    # TimeoutException Class
    """
    pass


def timeout_handler(signum, frame):
    """
    Function: timeout_handler
    This is a timeout handler function
    http://docs.python.org/2/library/signal.html
    Parameter:
    signum: the signal number
    frame: the current stack frame (None or a frame object)
    Return: None
    Output: raises exception
    """

    raise TimeoutException("ERROR: Device not responding!")


def set_timeout(timeout_value):
    """
    Function: set_timeout
    Sets the alarm using the timeout value provided
    Parameter:
    timeout_value: timeout value in seconds
    Return: None
    Output: sets alarm using the timeout_value
    """

    signal.signal(signal.SIGALRM, timeout_handler)
    # trigger alarm in "timeout_value" seconds
    signal.alarm(timeout_value)


def reset_timeout():
    """
    Function: reset_timeout
    resets the alarm
    Parameter: None
    Return: None
    Output: resets/disables alarm
    """

    signal.alarm(0)


"""
from automation_lib
"""


def end_auto_test_dict(test_dict):
    result_file_obj = test_dict['result_file_obj']
    start_time = test_dict['start_time']

    result_file_obj.close()

    end_time = time.asctime(time.localtime(time.time()))
    print (" ")
    print "Test completion time :%s [Start from: %s]" % (end_time, start_time)
    print (" ")
    print ("----")
    print ("Done")
    print ("----")
    print (" ")

    return end_time


def str_set_color(color, set_str):
    """
    Function: str_set_color
          the fun use to turn the color of str
    Parameters:
    color: have support  4 colors: red, green, yellow, blue
    str: the string needs to add color
    Return:
    col_str: the string have been added color
    Output: None
    """
    color_add = {'red': '31', 'green': '32', 'yellow': '33', 'blue': '34', 'amaranth': '35', 'cyanic': '36'}
    col_str = '\x1b[0;' + color_add[color] + 'm' + set_str + '\x1b[0m'
    return col_str


def get_abort_test_item_flag():
    """
    Function: get_abort_test_item_flag
    Returns ABORT_TEST_ITEM_FLAG
    Parameters: None
    Output: None
    Return: returns ABORT_TEST_ITEM_FLAG
    """
    return ABORT_TEST_ITEM_FLAG


def reset_abort_test_item_flag():
    """
    Function: reset_abort_test_item_flag
    resets ABORT_TEST_ITEM_FLAG to False
    Parameters: None
    Output: ABORT_TEST_ITEM_FLAG set to False
    Return: None
    """
    global ABORT_TEST_ITEM_FLAG
    ABORT_TEST_ITEM_FLAG = False
    return


def write_report(test_name, result, result_file_obj):
    """
    Function: write_report
    Writes report/result to .csv file
    Parameters:
    test_name: name of the test.
    result: PASSSED or FAILED
    result_file_obj: file obj of .csv file, the calling script must open
        the file first.
    Return: none
    Output:
    The .csv output file is updated with the test name and result
    """
    test_case_result = result.split(";")[0].strip()

    result_location = os.path.split(result_file_obj.name)[0]

    for case in switch(test_case_result):
        if case('PASSED'):
            print '%s:' % test_name + str_set_color('green', 'PASSED') + \
                  '%s\n' % result.replace(test_case_result, '')
            #remove all the temporary files if the test passes
            try:
                file_pattern = "%s/*%s*" % (result_location, test_name)
                for f in glob.glob(file_pattern):
                    if os.path.isdir(f):
                        # remove the directory
                        shutil.rmtree(f)
                    else:
                        if not f.endswith('.csv'):
                            os.remove(f)
            except OSError:
                pass

            break
        if case('FAILED'):
            print '%s:' % test_name + str_set_color('red', 'FAILED') + \
                  '%s\n' % result.replace(test_case_result, '')
            break
        if case('BLOCKED DEPENDENCY'):
            print '%s:' % test_name + \
                  str_set_color('yellow', 'BLOCKED DEPENDENCY') + \
                  '%s\n' % result.replace(test_case_result, '')
            break

    result_file_obj.write("%s,%s\n" % (test_name, result))
    result_file_obj.flush()
    return


def check_ignored_platforms(test_dict, test_case):
    """
    Function: check_ignored_platforms
    Checks for ignored platforms.  If ignored platform matches DUT,
    writes test result NA for test case and skips test.
    Parameters:
    test_dict: basic list of test data/parameters
    test_case: specific test case data/parameters
    Returns: True if test case should be skipped, else False
    """
    na_string = "NA\n"
    block_string = "BLOCKED DEPENDENCY\n"
    current_platform = test_dict['platform']
    result_file_obj = test_dict['result_file_obj']

    if test_case.get('ignore_platforms'):
        for platform in test_case.attrib['ignore_platforms'].split(','):
            if platform.strip() == current_platform:
                print("%s" % na_string)
                write_report(test_case.attrib['name'], na_string, result_file_obj)
                return True
    elif test_case.get('block_platforms'):
        for platform in test_case.attrib['block_platforms'].split(','):
            if platform.strip() == current_platform:
                print("%s" % block_string)
                write_report(test_case.attrib['name'], block_string, result_file_obj)
                return True

    return False


def get_full_build_no_device_id(deviceid):
    """
    Function: get_full_build_no
          the fun use to get full build number
    Parameters:
    Return: full build_no
    Output: None
    """

    if tc.DEVICE_ID_MASK == 0:
        adb_def = "adb"
    else:
        adb_def = "adb -s %s" % deviceid

    return commands.getoutput("%s shell getprop | grep ro.build.description" % adb_def)


def build_version_check_device_id(deviceid):
    """
    Function: build_version_check
    This fucntion use to check build version
    if build version is userdebug, promote adb permission as root

    Parameters: None
    Return: None
    Output: None
    """
    if tc.DEVICE_ID_MASK == 0:
        adb_def = "adb"
    else:
        adb_def = "adb -s %s" % deviceid

    if get_full_build_no_device_id(deviceid).count('userdebug') == 1:
        os.system("%s root" % adb_def)
        os.system("%s wait-for-device" % adb_def)


def check_file_device(file_name, adb_def):
    """
    Function check_file_phone
    This function checks if file exists on the phone
    Parameters:
    file_name: file name to be checked for existence
    adb_def: adb or adb -s device_id, taken from test_dict
    Return:
    True if exists, otherwise False
    """

    output_line = commands.getoutput("%s shell ls %s" % (adb_def, file_name))
    output_line = output_line.strip('\n')
    output_line = output_line.strip('\r')
    if output_line == file_name:
        return True
    else:
        return False


def check_file_host(file_name):
    """
    Function: check_file_host
    This function checks if file exists on the host machine
    Parameters:
    file_name: file name to be checked for existence
    Return:
    True if exists, otherwise False
    """

    if os.path.exists(file_name) is True:
        return True
    else:
        return False


def rm_file_host(file_name):
    """
    Function: remove_file_host
    This function remove file from the host machine, if
    the file exists
    Parameter:
    file_name: name of file to be removed
    Return:
    None

    """

    result = check_file_host(file_name)
    if result is True:
        os.remove(file_name)
    return


def read_fru_data(adb_def):
    """
    Function: read_fru_data(adb_def)

    This function read FRU data, return the entire file content
    Return:
    Fru data if successful
    None if fail
    """

    fru_file_name = "/sys/spid/fru"
    file_exist_flag = check_file_device(fru_file_name, adb_def)

    # If file does not exist, return None
    if not file_exist_flag:
        return None
    else:
        home_dir = os.getenv("HOME")
        test_output_dir = "%s/test_output" % home_dir
        host_fru_file_name = "%s/fru" % test_output_dir
        # pull fru descriptive file to host from device
        os_cmd = "%s pull %s %s" % (adb_def, fru_file_name, host_fru_file_name)
        os.system(os_cmd)
        # open fru descriptive file for reading
        fru_file = open(host_fru_file_name, 'r')
        fru_lines = fru_file.readlines()
        # close fru descriptive file
        fru_file.close()
        # remove fru descriptive file from host
        rm_file_host(host_fru_file_name)
        # return fru data
        return fru_lines


# def get_resource_product_prefix(product_prefix, product_prefix_common):
#     if product_prefix_common != '':
#         resource_product_prefix = product_prefix_common
#     else:
#         resource_product_prefix = product_prefix
#
#     return resource_product_prefix


def check_mofd_hdpi(adb_def):
    """
    Check if this is MOFD with high dpi

    """
    os_cmd = "%s shell dumpsys" % adb_def
    pipe = os.popen(os_cmd, "r")

    match_count = 0

    while True:
        line = pipe.readline()
        if not line:
            break
        else:
            # Both xdpi and ydpi must have the new HDPI values
            if ("x-dpi" in line) and (fc.MOFD_HDPI_XDPI in line):
                match_count += 1
            if ("y-dpi" in line) and (fc.MOFD_HDPI_YDPI in line):
                match_count += 1

    if match_count == 2:
        return True
    else:
        return False


def get_product_prefix_id(product_manifest_file_name, deviceid):
    """
    Function: get_product_prefix_id
    This function retrieves product info such as
    hardware platform, release version (based on API number)
    then returns product prefix, to be used in
    the tests.
    Product prefix is picked up from product manifest file.
    """

    if tc.DEVICE_ID_MASK == 0:
        adb_def = "adb"
    else:
        adb_def = "adb -s %s" % deviceid

    tree = ET()

    try:
        tree.parse(product_manifest_file_name)
    except:
        print ("    **** ERROR **** Failed to open product manifest file: %s" %
               product_manifest_file_name)

    product_root = tree.getroot()
    product_list = list(product_root)

    # Get API level
    adb_cmd = "%s shell getprop ro.build.version.sdk" % adb_def
    api_level = commands.getoutput(adb_cmd)

    # Get Platform/Hardware info
    adb_cmd = "%s shell getprop ro.hardware" % adb_def
    platform = commands.getoutput(adb_cmd)

    platform_string = "Platform: %s" % platform
    print str_set_color('green', platform_string)

    adb_cmd = "%s shell getprop | grep ro.build.version.release | awk '{print $2}'" % adb_def
    release = commands.getoutput(adb_cmd)
    release = release.replace('[', '').replace(']', '').strip()

    adb_cmd = "%s shell getprop ro.build.version.incremental" % adb_def
    build_id = commands.getoutput(adb_cmd)
    build_id = build_id.replace('-', '_')
    build_id = build_id.strip('\n')
    build_id = build_id.strip('\r')

    platform = platform.strip('\n')
    platform = platform.strip('\r')
    api_level = api_level.strip('\n')
    api_level = api_level.strip('\r')
    match_flag = 0
    product_prefix = "XXXX"
    product_prefix_common = ''
    manifest_display_width = 0
    manifest_display_height = 0
    manifest_virtual_core_count = 2

    manifest_platform = ''
    manifest_release = ''
    manifest_api_level = ''
    manifest_prefix = ''
    manifest_display_width = 0
    manifest_display_height = 0
    manifest_capture_image = ''
    manifest_virtual_core_count = ''
    platform_prefix = ''

    fru_list_index = 0
    manifest_fru_list_index = 0

    for product in product_list:
        for product_item in product:
            if product_item.tag == "product_item":
                for product_detail in product_item:
                    for parameter in product_detail:
                        if parameter.attrib['name'] == 'platform':
                            manifest_platform = parameter.text
                            platform_prefix = manifest_platform
                        if parameter.attrib['name'] == 'release':
                            manifest_release = parameter.text
                        if parameter.attrib['name'] == 'api_level':
                            manifest_api_level = parameter.text
                        if parameter.attrib['name'] == 'prefix':
                            manifest_prefix = parameter.text
                        if parameter.attrib['name'] == 'display_width':
                            manifest_display_width = int(parameter.text)
                        if parameter.attrib['name'] == 'display_height':
                            manifest_display_height = int(parameter.text)
                        if parameter.attrib['name'] == 'capture_image':
                            manifest_capture_image = parameter.text
                        if parameter.attrib['name'] == 'virtual_core_count':
                            manifest_virtual_core_count = parameter.text
                        if parameter.attrib['name'] == 'fru_list_index':
                            manifest_fru_list_index = int(parameter.text)
                            fru_list_index = manifest_fru_list_index

                    if ((manifest_platform == platform) and
                            (manifest_api_level == api_level)):

                        # Special case for now
                        # If hardware = merr_vv, there are 2 possibilities in the
                        # product manifest, based on release number: 4.2.1 and 4.2.2
                        # Therefore, check if the release number matches the release
                        # entry in the manifest.
                        # prefix can be merr_vv_jbmr1 or merr_pr0_jbmr1.1
                        if platform == "merr_vv":
                            if manifest_release == release:
                                product_prefix = manifest_prefix
                                match_flag = 1
                                break
                        elif platform == "byt_t_ffrd10":

                            os_cmd = "adb shell dumpsys"
                            pipe = os.popen(os_cmd)

                            byt_manu_width_flag = 0
                            byt_manu_height_flag = 0

                            while True:
                                device_info = pipe.readline()
                                if not device_info:
                                    break
                                if 'mDefaultViewport' in device_info:
                                    dimension_params = device_info.split(',')

                                    for dimension_param in dimension_params:
                                        if 'deviceWidth' in dimension_param:
                                            width_str = dimension_param.split('=')
                                            width_param = width_str[1]
                                            if '768' in width_param:
                                                byt_manu_width_flag = 1
                                                manifest_display_width = 768

                                        if 'deviceHeight' in dimension_param:
                                            dimension_param = dimension_param.strip('\n')
                                            dimension_param = dimension_param.strip('\r')
                                            dimension_param = dimension_param.strip('}')
                                            height_str = dimension_param.split('=')
                                            height_param = height_str[1]
                                            if '1024' in height_param:
                                                byt_manu_height_flag = 1
                                                manifest_display_height = 1024
                                    break

                            if (byt_manu_width_flag == 1) and (byt_manu_height_flag == 1):
                                print "BYT_MANU board found"
                                product_prefix = "byt_manu_jbmr1.1"
                                match_flag = 1
                                break
                            else:
                                product_prefix = manifest_prefix
                                match_flag = 1
                                break

                        # If hardware = mofd_*, decode FRU data
                        # to get product_prefix
                        elif "mofd_" in platform:
                            # Read FRU data

                            read_result = read_fru_data(adb_def)

                            if read_result is None:
                                print (" **** ERROR **** Failed to read FRU data")
                                print (" Unable to determine display type")
                                break
                            else:
                                # Decode FRU
                                # Lots of error checking to make sure
                                # we get correct FRU data
                                fru_data_count = len(read_result)
                                if fru_data_count != fc.FRU_DATA_COUNT:
                                    print (" **** ERROR **** FRU data count is wrong")
                                    print (" Unable to determine display type")
                                    break
                                else:
                                    fru = read_result[0]
                                    fru = fru.strip('\n')
                                    fru = fru.strip('\r')
                                    fru_digit = len(fru)
                                    if fru_digit != fc.FRU_DIGIT_COUNT:
                                        print (" **** ERROR **** FRU digit count is wrong")
                                        print (" Unable to determine display type")
                                        break
                                    else:
                                        display_data = fru[fc.FRU_DISPLAY_INDEX]
                                        for fru_data in fc.fru_list:
                                            if fru_data.display_id == display_data:
                                                match_flag = 1
                                                product_prefix = fru_data.product_prefix
                                                print (" Display: %s" % fru_data.description)
                                                product_prefix_common = fru_data.product_prefix_common
                                                manifest_display_width = fru_data.width
                                                manifest_display_height = fru_data.height
                                                break
                                        if match_flag == 1:
                                            # generate unique platform
                                            # prefix based on platform and
                                            # resolution
                                            prefix_words = \
                                                product_prefix.split("_")
                                            platform_prefix = "%s_%s" % \
                                                              (prefix_words[0],
                                                               prefix_words[1].split("-")[0])
                                        break

                        else:
                            product_prefix = manifest_prefix
                            match_flag = 1
                            break

                if match_flag == 1:
                    break
            if match_flag == 1:
                break
        if match_flag == 1:
            break

    # resource_product_prefix = get_resource_product_prefix(product_prefix, product_prefix_common)

    # Handle special case
    if product_prefix == "mofd_10x19-cmdmode_kk":
        mofd_hdpi_flag = check_mofd_hdpi(adb_def)
        # Override if MOFD with hdpi
        if mofd_hdpi_flag:
            product_prefix = "mofd_10x19hdpi-cmdmode_kk"
            resource_product_prefix = "mofd_10x19hdpi-common_kk"
            # generate unique platform prefix based on platform and resolution
            prefix_words = product_prefix.split("_")
            platform_prefix = "%s_%s" % (prefix_words[0],
                                         prefix_words[1].split("-")[0])

    print (" Final product prefix: %s" % product_prefix)
    # print (" Final resource prefix: %s" % resource_product_prefix)

    product_info = {
        'product_prefix': product_prefix,
        'product_prefix_common': product_prefix_common,
        # 'resource_product_prefix': resource_product_prefix,
        'display_width': manifest_display_width,
        'display_height': manifest_display_height,
        'build_id': build_id,
        'capture_image_func': manifest_capture_image,
        'virtual_core_count': manifest_virtual_core_count,
        # 'platform': manifest_platform,
        'platform': platform_string,
        'platform_prefix': platform_prefix,
    }

    if match_flag == 0:
        print ("\n **** ERROR **** Product not recognized\n")

    return product_info


def open_file_with_backup(file_name, mode):
    """
    Function: open_file_with_backup(file_name, mode)
    This function opens file for writing,
    before it opens the file, it checks if
    it already exists, if the file already exists,
    it creates a backup copy first.
    Parameter:
    file_name: Name of file to be opened
    mode: read or write mode
    Return:
    file_obj

    """

    if os.path.exists(file_name):
        file_size = os.path.getsize(file_name)
        if file_size > 0:
            date_format = "%Y%m%d%H%M%S"
            extension = os.path.splitext(file_name)[1][1:].strip()
            file_suffix = "%s.%s" % ((datetime.datetime.now().strftime(date_format)), extension)
            backup_file_name = file_name.replace(extension, file_suffix)
            shutil.copyfile(file_name, backup_file_name)
            print (" File: %s exists, creating a backup file first" % file_name)
            print (" Backup file name: %s" % backup_file_name)

    file_obj = open(file_name, mode)
    return file_obj


def adb_exec(adb_cmd_string):
    """ Executes the ADB command while logging the command and its output. """
    output = commands.getoutput(adb_cmd_string)
    logging.debug("%s - %s" % (adb_cmd_string, output))
    return output


def screen_timeout_check_device_id(expect, deviceid):
    """
    Function: screen_timeout_check_device_id
          the fun use check the screen off same as expect
    Parameters:expect: '15s','30s','1m','2m','5m','10m','30m'
    Return: 0: unexpect; 1: expect
    Output: None
    """

    if tc.DEVICE_ID_MASK == 0:
        adb_def = "adb"
    else:
        adb_def = "adb -s %s" % deviceid

    set_times = screen_off_timeout_times[expect]
    command = '''%s shell "echo 'select * from system;' | sqlite3
    data/data/com.android.providers.settings/databases/settings.db" | egrep "screen_off_timeout|set_times"''' % adb_def
    result = commands.getoutput(command)
    if int(result.count('%s' % set_times)) != 1:
        return False
    else:
        return True


def input_keyevent_device_id(event_key, deviceid):
    """
    Function: input_keyevent_device_id
          the fun use to execution adb input keyevent cmd
    Parameters:
    event_key: the key stay in input_keyevent
    device_id: deivce id
    Return: None
    Output: None
    """

    adb_def = "adb -s %s" % deviceid

    keyevent_cmd = input_keyevent[event_key]
    keyevent_cmd = keyevent_cmd.replace("adb", adb_def)
    os.system(keyevent_cmd)


def set_screen_timeout_device_id(value, deviceid):
    """
    Function: set_screen_off_timeout
    the fun use to set screen off timeout
    Parameters: value:'15s','30s','1m','2m','5m','10m','30m'
    Return: True: set ok; False: set failed
    Output: None
    """

    if tc.DEVICE_ID_MASK == 0:
        adb_def = "adb"
    else:
        adb_def = "adb -s %s" % deviceid

    result = screen_timeout_check_device_id(value, deviceid)
    if not result:
        commands.getstatusoutput('%s shell am start -W -a android.settings.DISPLAY_SETTINGS' % adb_def)

        # Sleep (screen timeout) is the third item on tablet (byt and mofd_25x16). it is the fourth item on phones.
        # it is the fourth item for all platforms if it is counted from the bottom
        input_keyevent_device_id('KEYCODE_MOVE_HOME', adb_def)
        # Navigate to screen timeout
        for i in range(1, 7):
            input_keyevent_device_id('KEYEVENT_DOWN', adb_def)
        for i in range(1, 4):
            input_keyevent_device_id('KEYEVENT_UP', adb_def)
        input_keyevent_device_id('KEYEVENT_ENTER', adb_def)

        input_keyevent_device_id('KEYCODE_MOVE_HOME', adb_def)
        # Select the appropriate screen timeout value
        for i in range(1, screen_off_timeout_times_adjust_offset[value]):
            input_keyevent_device_id('KEYEVENT_DOWN', adb_def)
        input_keyevent_device_id('KEYEVENT_ENTER', adb_def)
        input_keyevent_device_id('KEYEVENT_BACK', adb_def)

    if screen_timeout_check_device_id(value, deviceid) is True or result is True:
        return True
    else:
        return False


def get_build_version_release_device_id(deviceid):
    """
    Function: get_build_version_release_device_id
          the fun use to get build_version(eg:JB:4.1.1)
    Parameters:
    Return: build_version
    Output: None
    """

    if tc.DEVICE_ID_MASK == 0:
        adb_def = "adb"
    else:
        adb_def = "adb -s %s" % deviceid

    adb_cmd = "%s shell getprop | grep ro.build.version.release | awk '{print $2}'" % adb_def
    result = commands.getoutput(adb_cmd)
    return result.replace('[', '').replace(']', '').strip()


def stay_awake_state_check_device_id(expect, test_dict):
    """
    Function: stay_awake_state_check
          the fun use to check stay awake state
    Parameters: expect: disable ,enable
    Return: 0:unexpect, 1:expect
    Output: None
    """

    adb_def = test_dict['adb_def']
    deviceid = test_dict['deviceid']

    sys_version = get_build_version_release_device_id(deviceid).split('.')
    print "\n\n"
    print sys_version[0]
    print sys_version[0], sys_version[1]
    print type(int(sys_version[0]))
    if (int(sys_version[0]) + int(sys_version[1]) / 10.0) > 4.1:
        command = '''%s shell "echo 'select * from global;' | sqlite3
        data/data/com.android.providers.settings/databases/settings.db | grep 'stay_on_while_plugged_in'"''' % adb_def
    else:
        command = '''%s shell "echo 'select * from system;' | sqlite3
        data/data/com.android.providers.settings/databases/settings.db | grep 'stay_on_while_plugged_in'"''' % adb_def

    result = commands.getoutput(command)
    if expect == 'disable':
        if int(result.count('stay_on_while_plugged_in|0')) != 1:
            return 0
        else:
            return 1
    elif expect == 'enable':
        if int(result.count('stay_on_while_plugged_in|1')) == 1:
            return 0
        else:
            return 1
    else:
        return 1


# def setting_stay_awake_device_id(value, test_dict):
#     """
#     Function: setting_stay_awake
#           the fun use to set stay awake state
#     Parameters: value: disable ,enable
#     Return: True:set ok, False:set failed
#     Output: None
#     """
#
#     adb_def = test_dict['adb_def']
#
#     result = stay_awake_state_check_device_id(value, test_dict)
#     if result == 0:
#         # launch 'Developer options'
#         commands.getoutput('%s shell am start -W -n com.android.settings/.DevelopmentSettings' % adb_def)
#         input_keyevent_device_id('KEYCODE_MOVE_HOME', adb_def)
#         input_keyevent_device_id('KEYEVENT_DOWN', adb_def)
#         input_keyevent_device_id('KEYEVENT_DOWN', adb_def)
#         # enable/disable 'Stay awake'
#         input_keyevent_device_id('KEYEVENT_ENTER', adb_def)
#         input_keyevent_device_id('KEYEVENT_BACK', adb_def)
#         input_keyevent_device_id('KEYEVENT_BACK', adb_def)
#
#     if stay_awake_state_check_device_id(value, test_dict) == 1 or result == 1:
#         return True
#     else:
#         return False


def setup_common_test_dict(output_file_name, manifest_file_name, data_dict):
    """
    Function: setup_common_test_dict
    In the setup section, all directories are setup first.
    result_dir will always be $HOME/test_output regardless of test.
    tool_dir will always be tool directory, it is a relative directory location,
    not absolute path.
    Parameter:
    output_file_name:   This is the .csv file that lists the test name and execution result
    reference_dir:      directory with reference files
    manifest_file_name: xml file with test execution instructions
    data_dict:          data dictionary object containing device_id, batch_file_obj,\
                        cpu_frequency_set, vblank_set, relay_port_id, tool_dir
    Return:
    test_dict:          setup_dictionary
    """
    deviceid = data_dict['deviceid']

    if tc.DEVICE_ID_MASK == 0:
        adb_def = "adb"
        adb_shell = "adb shell"
    else:
        adb_def = "adb -s %s" % deviceid
        adb_shell = "adb -s %s shell" % deviceid

    # Time stamp is not really used this time.
    # It will be useful to implement watchdog function in the future.
    start_time = time.asctime(time.localtime(time.time()))
    print ("\n")
    print "Test start time : ", start_time
    print ("\n")

    #if build version is userdebug, give the higher permission
    build_version_check_device_id(deviceid)

    # if /dev/fb0 does not exist, create symbolic link
    result = check_file_device("/dev/fb0", adb_def)
    if not result:
        adb_cmd = "%s shell ln -s /dev/graphics/fb0 /dev/fb0" % adb_def
        os.system(adb_cmd)

    # If /dev/dri/card0 does not exist, create symbolic link.
    result = check_file_device("/dev/dri/card0", adb_def)
    if not result:
        adb_cmd = "%s shell ln -s /dev/card0 /dev/dri/card0" % adb_def
        os.system(adb_cmd)

    home_dir = os.getenv("HOME")
    result_dir = "%s/test_output" % home_dir

    current_path = str(os.getcwd())
    script_path_list = current_path.split("/")
    remove_str = script_path_list[len(script_path_list)-2] + "/" + script_path_list[len(script_path_list)-1]
    replace_str = "acs_test_suites/OTC/TC/ACS//Graphics/Graphics/scripts"
    path_to_file = current_path.replace(remove_str, replace_str)
    test_dir = path_to_file

    if not os.path.isdir(test_dir):
        print "The path you have provided for the test scripts does not exist. Please revise it in " \
              "setup_common_test_dict() in GFXFunctions."
    else:
        product_manifest_file_name = "%s/product_manifest.xml" % test_dir
        full_manifest_file_name = test_dir + "/" + manifest_file_name
        product_info = get_product_prefix_id(product_manifest_file_name, deviceid)

    # product_prefix = product_info['product_prefix']
    product_prefix_common = product_info['product_prefix_common']
    # resource_product_prefix = product_info['resource_product_prefix']
    display_width = product_info['display_width']
    display_height = product_info['display_height']
    build_id = product_info['build_id']
    capture_image_func = product_info['capture_image_func']
    virtual_core_count = product_info['virtual_core_count']
    platform = product_info['platform']
    platform_prefix = product_info['platform_prefix']

    # if ("XXXX" in product_prefix) or ("XXXX" in resource_product_prefix):
    #     print ("\n **** ERROR **** Product not recognized or resource not available")
    #     print ("\n Test aborted")

    # If result_dir does not exist, create one.
    if not os.path.exists(result_dir):
        os.makedirs(result_dir)

    # complete_prefix = "%s_%s_%s" % (product_prefix, deviceid, build_id)
    complete_prefix = "%s_%s" % (deviceid, build_id)
    full_output_file_name = "%s_%s" % (complete_prefix, output_file_name)
    output_file_name2 = "%s/%s" % (result_dir, full_output_file_name)
    result_file_obj = open_file_with_backup(output_file_name2, "w")

    min_cpu_freq = 0.5
    max_cpu_freq = 1.0

    cpu0_minfreq_filename = "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq"
    cpu0_maxfreq_filename = "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
    if check_file_device(cpu0_minfreq_filename, adb_def):
        min_cpu_rate = commands.getoutput("%s shell 'cat %s'" % (adb_def, cpu0_minfreq_filename))
        min_cpu_freq = float(min_cpu_rate) / 1000000

    if check_file_device(cpu0_maxfreq_filename, adb_def):
        max_cpu_rate = commands.getoutput("%s shell 'cat %s'" % (adb_def, cpu0_maxfreq_filename))
        max_cpu_freq = float(max_cpu_rate) / 1000000

    test_result = "NOT_YET_RUN\n"
    test_comment = ''

    # build setup_dict
    test_dict = {
        'deviceid': deviceid,
        'manifest_file_name': full_manifest_file_name,
        'result_dir': result_dir,
        'result_file_name': full_output_file_name,
        'adb_def': adb_def,
        'adb_shell': adb_shell,
        'result_file_obj': result_file_obj,
        # 'product_prefix': product_prefix,
        'product_prefix_common': product_prefix_common,
        # 'resource_product_prefix': resource_product_prefix,
        'start_time': start_time,
        'display_width': display_width,
        'display_height': display_height,
        'complete_prefix': complete_prefix,
        'test_dir': test_dir,
        'virtual_core_count': virtual_core_count,
        'max_cpu_freq': max_cpu_freq,
        'platform': platform,
        'platform_prefix': platform_prefix,
        'test_result': test_result,
        'test_comment': test_comment,
    }

    # check_and_recover_screen(test_dict)
    set_screen_timeout_device_id("30m", deviceid)
    # setting_stay_awake_device_id("enable", test_dict)

    return test_dict, output_file_name2


def get_all_prefix(test_dict):
    # product_prefix = test_dict['product_prefix']
    product_prefix_common = test_dict['product_prefix_common']
    # resource_product_prefix = test_dict['resource_product_prefix']
    # return product_prefix, product_prefix_common, resource_product_prefix
    return product_prefix_common

def check_result(paramlist, test_data):
    """
    Function: check_result
    read the output files and parse to see if they contain
    the keywords for pass or fail.
    Parameters:
    paramlist: contains the pass keyword, fail keyword,
               and how many pass keywords are needed
               to deem the test as a pass.
               also contains if the result file resides
               on host or device.
    test_data: generic structure that contains the test
               information.
    """

    result_dir = test_data['result_dir']
    complete_prefix = test_data['complete_prefix']
    test_name = test_data['test_name']
    result_file_obj = test_data['result_file_obj']
    device_result_file = ''

    adb_def = test_data['adb_def']
    adb_pull = "%s pull " % adb_def

    result_struct = {
        'host_result': 'no',
        'result_file': '',
        'pass_string': '',
        'pass_string_count': 1,
        'fail_string': ''
    }

    for param in paramlist:
        if param.attrib['name'] == 'result_file':
            result_struct['result_file'] = "%s/%s_%s" % (result_dir, complete_prefix, param.text)
            device_result_file = param.text
        elif param.attrib['name'] == 'pass_string':
            result_struct['pass_string'] = param.text
        elif param.attrib['name'] == 'pass_string_count':
            result_struct['pass_string_count'] = int(param.text)
        elif param.attrib['name'] == 'host_result':
            result_struct['host_result'] = param.text
        elif param.attrib['name'] == 'fail_string':
            result_struct['fail_string'] = param.text
        else:
            print "Warning: Unrecognized Tag: %s" % param.attrib['name']

    if result_struct['result_file'] != '':
        if result_struct['host_result'] == 'yes':
            #if the result file is on the host, directly open it
            result_file = open(result_struct['result_file'], 'r')
        else:
            #pull the file first:
            pulled_file = '%s/%s_%s' % (result_dir, complete_prefix, test_name + '.log')
            tmp_cmd = '%s %s %s' % (adb_pull, device_result_file, pulled_file)
            os.system(tmp_cmd)
            result_file = open(pulled_file, 'r')
    else:
        print 'Error: result file undeclared.\n'
        return

    pass_count = 0
    fail_count = 0
    for line in result_file:
        if result_struct['pass_string'] != '':
            if result_struct['pass_string'] in line:
                pass_count += 1
            if result_struct['fail_string'] != '':
                if result_struct['fail_string'] in line:
                    fail_count += 1
        elif result_struct['fail_string'] != '':
            if result_struct['fail_string'] != '':
                fail_count += 1
    if pass_count >= result_struct['pass_string_count'] and fail_count == 0:
        print test_data['test_name'] + ' test ' + str_set_color('green', 'passed!') + '\n'
        write_report(test_data['test_name'], 'PASSED\n', test_data['result_file_obj'])
    else:
        print test_data['test_name'] + ' test ' + str_set_color('red', 'failed!') + '\n'
        write_report(test_data['test_name'], 'FAILED\n', test_data['result_file_obj'])

    result_file.close()
    return


def test_sequence_handler(paramlist, test_data):
    """
    Function: test_sequence_handler
    This is a "universal" low level function that is the
    basis of test automation test sequence, based on
    pre-determined set of test syntax.
    New syntax can be added without affecting previous syntax.
    """

    result = "NOT_YET_RUN\n"
    pass_string = "PASSED\n"
    fail_string = "FAILED\n"
    result_file_obj = test_data['result_file_obj']
    test_name = test_data['test_name']
    test_case_name = test_data['test_case_name']
    result_file_obj = test_data['result_file_obj']
    result_dir = test_data['result_dir']
    device_id = test_data['deviceid']
    adb_def = test_data['adb_def']
    adb_shell = test_data['adb_shell']


    # product_prefix, product_prefix_common, resource_product_prefix = get_all_prefix(test_data)

    complete_prefix = test_data['complete_prefix']
    test_dir = test_data['test_dir']
    display_width = test_data['display_width']
    display_height = test_data['display_height']

    home_dir = os.getenv("HOME")
    test_output_dir = "%s/test_output" % home_dir

    for param in paramlist:
        logging.debug("TOKEN %s [%s]" % (param.attrib['name'], param.text))
        if param.attrib['name'] == 'adb_cmd':
            cmd = "%s\n" % param.text
            cmd = cmd.replace("adb", adb_def)
            os.system(cmd)
        elif param.attrib['name'] == 'deprecated':
            comment = param.text
            result = "NA"
            if comment is not None:
                result += ";%s" % comment
            result += '\n'
            write_report(test_name, result, result_file_obj)

        # if ABORT_TEST_ITEM_FLAG flag or ABORT_TEST_CASE_ITEM_FLAG is
        # set, abort
        if (get_abort_test_item_flag() or
                (get_test_case_status().status != TestCaseStatus.passed and
                     not get_test_case_status().is_test_case_cleanup)):
            break

    return

def check_result_output(logfile, check_string):
    # Check if the test passed or failed
    output_fail = commands.getoutput("cat %s | grep '%s' | grep -i 'failed'" % (logfile, check_string))
    output_pass = commands.getoutput("cat %s | grep '%s' | grep -i 'passed'" % (logfile, check_string))

    # Update result
    if len(output_fail) == 0 and len(output_pass) != 0:
        result = 'PASSED\n'
    elif len(output_fail) != 0 or len(output_pass) == 0:
        result = 'FAILED\n'
    return result


def ogles_conformance_test_run(paramlist, test_data):
    """
    Function: ogles_conformance_test_run
    Parameters: paramlist: C style parameter parsing using getopt
            test_data: test_data dictionary
    Return: none
    Output: It calls write_report function to write to .csv file
        the test name and result.

    """

    result_file_obj = test_data['result_file_obj']
    result_dir = test_data['result_dir']
    adb_def = test_data['adb_def']
    adb_shell = test_data['adb_shell']
    test_name = test_data['test_name']
    # product_prefix = test_data['product_prefix']
    complete_prefix = test_data['complete_prefix']
    # initialize
    result = 'NOT YET RUN\n'
    test_file_name = ''
    test_bin_name = ''
    test_bin_folder = ''
    path_level = 0
    extension_type = 'es1'
    # random seed
    random_seed = 32555

    # Generate temp log file with unique product_prefix, device_id, build_id, test_binary_name
    temp_log_file = result_dir + '/' + complete_prefix + '/' + "'" + test_name + "'" + '.log'
    # If result_dir does not exist, create one.
    if not os.path.exists(result_dir + '/' + complete_prefix):
        os.makedirs(result_dir + '/' + complete_prefix)

    cmd = 'date > %s' % temp_log_file
    os.system(cmd)

    for param in paramlist:
        if param.attrib['name'] == 'test_file_name':
            test_file_name = param.text
        elif param.attrib['name'] == 'test_bin_name':
            line = param.text
            words = line.split(';')
            test_bin_name = words[0]
            test_bin_folder = words[1]
        elif param.attrib['name'] == "log_param":
            log_option = param.text
            run_gtf_binary()
        elif param.attrib['name'] == 'path_0':
            path_level = 0
            run_native_binary()
        elif param.attrib['name'] == 'path_1':
            path_level = 1
            random_seed += 1
            run_native_binary()
        elif param.attrib['name'] == 'path_2':
            path_level = 2
            random_seed += 1
            run_native_binary()
        elif param.attrib['name'] == 'path_3':
            path_level = 3
            random_seed += 1
            run_native_binary()
        elif param.attrib['name'] == 'run_ext_es1':
            extension_type = 'es1'
            run_extension_test()
        elif param.attrib['name'] == 'run_ext_es2':
            extension_type = 'es2'
            run_extension_test()
        elif param.attrib['name'] == 'run_ext_egl':
            extension_type = 'egl'
            run_extension_test()
        elif param.attrib['name'] == 'result_check_string':
            test_name_string = param.text
            result = check_result_output(temp_log_file, test_name_string)
            write_report(test_name, result, result_file_obj)
        else:
            print "Warning: Unrecognized Tag: %s" % param.attrib['name']

        def run_native_binary():
            adb_cmd = "%s %s/%s -r %d -1 %s -p %d | tee --append %s" % (adb_shell, test_bin_folder,
                                                                        test_bin_name, random_seed, test_file_name,
                                                                        path_level, temp_log_file)
            os.system(adb_cmd)

        def run_extension_test():
            adb_cmd = "%s %s/%s -%s -test %s | tee --append %s" % (
                adb_shell, test_bin_folder, test_bin_name, extension_type, test_file_name, temp_log_file)
            os.system(adb_cmd)

        def run_gtf_binary():
            adb_cmd = "%s %s/%s -l=%s -seed=%d -run=%s/%s | tee --append %s" % (adb_shell, test_bin_folder, test_bin_name, log_option, random_seed, test_bin_folder, test_file_name, temp_log_file)
            os.system(adb_cmd)



"""
from test_manifest_lib
"""


def is_ignored_test_step(current_platform, test_step):
    """
    Function: is_ignored_test_step
    Checks for ignored platforms for specific test_step in XML. If test step
    is to be ignored, True is returned, else False.
    Parameters:
    current_platform: DUT platform
    test_case: specific test case data/parameters
    Returns: True if test step should be skipped, else False
    """

    # check for ignored platforms
    if test_step.get('only_platforms') is not None:
        for platform in test_step.attrib['only_platforms'].split(','):
            if platform.strip() == current_platform:
                return False
        return True

    return False


def parse_test_list(test_case):
    """
    Function: parese_test_list
    It parse a selected test. The primary function of this code
    is to get test_id, and test_name, both are strings.
    Parameters:
    test_case: the selected test (from manifest), this is basically
           manifest data.
    Return: test_number, and test_name
    Output: None
    """

    test_number = test_case.attrib['id']
    test_name = test_case.attrib['name']

    return test_number, test_name


def XMLCombiner(base, extra):
    """
    Function: XMLCombiner
    It return a list that contains test_cases in both "base" and "extra" files
    If there is a test_case in "extra" that has the same name as a test_case in "base"
    only the first test_case will be put into the list, not the second one
    Parameters:
    base: the manifest file name
    extra: the manifest file name with some product prefix
    Return: test_root, and combined test_list
    Output: None
    """

    tree = ET()
    test_list = []
    extra_test = []
    try:
        tree.parse(base)
        test_root = tree.getroot()
        test_list = list(test_root)
    except Exception, ex:
        print ("\n **** Notice **** Common test manifest file is not available: %s" % base)
        print (" Will use prefixed test manifest file if available: %s\n" % base)

    try:
        tree.parse(extra)
        test_root = tree.getroot()
        extra_test = list(test_root)
    except Exception, ex:
        print ("\n **** Notice **** Device specific test manifest file is not available: %s" % extra)
        print (" Will use common test manifest file if available: %s\n" % base)

    # if there is another manifest file named starts from the current product prefix,
    # read the tests out, add into "base" or replace the tests in "base" if the tests are
    # dups in "base"
    if len(extra_test) > 0:
        test_list0 = test_list
        for test_ecase in extra_test:
            flag = 0
            name1 = test_ecase.attrib['name']
            for n, test_bcase in enumerate(test_list0):
                name2 = test_bcase.attrib['name']
                if test_bcase.attrib['name'] == test_ecase.attrib['name']:
                    test_list[n] = test_ecase
                    flag = 1
                    break
            if flag == 0:
                test_list.append(test_ecase)

    if not test_list:
        return None, test_list
    else:
        return test_root, test_list


def run_test_item_dict(test_dict, test_item):
    """
    Function run_test_item_dict
    This function host a single test item to test_stepdef run_test_item(test_data):
    """

    adb_def = test_dict['adb_def']
    current_platform = test_dict['platform']

    for test_step in test_item:
        if test_step.tag != 'test_step':
            print 'tag is not supported!'

        # if test case failed, exit test case
        if (((get_test_case_status().status != TestCaseStatus.passed) and
                 ((test_item.get('name') is None) or
                      (test_item.attrib['name'] != 'test_case_cleanup'))) or
                (is_ignored_test_step(current_platform, test_step))):
            continue

        # for command type step, issue the command directly
        if test_step.attrib['type'] == 'command':
            #TODO: need more error handling here
            for cmd in test_step:
                adb_cmd = cmd.text
                adb_cmd = adb_cmd.replace("adb", adb_def)
                os.system(adb_cmd)
        # for function type step, call the function with parameter list
        if test_step.attrib['type'] == 'fun':
            #A fast way to concatenate parameter list
            #test_step is a list of parameters
            globals()[test_step.attrib['fun_name']](test_step, test_dict)

        # if ABORT_TEST_ITEM_FLAG flag is set, abort
        if get_abort_test_item_flag():
            reset_abort_test_item_flag()
            break


def execute_test_dict(test_case, test_dict):
    """
    Function: execute_test_dict
    First, it checks if the test_selected is valid.
    If it is valid, it will parse the manifest file
    When it finds the test in the manifest,
    it will do the needed preparations, such as installing app, or file,
    finally it will dispatch the test cmd.
    Parameters:
    test_selected: The test_number selected. This is integer.
    last_test_number: The last number in the manifest. This is used for
        boundary checking.
    test_dict:
        This is basically the record of the test data/parameters
    Return: None
    Output: Depends on the type of test.
    """
    deviceid = test_dict['deviceid']
    manifest_file_name = test_dict['manifest_file_name']
    result_dir = test_dict['result_dir']
    # tool_dir = test_dict['tool_dir']
    result_file_obj = test_dict['result_file_obj']
    adb_def = test_dict['adb_def']
    # product_prefix = test_dict['product_prefix']

    test_number, test_name = parse_test_list(test_case)

    print ("   Executing test: %d: %s" % (int(test_number), test_name))

    # check for ignored platform test
    if check_ignored_platforms(test_dict, test_case):
        return

    # check for flag to log result at end of test case
    log_result = False
    if test_case.get('log_result'):
        if test_case.attrib['log_result'] == 'true':
            log_result = True

    # clear logcat
    os.system("%s logcat -c" % adb_def)

    for item in test_case:

        # if test case failed, exit test case or
        # go to test_case_cleanup test_item
        if ((get_test_case_status().status != TestCaseStatus.passed) and
                ((item.tag != 'test_item') or
                     (item.get('name') is None) or
                     (item.attrib['name'] != 'test_case_cleanup'))):
            continue
        elif ((item.get('name') is not None) and
                  (item.attrib['name'] == 'test_case_cleanup')):
            set_test_case_cleanup(True)
                # for sub tests, if there is only one single test, then use 1 item
        elif item.tag == 'test_item':
            if item.attrib['name'] == '' or item.attrib['name'] == 'test_case_cleanup':
                test_dict['test_name'] = test_name
            else:
                test_dict['test_name'] = test_name + '_' + item.attrib['name']

            test_dict['test_item'] = item
            test_dict['test_case_name'] = test_name
            run_test_item_dict(test_dict, item)
        else:
            print "ERROR: Wrong Manifest Format! + %s \n" % item.tag

    # if test case requests to log result at end of test case,
    # log result
    if log_result:
        write_report(test_name, '%s;%s\n' % (TestCaseStatus.tostring(
            get_test_case_status().status), get_test_case_status().info), result_file_obj)

    reset_test_case_status()

    return


def automation_dict_execution_handler(test_dict, exit_after=True):
    """
    Function: automation_dict_execution_handler
    Parameters:
    test_dict: the needed test data
    exit_after: OPTIONAL - enable/disable exiting in this function
    ex. if a caller needs to cleanup before exit
    """
    deviceid = test_dict['deviceid']
    manifest_file_name = test_dict['manifest_file_name']
    result_dir = test_dict['result_dir']
    result_file_obj = test_dict['result_file_obj']

    # product_prefix = test_dict['product_prefix']
    # resource_product_prefix = test_dict['resource_product_prefix']

    # test_root, test_list = XMLCombiner(manifest_file_name, resource_product_prefix + '_' + manifest_file_name)
    test_root, test_list = XMLCombiner(manifest_file_name, '-' + '_' + manifest_file_name)
    # Abort test if unable to get test list from test manifest file
    if not test_list:
        print ("\n **** ERROR **** Test aborted, unable to get test list from test manifest file\n")
        sys.exit(tc.TEST_ERROR)

    test_dict['test_suite'] = test_root.tag

    print ("    Running full automation mode")
    for test_case in test_list:

        try:
            execute_test_dict(test_case, test_dict)
        except KeyboardInterrupt:
            print ("    **** Test interrupted ****")
            break

    end_auto_test_dict(test_dict)
    return


def device_id_check(deviceid):
    """
    Function: device_id_check
    Description:
    Do error checking here
    Count devices connected.
    Read device_id from the device

    Decision making table

    --------------------------------------------------------------------
    | device count | device_id passed | device_id read | Decision      |
    |--------------|------------------|----------------|---------------|
    |      0       |    don't care    |    don't care  | Abort (1)     |
    |--------------|------------------|----------------|---------------|
    |      1       |    None          | dut_device_id  | Execute       |
    |--------------|------------------|----------------|---------------|
    |      1       |    device_id     | dut_device_id  | If == execute |
    |              |                  |                | else abort (2)|
    |--------------|------------------|----------------|---------------|
    |     > 1      |    None          | dut_device_id  | Abort (3)     |
    |--------------|------------------|----------------|---------------|
    |     > 1      |    device_id     | dut_device_id  | If == execute |
    |              |                  |                | else abort (4)|
    --------------------------------------------------------------------

    Note:
    (1): Test aborted, no device detected
    (2): If device_id matches dut_device_id, execute, otherwise,
         abort. We don't want to run test on incorrect device
    (3): No parameter is passed on the cmd line, but multiple
           devices are detected. Must abort.
    (4): execute test only if one of the multiple devices detected
           matches the cmd line parameter's device_id
    Return: device_id if valid
    Output: device_id
    """

    device_count = commands.getoutput("adb devices").count('device')
    device_count -= 1

    if device_count == 0:
        # If device_count == 0, exit, abort test
        print ("    **** ERROR **** No device connected, test aborted")
        # sys.exit(tc.TEST_ERROR)

    elif device_count == 1:
        # One device connected.
        # Retrieve the device ID
        # if None is passed, adopt this device_id
        # if device_id parameter does not match
        # the actual device_id, abort test
        dut_device_id = " "
        cmd = "adb devices"
        pipe = os.popen(cmd, "r")
        while True:
            line = pipe.readline()
            if not line:
                break
            else:
                line = line.strip('\n')
                line = line.strip('\r')
                line = line.replace('\t', ' ')
                words = line.split(" ")
                if len(words) > 1:
                    word = words[1]
                    if word == "device":
                        dut_device_id = words[0]
                        if deviceid == "None":
                            deviceid = dut_device_id
                            print (" Device ID retrieved: %s" % deviceid)
                            break
                        else:
                            if deviceid != dut_device_id:
                                print ("    **** ERROR **** Incorrect device ID passed")
                                print ("        Device ID passed: %s" % deviceid)
                                print ("        Device ID retrieved: %s" % dut_device_id)
                                print ("        Test aborted")
                            else:
                                print (" Device ID = %s" % deviceid)
                                break
    else:
        if deviceid == 'None':
            # If no device_id is passed, abort
            print ("    **** ERROR **** Multiple devices connected")
            print ("        Must use -d device_id option")
            print ("        Test aborted")
        else:
            match_flag = 0
            dut_device_id = " "
            cmd = "adb devices"
            pipe = os.popen(cmd, "r")
            while True:
                line = pipe.readline()
                if not line:
                    break
                else:
                    line = line.strip('\n')
                    line = line.strip('\r')
                    line = line.replace('\t', ' ')
                    words = line.split(" ")
                    if len(words) > 1:
                        word = words[1]
                        if word == "device":
                            dut_device_id = words[0]
                            if dut_device_id == deviceid:
                                match_flag = 1
                                break

            if match_flag == 0:
                print ("    **** ERROR **** Multiple devices connected")
                print ("        No matching device ID found")
                print ("        Test aborted")
            else:
                print (" Device ID = %s" % deviceid)

    return deviceid


def cpu_frequency_range_check(x):
    """
    Function: cpu_frequency_range_check
    This function checks if cpu frequency is in the valid range
    TODO: Modify range based on platform
    Parameters:
    x: input argument passed
    Return:
    x: if valid, return back argument passed
    """

    x = float(x)
    if x < 0.5 or x > 2.0:
        raise argparse.ArgumentTypeError("%r not in range [0.5, 2.0]" % x)
    return x


def automation_argument_parser_handler():
    """
    Function: automation_argument_parser_handler
    This function checks for optional arguments such as device_id,
    cpu_frequency and vblank
    Documentation: http://docs.python.org/dev/library/argparse.html#creating-a-parser
    Parameters:
    cmd_arg: command line parameters
    Return:
    data_dict containing
    device_id:
    cpu_frequency_set: cpu frequency in GHz
    vblank_set: vsync 'off/on'
    relay_port: relay port id
    Note: Device ID is ID returned by adb devices cmd.
    Output: None
   """
    relay_port_id = port_relay_0

    # Error check on device_id
    deviceid2 = "None"
    deviceid = device_id_check(deviceid2)

    # Setup data dictionary
    data_dict = dict(deviceid=deviceid, relay_port=relay_port_id)

    return data_dict


def execute_test_group(manifest_file_name, result_file_name, exit_after=True):
    """
    Function: execute_test_group
    This function execute a group of tests.
    (the lowest level test building block)

    Parameters:
    cmd_line_param: command line parameters passed to the test group
    manifest_file_name: manifest file for the group
    reference_dir: directory of where the reference files or images are
        stored
     result_file_name: name of the file where test results are reported
    exit_after: OPTIONAL - enable/disable exiting in automation_dict_execution_handler()
    """
    data_dict = automation_argument_parser_handler()

    # Common setup code
    test_dict, result_filename = setup_common_test_dict(result_file_name, manifest_file_name, data_dict)

    automation_dict_execution_handler(test_dict, exit_after)

    return result_filename