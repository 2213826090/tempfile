#!/usr/bin/env python

# Copyright (c) 2016, Intel Corporation.
# Author: Florent Auger <florent.auger@intel.com>

# The script runs back to back reset test, and logs in a file the reset type
# and source of the last executed reset by pulling and parsing the RSCI table
# from the device. It also displays the bootreason for information.
#
# In case of global reset, it reads the SSRAM and logs the firmware error code.
# This code is obtained by parsing the SSRAM dump if the "bxt-ssram-dump"
# executable is available in the current path. Otherwise, it reads directly
# the SSRAM with a peeknpoke command.
#
# It requires a USB cable to the host and ADB.
#
# The script first waits for a device to be detected through ADB, then it
# waits for the boot to complete (Android booted to UI), read and parse
# RSCI table, and finally sleep for a random number of seconds
# (default 15 to 60s) before triggering a reset by:
# - running "adb reboot":
#   => usage: "test_reboot_BXT.xx reboot"
# - triggering a kernel panic that ends in a reset:
#   => usage: "test_reboot_BXT.xx panic"
# - stopping watchdog daemon to trigger a TCO timeout:
#   => usage: "test_reboot_BXT.xx tco"
#
# The usage is displayed if no parameter is passed or with an incorrect one.
# It is a while loop that can only stop with CTRL+C.


import sys
import struct
import array
import string
import subprocess
import os
import platform
import time
import random

rsci_table = "RSCI"
f_reset_info = "reset_info.txt"

p_tmp_path = "/data/local/tmp/"
e_bxt_ssram_dump = "bxt-ssram-dump"
f_ssram_dump = "bxt-ssram-dump.bin"

no_code = '0xFFFF'

# PMC firmware error code offset
FW_ERROR_CODE_OFFSET = 0x720
# SSRAM buffer base address
SSRAM_BASE_ADDR = 0xFE900000

fw_error_code_dict = {
    '0x1311': "PMC_RESET_ENTRY_IP_RESET_PREP_ACK_TIMEOUT_ERROR",
    '0x0b11': "PMC_CSE_CLD_RST_IP_RESET_PREP_ACK_TIMEOUT_ERROR",
    '0x136b': "PMC_RESET_ENTRY_CSE_RESET_WARN_ACK_TIMEOUT_ERROR",
    '0x076b': "PMC_BOOT_CSE_RESET_WARN_ACK_TIMEOUT_ERROR",
    '0x0b6b': "PMC_CSE_CLD_RST_CSE_RESET_WARN_ACK_TIMEOUT_ERROR",
    '0x1310': "PMC_RESET_ENTRY_PUNIT_RESET_WARN_ACK_TIMEOUT_ERROR",
    '0x104c': "PMC_RESET_ENTRY_PLATFORM_EVENT_LOGGED",
    '0x17b1': "PMC_RT_EVENT_LOOP_ACK_SX_MSG_TIMEOUT_ERROR"
}

rsci_header = struct.Struct('< 4c I B B 6c 8c I 4c I')
rsci_fields_v2 = struct.Struct('< B B B B I I')
# would need a specific dict to support RSCI rev 1 table
rsci_fields_v1 = struct.Struct('< B B B B I')

# dictionary to translate values into a human readable format
translate_dict = {'wake_source': ["WAKE_NOT_APPLICABLE",
                                  "WAKE_BATTERY_INSERTED",
                                  "WAKE_USB_CHARGER_INSERTED",
                                  "WAKE_ACDC_CHARGER_INSERTED",
                                  "WAKE_POWER_BUTTON_PRESSED",
                                  "WAKE_RTC_TIMER",
                                  "WAKE_BATTERY_REACHED_IA_THRESHOLD",
                                  "WAKE_ERROR"],
                  'reset_source': ["RESET_NOT_APPLICABLE",
                                   "RESET_OS_INITIATED",
                                   "RESET_FORCED",
                                   "RESET_FW_UPDATE",
                                   "RESET_KERNEL_WATCHDOG",
                                   "RESERVED_5",
                                   "RESERVED_6",
                                   "RESERVED_7",
                                   "RESET_EC_WATCHDOG",
                                   "RESET_PMIC_WATCHDOG",
                                   "RESERVED_10",
                                   "RESET_SHORT_POWER_LOSS",
                                   "RESET_PLATFORM_SPECIFIC",
                                   "RESET_UNKNOWN"],
                  'shutdown_source': ["SHTDWN_NOT_APPLICABLE",
                                      "SHTDWN_POWER_BUTTON_OVERRIDE",
                                      "SHTDWN_BATTERY_REMOVAL",
                                      "SHTDWN_VCRIT",
                                      "SHTDWN_THERMTRIP",
                                      "SHTDWN_PMICTEMP",
                                      "SHTDWN_SYSTEMP",
                                      "SHTDWN_BATTEMP",
                                      "SHTDWN_BATTEMP",
                                      "SHTDWN_SYSOVP",
                                      "SHTDWN_SECURITY_WATCHDOG",
                                      "SHTDWN_SECURITY_INITIATED",
                                      "SHTDWN_PMC_WATCHDOG",
                                      "SHTDWN_EC_WATCHDOG"],
                  'reset_type': ["NOT_APPLICABLE",
                                 "WARM_RESET",
                                 "COLD_RESET",
                                 "RESERVED_3",
                                 "RESERVED_4",
                                 "RESERVED_5",
                                 "RESERVED_6",
                                 "GLOBAL_RESET"]
}

def translate(field, value):
    ret = "NO_TRANSLATION_AVAILABLE"
    if (field in translate_dict.keys()):
        if (value < len(translate_dict[field])):
            ret = translate_dict[field][value]
        else:
            ret = "UNKNOWN_VALUE"
    return ret

def verify_checksum(f):
    fd = open(f, mode='rb')
    rsci_table = array.array('B', fd.read())
    fd.close()
    return sum(rsci_table) % 0x100

def dump_binary_table(f):
    fd = open(f, mode='rb')
    header = rsci_header.unpack(fd.read(rsci_header.size))
    revision = header[5]
    if (revision == 1):
        print "Error: unsupported RSCI rev 1"
    elif (revision == 2):
        fields = rsci_fields_v2.unpack(fd.read(rsci_fields_v2.size))
    else:
        print "Error: Unknown revision {}".format(revision)
        fd.close()
        exit(-1)

#    reset_info = translate('reset_type', fields[2]) + " - " + translate('reset_source', fields[1])
    reset_type = translate('reset_type', fields[2])
    reset_source = translate('reset_source', fields[1])

    return reset_type, reset_source

    fd.close()

# unfortunately, subprocess.check_output(cmd) stops on error,
# so here is the equivalent custom version that allows retries
def run_cmd(cmd):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    (output, error) = process.communicate()
    return (output, error)

def get_reset_info():
    # remove previous version of RSCI table if exists
    if os.path.exists(rsci_table):
        if (host_os == 'Linux'):
            subprocess.call(["rm", rsci_table])
        else:
            # assuming this is Winwdows
            subprocess.call(["del", rsci_table], shell=True)

    # get new RSCI table
    subprocess.call(["adb", "pull", "/sys/firmware/acpi/tables/RSCI", rsci_table])

    # process it if the table was able to be pulled from the device
    if os.path.exists(rsci_table):
        if (verify_checksum(rsci_table) == 0):
            return dump_binary_table(rsci_table)
        else:
            print "Error: Table is corrupted!"
            exit(-1)
    else:
        print "Cannot find a RSCI table to parse"
        exit(-1)

def get_fw_error_code(bert_enable):
    if (bert_enable == 0):
        address = "%x" % (SSRAM_BASE_ADDR+FW_ERROR_CODE_OFFSET)
        response = run_cmd(["adb", "shell", "peeknpoke r", address, "16" ])[0]
        # typical output if no error : "The value of register 0xfe900720 is 0x104c"
        fw_error_code = response.split()
        fw_error_code = fw_error_code[6]
    else:
        if os.path.exists(f_ssram_dump):
            if (host_os == 'Linux'):
                subprocess.call(["rm", f_ssram_dump])
            else:
                # assuming this is Winwdows
                subprocess.call(["del", f_ssram_dump], shell=True)

        # removing previous version of SSRAM dump binary
        run_cmd(["adb", "shell", "rm", p_tmp_path + f_ssram_dump])
        # dump the SSRAM to a binary file
        run_cmd(["adb", "shell", p_tmp_path + e_bxt_ssram_dump, p_tmp_path + f_ssram_dump])
        # pull the generated binary to the host
        subprocess.call(["adb", "pull", p_tmp_path + f_ssram_dump, f_ssram_dump])

        if not os.path.exists(f_ssram_dump):
            print "Returning a default code as %s is not accessible" % f_ssram_dump
            return no_code

        fd = open(f_ssram_dump,'rb')
        fd.seek(FW_ERROR_CODE_OFFSET)

        fw_error_code = fd.read(2)
        fd.close()
        fw_error_code = struct.unpack('h', fw_error_code)
        fw_error_code = hex(fw_error_code[0])

    for elem in fw_error_code_dict:
        if (elem == fw_error_code):
            fw_error_code = fw_error_code_dict[elem]

    return fw_error_code

def usage():
    print "Usage:"
    print "  Trigger a reset from adb:"
    print "    %s reboot" % sys.argv[0]
    print "  Trigger a reset from a kernel panic:"
    print "    %s panic" % sys.argv[0]
    print "  Trigger a reset from a TCO timer expiration:"
    print "    %s tco" % sys.argv[0]
    exit(-1)

def main():

    if len(sys.argv) < 2:
        usage()

    global host_os
    host_os = platform.system()

    if os.path.exists(f_reset_info):
        if (host_os == 'Linux'):
            subprocess.call(["mv", f_reset_info, f_reset_info + ".prev"])
        else:
            # assuming this is Winwdows
            subprocess.call(["move", f_reset_info, f_reset_info + ".prev"], shell=True)

    bert_enable = 0
    if os.path.exists(e_bxt_ssram_dump):
        print "waiting for device"
        subprocess.call(["adb", "wait-for-device"])
        run_cmd(["adb", "push", e_bxt_ssram_dump, p_tmp_path + e_bxt_ssram_dump])
        run_cmd(["adb", "shell", "chmod 777", p_tmp_path + e_bxt_ssram_dump])
        bert_enable = 1

    test_num = 0

    while True:
        print "*******************************"

        test_num += 1
        test_time = time.strftime("%a %b %d %H:%M:%S")
        print "Reset %d - %s" % (test_num, test_time)

        print "waiting for device"
        subprocess.call(["adb", "wait-for-device"])

        print "waiting for sys.boot_completed"
        boot_completed = 0
        while (boot_completed != ['1']):
            boot_completed = run_cmd(["adb", "shell", "getprop", "sys.boot_completed"])[0]
            # method to avoid empty string
            boot_completed = boot_completed.split()
            time.sleep(2)

        print "getting root access"
        run_cmd(["adb", "root"])
        subprocess.call(["adb", "wait-for-device"])

        reset_type, reset_source = get_reset_info()
        reset_info = reset_type + " - " + reset_source
        print "Reset type and source are %s" % reset_info
        print "Bootreason is " + run_cmd(["adb", "shell", "getprop", "ro.boot.bootreason"])[0]

        # if the reset type is global and BERT is enable, then log the PMC fw error code
        fw_error_code = 0
        if (reset_type == "GLOBAL_RESET"):
            fw_error_code = get_fw_error_code(bert_enable)

        # log result into a file
        fd = open(f_reset_info,'a')
        fd.write("Reset %d - %s\n" % (test_num, test_time))
        fd.write("%s\n" % reset_info)
        if (fw_error_code != 0):
            fd.write("Firmware error code = %s\n" % fw_error_code)
            print "Firmware error code = %s\n" % fw_error_code
        fd.close()

        # wait for some random time before rebooting the device
        # this can be customized
        delay = random.randrange(15,60)
        print "Sleeping for %d seconds" % delay
        time.sleep(delay)

        if (sys.argv[1] == "reboot"):
            print "Rebooting now..."
            subprocess.call(["adb", "reboot"])
        elif (sys.argv[1] == "panic"):
            print "Rebooting in about 10s..."
            subprocess.call(["adb", "shell", "echo c > /proc/sysrq-trigger"])
        elif (sys.argv[1] == "tco"):
            # Cannot perform more than 2 TCO expirations in a raw within
            # 10 minutes, otherwise kernelflinger enters in crashdump mode.
            # So need to interleave a proper "reset" every 2 expirations.
            if (test_num % 3):
                print "Rebooting after TCO timer expiration..."
                subprocess.call(["adb", "shell", "stop watchdogd"])
            else:
                print "Rebooting now..."
                subprocess.call(["adb", "reboot"])
        else:
            usage()

        # wait for some time for reset to be effective before pinging again
        # the device, and it must be longer than TCO timer expiration time
        time.sleep(45)

if __name__ == "__main__":
    main()