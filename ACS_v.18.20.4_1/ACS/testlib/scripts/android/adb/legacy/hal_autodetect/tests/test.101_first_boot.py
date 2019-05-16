#!/usr/bin/env python

##############################################################################
#
# @filename:    test.101_first_boot.py
#
# @description: Checking boot errors by inspecting dmesg and logcat
#
# @author:      alexandrux.n.branciog@intel.com
#
##############################################################################

import sys
from testlib.scripts.hal_autodetect import hal_steps as steps
from testlib.base.base_utils import get_args

# usage:
#       python test.<test_name>.py --serial 192.168.1.1:5555
args = get_args(sys.argv)
globals().update(vars(args))

dmesg_file = 'dmesg'
logcat_file = 'logcat'

ignore_errors = {
"ecs_e7":{
    "dmesg":[
        "HAL: libhal_log_set_level: log level set to 3",
        "HAL: libhal_alloc_binding: Modalias of 525 bytes received",
        "HAL: libhal_map: No HAL binding for [keystore] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio_policy] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.primary] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [local_time] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [hwcomposer] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [memtrack] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.a2dp] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.usb] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [consumerir] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [gps] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [flp] - Falling back to legacy mode",
        "Could not allocate telemetry record"],
    "elogcat":[
        "Could not find a HAL module for audio",
        "Could not find a HAL module for consumerir",
        "Could not find a HAL module for gps",
        "Could not find a HAL module for flp",
        "Could not find a HAL module for hwcomposer",
        "Could not find a HAL module for memtrack",
        "Could not find a HAL module for local_time",
        "Could not find a HAL module for audio_policy",
        "Could not find a HAL module for keystore"],
    "ilogcat":[
        "HAL_IPC_MAP failed",
        "Got HAL server module"]
},
"t100":{
    "dmesg":[
        "HAL: libhal_log_set_level: log level set to 3",
        "HAL: libhal_alloc_binding: Modalias of 525 bytes received",
        "HAL: libhal_map: No HAL binding for [keystore] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio_policy] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.primary] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [local_time] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [hwcomposer] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [memtrack] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.a2dp] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.usb] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [consumerir] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [gps] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [flp] - Falling back to legacy mode",
        "Could not allocate telemetry record"],
    "elogcat":[
        "Could not find a HAL module for audio",
        "Could not find a HAL module for consumerir",
        "Could not find a HAL module for gps",
        "Could not find a HAL module for flp",
        "Could not find a HAL module for hwcomposer",
        "Could not find a HAL module for memtrack",
        "Could not find a HAL module for local_time",
        "Could not find a HAL module for audio_policy",
        "Could not find a HAL module for keystore"],
    "ilogcat":[
        "HAL_IPC_MAP failed",
        "Got HAL server module"]
},
"xps12":{
    "dmesg":[
        "HAL: libhal_log_set_level: log level set to 3",
        "HAL: libhal_alloc_binding: Modalias of 525 bytes received",
        "HAL: libhal_map: No HAL binding for [keystore] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio_policy] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.primary] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [local_time] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [hwcomposer] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [memtrack] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.a2dp] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [audio.usb] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [consumerir] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [gps] - Falling back to legacy mode",
        "HAL: libhal_map: No HAL binding for [flp] - Falling back to legacy mode"],
    "elogcat":[
        "Could not find a HAL module for audio",
        "Could not find a HAL module for consumerir",
        "Could not find a HAL module for gps",
        "Could not find a HAL module for flp",
        "Could not find a HAL module for hwcomposer",
        "Could not find a HAL module for memtrack",
        "Could not find a HAL module for local_time",
        "Could not find a HAL module for audio_policy",
        "Could not find a HAL module for keystore"],
    "ilogcat":[
        "HAL_IPC_MAP failed",
        "Got HAL server module"]
}
}

steps.boot_error_check(
    print_error = "Error - boot errors",
    serial = serial,
    ignore_errors = ignore_errors,
    dmesg_file = dmesg_file,
    logcat_file = logcat_file
)()
