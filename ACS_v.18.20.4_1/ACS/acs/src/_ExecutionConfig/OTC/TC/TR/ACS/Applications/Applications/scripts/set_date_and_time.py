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
:description: Test Case for all the major functionalities of the Date & time settings pane
:since: 7/29/14
:author: mmaracix
"""

from date_time_ui_lib import *
import time
VERDICT = SUCCESS
OUTPUT = '\n'

#TC params

new_timezone_value = TC_PARAMETERS('new_timezone_value')
new_time_value = TC_PARAMETERS('new_time_value')
new_date_value = TC_PARAMETERS('new_date_value')

d.press(82)
### launch Date & time settings panel
if launch_date_time_settings():
    OUTPUT += 'Success, the Date & Time settings panel was reached successfully.\n'
else:
    VERDICT = FAILURE
    OUTPUT += 'Failure, the Date & Time settings panel could not be reached.\n'

#enabling automatic time

if enable_automatic_time():
    OUTPUT += "Success, Automatic Date & time enabled.\n"
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, Automatic Date & time were not enabled.\n"

# disabling automatic time
if disable_automatic_time():
    OUTPUT += "Success, Automatic Date & time disabled.\n"
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, Automatic Date & time were not disabled.\n"

#change timezone
initial_tz = get_timezone().split(',')[0]
initial_time = get_time()
initial_date = get_date()
if d(text="Select time zone").exists:
    d(text="Select time zone").click()
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, could not reach the 'Select time zone menu'.\n"


new_tz = set_timezone(new_timezone_value)
changed_tz = get_timezone().split(',')[0]
current_date = get_date()
if compare_timezone(new_tz, changed_tz):
    OUTPUT += "Success, changed old timezone value %s with %s.\n" % (initial_tz, changed_tz)
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, given timezone value %s does not match the provided value of %s.\n" % (new_tz, changed_tz)

#check time after changing the timezone

new_time = get_time()
new_tz = get_timezone()
if check_time_after_change_timezone(initial_time, initial_tz, new_time, new_tz, initial_date, current_date):
    OUTPUT += "Success, the new time value is as expected after changing the timezone.\n"
else:
    VERDICT = FAILURE
    OUTPUT += 'Failure, the new time value is not changed as expected; initial value was %s and current value is %s' \
              'with initial timezone value of %s and current timezone value of %s.\n'\
              % (initial_time, new_time, initial_tz, new_tz)


#changing the time and date
#setting the time manually and checking it
if disable_automatic_time():
    initial_time = get_time()
    d(text="Set time").click()
    defined_time = set_time(new_time_value)
    try:
        compare_time(new_time_value, defined_time)
        OUTPUT += "Success, the time was changed with the provided value.\n"
    except False:
        try:
            compare_time(new_time_value, defined_time, offset_minutes=1)
            OUTPUT += "Success, the time was changed with the provided value.\n"
        except False:
            VERDICT = FAILURE
            OUTPUT += "Failure, the time was not changed with the provided value of %s.\n" % new_time_value

#setting the date
initial_date_check = get_date()
d(text="Set date").click()
changed_date = set_date(new_date_value)
if compare_date(initial_date, changed_date):
    #this function returns True if the provided dates are different
    OUTPUT += "Success, the initial date %s was successfully changed with %s.\n" % (initial_date, changed_date)
else:
    VERDICT = FAILURE
    if initial_date == changed_date:
        OUTPUT += "Failure caused by the fact that the new date value %s is identical to the initial date value" \
                  "%s.\n" % (new_date_value, initial_date)
    else:
        OUTPUT += "Failure,  the initial date %s was not changed with %s.\n" % (initial_date, changed_date)

#setting and checking the time-format
tf1 = get_time_format()
if set_time_format():
    tf2 = get_time_format()
    if check_time_format(tf1, tf2):
        OUTPUT += "Success, Time format was changed from %s to %s.\n" % (tf1, tf2)
    else:
        # raise EnvironmentError, "Time format was not succesfully changed as planned"
        VERDICT = FAILURE
        OUTPUT += 'Failure, Time format was not successfully changed as planned.\n'
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, Time format was not changed at all.\n"

#check the time on the indicator bar

if d(text="Set time").exists:
    my_time = get_time_absolute()
    try:
        check_time_indicator_bar(my_time)
        OUTPUT += "Success, Time on the indicator bar has the correct value.\n"
    except False:
        check_time_indicator_bar(my_time, offset_minutes=1)
        OUTPUT += "Success, Time on the indicator bar has the correct value.\n"
    except False:
        VERDICT = FAILURE
        OUTPUT += "Failure, time on the indicator bar is different.\n"

# lock screen unlock screen and check time and date
d.press(82)
launch_date_time_settings()
if d(text="Date & time").exists:
    my_lock_time = get_time_absolute()

    my_lock_date = get_date()
    time_format = get_time_format()
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, Could not get the current date and time.\n"
d.press.power()
d.wait(4)
d.press.power()
if check_time_date_lockscreen(my_lock_time, my_lock_date, time_format):
    OUTPUT += "Success, Time & date on lockscreen match.\n"
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, Time and date on lock screen don't match.\n"

d.press(82)
#reboot and check time and date
if not launch_date_time_settings():
    launch_date_time_settings()
else:
    d(text="Set time").click()
    set_time("4:59 pm")
    my_reboot_time = get_time()
    my_reboot_date = get_date()
    my_time_format = get_time_format()
    reboot_cmd = "adb reboot"
    exec_status, output = DEVICE.run_cmd(reboot_cmd, 15)

    time.sleep(30)
    try:
        check_time_date_lockscreen(my_reboot_time, my_reboot_date, my_time_format)
        OUTPUT += "Success, Time and date still match after reboot.\n"
    except False:
        try:
            check_time_date_lockscreen(my_reboot_time, my_reboot_date, my_time_format, offset_minutes=1)
            OUTPUT += "Success, Time and date still match after reboot.\n"
        except False:
            VERDICT = FAILURE
            OUTPUT += "Failure, time and date no longer match.\n"