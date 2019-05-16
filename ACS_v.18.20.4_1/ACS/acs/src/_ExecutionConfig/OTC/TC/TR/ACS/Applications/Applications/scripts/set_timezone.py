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
:description: Test Case for changing the timezone, with preconditions
:since: 7/29/14
:author: mmaracix
"""

from date_time_ui_lib import *
VERDICT = SUCCESS
OUTPUT = '\n'

#TC params
precondition_timezone_value = "London, Dublin"
precondition_time_format = "AM/PM"
precondition_time_value = "01:00 AM"
new_timezone_value = TC_PARAMETERS('new_timezone_value')

d.press(82)
### launch Date & time settings panel
if launch_date_time_settings():
    OUTPUT += 'Success, the Date & Time settings panel was reached successfully.\n'
else:
    VERDICT = FAILURE
    OUTPUT += 'Failure, the Date & Time settings panel could not be reached.\n'

#enabling pre-conditions
d(text="Select time zone").click()
set_timezone(precondition_timezone_value)
if get_time_format() == "AM/PM":
    pass
else:
    set_time_format()
disable_automatic_time()
d(text="Set time").click()
set_time(precondition_time_value)

#actual test

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
