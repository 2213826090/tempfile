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
:description: This TC sets the value and format of the time
:since: 7/31/14
:author: mmaracix
"""

from date_time_ui_lib import *
VERDICT = SUCCESS
OUTPUT = '\n'

#TC params
new_time_value = TC_PARAMETERS('new_time_value')

d.press(82)
### launch Date & time settings panel
if launch_date_time_settings():
    OUTPUT += 'Success, the Date & Time settings panel was reached successfully.\n'
else:
    VERDICT = FAILURE
    OUTPUT += 'Failure, the Date & Time settings panel could not be reached.\n'

# disabling automatic time - step 1
if disable_automatic_time():
    OUTPUT += "Success, Automatic Date & time disabled.\n"
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, Automatic Date & time were not disabled.\n"

#changing the time and time-format
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
#setting and checking the time-format
def set_time_format_local(tf1=''):
    VERDICT = SUCCESS
    OUTPUT = ''
    if tf1 == '':
        tf1 = get_time_format()
    else:
        pass
    if tf1 == get_time_format():
        OUTPUT += "The time format is already equal to the value given as a parameter"
    else:
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

    return VERDICT, OUTPUT

#change time format randomly:
set_time_format_local()

#set hour format specifically to 24-hour
set_time_format_local(tf1="24-hour")
#set hour format specifically to AM/PM
set_time_format_local(tf1="AM/PM")

#enabling automatic time
if enable_automatic_time():
    OUTPUT += "Success, Automatic Date & time enabled.\n"
else:
    VERDICT = FAILURE
    OUTPUT += "Failure, Automatic Date & time were not enabled.\n"

#gotta also check network time, network connectivity is needed
