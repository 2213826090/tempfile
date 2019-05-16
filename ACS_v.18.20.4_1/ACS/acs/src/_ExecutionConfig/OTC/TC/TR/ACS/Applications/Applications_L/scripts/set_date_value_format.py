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
:description: This TC sets the value and format of the date, looping through all the formats specified
:since: 7/31/14
:author: mmaracix
"""

from date_time_ui_lib import *

VERDICT = SUCCESS
OUTPUT = '\n'

# TC params
new_date_value = TC_PARAMETERS('new_date_value')

regional_date_format = "%m/%d/%Y"
second_date_format = "%m/%d/%Y"
third_date_format = "%d/%m/%Y"
fourth_date_format = "%Y/%m/%d"

d.press(82)
d.press.home()
# ## launch Date & time settings panel
if launch_date_time_settings():
    OUTPUT += 'Success, the Date & Time settings panel was reached successfully.\n'
else:
    VERDICT = FAILURE
    OUTPUT += 'Failure, the Date & Time settings panel could not be reached.\n'

if VERDICT == SUCCESS:
    # enabling automatic time
    # this is a precondition, as the first step is disabling automatic date and time
    if enable_automatic_time():
        OUTPUT += "Success, Automatic Date & time enabled.\n"
    else:
        VERDICT = FAILURE
        OUTPUT += "Failure, Automatic Date & time were not enabled.\n"

    # disabling automatic time - step 1
    if disable_automatic_time():
        OUTPUT += "Success, Automatic Date & time disabled.\n"
    else:
        VERDICT = FAILURE
        OUTPUT += "Failure, Automatic Date & time were not disabled.\n"

    # setting the date
    if d(text="Set date").enabled:
        initial_date = get_date()
        d(text="Set date").click()
        if d(className="android.widget.DatePicker").wait.exists(timeout=5000):
            changed_date = set_date(new_date_value)
            saved_date_value = get_date()
            if compare_saved_date(string_to_date(new_date_value), changed_date) and compare_saved_date(
                    string_to_date(new_date_value), saved_date_value):
                # this function returns True if the provided dates are identical
                OUTPUT += "Success, the initial date %s was successfully changed with %s.\n" % (
                    initial_date, saved_date_value)
            else:
                VERDICT = FAILURE
                if initial_date == changed_date:
                    OUTPUT += "Failure caused by the fact that the new date value %s is identical in date picker to" \
                              " the initial date value %s.\n" % (new_date_value, initial_date)
                else:
                    OUTPUT += "Failure,  the initial date %s was not changed with %s.\n" % (initial_date,
                                                                                            new_date_value)
        else:
            VERDICT = FAILURE
            OUTPUT += "Failure, date could not be changed because date picker not opened.\n"
    else:
        VERDICT = FAILURE
        OUTPUT += "Failure, 'Set date' option is disabled.\n"

    # setting the date format
    for i in (second_date_format, third_date_format, fourth_date_format):
        d(text="Choose date format").click()
        if d(resourceId="android:id/select_dialog_listview").wait.exists(timeout=5000):
            new_format = set_date_format(i)
            format_date = check_format_on_date()
            if compare_date_format(i, new_format):
                if compare_date_format(new_format, format_date):
                    OUTPUT += "Success, The date format was successfully changed to %s.\n" % i
                else:
                    VERDICT = FAILURE
                    OUTPUT += "The new date format %s was changed, but the change is not reflected in the current" \
                              " date.\n" % i
            else:
                VERDICT = FAILURE
                OUTPUT += "Failure, the date format was not changed accordingly.\n"
        else:
            VERDICT = FAILURE
            OUTPUT += "Failure, the dialog for setting date format not opened.\n"

    # enabling automatic time
    if enable_automatic_time():
        OUTPUT += "Success, Automatic Date & time enabled.\n"
    else:
        VERDICT = FAILURE
        OUTPUT += "Failure, Automatic Date & time were not enabled.\n"

#gotta also check network time, network connectivity is needed
d.press.home()
