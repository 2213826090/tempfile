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
:description: This file contains Python UIAutomator functions for the Date & time settings pane
:since: 7/21/14
:author: mmaracix
"""
from uiautomator import device as d
import datetime


def enable_automatic_time():
    if not d(className="android.widget.ListView").child(className="android.widget.LinearLayout").child(className="android.widget.CheckBox").checked:
        d(text="Automatic date & time").click()
    elif d(text="Automatic date & time").checked:
        pass
    if d(text="Set date").enabled | d(text="Set time").enabled:
        return False
    else:
        return True


def disable_automatic_time():
    if not d(className="android.widget.ListView").child(className="android.widget.LinearLayout").child(className="android.widget.CheckBox").checked:
        pass
    else:
        d(text="Automatic date & time").click()
    if d(text="Set date").enabled | d(text="Set time").enabled:
        return True
    else:
        return False


def get_time():
    try:
        if d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(16).info['text'] == 'Set time':
            time_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(17).info
            time_elem = datetime.datetime.strptime(time_str['text'], "%I:%M %p").time()
        elif d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(13).info['text'] == 'Set time':
            time_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(14).info
            time_elem = datetime.datetime.strptime(time_str['text'], "%I:%M %p").time()
        else:
            raise Exception, 'Could not find the time field at the suggested locations. Please revise the xml structure against the code written here'
    except ValueError:
        if d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(16).info['text'] == 'Set time':
            time_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(17).info
            time_elem = datetime.datetime.strptime(time_str['text'], "%H:%M").time()
        elif d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(13).info['text'] == 'Set time':
            time_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(14).info
            time_elem = datetime.datetime.strptime(time_str['text'], "%H:%M").time()
        else:
            raise Exception, 'Could not find the time field at the suggested locations. Please revise the xml structure against the code written here'
    return time_elem


def get_time_absolute():
    try:
        if d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(16).info['text'] == 'Set time':
            time_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(17).info
            time_elem = datetime.datetime.strptime(time_str['text'], "%H:%M %p").time()
        elif d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(13).info['text'] == 'Set time':
            time_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(14).info
            time_elem = datetime.datetime.strptime(time_str['text'], "%H:%M %p").time()
        else:
            raise Exception, 'Could not find the time field at the suggested locations. Please revise the xml structure against the code written here'
    except ValueError:
        if d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(16).info['text'] == 'Set time':
            time_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(17).info
            time_elem = datetime.datetime.strptime(time_str['text'], "%H:%M").time()
        elif d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(13).info['text'] == 'Set time':
            time_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(14).info
            time_elem = datetime.datetime.strptime(time_str['text'], "%H:%M").time()
        else:
            raise Exception, 'Could not find the time field at the suggested locations. Please revise the xml structure against the code written here'
    return time_elem


def set_time(time_string):
    """
    Method to change the time
    Paramaters:
    time_string: string of time in the format %H:%M %AM/PM
    AM or PM are optional and they must only be provided when the 24-hour format is not selected
    returns the new time value in a datetime.time type
    """
    time_set = ''
    time_struct = string_to_time(time_string)

    if (d(text="PM").exists or d(text="AM").exists) and time_struct.hour > 12:
        new_hour = time_struct.hour - 12
    else:
        new_hour = time_struct.hour
    # new_hour = time_struct.hour
    if d(className="android.widget.TimePicker").child_by_instance(0, className="android.widget.NumberPicker").child(className = "android.widget.EditText").set_text(new_hour):
        hour_val = d(className="android.widget.TimePicker").child_by_instance(0, className="android.widget.NumberPicker").child(className = "android.widget.EditText").info['text']
        if hour_val != '0':
            pass
        else:
            hour_val = '12'
        time_set += str(hour_val)+":"

     #set the hour accordingly
    if d(className="android.widget.TimePicker").child_by_instance(1, className="android.widget.NumberPicker").child(className = "android.widget.EditText").set_text(time_struct.minute):
        minutes_val = d(className="android.widget.TimePicker").child_by_instance(1, className="android.widget.NumberPicker").child(className = "android.widget.EditText").info['text']
        time_set += str(minutes_val)

    if "pm" in time_string.lower():
        if d(text="PM").exists:
            d(text="PM").click()
            time_set += " PM"
    elif "am" in time_string.lower():
        if d(text="AM").exists:
            d(text="AM").click()
            time_set += " AM"

    if 'am' in time_set.lower() or 'pm' in time_set.lower():
        time_set = datetime.datetime.strptime(time_set, "%I:%M %p").time()
    else:
        time_set = datetime.datetime.strptime(time_set, "%H:%M").time()

    d.press.enter()
    d(text="Done").click()
    return time_set


def compare_time(initial_time, second_time, offset_hour = 0, offset_minutes = 0):
    """
    offset is given for minutes and hours, in their measure(hours or minutes)
    returns True if the time is the same
    """
    if type(initial_time) != str:
        try:
            initial_time = initial_time.strftime("%H:%M")
        except ValueError:
            initial_time = initial_time.strftime("%H:%M:%S")
    else:
        pass
    first_time_hour = initial_time.split(":")[0]
    offset_h = int(first_time_hour) + offset_hour
    first_time_minute = initial_time.split(":")[1]
    offset_m = int(first_time_minute) + offset_minutes
    if offset_m >= 60:
        offset_m -= 60
        offset_h += 1
    initial_time = initial_time.replace(first_time_hour, str(offset_h))

    if offset_m != 0:
        initial_time = initial_time.replace(first_time_minute, str(offset_m))
    elif offset_m == 0:
        initial_time = initial_time.replace(first_time_minute, str(offset_m) + "0")

    if type(initial_time) == str:
        initial_time = string_to_time(initial_time)
    else:
        pass
    if type(second_time) == str:
        second_time = string_to_time(second_time)
    else:
        pass
    if initial_time != second_time:
        return False
    else:
        return True


def get_timezone():
    """
    get the current time zone form the Date & time settings pane
    """
    if "GMT" not in d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(22).info['text']:
        zone_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(18).info['text']
    else:
        zone_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(22).info['text']
    return zone_str

def set_timezone(new_timezone_name):
    """
    Set a new timezone with a specified value
    Returns the associated time value for the selected timezone
    """
    if d(scrollable=True).scroll.to(text=new_timezone_name):
        tz_value = d(text=new_timezone_name).sibling(resourceId="android:id/text2").info['text']
        d(text=new_timezone_name).click()
        #TODO: raise error here if the else isn't fulfilled
    return tz_value

def compare_timezone(expected_tz, actual_tz):
    if "+" in expected_tz:
        new_expected_tz = expected_tz.split('+')[1].split(':')[0]
    else:
        new_expected_tz = expected_tz.split('-')[1].split(':')[0]
    if int(new_expected_tz) < 10:
        expected_tz = expected_tz.replace(new_expected_tz, "0"+new_expected_tz)
    if expected_tz == actual_tz:
        return True
    else:
        return False

def get_tz_value_from_string(tz_string):
    try:
        return tz_string.split(',')[0].strip('GMT').split(":")
    except ValueError:
        raise SyntaxError
def check_time_after_change_timezone(old_time, old_tz, new_time, new_tz, initial_date, current_date):
    """
    Still need to add functionality for checking changes were effective
    for timezones that are fractional, i.e. +/-4:45, +/-2:30 etc
    """
    new_time_int_hour = int(new_time.hour)
    old_time_int_hour = int(old_time.hour)

    new_time_int_min = int(new_time.minute)
    old_time_int_min = int(old_time.minute)

    #replacing 0-hours with 24 for the math to work
    if new_time_int_hour == 0 and old_time_int_hour > 12:
        new_time_int_hour = 24
    elif old_time_int_hour == 0 and new_time_int_hour > 12:
        old_time_int_hour = 24

    #parsing timezone values unconstrained
    old_tz_int_hour = int(get_tz_value_from_string(old_tz)[0])
    new_tz_int_hour = int(get_tz_value_from_string(new_tz)[0])

    old_tz_int_min = int(get_tz_value_from_string(old_tz)[1])
    new_tz_int_min = int(get_tz_value_from_string(new_tz)[1])

    tz_delta_hour = new_tz_int_hour - old_tz_int_hour
    delta_tz_min = new_tz_int_min - old_tz_int_min

    time_delta = new_time_int_hour - old_time_int_hour
    if not compare_date(initial_date, current_date):
        time_delta = new_time_int_hour - old_time_int_hour
    elif current_date == initial_date+datetime.timedelta(days=1):
        new_time_int_hour += 24
    elif current_date == initial_date+datetime.timedelta(days=-1):
        new_time_int_hour -= 24
        time_delta = new_time_int_hour - old_time_int_hour

    delta_tz_min = new_tz_int_min - old_tz_int_min
    if delta_tz_min == 0:
        time_delta = new_time_int_hour - old_time_int_hour
    elif delta_tz_min != 0 and delta_tz_min < 60:
        min_delta_plus = old_time_int_min + delta_tz_min
        if old_time_int_min != 0 and new_tz_int_min != 0:
            if min_delta_plus >= 60:
                new_time_int_hour -= 1
                time_delta = new_time_int_hour - old_time_int_hour

    elif delta_tz_min > 60:
        if time_delta > 0:
            new_time_int_hour += 1
            time_delta = new_time_int_hour - old_time_int_hour
        elif time_delta < 0:
            new_time_int_hour -= 1
            time_delta = new_time_int_hour - old_time_int_hour

    if new_tz_int_hour < 0 and old_tz_int_hour < 0:
        if old_time_int_hour > 12 > new_time_int_hour:
            time_delta = new_time_int_hour - (24-old_time_int_hour)
        elif new_time_int_hour == 24:
            new_time_int_hour = 0
            time_delta = new_time_int_hour - old_time_int_hour

    if delta_tz_min < 0 and tz_delta_hour == 0:
        tz_delta_hour = 1

    if tz_delta_hour == time_delta:
        return True
    else:
        return False

regional_format = "%m/%d/%Y"
second_format = "%m/%d/%Y"
third_format = "%d/%m/%Y"
fourth_format = "%Y/%m/%d"
lockscreen_format = "%a, %B %d"


def set_date_format(format_string):
    """
    Takes as parameter a string with one of the 4 formats available.
    If a format that does not exist is provided, a SyntaxError will be raised and the "Cancel" button will be clicked.
    Returns the value that was enabled.
    """
    if 'regional' in format_string.lower():
        d(resourceId="android:id/text1", index=0).click()
        date_format = regional_format
    elif format_string == second_format:
        d(resourceId="android:id/text1", index=1).click()
        date_format = second_format
    elif format_string == third_format:
        d(resourceId="android:id/text1", index=2).click()
        date_format = third_format
    elif format_string == fourth_format:
        d(resourceId="android:id/text1", index=3).click()
        date_format = fourth_format
    else:
        d(text="Cancel").click()
        raise SyntaxError("You have provided a date format that is not present in the option pane. Please revise.")
    return date_format

def get_date_format():
    date_format = ''
    if d(text="Choose date format").exists:
        d(text="Choose date format").click()
    if d(resourceId="android:id/text1", index=0).info['checked'] == 'True':
        date_format = 'Regional' + regional_format
    elif d(resourceId="android:id/text1", index=1).click() == 'True':
        date_format = second_format
    elif d(resourceId="android:id/text1", index=2).click() == 'True':
        date_format = third_format
    elif d(resourceId="android:id/text1", index=3).click() == 'True':
        date_format = fourth_format
    else:
        date_format = 'Could not establish the date format used'
    d(text="Cancel").click()
    return date_format

def compare_date_format(reference_format, provided_format):
    if provided_format == reference_format:
        return True
    else:
        return False

def check_format_on_date():
    """
    Takes as parameter a date, either as string or datetime.date and returns the format that it fits.
    If the format does not match, it raises a ValueError
    """
    date_format = ''
    date_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(10).info['text']
    if date_str == '':
        date_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(12).info['text']
    try:
        date_val = datetime.datetime.strptime(date_str, second_format).date()
        date_format = second_format
    except ValueError:
        try:
            date_val = datetime.datetime.strptime(date_str, third_format).date()
            date_format = third_format
        except ValueError:
            try:
                date_val = datetime.datetime.strptime(date_str, fourth_format).date()
                date_format = fourth_format
            except ValueError:
                raise SyntaxError("The format for %s could not be found." % date_str)

    return date_format


def get_date():
    # for Nexus at the moment

    date_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(10).info['text']
    if date_str == '':
        date_str = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(12).info['text']
    try:
        date_val = datetime.datetime.strptime(date_str, regional_format).date()
    except ValueError:
        try:
            date_val = datetime.datetime.strptime(date_str, second_format).date()
        except ValueError:
            try:
                date_val = datetime.datetime.strptime(date_str, third_format).date()
            except ValueError:
                try:
                    date_val = datetime.datetime.strptime(date_str, fourth_format).date()
                except ValueError:
                    raise SyntaxError("The format for %s could not be found." % date_str)

    return date_val

def string_to_time(time_string):
    """
    Takes as parameter a string that is suppose to be a time provided and transforms it accordingly
    Returns a datetime.time element
    """

    if 'am' in time_string.lower() or 'pm' in time_string.lower():
        try:
            time_struct = datetime.datetime.strptime(time_string, "%I:%M:%S %p").time()
        except ValueError:
            time_struct = datetime.datetime.strptime(time_string, "%I:%M %p").time()
        try:
            if time_struct.hour < 12:
                pass
        except False:
            raise SyntaxError("You have provided a time greater that 12 in a PM/AM time-format")
    elif 'am' not in time_string.lower() or 'pm' not in time_string.lower():
        try:
            time_struct = datetime.datetime.strptime(time_string, "%H:%M:%S").time()
        except ValueError:
            time_struct = datetime.datetime.strptime(time_string, "%H:%M").time()
        try:
            if time_struct.hour < 24:
                pass
        except False:
                raise SyntaxError("You have provided a time greater that 24 in a 24-h time-format")
        try:
            if time_struct.hour > 0 or time_struct.minute > 0:
                pass
        except False:
                raise SyntaxError("You have provided a negative value for the time parameter, please revise")

    return time_struct

def string_to_date(date_string):
    """
    Takes a string as an argument and returns a datetime.date() value
    """
    try:
        string_date = datetime.datetime.strptime(date_string, third_format)
    except ValueError:
        try:
            string_date = datetime.datetime.strptime(date_string, regional_format)
        except ValueError:
            try:
                string_date = datetime.datetime.strptime(date_string, second_format)
            except ValueError:
                raise SyntaxError("Please provide a date in the format DD/MM/YYYY or MM/DD/YYYY")
    return string_date.date()

def compare_date(old_date, new_date):
    """
    both dates are in datetime.date() format
    function returns True if the provided dates are different
    """
    try:
        if old_date == new_date:
            return False
        else:
            return True
    except ValueError:
        raise Exception, "Dates were not in an appropriate format"

def set_date(date_string):
    """
    Takes a date as a string for parameter
     Returns a datetime.date() object
    """
    new_value = ''
    string_date = string_to_date(date_string)
    new_year = string_date.year
    new_day = string_date.day
    new_month = string_date.strftime("%b")

    if d(className="android.widget.DatePicker").child_by_instance(0, className="android.widget.NumberPicker").child(className = "android.widget.EditText").set_text(new_month):
        month_val = d(className="android.widget.DatePicker").child_by_instance(0, className="android.widget.NumberPicker").child(className = "android.widget.EditText").info['text']
        new_value += month_val
    if d(className="android.widget.DatePicker").child_by_instance(1, className="android.widget.NumberPicker").child(className = "android.widget.EditText").set_text(new_day):
        day_val = d(className="android.widget.DatePicker").child_by_instance(1, className="android.widget.NumberPicker").child(className = "android.widget.EditText").info['text']
        new_value += "/" + day_val
    if d(className="android.widget.DatePicker").child_by_instance(2, className="android.widget.NumberPicker").child(className = "android.widget.EditText").set_text(new_year):
        year_val = d(className="android.widget.DatePicker").child_by_instance(2, className="android.widget.NumberPicker").child(className = "android.widget.EditText").info['text']
        new_value += "/" + year_val

    d(text="Done").click()
    return datetime.datetime.strptime(new_value, '%b/%d/%Y')

def check_time_date_lockscreen(time_to_check, date_to_check, time_format, offset_hour = 0, offset_minutes = 0):
    """
    time_to_check is of time_struct format, as retuned by get_time()
    date_to_check as returned by get_date()
    """
    if d(resourceId="com.android.keyguard:id/clock_view").wait(2000) and d(resourceId="com.android.keyguard:id/date_view").wait(2000):
        time_value = d(resourceId="com.android.keyguard:id/clock_view").info['text']
        date_value = d(resourceId="com.android.keyguard:id/date_view").info['text']
    else:
        raise Exception("please wake up the screen")
    time_value = datetime.datetime.strptime(time_value, "%H:%M").time()
    if time_format == '24-hour':
        pass
    elif time_format == 'AM/PM':
        if time_to_check.hour > 12:
            time_to_check_hour = int(time_to_check.hour) - 12
            time_to_check = str(time_to_check_hour) + ":" + str(time_to_check.minute)

    date_value = datetime.datetime.strptime(date_value, lockscreen_format)
    if compare_time(time_to_check, time_value, offset_hour, offset_minutes):
        if compare_date(date_value, date_to_check):
            return True
    else:
        return False


def check_time_indicator_bar(given_time, offset_hour = 0, offset_minutes = 0):
    """
    given_time is in strptime.time() format
    """
    d.open.notification()
    try:
        if d(resourceId='com.android.systemui:id/datetime').wait(2000):
            time_str = d(resourceId='com.android.systemui:id/clock').info['text']
            time_str = datetime.datetime.strptime(time_str, "%H:%M")
        else:
            raise Exception("Did not reach the Notification panel")
    except UnboundLocalError:
        exit(0)
    d(resourceId="com.android.systemui:id/notification_icon_area").click()
    if compare_time(given_time, time_str, offset_hour, offset_minutes):
        return True
    else:
        return False


def get_time_format():
    if d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(21).info['text'] == "Use 24-hour format":
        time_format = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(22).info['text']
    elif d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(26).info['text'] == "Use 24-hour format":
        time_format = d(className="android.widget.LinearLayout").child(className="android.widget.ListView").child_by_instance(27).info['text']
    else:
        raise EnvironmentError("Could not find the value for the time format due to the UI structure. Please adjust UIAutomator code")
    try:
        time_form = datetime.datetime.strptime(time_format, "%H:%M")
        return_format = "24-hour"
    except ValueError:
            time_form = datetime.datetime.strptime(time_format, "%H:%M %p")
            return_format = "AM/PM"

    return return_format

def set_time_format():
    """
    This function changes the time format with a click
    """
    if d(text="Use 24-hour format").exists:
        if d(text="Use 24-hour format").click():
            return True
        else:
            return False
    else:
        return False


def check_time_format(first_time_format, second_time_format):
    """
    the time_format strings are in str format as returned by get_time_format, str
    format_to_check: string
    returns True if the value is different from the initial one and False if it is the same
    """
    #TODO: should it also check if the button is checked or not in the interface for this assert?
    if first_time_format != second_time_format:
        return True
    else:
        return False


def launch_settings_panel():
    d.press.home()
    d(description="Apps").click()
    if d(text="Settings").exists:
        d(text="Settings").click()
        d(packageName="com.android.settings", className="android.widget.ListView").fling.toEnd()
        return True
    elif d(scrollable=True).scroll.horiz.to(text="Settings"):
        d(text="Settings").click()
        d(packageName="com.android.settings", className="android.widget.ListView").fling.toEnd()
        return True
    else:
        return False

def launch_date_time_settings():
    if not launch_settings_panel():
        return False
    else:
        if d(text="Date & time").exists:
            d(text="Date & time").click()
            return True
        else:
            return False