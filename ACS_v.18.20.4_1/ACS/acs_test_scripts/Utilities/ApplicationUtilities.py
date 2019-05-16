"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
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

:organization: INTEL OTC Android
:summary: methods for applications that are package-independent and can be solved through intents
          so far: alarm clock
:since: 4/21/15
:author: mmaraci
"""
from ErrorHandling.DeviceException import DeviceException
from uiautomator import Device


class ApplicationUtilities():
    def __init__(self, device):
        self.device = device
        self._uidevice = Device(device.retrieve_serial_number())

    def set_alarm_clock(self, extra_message, extra_hour, extra_minutes):
        """
        This method sets an alarm clock with the specified parameters
        :param extra_message: string
        :param extra_hour: int, 24h format
        :param extra_minutes: int
        :return:
        """

        intent = "adb shell am start -a android.intent.action.SET_ALARM "
        put_extra_message = " --es android.intent.extra.alarm.MESSAGE " + str(extra_message)
        put_extra_hour = " --ei android.intent.extra.alarm.HOUR " + str(extra_hour)
        put_extra_minutes = " --ei android.intent.extra.alarm.MINUTES " + str(extra_minutes)

        cmd = intent + put_extra_message + put_extra_hour + put_extra_minutes

        result, result_output = self.device.run_cmd(cmd, 20)

        if ('Error' in result_output) or ('Warning' in result_output):
            # an error occurred, so we return a False, just to know this is a game killer
            return False

        return True

    def show_alarm(self):

        intent = "adb shell am start -a android.intent.action.SHOW_ALARMS  "

        result, result_output = self.device.run_cmd(intent, 20)
        control_string = 'Error: Activity not started, unable to resolve Intent'

        if control_string in result_output:
            # an error occurred, so we return a False, just to know this is a game killer
            return False

        return True

    def check_alarm_clock(self, int_hour, int_minutes):
        """
        This methods greps for the alarms on the device and checks that there is one set for
        the specified given time
        :param int_hour: int, 24h format
        :param int_minutes: int
        :return:
        """
        self.show_alarm()

        if int(int_minutes) < 10:
            int_minutes = "0" + int_minutes
        grep_cmd = "adb shell dumpsys alarm|grep -B 5 -A 5 deskclock|grep -A 5 indicator|grep " + int_hour + ':' + int_minutes
        control_string = int_hour + ':' + int_minutes

        result, result_output = self.device.run_cmd(grep_cmd, 20)

        if control_string not in result_output:
            # this means that no such alarm is reported
            return False

        return True

    def get_system_clock(self):
        """
        Get the current device time
        :return: the current system time as a tuple: hour and minutes
        """

        get_date = " adb shell date '+%H:%M'"

        result, result_output = self.device.run_cmd(get_date, 20)

        if not result_output:
            result_output = "0:0"

        print result_output
        current_hour = int(result_output.split(':')[0])
        current_minutes = int(result_output.split(':')[1])

        return current_hour, current_minutes

    def force_stop_clock(self):

        force_stop_cmd = ' adb shell am force-stop com.google.android.deskclock '
        result, output = self.device.run_cmd(force_stop_cmd, 20)
