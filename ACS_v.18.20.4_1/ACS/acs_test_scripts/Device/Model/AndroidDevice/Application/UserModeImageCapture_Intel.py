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

:organization: INTEL MCG PSI
:summary: This script implements the User mode image capture for power measurement
:since: 19/04/2013
:author: pbluniex
"""
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IApplication import IApplication
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Multimedia.Camera import Camera
from ErrorHandling.AcsConfigException import AcsConfigException
import re


class UserModeImageCapture_Intel(IApplication):

    """
    Implementation of user mode image capture
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IApplication.__init__(self, device)
        self._camera = None
        self._camera_name = "camera"
        self._arguments = None
        self._capture_number = None
        self._sleep_time_between_capture = None
        self._flash_mode = None
        self._screen_mode = None
        self._execution_file = None
        self._device = device
        self.__umic_report_path = None
        self.__um_capture_pid = None

    def start(self):
        """
        Start UMIC application
        """
        self._camera.launch()

        # take a screenshot
        filename = self.__umic_report_path + "/" + time.strftime("%Y%m%d.%H%M") + ".png"
        self._device.screenshot(filename=filename)

    def __get_arguments_value(self, arg_name):
        """
        get the value of an argument
        """
        val = None
        if self._arguments is not None:
            # split arguments list
            args_list = self._arguments.split(";")
            for args in args_list:
                # split argument name and argument value
                curr_args = args.split("=")
                if arg_name in curr_args[0] and len(curr_args) == 2:
                    val = curr_args[1].strip()
        return val

    def __generate_and_push_sh(self):
        """
        generate .sh file and push it on device
        """

        # create dir
        cmd = "mkdir /data/usermode"
        self.adb_shell(cmd, 3)

        # write commend list in sh file
        cmd_list = "sleep 30;\n" \
                   "for i in `seq 1 %d`;\n"\
                   "do input keyevent 27;\n"\
                   "sleep %d;\n"\
                   "done" % (self._capture_number, self._sleep_time_between_capture)

        cmd = "echo '%s' > /data/usermode/user_mode_capture.sh" % cmd_list
        self.adb_shell(cmd, 3)

        # change rights on file
        cmd = "chmod 777 /data/usermode/*"
        self.adb_shell(cmd, 3)

    def __get_existing_sh_pids(self):
        """
        From the ps command, it greps all shell processes to be
         taken into account to kill User Mode shell script in
         the function "__stop_um_capture".

        @return: a list of shell pids
        @rtype: list of ints
        """
        #Get the full ps output
        pids = "ps"
        #ps_output_list = self.adb_shell(pids, 10)
        (ret_code, ps_output_list) = self._device.run_cmd("adb shell ps", 10)

        #Generate the RegEx
        ps_line_str = "(root|shell)\s+(?P<pid>\d{1,5})(.*?)\s+(/system/bin/sh|sh)"
        #Iterator
        iterator = re.finditer(ps_line_str, ps_output_list)
        #List comprehension
        ps_shell_list = [int(ps_element.group("pid")) for ps_element in iterator]

        self._logger.debug("Shell pid list: %s " % str(ps_shell_list))

        return ps_shell_list

    def __stop_um_capture(self):
        """
        Stops User Mode Image Capture shell script.

        @return: the return code of the operation
        @rtype: str
        """
        cmd_str = "kill %s" % self.__um_capture_pid
        response = self.adb_shell(cmd_str, 5)
        self._logger.debug("Response: %s" % str(response))

        return response

    def __get_umic_pid(self, existing_sh_pids):
        """
        Stops User Mode Capture shell script by killing the shell process.
        It takes into account other shell scripts previously existing.

        @param existing_sh_pids: previously-existing list of shell pids
        @type existing_sh_pids: list of str

        @return: returns User Mode Image Capture pid value.
        @rtype: int
        """
        self._logger.debug("Stop User Mode Image Capture shell script.")
        #Get list of shell pids, including user_mode_image_capture pid
        ps_shell_list = self.__get_existing_sh_pids()

        #Substraction of user_mode_pid from before and after list
        user_mode_pid_list = list(set(ps_shell_list) - set(existing_sh_pids))

        #Prevention of wrong cases
        if len(user_mode_pid_list) == 0:
            self._logger.warning("No user_mode_image_capture pid. Continue.")
            return
        elif len(user_mode_pid_list) > 1:
            self._logger.warning("PID list: %s" % str(user_mode_pid_list))
            self._logger.warning("System can't extract which one is user_mode_image_capture \
                pid. Continue.")
            return

        user_mode_pid = user_mode_pid_list[0]
        self._logger.debug("user_mode_pid: %s" % str(user_mode_pid))

        return user_mode_pid

    def install(self, appuri, additionnals=None, arguments=None, destination=None):
        """
        install the application : create .sh file and push it on device
        """
        IApplication.install(self, appuri, additionnals, arguments, destination)

        # get number of capture
        capture_number = self.__get_arguments_value("NumberOfIteration")
        # default value_execution_file
        if capture_number is None:
            self._capture_number = 10
        else:
            self._capture_number = int(capture_number)

        # get sleep time between capture
        sleep_time = self.__get_arguments_value("SleepTimeBetweenCapture")

        # default value
        if sleep_time is None:
            self._sleep_time_between_capture = 30
        else:
            self._sleep_time_between_capture = int(sleep_time)

        self._camera = Camera(self._device)

        # generate and push application file
        self.__generate_and_push_sh()

    def drive(self):
        """
        drive the application
        """
        existing_sh_pids = self.__get_existing_sh_pids()

        cmd = "nohup sh /data/usermode/user_mode_capture.sh "
        self.adb_shell(cmd, 3)
        self.__um_capture_pid = self.__get_umic_pid(existing_sh_pids)

    def stop(self):
        """
        Stop UMIC application
        """
        self._camera.legacy_shutdown()
        self.__stop_um_capture()

    def post_install(self):
        """
        Post installation configuration
        """
        # deactivate auto brightness
        self._phonesystem.set_brightness_mode("manual")

        # configure camera configuration
        self._flash_mode = self.__get_arguments_value("FlashMode")
        # default value
        if self._flash_mode is None:
            self._flash_mode = "Off"

        self._screen_mode = self.__get_arguments_value("ScreenMode")
        # default value
        if self._screen_mode is None:
            self._screen_mode = "Standard"

        self._camera.setup(self._flash_mode, self._screen_mode, self._camera_name)

        # create a sub folder in the report folder which contains screenshots
        result_folder = "screenshots"

        report_dir = self._get_report_tree()
        report_dir.create_subfolder(result_folder)
        self.__umic_report_path = report_dir.get_subfolder_path(result_folder)

    def uninstall(self):
        """
        remove shell file from the device
        """
        cmd = "rm -r /data/usermode"
        self.adb_shell(cmd, 3)
