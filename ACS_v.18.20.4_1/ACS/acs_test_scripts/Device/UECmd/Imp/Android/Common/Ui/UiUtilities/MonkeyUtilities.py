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
:summary: Host Interface with embedded monkey engine
All Monkey commands can be find at
http://sourcebrowser.tl.intel.com:8080/source/xref/ABSP-gingerbread/development/cmds/monkey/

:since: 2011-5-19
:author: fhu2
"""
import select
import socket
import time
import errno

from UtilitiesFWK.Utilities import Global, internal_shell_exec, run_local_command
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class MonkeyUtilities():

    def __init__(self,
                 host="localhost",
                 port=6666,
                 protect_from_oom=False,
                 device=None):
        """
        Constructor
        """
        self._monkey_process = None
        self._adb_monkey_socket = None
        self._ui_autom_process = None
        self.__protect_from_oom = protect_from_oom
        self._device = device
        self._host = host
        self._port = port
        self._logger = LOGGER_TEST_SCRIPT

    def get_process_id(self, process_name):
        """
        Retrieve the process id according to his name

        :type process_name: str
        :param process_name: the process name we are looking for

        :rtype: str or None
        :return: process id if has been found
        """
        monkey_id = None
        if process_name:
            ps_result, ps_output = self._device.run_cmd(cmd="adb shell ps", timeout=1, silent_mode=True)
            if ps_result == Global.SUCCESS and ps_output:
                ps_lines = ps_output.splitlines()
                # for each process line
                for ps_line in ps_lines:
                    # split info in a list and remove empty char
                    ps_info = ps_line.split()
                    if ps_info and process_name in ps_info[-1]:
                        monkey_id = ps_info[1] if ps_info[1].isdigit() else None
                        break
        return monkey_id

    def _monkey_start(self, timeout):
        # Map embedded port to host port
        result, output = internal_shell_exec(self._device.format_cmd(
            "adb forward tcp:%s tcp:%s" % (self._port, self._port)), 5)

        if result == Global.SUCCESS:
            # Kill all previous monkey application
            monkey_process = self.get_process_id("com.android.commands.monkey")
            if monkey_process:
                result, output = self._device.run_cmd("adb shell kill -9 {0}".format(monkey_process), timeout)
            else:
                self._device.get_logger().debug("No running monkey process")
        if result == Global.SUCCESS:
            # monkey is a blocking application => start it and keep a ref on the process
            # Used with logwrapper, all monkey output will be written in logcat
            self._monkey_process, _ = run_local_command(
                self._device.format_cmd("adb shell logwrapper monkey --port %s" % self._port))
            time.sleep(1)

        if result == Global.SUCCESS and self._monkey_process.poll() is None and self.__protect_from_oom:
            monkey_process = self.get_process_id("monkey")
            if monkey_process:
                # Update "out of memory" policy to kill monkey only on critical situation
                self._device.run_cmd("adb shell echo -15 > /proc/%s/oom_adj" % monkey_process, timeout)
                # Create the socket connection
                self._adb_monkey_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._adb_monkey_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._adb_monkey_socket.settimeout(1)
                self._adb_monkey_socket.connect((self._host, self._port))
            else:
                output = "Cannot update oom monkey rules, monkey process not found with ps"
                result = Global.FAILURE

        return result, output

    def _monkey_send(self, data):
        if self._adb_monkey_socket is not None:
            try:
                self._adb_monkey_socket.send(data)
            except socket.error as socket_ex:
                self._logger.error(" adb socket monkey send - Following error occurs (error %s)" (str(socket_ex)))
            except IOError as io_ex:
                if io_ex.errno == errno.EPIPE:
                    self._logger.error(" adb socket monkey send - Following error occurs (error %s)" (str(io_ex)))

    def _monkey_receive(self, timeout):
        data = ""
        if self._adb_monkey_socket is not None:
            done = False
            end_time = time.time() + timeout

            while time.time() < end_time and not done:
                # Wait for data in the input socket
                inputready, _, _ = select.select([self._adb_monkey_socket], [], [], .1)
                # Read data from the socket
                for s in inputready:
                    if s == self._adb_monkey_socket:
                        # Receive new data
                        newdata = self._adb_monkey_socket.recv(512)
                        # Check whether we expect more data or not
                        # We expect more data if:
                        # - the data buffer is still empty or
                        # - we still have read some data from the socket
                        data += str(newdata)
                        if self._adb_monkey_socket and data.endswith('\n'):
                            done = True
        # Return the data
        return data

    def _monkey_stop(self):
        self._monkey_send("quit\n")
        if self._adb_monkey_socket is not None:
            try:
                self._adb_monkey_socket.shutdown(socket.SHUT_RDWR)
                self._adb_monkey_socket.close()
                self._adb_monkey_socket = None
            except socket.error as socket_ex:
                self._logger.error(" adb socket monkey stop - Following error occurs (error %s)" %(str(socket_ex)))
            if self._monkey_process.poll() is None:
                try:
                    self._monkey_process.terminate()
                except socket.error as socket_ex:
                    self._logger.error(" adb socket monkey stop - Following error occurs (error %s)" %(str(socket_ex)))
            self._monkey_process = None

    def enable_ui_automator_log(self):
        """
        UI Automator can log more UI events in logcat.
        It can be usefull to set a verdict on UI actions
        """
        if self._ui_autom_process is None:
            # uiautomator is a blocking application => start it and keep a ref on the process
            # Used with logwrapper, all uiautomator output will be written in logcat
            self._ui_autom_process, _ = run_local_command(
                self._device.format_cmd("adb shell logwrapper uiautomator events"))

    def _ui_autom_stop(self):
        if self._ui_autom_process is not None and self._ui_autom_process.poll() is None:
            self._ui_autom_process.terminate()
        self._ui_autom_process = None

    def connect(self):
        return self._monkey_start(10)

    def disconnect(self):
        self._monkey_stop()
        self._ui_autom_stop()

    def long_touch(self, loc_x, loc_y, timeout, touch_delay=5):
        command = "touch down %s %s\n" % (loc_x, loc_y)
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        if response is None or response.find("OK") == -1:
            return Global.FAILURE

        time.sleep(touch_delay)

        command = "touch up %s %s\n" % (loc_x, loc_y)
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        if response is None or response.find("OK") == -1:
            return Global.FAILURE
        else:
            return Global.SUCCESS

    def double_touch(self, loc_x, loc_y, timeout):
        time_between_commands = 0.25
        command = "tap %s %s\n" % (loc_x, loc_y)

        # Initial tap
        self._monkey_send(command)
        response_initial_tap = self._monkey_receive(timeout)
        time.sleep(time_between_commands)

        # 1st tap
        self._monkey_send(command)
        response_first_tap = self._monkey_receive(timeout)
        time.sleep(time_between_commands)

        # 2nd tap
        self._monkey_send(command)
        response_second_tap = self._monkey_receive(timeout)

        # Check responses
        if response_first_tap is None or response_first_tap.find("OK") == -1 or response_second_tap is None \
            or response_second_tap.find("OK") == -1 or response_initial_tap is None \
            or response_initial_tap.find("OK") == -1:
            return Global.FAILURE

        return Global.SUCCESS

    def touch(self, loc_x, loc_y, timeout):
        command = "tap %s %s\n" % (loc_x, loc_y)
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        if response is None or response.find("OK") == -1:
            return Global.FAILURE
        else:
            return Global.SUCCESS

    def type(self, str_to_type, timeout):  # @ReservedAssignment
        if str_to_type:
            starts_with_space = str_to_type.startswith(" ")
            ends_with_space = str_to_type.endswith(" ")

            strings = str_to_type.split(" ")

            # space in the head
            if starts_with_space:
                if self.press("KEYCODE_SPACE", timeout) == Global.FAILURE:
                    return Global.FAILURE

            if self.type_word(strings[0], timeout) == Global.FAILURE:
                return Global.FAILURE

            # strings
            for string in strings[1:]:
                if self.press("KEYCODE_SPACE", timeout) == Global.FAILURE:
                    return Global.FAILURE

                if self.type_word(string, timeout) == Global.FAILURE:
                    return Global.FAILURE

            # space in the end
            if ends_with_space:
                if self.press("KEYCODE_SPACE", timeout) == Global.FAILURE:
                    return Global.FAILURE

    def type_word(self, word, timeout):
        command = "type %s\n" % word
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        if response is None or response.find("OK") == -1:
            status = Global.FAILURE
        else:
            status = Global.SUCCESS
        return status

    def generate_step(self, distance):
        return_value = 1
        if distance >= 10:
            return_value = 10
        elif distance >= 5:
            return_value = 5
        return return_value

    def generate_steps(self, start_x, start_y, stop_x, stop_y):
        distance_x = (stop_x - start_x) if stop_x >= start_x else (start_x - stop_x)
        distance_y = (stop_y - start_y) if stop_y >= start_y else (start_y - stop_y)
        coordinates = (0, 0)
        if distance_x >= distance_y:
            if distance_x != 0:
                step_x = self.generate_step(distance_x)
                step_y = step_x * distance_y / distance_x
                coordinates = (step_x, step_y)
        elif distance_y != 0:
            step_y = self.generate_step(distance_y)
            step_x = step_y * distance_x / distance_y
            coordinates = (step_x, step_y)
        return coordinates

    def drag(self, start_x, start_y, stop_x, stop_y, timeout):
        self.touch_move(start_x, start_y, stop_x, stop_y, timeout)

    def touch_down(self, loc_x, loc_y, timeout):
        command = "touch down %s %s\n" % (loc_x, loc_y)
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        status = Global.FAILURE
        if response is None or response.rstrip("\r\n") != "OK":
            status = Global.FAILURE
        else:
            status = Global.SUCCESS
        return status

    def touch_move(self, start_x, start_y, stop_x, stop_y, timeout):
        # PRESS
        command = "touch down %s %s\n" % (start_x, start_y)
        self._monkey_send(command)

        # COMPUTE MOUVEMENT
        touch_move_loc_list = []

        (step_x, step_y) = self.generate_steps(start_x, start_y, stop_x, stop_y)

        if step_x > step_y:
            if start_x < stop_x:
                while start_x + step_x < stop_x:
                    start_x += step_x
                    start_y = (start_y + step_y) if start_y < stop_y else (start_y - step_y)
                    touch_move_loc_list.append((start_x, start_y))
            else:
                while start_x - step_x > stop_x:
                    start_x -= step_x
                    start_y = (start_y + step_y) if start_y < stop_y else (start_y - step_y)
                    touch_move_loc_list.append((start_x, start_y))
        else:
            if start_y < stop_y:
                while start_y + step_y < stop_y:
                    start_y += step_y
                    start_x = (start_x + step_x) if start_x < stop_x else (start_x - step_x)
                    touch_move_loc_list.append((start_x, start_y))
            else:
                while start_y - step_y > stop_y:
                    start_y -= step_y
                    start_x = (start_x + step_x) if start_x < stop_x else (start_x - step_x)
                    touch_move_loc_list.append((start_x, start_y))

        # DO THE DRAG
        for movement in touch_move_loc_list:
            command = "touch move %s %s\n" % (movement[0], movement[1])
            self._monkey_send(command)

        # UNPRESS
        command = "touch up %s %s\n" % (stop_x, stop_y)
        self._monkey_send(command)

        response = self._monkey_receive(timeout)
        if response is not None and response.find("ERROR") == -1:
            return Global.SUCCESS
        else:
            return Global.FAILURE

    def touch_up(self, loc_x, loc_y, timeout):
        command = "touch up %s %s\n" % (loc_x, loc_y)
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        if response is None or response.rstrip("\r\n") != "OK":
            return Global.FAILURE
        else:
            return Global.SUCCESS

    def press(self, key, timeout):
        command = "press %s\n" % (key)
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        if response is None or response.find("OK") == -1:
            return Global.FAILURE
        else:
            return Global.SUCCESS

    def key_down(self, key, timeout):
        command = "touch down %s\n" % (key)
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        if response is None or response.find("OK") == -1:
            return Global.FAILURE
        else:
            return Global.SUCCESS

    def key_up(self, key, timeout):
        command = "touch up %s\n" % (key)
        self._monkey_send(command)
        response = self._monkey_receive(timeout)
        if response is None or response.find("OK") == -1:
            return Global.FAILURE
        else:
            return Global.SUCCESS

    def wake(self, timeout):
        command = "wake\n"
        try_nb = 0
        while try_nb < 2:
            self._monkey_send(command)
            response = self._monkey_receive(timeout)
            if response is not None and response.find("OK") != -1:
                return Global.SUCCESS
            try_nb += 1

        return Global.FAILURE
