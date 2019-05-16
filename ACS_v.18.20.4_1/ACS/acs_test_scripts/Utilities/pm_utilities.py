"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL AVE SV
@since: 3/11/2014
@author: Christopher Hyatt christopher.r.hyatt@intel.com
@summary:
Common functions that are used in the PM regressions.
Class and Descriptions:

PowerStates:
Supports all commands to limit and expand sleep
states for the Device.

PowerGating:
Supports all the power gating test.

Utilities:
Support functions
"""

import re
import subprocess
import time
import tempfile
import serial
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Global, internal_shell_exec
from collections import defaultdict


'''
The DriverStates dictionary is used for the power gating test
and helps parsing the mid_pmu_states file when testing for
multiple drivers.
'''
DriverStates = {
                "EMMC": ["lss:02"],
                "SDIO0" : ["lss:01"],
                "SDIO2" : ["lss:04"],
                "UART" : ["lss:31"],
                "SSP3" : ["lss:17"],
                "SSP5" : ["lss:18"],
                "SSP6" : ["lss:19"],
                "I2C1" : ["lss:20"],
                "I2C2" : ["lss:21"],
                "I2C3" : ["lss:22"],
                "I2C4" : ["lss:23"],
                "I2C5" : ["lss:24"],
                "I2C6" : ["lss:26"],
                "I2C7" : ["lss:27"],
                "MIPI": ["lss:05"],
                "CHAABI" : ["lss:06"],
                "I2C1" : ["lss:20"],
                "AUDIO FABRIC" : ["lss:10"],
                "USB HSIC" : ["lss:08"],
                "USB3" : ["lss:09"],
                "pciPTI" : ["lss:16"],
                "DMA" : ["lss:25"],
                "PSH" : ["lss:00"],
                "PWM" : ["lss:36"],
                "pci" : ["lss:15"],
                "SSIC" : ["lss:07"],
                "GFX" : ["GFXSLC", "GSDKCK", "GRSCD"],
                "VED" : ["VED"],
                "VEC" : ["VEC"],
                "ISP" : ["ISP"],
                "3D_MARK" : ["GFXSLC", "GSDKCK", "GRSCD", "lss:10"],
                "CAMERA" : ["GFXSLC", "GSDKCK", "ISP", "VEC", "DPA", "MIO"],
                "VIDEO_PLAYBACK" : ["GFXSLC", "GSDKCK", "VED", "DPA", "MIO"],
                "SDIO" : ["lss:31", "lss:04"]}

class PowerStates:

    '''
        Function Name: s0i1_only
        Description: Limits sleep states to s0i1
        Inputs: None
        Returns: None
    '''
    def s0i1_only(self):
        self.reset_all_s_states()
        self.disable_s3()
        self.disable_s0i3()

    '''
        Function Name: s0i3_only
        Description: Limits sleep states to s0i3
        Inputs: None
        Returns: None
    '''
    def s0i3_only(self):
        self.reset_all_s_states()
        self.disable_s3()
        self.disable_s0i1()

    '''
        Function Name: disable_s0i3
        Description: Blocks s0i3 as a sleep state
        Inputs: None
        Returns: None
    '''
    def disable_s0i3(self):

        cmd = ["adb", "shell", "echo", "9", ">", "/d/cstate_ignore_add"]
        internal_shell_exec(cmd, 100)

    '''
        Function Name: enable_s0i3
        Description: Allows s0i3 sleep state
        Inputs: None
        Returns: None
    '''
    def enable_s0i3(self):

        cmd = ["adb", "shell", "echo", "9", ">", "/d/cstate_ignore_remove"]
        internal_shell_exec(cmd, 100)

    '''
        Function Name: disables_s0i1
        Description: Blocks s0i1 sleep state
        Inputs: None
        Returns: None
    '''
    def disable_s0i1(self):

        cmd = ["adb", "shell", "echo", "7", ">", "/d/cstate_ignore_add"]
        internal_shell_exec(cmd, 100)

    '''
        Function Name: enable_s0i1
        Description: Allows s0i1 sleep states
        Inputs: None
        Returns: None
    '''
    def enable_s0i1(self):

        cmd = ["adb", "shell", "echo", "7", ">", "/d/cstate_ignore_remove"]
        internal_shell_exec(cmd, 100)

    '''
        Function Name: disable_s3
        Description: Blocks s3 sleep state
        Inputs: None
        Returns: None
    '''
    def disable_s3(self):

        cmd = ["adb", "shell", "echo", "0", ">", "/d/s3_ctrl"]
        internal_shell_exec(cmd, 100)

    '''
        Function Name: enable_s3
        Description: Allows s3 sleep states
        Inputs: None
        Returns: None
    '''
    def enable_s3(self):

        cmd = ["adb", "shell", "echo", "1", ">", "/d/s3_ctrl"]
        internal_shell_exec(cmd, 100)

    '''
        Function Name: reset_all_s_states
        Description: Allows s3, s0i3, and s0i1 sleep states
        Inputs: None
        Returns: None
    '''
    def reset_all_s_states(self):

        self.enable_s0i1()
        self.enable_s0i3()
        self.enable_s3()
    '''
    Fucntion Name: clear_mid_pmu_states()
    Inputs: None
    Returns: None
    Description: Clears all counters and residencies from the mid_pmu_states
    file
    '''
    def clear_mid_pmu_states(self):
        cmd = ["adb", "shell", "echo", "clear", ">", "/d/cstate_ignore_remove"]
        internal_shell_exec(cmd, 100)


class Utilities:

    def __init__(self, DEVICE):

        self.SERIALOBJ = []
        self.serial_port_names = []
        self.serial_port_names = str(DEVICE.get_config("serialPort", "", str)).split(";")
        for name in self.serial_port_names:
            LOGGER_TEST_SCRIPT.debug("Serial Name: %s" % name)
            '''
            If name is blank it will not add port object
            '''
            if not name == "":
                self.SERIALOBJ.append(serial.Serial(baudrate=115200, port=name))

    '''
        Function Name: write_all_serial_ports
        Description: Writes to all serial ports defined in
            Bench_Config.xml
        Inputs: message
        Returns: None
    '''
    def write_all_serial_ports(self, message):

        if len(self.SERIALOBJ) == 0:
            LOGGER_TEST_SCRIPT.warning("NO SERIAL OBJECTS FOUND")
            return
        for obj in self.SERIALOBJ:
            obj.write(message)

class PowerGating:

    def __init__(self):
        self.power_states_api = PowerStates()
        self._logger = LOGGER_TEST_SCRIPT

    '''
    Function Name: test_power_gating
    Inputs:    Defined time in sec to sleep during power down phase (time_to_sleep)
            What driver_entry to read from the DriverStates dictionary (what_to_read)
            A command to run app/script to stress drivers (cmd_to_run)
    Returns: verdict (PASS, FAIL, BLOCKED) and an output message
    Description:  The function starts an app/script and while it is running it
    takes readings using the take_power_gate_reading() function to parse the
    mid_pmu_states file for a drivers power state.  This function will fail
    if at any point the drivers are not power gating during the stress phase.
    At the end of the stress phase mid_pmu_states are read at the beginning
    and end of a defined sleep window.  The results are compared to insure
    that drivers can return to sleep state after being stressed.
    '''
    def test_power_gating(self, time_to_sleep, what_to_read, cmd_to_run):
        f_stdout = tempfile.TemporaryFile()
        self.power_states_api.clear_mid_pmu_states()
        cmd = []
        message = ""
        cmd = str(cmd_to_run).split()
        proc = subprocess.Popen(cmd, stdout=f_stdout, stderr=subprocess.STDOUT)
        state_dic = defaultdict(list)
        time.sleep(10)
        while proc.poll() is None:
            self._logger.info("App is running")
            for driver_entry in DriverStates[what_to_read]:
                self._logger.debug("driver_entry: %s" % driver_entry)
                state, D0_count, D0_residency = self.take_power_gate_reading(driver_entry)
                self.process_readings(state, D0_count, D0_residency, state_dic, message, driver_entry)
                if state_dic is None:
                    proc.kill()
                    return Global.BLOCKED, message
            time.sleep(10)
        del(state_dic)
        self._logger.info("Finished with App")
        time.sleep(60)
        verdict = Global.FAILURE
        output = "failure"
        for driver_entry in DriverStates[what_to_read]:
            state1, D0_count1, D0_residency1 = self.take_power_gate_reading(driver_entry)
            time.sleep(60)
            state2, D0_count2, D0_residency2 = self.take_power_gate_reading(driver_entry)
            if D0_count1 is None and D0_count2 is None and state1 != "D0" and state1 == state2:
                verdict = Global.SUCCESS
                output = "Success"
            elif (state1 == state2 and D0_count1 == D0_count2) or (D0_residency2 is not None and float(D0_residency2) < float(D0_residency1)):
                verdict = Global.SUCCESS
                output = "Success"
            else:
                return Global.FAILURE, "Driver did not return to D0i3 state after stimulus"
        return verdict, output
    '''
    Function Name: take_power_gate_readings()
    Inputs: A string of what to grep for in mid_pmu_states
    Returns: A triple tuple
        state of driver (sleep states)
        count of being in D0
        residency of being in D0
    Description: The function will grep for a given driver line
    in the mid_pmu_states file and parses the line for state,
    D0 count and residency of teh D0 state.  This function relies on
    the formating of mid_pmu_states to stay the same.
    '''
    def take_power_gate_reading(self, what_to_read):
        self._logger.debug("Looking for: %s" % what_to_read)
        verdict, grep_output = internal_shell_exec(["adb", "shell", "cat",
                      "/d/mid_pmu_states", "|", "grep", what_to_read], 100)
        if not verdict == Global.SUCCESS:
            return None, None, None
        for line in grep_output.split("\n"):
            self._logger.debug("Line %s:" % line)
            line_array  = line.split()
            self._logger.debug(len(line_array))
            if len(line_array) == 11:
                return line_array[7], line_array[8], line_array[9]
            elif len(line_array) == 10:
                return line_array[6], line_array[7], line_array[8]
            elif len(line_array) == 9:
                return line_array[8], None, None
            elif len(line_array) == 6:
                return line_array[2], line_array[3], line_array[4]
            elif len(line_array) == 3:
                return line_array[2], None, None

    '''
    Function Name: bool_power_gating
    Inputs: The amount of time to test (timeout) and an driver_entry in the DriverStates
        dictionary (what_to_read)
    Returns: A bool array NOT THE BEST WAY TO DO THIS
    Description: The function will pares the mid_pmu_states for a
    given driver_entry in the DriverStates dictionary.  If the driver is
    is in a D0 state a True will be appended to the power_gating
    bool array, else a False is appended.
    '''
    def bool_power_gating(self, timeout, what_to_read):
        '''
        Returns an array of bools
        '''
        start_time = time.time()
        end_time = start_time + timeout
        power_gating = []
        while start_time < end_time:
            for driver_entry in DriverStates[what_to_read]:
                state1, D0_count1, D0_residency1 = self.take_power_gate_reading(driver_entry)
                self._logger.debug("State: %s D0_count: %s D0_res %s" %(state1, D0_count1, D0_residency1))
                time.sleep(10)
                state2, D0_count2, D0_residency2 = self.take_power_gate_reading(driver_entry)
                self._logger.debug("State: %s D0_count: %s D0_res %s" %(state2, D0_count2, D0_residency2))
                if D0_count1 is None and D0_count2 is None:
                    if state1 == "D0" and state1 == state2:
                        self._logger.info("%s IS IN D0" % driver_entry)
                        power_gating.append(True)
                    elif state1 == state2:
                        self._logger.debug("%s IS NOT IN D0" % driver_entry)
                        power_gating.append(False)
                elif D0_count1 == D0_count2:
                    self._logger.info("%s IS NOT POWER GATING" % driver_entry)
                    power_gating.append(False)
                else:
                    self._logger.debug("%s IS POWER GATING" % driver_entry)
                    power_gating.append(True)
            start_time = time.time()
        return power_gating

    '''
    Function Name: bool_power_gating2
    Inputs: The amount of time to test (timeout), an driver_entry in the DriverStates
        dictionary (what_to_read) and a bool so that the function can be
        for both the stimulated and non-stimulated scenarios.
    Returns: A single bool
    Description: The function will parse the mid_pmu_states for a
    given driver_entry in the DriverStates dictionary.  This function
    differ in many ways than the one above.  This one runs faster
    by parsing the mid_pmu_states rather than using grep.  It also
    keeps a running count of D0 states and then makes a decision
    at the end of the function call.
    '''
    def bool_power_gating2(self, timeout, what_to_read, stimulated=True):

        start_time = time.time()
        end_time = start_time + timeout
        state_dic = defaultdict(list)
        power_gating = True
        message = ""
        while start_time < end_time:
            verdict, mid_pmu_states = internal_shell_exec(["adb", "shell", "cat",
                      "/d/mid_pmu_states"], 100)
            if not verdict == Global.SUCCESS:
                return None, "Could not find mid_pmu_states"
            for driver_entry in DriverStates[what_to_read]:
                state, D0_count, D0_residency = self.get_readings_from_given_mid_pmu_states2(driver_entry, mid_pmu_states)
                self._logger.debug("State: %s D0_count: %s D0_res %s" %(state, D0_count, D0_residency))
                self.process_readings(state, D0_count, D0_residency, state_dic, message, driver_entry)
                if state_dic is None:
                    return Global.BLOCKED, message
            time.sleep(10.00)
            start_time = time.time()
        result_out = ""
        for driver_entry in DriverStates[what_to_read]:
            if stimulated and state_dic[driver_entry][3] > 0:
                result_out = result_out + "%s power gated %d times\n" % (driver_entry, int(state_dic[driver_entry][3]))
            elif not stimulated and state_dic[driver_entry][3] < 20:
                result_out = result_out + "%s power gated %d times\n" % (driver_entry, int(state_dic[driver_entry][3]))
            else:
                power_gating = False
                result_out = result_out + "%s power gated %d times\n" % (driver_entry, int(state_dic[driver_entry][3]))

        return power_gating, result_out

    '''
    Function Name: test_power_gating2
    Inputs: Defined time in sec to sleep during power down phase (time_to_sleep)
            What driver_entry to read from the DriverStates dictionary (what_to_read)
            A command to run app/script to stress drivers (cmd_to_run)
            Timeout value in seconds (timeout)
    Returns: verdict (PASS, FAIL, BLOCKED) and an output message
    Description:  The function starts an app/script and while it is running it
    takes readings using the take_power_gate_reading() function to parse the
    mid_pmu_states file for a drivers power state.  This function will fail
    if the running count for D0 states is 0 while stressed or more than 0
    during the power down phase of the test.
    At the end of the stress phase mid_pmu_states are read at the beginning
    and end of a defined sleep window.  The results are compared to insure
    that drivers can return to sleep state after being stressed.
    '''
    # MODIFIED
    def test_power_gating2(self, time_to_sleep, what_to_read, cmd_to_run, timeout):
        f_stdout = tempfile.TemporaryFile()
        self.power_states_api.clear_mid_pmu_states()
        cmd = []
        message = ""
        cmd = str(cmd_to_run).split()
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr = subprocess.PIPE)
        state_dic = defaultdict(list)
        # time.sleep(10)
        self._logger.debug("PROC IS %s" % proc.poll())
        while (proc.poll() is None and timeout > 0):
            t0 = time.time()
            verdict, mid_pmu_states = internal_shell_exec(["adb", "shell", "cat", "/d/mid_pmu_states"], 100)
            if not verdict == Global.SUCCESS:
                return Global.BLOCKED, "Did not receive mid_pmu_states back from device"
            for driver_entry in DriverStates[what_to_read]:
                state, D0_count, D0_residency = self.get_readings_from_given_mid_pmu_states2(driver_entry, mid_pmu_states)
                self._logger.debug("State: %s D0_count: %s D0_res %s" %(state, D0_count, D0_residency))
                self.process_readings(state, D0_count, D0_residency, state_dic, message, driver_entry)
                if state_dic is None:
                    return Global.BLOCKED, message
            t1 = time.time()
            timeout -= (t1 - t0)
        power_gating = True
        result_out = ""
        self._logger.debug(f_stdout.read())
        for driver_entry in DriverStates[what_to_read]:
            if int(state_dic[driver_entry][3]) > 0:
                result_out = result_out + "%s power gated %d times\n" % (driver_entry, int(state_dic[driver_entry][3]))
                self._logger.debug(result_out)
            else:
                power_gating = False
                result_out = result_out + "%s power gated %d times\n" % (driver_entry, int(state_dic[driver_entry][3]))
                self._logger.debug(result_out)
        if power_gating is False:
            self._logger.debug(result_out)
            return Global.FAILURE, result_out
        else:
            self._logger.debug(result_out)
            return Global.SUCCESS, result_out

    '''
    Function Name: process_readings
    Inputs: state, D0_count, D0_residency, state_dic, message
    Returns: an updated state_dic and message
    Description:  The function takes the state, D0_count, and
    D0_residency and updates the state_dic depending on the
    values of the three.  If there is an error than the
    state_dic will be returned a None value and a message will
    describe the error.
    '''
    def process_readings(self, state, D0_count, D0_residency, state_dic, message, driver_entry):

        self._logger.debug("State: %s D0_count: %s D0_res %s\
                " %(state, D0_count, D0_residency))
        if state is None:
            state_dic = None
            message = "No Driver Was Found"
            return
        elif D0_count is None:
            if len(state_dic[driver_entry]) == 0:
                self._logger.debug("First Time driver_entry\
                     for %s" % driver_entry)
                state_dic[driver_entry].append(state)
                state_dic[driver_entry].append("N/A")
                state_dic[driver_entry].append("N/A")
                state_dic[driver_entry].append(0)
            else:
                if state == "D0":
                    state_dic[driver_entry][3] = int(state_dic[driver_entry][3]) + 1
            self._logger.info("Driver %s is in %s" %(driver_entry, state))
        elif D0_residency is not None:
            self._logger.info("Driver %s is in %s for %s \
                percent of the time and has a count of %s\
                " % (driver_entry, state, D0_residency, D0_count))
            if len(state_dic[driver_entry]) == 0:
                self._logger.debug("First time: loading dictionary")
                state_dic[driver_entry].append(state)
                state_dic[driver_entry].append(D0_count)
                state_dic[driver_entry].append(D0_residency)
                state_dic[driver_entry].append(0)
            else:
                if float(state_dic[driver_entry][1]) < float(D0_count):
                    self._logger.debug("The driver is power gating")
                    state_dic[driver_entry][0] = state
                    state_dic[driver_entry][1] = D0_count
                    state_dic[driver_entry][2] = D0_residency
                    state_dic[driver_entry][3] = int(state_dic[driver_entry][3]) + 1
                else:
                    self._logger.error("D0_COUNT: %s" % D0_count)
                    self._logger.error("D0_COUNT last run: %s" % state_dic[driver_entry][1])
        else:
            self._logger.error("ERROR IN PROCESSING driver_entry")
            state_dic = None
            message = "ERROR IN PROCESSING driver_entry"



    '''
    Function Name: get_readings_from_given_mid_pmu_states
    Inputs: string for driver name in mid_pmu_states(what_to_read)
            the mid_pmu_states file copied from the device(mid_pmu_states)
    Returns:  Triple Tuple for device state, D0 count, and D0 residency
    Description: Given a copy of mid_pmu_states the function will parse
    file for a given driver name and return driver state, D0 count, and
    D0 residency
    '''
    def get_readings_from_given_mid_pmu_states(self, what_to_read, mid_pmu_states):
        self._logger.debug("LOOKING FOR : %s" % what_to_read)
        for line in mid_pmu_states.split("\n"):
            self._logger.debug("LOOKING AT LINE : %s" % line)
            if re.search(what_to_read, line) is not None:
                self._logger.debug("Line %s:" % line)
                line_array  = line.split()
                self._logger.debug(len(line_array))
                if len(line_array) == 11:
                    return line_array[7], line_array[8], line_array[9]
                elif len(line_array) == 10:
                    return line_array[6], line_array[7], line_array[8]
                elif len(line_array) == 9:
                    return line_array[8], None, None
                elif len(line_array) == 6:
                    return line_array[2], line_array[3], line_array[4]
                elif len(line_array) == 3:
                    return line_array[2], None, None

    '''
    Function Name: get_readings_from_given_mid_pmu_states2
    Inputs: string for driver name in mid_pmu_states(what_to_read)
            the mid_pmu_states file copied from the device(mid_pmu_states)
    Returns:  Triple Tuple for device state, D0 count, and D0 residency
    Description: Given a copy of mid_pmu_states the function will parse
    file for a given driver name and return driver state, D0 count, and
    D0 residency.  This takes care of some formatting disparities between
    mid_pmu_states formatting issues.
    '''
    def get_readings_from_given_mid_pmu_states2(self, what_to_read, mid_pmu_states):
        state = None
        count = None
        res = None
        self._logger.debug("Looking for : %s" % what_to_read)
        for line in mid_pmu_states.split("\n"):
            self._logger.debug("Looking at line : %s", line)
            line_match = re.search(r"\s+(%s.+)\s+" % what_to_read, mid_pmu_states)
            if line_match is not None:
                self._logger.debug("Phish Phish got my wish : %s" % line_match.group(1))
                state_match = re.search(r"\s+(D0.*)\s+", line_match.group(0))
                if state_match:
                    self._logger.debug("Found State : %s" % state_match.group(1))
                    state = state_match.group(1)
                count_match = re.search(r"\s+(\d+)\s+", line_match.group(0))
                if count_match:
                    self._logger.debug("Found Count : %s" % count_match.group(1))
                    count = count_match.group(1)
                res_match = re.search(r"\s+(\d+\.\d+)\s+", line_match.group(0))
                if res_match:
                    self._logger.debug("Found Residency : %s" % res_match.group(1))
                    res = res_match.group(1)
                return state, count, res
        #fail case returns all None calling function should take care of the error
        self._logger.error("Line was not found for %s" % what_to_read)
        return None, None, None
