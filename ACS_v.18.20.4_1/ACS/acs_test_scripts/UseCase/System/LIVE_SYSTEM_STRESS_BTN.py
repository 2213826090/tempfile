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
:summary: This UseCase is a stress test with the power button
:since: 16/07/2014
:author: pbluniex
"""
import time
from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from SYSTEM_SLEEP_BASE import SystemSleepBase
from Core.Report.Live.LiveReporting import LiveReporting
import re


class LiveSystemStressButton(SystemSleepBase):
    """
    Stress button class
    """
    def __init__(self, tc_name, global_config):
        """
        Setup of usecase
        """
        SystemSleepBase.__init__(self, tc_name, global_config)
        self._sysdebug_api = self._device.get_uecmd("SysDebug")
        self._system_api = self._device.get_uecmd("System")
        self._timeout = self._tc_parameters.get_param_value("TIMEOUT", 1800, int)
        self._duration = self._tc_parameters.get_param_value("ITERATION_DURATION", 180, int)
        self._cycles = self._tc_parameters.get_param_value("SUSPEND_CYCLES_REQUIRED_NB", 1, int)
        self._sleep_time = self._tc_parameters.get_param_value("POWER_BUTTON_INTERVAL", 30, int)
        if self._io_card is None:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "No IOCard configured")
    def __unplug(self):
        """
        Unplug the device
        """
        self._device.disconnect_board()
        self._io_card.usb_host_pc_connector(False)

        # Unplug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            self._io_card.wall_charger_connector(False)

    def __plug(self):
        """
        Plug the device
        """
        # plug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            self._io_card.wall_charger_connector(True)

        self._io_card.usb_host_pc_connector(True)
        time.sleep(self._wait_btwn_cmd)
        self._device.connect_board()

    def __is_critical_crashes(self):
        """
        Extract critical crashes from crashinfo xml
        """
        result = False
        crashtool_filter = "WDT|WDT_UNHANDLED|HWWDTWITHLOG|HWWDT_RESET|HWWDT_UNHANDLED|" \
                           "SWWDT_UNHANDLED|FABRICERR|MEMERR|INSTERR|SRAMECCERR|HWWDTLOGERR" \
                           "|SCUWDT|PLLLOCKERR|CHAABIHANG|CHAABIWDT|KERNELWDT|SCUCHAABIWDT|FABRICXML|" \
                           "UNDEFL1ERR|PUNITMBBTIMEOUT|VOLTKERR|VOLTSAIATKERR|LPEINTERR|PSHINTERR|FUSEINTERR" \
                           "|IPC2ERR|KWDTIPCERR|NORTHFUSEERR|IPANIC|IPANIC_SWWDT|UIWDT|MSHUTDOWN|KDUMP|VMMTRAP" \
                           "|VMM_UNHANDLED|PANIC|ASSERTION|EXCEPTION"
        critical_crash_types = crashtool_filter.split("|")
        xml_node = self._sysdebug_api.report()
        crash_infos = xml_node.find("CrashInfo")
        if crash_infos is not None:
            for crash in crash_infos.findall('crash'):
                crash_type = crash.get("type")
                if crash_type is not None and crash_type in critical_crash_types:
                    self._logger.debug("Critical crash detected, id: %s, type: %s" % (crash.get("eventid"), crash_type))
                    result = True
        return result

    def __get_s3_counter(self):
        """
        Extract s3 counter informations from the device
        """
        _, output = self._device.run_cmd("adb shell test -f /sys/kernel/debug/suspend_time"
                                         " && cat /sys/kernel/debug/suspend_time"
                                         " || cat /sys/kernel/debug/sleep_time", 5)
        groups = re.findall("\s*\d+\s-\s+\d+\s+(\d+)", output)
        s3_value = 0
        for value in groups:
            s3_value += int(value)

        return s3_value

    def run_test(self):
        """
        Execute the test
        """
        # Call live sleep use case base
        SystemSleepBase.run_test(self)

        # Get s3 counters base value
        base_value = self.__get_s3_counter()

        # Initialize CrashInfo Sysdebug module
        self._sysdebug_api.init("CrashInfo")

        # Initialize and start iterations
        counter = 0
        iteration_num = 1
        iterations_crash = []
        iterations_reboot = []

        endtime = time.time() + self._timeout
        while counter < self._cycles:
            # Start iteration
            verdict = "FAILED"
            LiveReporting.instance().send_start_tc_info(tc_name="suspend_resume_stress_test", iteration=True)

            # RAZ Sysdebug module
            self._sysdebug_api.reset()
            self._sysdebug_api.start()

            # Get device uptime before usb unplug
            base_time = self._system_api.get_uptime()

            # Unplug USB
            self.__unplug()

            # Run suspend resume
            cycle_time = time.time() + self._duration
            while time.time() < cycle_time:
                self._io_card.press_power_button(0.2)
                time.sleep(self._sleep_time)

            # plug USB
            self.__plug()
            self._sysdebug_api.stop()

            # Get device uptime after USB replug
            current_time = self._system_api.get_uptime() + (self._duration*1000)

            # Check and update s3 counter
            current_value = self.__get_s3_counter()
            if counter > 0:
                iteration_value = current_value - (base_value + counter)
            else:
                iteration_value = current_value - base_value

            if iteration_value > 0:
                counter += iteration_value
                verdict = "PASSED"

            # Check if critical crash occurred
            if self.__is_critical_crashes():
                iterations_crash.append(iteration_num)
                self._logger.error("One or several critical crashes happened on the board")
                verdict = "FAILED"

            # Check if board reboot occurred
            if current_time < base_time:
                iterations_reboot.append(iteration_num)
                self._logger.error("Reboot of the board detected")
                verdict = "FAILED"

            # push iteration value to TCR
            LiveReporting.instance().update_running_tc_info(test_info={"metrics": [{"name": "s3_reached_count", "value": iteration_value, "unit": "NA"}]},
                                                            iteration=True)
            # End of iteration
            LiveReporting.instance().send_stop_tc_info(verdict=verdict,
                                                       execution_nb=1,
                                                       success_counter=1,
                                                       max_attempt=1,
                                                       acceptance_nb=1,
                                                       tc_comments="",
                                                       iteration=True)
            # Fail test if time is out
            if time.time() > endtime:
                msg = "TIMEOUT, failed to reach %d times s3 sleep mode, only %d reached" % (self._cycles, counter)
                if len(iterations_crash) > 0:
                    msg += ", crash detected during iterations: %s" % iterations_crash
                if len(iterations_reboot) > 0:
                    msg += ", reboot of the board detected during iterations: %s" % iterations_reboot
                return Global.FAILURE, msg

            # Increment for next iteration
            iteration_num += 1

        # push global value to TCR
        LiveReporting.instance().update_running_tc_info(test_info={"metrics": [{"name": "s3_reached_count", "value": counter, "unit": "NA"}]})

        return Global.SUCCESS, "No Error"