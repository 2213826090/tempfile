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
:summary: This file implements the System UEcmd for Android device
:author: pbluniex
"""
import re
import os
import time
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.ISleepMode import ISleepMode
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.PnPUtilities import grouped_regex_from_list

####################################################
# Classes for device-dependant counters operations #
####################################################

class SleepModeGeneric(Base):
    """
    :summary: SleepMode interface for common methods
    using an C{Intent} based communication to the I{DUT}.
    """
    _sleep_modes = ["s0"]                  # list of the sleep modes available on the device
    _sleep_mode_file = None            # path of the residency file
    _sleep_mode_type = "UNKNOWN"       # type of sleep mode used
    _dstates_files = []                # list of files where d-states informations can be found
    _clearable = False                 # True if echo 'clear' > _sleep_mode_file clears the residencies
    _states = []  # Define here a 'hardcoded' list of states, in order to handle them smartly :
                  # 'name' is the state name
                  # 'disable' is the list of c-states to disable to enter in it
                  # 'other_names' is a list of names/nicknames that can be given to this state
                  # 'suspend' should be False if acquiring a wakelock helps to enter this state
    _default_counters_fmt = "[\d\.-]*" # Default regexp for counters values
    _counters_fmt = {                  # define here format of fields not matching this regexp
        'mode':'[^\s]*',
    }
    _counters_eol = "$"                # define here the pattern of the end of the line
    _pattern = ["^s0", "^s0i.*", "^lpmp3", "^s3"]

    def __init__(self, device):
        """
        Constructor
        """
        Base.__init__(self, device)
        self._system_api = self._device.get_uecmd("System")

    def get_sleep_modes(self):
        """
        Get the sleep modes available on this device (should be by parsing the residency file)
        """
        return self._sleep_modes

    def get_sleep_mode_file(self):
        """
        Get the sleep mode file path
        """
        return self._sleep_mode_file

    def get_sleep_mode_type(self):
        """
        Get the sleep mode type (PMU_STATES, PMC_ATOM, TELEMETRY_ATOM...)
        """
        return self._sleep_mode_type

    def is_sleep_mode_clearable(self):
        """
        True if the residencies can be cleared by echoing 'clear' in the sleep mode file, else False
        """
        return self._clearable

    def get_d_states_files(self):
        """
        Returns the list of files where d-states informations can be retrieved
        """
        return self._dstates_files

    def get_states(self):
        """
        Returns the list of states pre-defined
        """
        return self._states

    def clear_counters(self):
        """
        Device-dependant operations to do in order to clear the counters
        """
        pass

    def parse_counters_header(self, header):
        """
        Put here the operations to do on the sleep mode file's header to have proper field names
        """
        return header

    def get_counter_fmt(self, counter_name):
        """
        Returns the regexp to be used for the given field name
        """
        if counter_name in self._counters_fmt:
            return self._counters_fmt[counter_name]
        return self._default_counters_fmt

    def get_counters_eol(self):
        """
        The string to add at the end of the regexp (default should be "$" for most platforms)
        """
        return self._counters_eol

    def pre_format_residencies(self, residencies):
        """
        Put here the operations to do on the residency list
        """
        for element in residencies:
            element["mode"] = self.format_state_name(element["mode"])

    def get_pattern(self):
        """
        Get the pattern to read residencies
        """
        return self._pattern

    def format_state_name(self, name):
        """
        Format the sleep mode name
        """
        return name.strip().replace(":", "").lower()

class SleepModeLegacy(SleepModeGeneric):
    """
    :summary: SleepMode management for legacy platforms (with mid_pmu_states)
    using an C{Intent} based communication to the I{DUT}.
    """
    _sleep_modes = []
    _sleep_mode_file = "/sys/kernel/debug/mid_pmu_states"
    _sleep_mode_type = "PMU_STATES"
    _dstates_files = ["/sys/kernel/debug/mid_pmu_states"]
    _clearable = True
    _states = [
        {'name':"lpmp3", 'disable': ["s0i3"], 'other_names': ["s0i1-lpe", "mp3", "s0i1", "audio"], 'suspend':False},
        {'name':"s0i1-lpe", 'disable': ["s0i3"], 'other_names': ["lpmp3", "mp3", "s0i1", "audio"], 'suspend':False},
        {'name':"s0i1-psh", 'disable': ["s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i1-disp", 'disable': ["s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i1-lpe-psh", 'disable': ["s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i1-lpe-disp", 'disable': ["s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i1-psh-disp", 'disable': ["s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i1-lpe-psh-disp", 'disable': ["s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i1", 'disable': ["s0i3"], 'other_names': ["idle-s0i1", "audio"], 'suspend':False},
        {'name':"s0i2", 'disable': ["s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i3-psh-ret", 'disable': ["s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i3", 'disable': ["s0i1"], 'other_names': ["idle-s0i3"], 'suspend':False},
        {'name':"s3", 'disable': [], 'other_names': ["suspend", "legacy-suspend"], 'suspend':True},
        {'name':"s0", 'disable': [], 'other_names': ["active", "audio"], 'suspend':False},
        # on sofia, we are in s0 for audio playback, as s0ix does not exist
    ]
    _default_counters_fmt = "[\d\.-]*"
    _pattern = ["^s0", "^s0i.*", "^lpmp3", "^s3"]

    def __init__(self, device):
        """
        Constructor
        """
        SleepModeGeneric.__init__(self, device)

    def pre_format_residencies(self, residencies):
        """
        Put here the operations to do on the residency list
        """
        # add a fake s0 residency for sofia
        if "s0" not in [element["mode"] for element in residencies]:
            tmp = {}
            tmp["mode"] = "s0"
            uptime = self._system_api.get_uptime()
            s0_time = uptime - sum([float(element["time"]) for element in residencies])
            tmp["time"] = s0_time
            tmp["residency"] = 100. * s0_time / uptime
            tmp["count"] = 0
            residencies.append(tmp)

    def get_sleep_modes(self):
        """
        get the list of available sleep modes on the device
        """
        if self._sleep_modes:
            return self._sleep_modes

        cmd = "grep -i -e '%s' %s" % ("' -e'".join(self._pattern), self._sleep_mode_file)
        lines = self._exec("adb shell %s" % cmd, force_execution=True).splitlines()
        for line in lines:
            state = self.format_state_name(line.split()[0])
            self._sleep_modes.append(state)

        if "s0" not in self._sleep_modes:
            self._sleep_modes.append("s0")

        return self._sleep_modes

class SleepModePmcAtom(SleepModeGeneric):
    """
    :summary: SleepMode management for pmc_atom (CHT) platforms
    using an C{Intent} based communication to the I{DUT}.
    """
    _sleep_modes = []
    _sleep_mode_file = "/sys/kernel/debug/pmc_atom/sleep_state"
    _sleep_mode_type = "PMC_ATOM_SLEEP"
    _dstates_files = ["/sys/kernel/debug/pmc_atom/dev_state",
                      "/sys/kernel/debug/nc_atom/nc_dev_state"]
    _clearable = True
    _states = [
        {'name':"s0", 'disable': [], 'other_names': ["active"], 'suspend':False},
        {'name':"s0ir", 'disable': ["s0i1", "s0i3"], 'other_names': [], 'suspend':False},
        {'name':"s0i1", 'disable': ["s0i3"], 'other_names': ["lpmp3", "mp3", "audio"], 'suspend':False},
        {'name':"idle-s0i3", 'disable': ["s0i1"], 'other_names': ["s0i3"], 'suspend':False},
        {'name':"legacy-suspend", 'disable': [], 'other_names': ["s3"], 'suspend':True},
    ]

    _default_counters_fmt = "\d*"
    _counters_fmt = {
        'mode':'[^\s]*',
        'residency':None, # nothing to do : format is "Residency Time" where Residency = Mode
    }
    _counters_eol = "\s.*$" # we have to match " ms$" on CHT
    _pattern = ["^s0", "^s0i.*", "^lpmp3", "^s3", "^legacy-suspend", "^idle-s0i.*"]

    def __init__(self, device):
        """
        Constructor
        """
        SleepModeGeneric.__init__(self, device)

    def get_sleep_modes(self):
        """
        get the list of available sleep modes on the device
        """
        if self._sleep_modes:
            return self._sleep_modes

        cmd = "grep -i -e '%s' %s" % ("' -e'".join(self._pattern), self._sleep_mode_file)
        lines = self._exec("adb shell %s" % cmd, force_execution=True).splitlines()
        for line in lines:
            state = self.format_state_name(line.split()[0])
            self._sleep_modes.append(state)
        return self._sleep_modes

    def pre_format_residencies(self, residencies):
        """
        Put here the operations to do on the residency list
        """
        for element in residencies:
            # time is in ms
            element["time"] = float(element.get("time")) / 1000.0
            element["mode"] = self.format_state_name(element["mode"])

class SleepModeTelemetryAtom(SleepModeGeneric):
    """
    :summary: SleepMode management for telemetry_atom (BXT) platforms
    using an C{Intent} based communication to the I{DUT}.
    """
    _sleep_modes = []
    _sleep_mode_file = "/sys/kernel/debug/telemetry_atom/soc_states"
    _sleep_mode_type = "TELEMETRY_ATOM_SLEEP"
    _dstates_files = ["/sys/kernel/debug/telemetry_atom/soc_states",
                      "/sys/kernel/debug/telemetry_atom/ioss_sample_info",
                      "/sys/kernel/debug/telemetry_atom/pss_sample_info"]
    _clearable = False
    _states = [
        {'name':"s0ix_shallow", 'disable': ["C10"], 'other_names': ["s0i1", "s0i2", "audio"], 'suspend':False},
        {'name':"s0ix_deep", 'disable': ["C8", "C9"], 'other_names': ["s0i3", "idle-s0i3"], 'suspend':False},
        #{'name':"suspend_with_s0ixshallow", 'disable': [], 'other_names': [], 'suspend':True},
        #{'name':"suspend_with_s0ixdeep", 'disable': [], 'other_names': ["s3", "legacy-suspend"], 'suspend':True},
        #{'name':"suspend_with_shallow_wakes", 'disable': [], 'other_names': [], 'suspend':True},
        {'name':"suspend_total", 'disable': [], 'other_names': ["s3", "legacy-suspend"], 'suspend':True},
        {'name':"s0ix_suspend_total", 'disable': [], 'other_names': [], 'suspend':True},
    ]
    _default_counters_fmt = "\d*"
    _counters_fmt = {
        'mode':'[^\s]*\s[^\s]*',
    }
    _pattern = ["^S0IX Shallow", "^S0ix deep", "^Suspend(With S0ixDeep)",
                "^Suspend(With Shallow-Wakes)", "^Suspend(With S0ixShallow)"]

    def __init__(self, device):
        """
        Constructor
        """
        SleepModeGeneric.__init__(self, device)
        _uptime_clear = 0.

    def get_sleep_modes(self):
        """
        get the list of available sleep modes on the device
        """
        if self._sleep_modes:
            return self._sleep_modes

        cmd = "grep -i -e '%s' %s" % ("' -e'".join(self._pattern), self._sleep_mode_file)
        lines = self._exec("adb shell %s" % cmd, force_execution=True).splitlines()
        for line in lines:
            state = ' '.join([l.lower() for l in line.split() if not l.isdigit()])
            fmt_state = self.format_state_name(state)
            if "suspend_with" not in fmt_state:
                self._sleep_modes.append(fmt_state)

        if "s0" not in self._sleep_modes:
            self._sleep_modes.append("s0")
        if "suspend_total" not in self._sleep_modes:
            self._sleep_modes.append("suspend_total")

        return self._sleep_modes

    def parse_counters_header(self, header):
        """
        Put here the operations to do on the sleep mode file's header to have proper field names
        """
        ret = header.lower().replace("residency", "time").replace("occurrence", "count")
        ret = ret.replace("s0ix", "").replace("type", "")
        return ret

    def pre_format_residencies(self, residencies):
        """
        Put here the operations to do on the residency list
        """
        uptime = self._system_api.get_uptime()
        s0_time = uptime

        for element in residencies:
            # time is in us
            element["time"] = float(element["time"]) / 1000000.0
            s0_time -= float(element["time"])
            if uptime:
                element["residency"] = 100.0 * element["time"] / uptime
            element["mode"] = self.format_state_name(element["mode"])

        if "s0" not in [element["mode"] for element in residencies]: # add a fake s0 residency
            tmp = {}
            tmp["mode"] = "s0"
            tmp["time"] = s0_time
            tmp["residency"] = 100. * s0_time / uptime
            tmp["count"] = 0
            residencies.append(tmp)

        # Hack : replace all suspend counters by their sum
        total = {}
        total["mode"] = "suspend_total"
        total["time"] = 0
        total["residency"] = 0
        total["count"] = 0
        for suspend in [x for x in residencies if "suspend_with" in x["mode"]]:
            for field in ("time", "residency", "count"):
                total[field] += suspend[field]
        for i in range(0, len(residencies))[::-1]:
            if residencies[i]["mode"] and "suspend_with" in residencies[i]["mode"]:
                del residencies[i] # delete all "suspend_with" modes in the list
        residencies.append(total)

    def format_state_name(self, name):
        """
        Format the sleep mode name
        """
        state = name.replace('+', "_").replace('-', "_")
        state = '_'.join([x for x in state.replace('(', " ").replace(')', " ").split() if x])
        return state.lower()

###################
# SleepMode class #
###################

class SleepMode(Base, ISleepMode):
    """
    :summary: SleepMode management
    using an C{Intent} based communication to the I{DUT}.
    """
    _sleep_modes = []
    _sleep_mode_files = [(SleepModeLegacy, "/sys/kernel/debug/mid_pmu_states"),
                         (SleepModePmcAtom, "/d/pmc_atom/sleep_state"),
                         (SleepModeTelemetryAtom, "/d/telemetry_atom/soc_states")
                        ]
    _sleepmode_instance = None
    _residencies = None
    _residencies_before_clear = None

    def __init__(self, device):
        """
        Constructor
        """
        Base.__init__(self, device)
        ISleepMode.__init__(self, device)

        # instanciate the correct class according to the device and set self._sleepmode_instance
        self.__get_sleepmode_instance(device)

        self._sleep_modes = self._sleepmode_instance.get_sleep_modes()

        if self.device_has_residencies() and not self._sleep_modes:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Failed to retrieve sleep modes")

        self._logger.info("sleep modes")
        self._logger.info(self._sleep_modes)
        self._requested_mode = None
        self._real_sleep_mode = ""
        self._disabled_states = []
        self._allow_suspend = True
        self._audio_file = None
        self._audio_volume = 10
        self._music_player = None

        self.__cpudir = "/sys/devices/system/cpu"
        self.__cpus = []
        self.__s0ix_state = {}
        self._audio_api = None
        self._system_api = None

    def __get_sleepmode_instance(self, device):
        """
        get the instance of the class used to handle the sleep modes (self.sleepmode_instance)
        """
        if self._sleepmode_instance:
            return self._sleepmode_instance

        for class_name, file_path in self._sleep_mode_files:
            cmd = "adb shell test -f {0} && wc -l {0} | cut -d' ' -f1".format(file_path)
            output = self._exec(cmd, force_execution=True).splitlines()
            if len(output) > 0 and int(output[0]) > 0:
                self._sleepmode_instance = class_name(device)
                return self._sleepmode_instance

        if not self._sleepmode_instance:
            self._logger.error("This device does not have any known file to retrieve residencies")
            self._sleepmode_instance = SleepModeGeneric(device) # measurement without residency
        return self._sleepmode_instance

    def device_has_residencies(self):
        """
        To know if the device has a residency file

        :rtype: bool
        :return: true if the device has a residency file, else false
        """
        return not not [x for x in self._sleep_mode_files if isinstance(self._sleepmode_instance, x[0])]

    def get_sleep_mode_file(self):
        """
        get the sleep_mode file path
        """
        return self._sleepmode_instance.get_sleep_mode_file()

    def get_d_states_files(self):
        """
        get the path of the file for d-states
        """
        return self._sleepmode_instance.get_d_states_files()

    def get_sleep_mode(self):
        """
        Return the real sleep mode

        :rtype: str
        :return: Real sleep mode
        """
        return str(self._real_sleep_mode)

    def _get_real_sleepmode(self, mode):
        """
        Return the real sleep mode
        """
        self._disabled_states = []
        self._allow_suspend = True
        self._real_sleep_mode = mode

        declared_states = self._sleepmode_instance.get_states()
        # check in real names first
        for state in declared_states:
            if "name" in state and state["name"] == mode and mode in self._sleep_modes:
                self._disabled_states = state["disable"]
                self._allow_suspend = state["suspend"]
                self._real_sleep_mode = mode
                return self._real_sleep_mode
        # then in alternative names
        for state in declared_states:
            if "other_names" in state and mode in state["other_names"] and state["name"] in self._sleep_modes:
                self._disabled_states = state["disable"]
                self._allow_suspend = state["suspend"]
                self._real_sleep_mode = state["name"]
        return self._real_sleep_mode

###############################
# cpu configuration functions #
###############################

    def __get_cpus(self):
        """
        Get all cpus
        """
        if len(self.__cpus) > 0:
            return self.__cpus

        # Get the CPU names
        cmd = "find %s -name 'cpu[0-9]*'" % self.__cpudir
        output = self._exec("adb shell %s" % cmd, force_execution=True)

        for line in output.splitlines():
            cpuname = os.path.basename(line)
            if re.match("cpu[0-9]+", cpuname):
                self.__cpus.append(os.path.basename(line))

        if len(self.__cpus) == 0:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Failed to retrieve CPU names")

        return self.__cpus

    def __get_cpuidle_states_dirs(self):
        """
        Return a dictionary of corresponding state directory for s0ix mode
        """
        if len(self.__s0ix_state) > 0:
            return self.__s0ix_state

        cpus = self.__get_cpus()
        cpuidledir = self.__cpudir + "/" + cpus[0] + "/cpuidle"

        cmd = "grep '^.*$' %s/state*/name" % cpuidledir
        output = self._exec("adb shell %s" % cmd, force_execution=True)
        if output == "" or "no such file" in output.lower():
            self._logger.info("No modes available on this device")
            return self.__s0ix_state

        for line in output.splitlines():
            match = re.search("%s/(?P<state>state\d)/name:(?P<mode>.*)$" % cpuidledir, line)

            if match and match.group("mode"):
                self.__s0ix_state[match.group("mode")] = match.group("state")

        self._logger.debug("__get_cpuidle_states_dirs returns '%s'" % self.__s0ix_state)
        return self.__s0ix_state

    def __configure_cpuidle_state(self, mode=""):
        """
        Set the cpu in requested idle state mode

        :type mode: str
        :param mode: the mode to find in cpuidle

        :type disable: bool
        :param disable:
        """
        if not self.device_has_residencies():
            return

        cpus = self.__get_cpus()
        states = self.__get_cpuidle_states_dirs()

        cmd = ""
        for state in states.keys():
            if not mode or not [x for x in self._disabled_states if x.lower() in state.lower()]:
                disable = 0
            else:
                disable = 1

            for cpu in cpus:
                cmd += "echo %d > %s/%s/cpuidle/%s/disable;" % \
                    (disable, self.__cpudir, cpu, states[state])

            if cmd != "":
                self._exec("adb shell %s" % cmd, force_execution=True)
                cmd = ""

################################
# generic init/clear functions #
################################

    def _acquire_acs_wakelock(self):
        """
        Acquire a wakelock to prevent to enter in S3 state
        """
        self._logger.debug("Hold wake lock")
        self._exec("adb shell echo 'acs' > /sys/power/wake_lock", force_execution=True)

    def _release_acs_wakelock(self):
        """
        Release acs wakelocks
        """
        self._logger.debug("Clear acs wake lock")
        self._exec("adb shell echo 'acs' > /sys/power/wake_unlock", force_execution=True)

    def _clear_default_mode(self):
        """
        Common sleep mode cleaner
        """
        self._logger.info("Clear sleep mode configuration")

        self.__configure_cpuidle_state()
        self._release_acs_wakelock()

    def init(self, mode, settle_time=0, audio_file=None,
             audio_volume=10, music_player=None):
        """
        Sleep mode initialization on the device

        :type mode: str
        :param mode: Requested mode to enter

        :type audio_file: str
        :param audio_file: The path of the audio file on the board

        :type settle_time: int
        :param settle_time: Time to wait before sleep mode
        """
        self._requested_mode = str(mode).lower()
        self._audio_file = audio_file
        self._audio_volume = audio_volume
        self._music_player = music_player

        self._get_real_sleepmode(self._requested_mode)

        if not self._allow_suspend:
            self._acquire_acs_wakelock()

        self.__configure_cpuidle_state(self._real_sleep_mode)

        funcname = "_init_%s" % self._requested_mode
        if hasattr(self, funcname):
            getattr(self, funcname)()

        if settle_time > 0:
            self._logger.info("Wait for %d seconds before sleep mode", settle_time)
            time.sleep(settle_time)

    def clear(self):
        """
        Clear sleep mode
        """
        self._clear_default_mode()
        funcname = "_clear_%s" % self._requested_mode
        if hasattr(self, funcname):
            getattr(self, funcname)()

        self._requested_mode = None

################################################################
# init/clear functions for each specific mode (lpmp3/s0/audio) #
################################################################

    def _init_s0(self):
        """
        Init static content on display mode
        """
        phonesysapi = self._device.get_uecmd("PhoneSystem")
        phonesysapi.display_on()

    def _clear_s0(self):
        """
        Clear static content on display mode
        """
        phonesysapi = self._device.get_uecmd("PhoneSystem")
        phonesysapi.display_off()

    def _init_lpmp3(self):
        """
        Init LPMP3 state mode
        """
        if self._audio_file is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "MEDIA_PATH: Please specify media path "
                                     "for lpmp3 test.")

        self._acquire_acs_wakelock()

        self._logger.debug("Launch audio file: %s" % self._audio_file)
        self._audio_api = self._device.get_uecmd("Audio")
        self._system_api = self._device.get_uecmd("System")
        phonesysapi = self._device.get_uecmd("PhoneSystem")
        phonesysapi.display_on()
        self._system_api.adjust_specified_stream_volume("Media", self._audio_volume)

        if self._music_player and self._music_player.lower() == "native":
            self._audio_api.play_native(self._audio_file)
        else:
            self._audio_api.play(self._audio_file, True)

        phonesysapi.display_off()

    def _clear_lpmp3(self):
        """
        Clear LPMP3 mode
        """
        if self._audio_file is None:
            return

        self._logger.debug("Stop audio")
        if self._music_player and self._music_player.lower() == "native":
            self._audio_api.stop_native()
        else:
            self._audio_api.stop()

        self._audio_file = None

###############################
# Residency counters handling #
###############################

    def clear_residency_counters(self, delay_time_s=0):
        """
        Clear residencies
        """
        if not self.device_has_residencies():
            return

        self._residencies = None
        self._residencies_before_clear = None
        self._logger.info("Reset residency counters")

        if not self._sleepmode_instance.is_sleep_mode_clearable():
            self._residencies_before_clear = self.get()
            self._residencies = None
            self._logger.info("Residency cannot be cleared on this device, so we make a diff before/after")

        self._sleepmode_instance.clear_counters()

        if not self._sleepmode_instance.is_sleep_mode_clearable():
            return

        if delay_time_s:
            # Acquire a wakelock, wait for X seconds, clear residency file and release the wakelock
            # The wakelock is only mandatory for s3, but not impacting s0ix
            # Thanks to the nohup, the generated script will still run after the usb is unplugged
            tmp_script_file = "/data/sleep_mode_clear_cmd.sh"
            wlname = "waitforclear"
            wllock = "/sys/power/wake_lock"
            wlunlock = "/sys/power/wake_unlock"
            self._exec("adb shell echo \"echo %s > %s; sleep %d; echo clear > %s; echo %s > %s\" > %s" %
                       (wlname, wllock, delay_time_s, self._sleepmode_instance.get_sleep_mode_file(), wlname, wlunlock,
                        tmp_script_file), force_execution=True)
            self._exec("adb shell chmod 777 %s" % tmp_script_file,
                       force_execution=True)
            self._exec("adb shell nohup sh %s" % tmp_script_file,
                       force_execution=True, wait_for_response=False, timeout=1)

        self._exec("adb shell echo clear > %s" % self._sleepmode_instance.get_sleep_mode_file(),
                   force_execution=True)

    def _fetch_residency_counters(self):
        """
        Fetch residencies
        """
        if self._residencies is not None:
            return

        if not self.device_has_residencies():
            self._residencies = []
            return

        compute_residencies = False
        residency_pattern = grouped_regex_from_list(self._get_pattern())
        self._logger.info("Pattern : %s" % residency_pattern)
        self._residencies = []
        cmd = "grep -i -e '%s' %s" % ("' -e '".join(self._sleepmode_instance.get_pattern()),
                                       self._sleepmode_instance.get_sleep_mode_file())
        residency_lines = self._exec("adb shell %s" % cmd, force_execution=True)
        residencies = residency_lines.splitlines()

        for residency in residencies:
            if residency:
                match = re.search(residency_pattern, residency)
                if match:
                    self._residencies.append(self._get_table(match))

        # Pre-format residencies
        for element in self._residencies:
            if "mode" in element:
                element["mode"] = element["mode"].replace(":", "")
            if "count" in element and str(element["count"]).isdigit():
                element["count"] = int(element["count"])
            else:
                element["count"] = 0
            element["time"] = float(element["time"])

        self._sleepmode_instance.pre_format_residencies(self._residencies)

        # Handle the diff before/after for platform where residency data cannot be cleared
        if not self._sleepmode_instance.is_sleep_mode_clearable() and self._residencies_before_clear:
            self._make_diff_from_beginning(self._residencies)
            compute_residencies = True

        # Compute residencies if not done (or needed)
        total_time = sum([e["time"] for e in self._residencies])
        if total_time:
            for element in self._residencies:
                if compute_residencies or "residency" not in element:
                    element["residency"] = 100.0 * element["time"] / total_time

    def _make_diff_from_beginning(self, res):
        """
        Replace the values by the diff between old and new values
        """

        if not self._sleepmode_instance.is_sleep_mode_clearable() and self._residencies_before_clear:
            total_time = 0.
            for element in res:
                for attr in element:
                    elemtype = type(element[attr])
                    if elemtype in [int, float]:
                        clres = self._residencies_before_clear
                        old_value = [x for x in clres if "mode" in x and x["mode"] == element["mode"]]
                        if old_value and attr in old_value[-1]:
                            try:
                                element[attr] -= elemtype(old_value[-1][attr])
                            except:
                                pass
                total_time += element["time"]

            # Compute new residencies after the diff
            for element in res:
                if total_time:
                    element["residency"] = 100.0 * element["time"] / total_time

    def _get_pattern(self):
        """
        Get the pattern to read residencies

        :rtype: list
        :return: list of pattern group to fetch residencies.
        """
        if not self.device_has_residencies():
            return []

        residency_group = []
        patterns = self._sleepmode_instance.get_pattern()
        cmd = "grep -v '\-\-\-' %s|grep -C 1 -i -e '%s'|head -n1" % \
              (self._sleepmode_instance.get_sleep_mode_file(), "' -e '".join(patterns))
        residency_format = self._exec("adb shell %s" % cmd, force_execution=True)
        residency_format = "mode " + residency_format # append mode at the beginning
        residency_format = self._sleepmode_instance.parse_counters_header(residency_format)
        groups = re.split('\s+', residency_format.strip("\t"))

        for group in groups:
            groupname = group.split("(")[0].split("[")[0].strip().replace(".", "_").lower()
            if groupname != "" and groupname != "state":
                regexp = self._sleepmode_instance.get_counter_fmt(groupname)
                if regexp:
                    if not residency_group: # first element
                        residency_group.append([groupname, "^", regexp, "\s"])
                    else:
                        residency_group.append([groupname, "\s*", regexp, "\s"])

        residency_group[-1][3] = self._sleepmode_instance.get_counters_eol()

        return residency_group

    def _get_table(self, matchobj):
        """
        Get table of residency output

        :type matchobj: MatchObj
        :param matchobj: A matched line of residency output
        """
        if not matchobj:
            return {}

        ret = {}
        for group in matchobj.groupdict().keys():
            residency = matchobj.group(group)
            ret[group] = residency.lower()
        return ret

    def get(self):
        """
        Return the residencies table

        :rtype: dict
        :return: the residencies table
        """
        self._fetch_residency_counters()
        if self._residencies:
            return self._residencies
        else:
            return []
