#!/usr/bin/env python

#######################################################################
#
# @filename:    audio_steps.py
# @description: Audio test steps
# @author:      saddam.hussain.abbas@intel.com
#
#######################################################################

# Buildin


# Used defined
from testlib.scripts.audio.audio_step import AudioStep
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.audio import audio_utils
from testlib.base import base_utils
from testlib.utils.connections.local import Local


class LaunchMusicPlayer(AudioStep):
    def __init__(self, view_to_find={"text": "Google Play Music"},
                 view_to_check="Google Play Music", timeout=5000,
                 view_presence=True, **kwargs):
        AudioStep.__init__(self, **kwargs)
        self.view_to_find = view_to_find
        self.view_to_check = view_to_check
        self.timeout = timeout
        self.view_presence = view_presence
        self.set_passm("Launched music player {0} checking {1}".format(
            view_to_find, view_to_find))
        self.set_errorm("", "Not able to launch music player {0} checking {1}".format(view_to_find, view_to_check))
        self.step_data = False

    def do(self):
        if self.device_info.all_apps_icon is None:
            ui_steps.press_media(serial=self.serial)()
            ui_steps.click_button_common(serial=self.serial,
                              view_to_find=self.view_to_find)()
        else:
            ui_steps.open_app_from_allapps(serial=self.serial,
                                           view_to_find=self.view_to_find)

    def check_condition(self):
        if self.view_to_check is not None:
            self.step_data = ui_steps.wait_for_view_common(serial=self.serial,
                                         view_to_find=self.view_to_find,
                                         view_presence=self.view_presence)()
        return self.step_data


class EavbSetMode(AudioStep):
    def __init__(self, mode, timeout=5000, **kwargs):
        AudioStep.__init__(self, **kwargs)
        self.mode = mode.lower()
        self.timeout = timeout
        self.set_passm("Successfully set dut in '{0}' mode".format(mode))
        self.step_data = False
        self.error = ""

    def do(self):
        if self.mode == "master" or self.mode == "m":
            if audio_utils.check_eavb_mode(serial=self.serial) != "m":
                self.step_data = audio_utils.set_eavb_mode(serial=self.serial,
                                                         mode="m")
            else:
                self.step_data = True
        elif self.mode == 'slave' or self.mode == "s":
            if audio_utils.check_eavb_mode(serial=self.serial) != "s":
                self.step_data = audio_utils.set_eavb_mode(serial=self.serial,
                                                         mode="s")
            else:
                self.step_data = True
        else:
            raise Exception("Mode '{0}' not supported. \
                       Use only 'master' or 'slave' mode.".format(self.mode))
        if self.step_data:
            if self.adb_connection.reboot_device():
                if self.adb_connection.adb_root():
                    self.step_data = True
                else:
                    self.error = "Failed to start device as root after set " \
                                  "eavb mode '{0}'".format(self.mode)
            else:
                self.error = "Failed to reboot device after eavb mode '{0}' is set".format(self.mode)
        else:
            self.error = "Failed to set property persist.eavb.mode to {" \
                         "0}".format(self.mode)

    def check_condition(self):
        if self.step_data:
            current_mode = audio_utils.check_eavb_mode(serial=self.serial)
            if self.mode == "master" or self.mode == "m":
                if current_mode == "m":
                    self.step_data = True
            else:
                if current_mode == "s":
                    self.step_data = True
        self.set_errorm(self.error, "Not able to set dut in '{0}' "
                                    "mode".format(self.mode))
        return self.step_data


class EavbSetup(AudioStep):
    def run_cmd(self, command, input = None, timeout=5):
        """this is a custom implementation of run_cmd to pass input to the
        child shell created by the command"""
        import subprocess
        import signal

        cmd_prefix = self.adb_connection.cmd_prefix[:]
        cmd_prefix.extend(["shell"])
        command = cmd_prefix + command.split()

        p = subprocess.Popen(command, stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        __err = 'Timeout {0} second(s) reached while executing "{1}"'.format(
                   timeout, " ".join(command))
        def handler(signum, frame):
            raise base_utils.TimeoutError(__err)

        signal.signal(signal.SIGALRM, handler)
        signal.alarm(timeout)

        if input is not None:
            out, err = p.communicate(input)

        status = p.poll()
        if status != 0:
            signal.alarm(0)
            raise AssertionError("Error encountered: Failure in "
                      "executing {0}\n{1}".format(command, err))
        signal.alarm(0)
        return out

    def __init__(self, mode, timeout=5000, **kwargs):
        AudioStep.__init__(self, **kwargs)
        self.mode = mode.lower()
        self.timeout = timeout
        self.set_passm("Successfully setup dut for '{0}' mode".format(mode))
        self.step_data = False
        self.error = ""

    def do(self):
        if self.mode not in ["master", "m", "slave", "s"]:
            raise Exception("Mode '{0}' not supported. \
                     Use only 'master' or 'slave' mode.".format(self.mode))
        if audio_utils.set_gptp_automotive_profile(
                                               serial=self.serial, state="on"):
            self.step_data = True

        # Todo add d6 avb mode if needed

    def check_condition(self):
        if self.step_data == True:
            if self.mode == "master" or self.mode == "m":
                check_properties = ["asCapable True", "Port State 7"]
            else:
                check_properties = ["asCapable True", "Port State 9"]
            if audio_utils.check_gptp_automotive_profile(serial=self.serial)!="y":
                self.step_data = False
                self.error = "Failed: persist.gptp.automotive_profile is not equal " \
                             "to 'y'"
                return False
            if audio_utils.check_gptp_service(serial=self.serial)!="running":
                self.step_data = False
                gptp_service_name = "init.svc.gptp" if self.mode in [
                    "master", "m"] else "init.svc.gptp_s"
                self.error = "Failed: {0} is not running".format(gptp_service_name)
                return False
            if audio_utils.check_avb_service(serial=self.serial)!="running":
                self.step_data = False
                self.error = "Failed: init.svc.avbstreamhandler is not running"
                return False
            if self.device_info.dessert < "O":
                eavb_status_check_tool = "open_avb_shm_test_32"
                match = self.adb_connection.parse_cmd_output(
                    cmd=eavb_status_check_tool, grep_for=check_properties,
                    multiple_grep="OR")
            else:
                eavb_status_check_tool = "open_avb_shm_test"
                # In Android O, eavb 'open_avb_shm_test' can be run only by
                # 'audioserver' user

                match = self.adb_connection.parse_cmd_output(
                    cmd="su audioserver " + eavb_status_check_tool, grep_for=check_properties,
                    multiple_grep="OR")

            if not all(prop in match for prop in check_properties):
                self.error = "Failed: Boards are not synchronized\n{" \
                             "0}".format(match)
                self.step_data = False
            self.set_errorm(self.error, "Not able to setup dut for '{0}' "
                                        "mode".format(self.mode))
        return self.step_data


class EavbStreamAudio(AudioStep):
    def run_cmd(self, command, mode="sync", timeout=300):
        """this is a work around implemented to make alsa_aplay to work till
         the bug it has is fixed
         Bug:
             alsa_aplay returns output in error stream. Because of this,
                not able to determine execution status"""
        import subprocess
        import time
        import signal

        cmd_prefix = self.adb_connection.cmd_prefix[:]
        cmd_prefix.extend(["shell"])
        command = cmd_prefix + command.split()
        # stderr is not redirected to pipe because of known error which is
        # causing p.poll to always return None though process is terminated
        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if mode == "sync":
            __err = 'Timeout {0} second(s) reached while executing "{1}"'.format(
                timeout, " ".join(command))
            def handler(signum, frame):
                raise base_utils.TimeoutError(__err)

            signal.signal(signal.SIGALRM, handler)
            signal.alarm(timeout)
            while True:
                time.sleep(1)
                status = p.poll()
                if status != None:
                    if status != 0:
                        signal.alarm(0)
                        raise AssertionError(
                            "Error encountered: Failure in executing {"
                            "0}\n{1}".format(command, p.stderr.read()))
                    signal.alarm(0)
                    return p
        else:
            time.sleep(0.5)
            if p.poll() not in [0, None]:
                return False
            return p

    def __init__(self, input_audio_file, eavb_stream, mode="sync", timeout=5000, **kwargs):
        AudioStep.__init__(self, **kwargs)
        self.input_audio_file = input_audio_file
        self.mode = mode.lower()
        self.timeout = timeout
        self.eavb_stream = eavb_stream
        self.set_passm("Successfully streamed audio: {0} through EAVB".format(
            input_audio_file))
        self.set_errorm("", "Not able stream audio: {0} through EAVB".format(
            input_audio_file))
        self.step_data = False

    def do(self):
        alsa_aplay_tool = "alsa_aplay" if self.device_info.dessert == "O" else "alsa_aplay_32"
        stream_cmd = " -v -Dplug:avb:{0} ".format(self.eavb_stream)
        command = alsa_aplay_tool + stream_cmd + \
                  self.input_audio_file

        if self.mode not in ["sync", 'async']:
            raise Exception("Mode '{0}' not supported. \
                            Use only 'sync' or 'async'.".format(self.mode))
        self.step_data = self.run_cmd(command, self.mode)

    def check_condition(self):
        return self.step_data


class EavbStartRecordAudio(AudioStep):
    def __init__(self, eavb_stream, output_audio_file, mode="async",timeout=5000, **kwargs):
        AudioStep.__init__(self, **kwargs)
        self.output_audio_file = output_audio_file
        self.mode = mode.lower()
        self.timeout = timeout
        self.eavb_stream = eavb_stream
        self.set_passm("Successfully started recording EAVB streaming audio "
                       "in {0}".format(output_audio_file))
        self.set_errorm("", "Not able start recording EAVB streaming audio")
        self.step_data = False

    def do(self):
        alsa_arecord_tool = "alsa_arecord" if self.device_info.dessert == \
                                              "O" else "alsa_arecord_32"
        record_cmd = " -v -Dplug:avb:{0} -f dat ".format(self.eavb_stream)
        command = alsa_arecord_tool + record_cmd + self.output_audio_file
        self.adb_connection.run_cmd(command = command, mode=self.mode)
        self.step_data = True


class EavbStopRecordAudio(AudioStep):
    def __init__(self, timeout=5000, **kwargs):
        AudioStep.__init__(self, **kwargs)
        self.timeout = timeout
        self.set_passm("Successfully stoped recording streaming audio")
        self.set_errorm("", "Not able stop recording streaming audio")
        self.step_data = False

    def do(self):
        pid = self.adb_connection.pgrep_common(args="alsa_arecord")
        self.adb_connection.kill_all(pid)
        self.step_data = True

    def check_condition(self):
        if self.step_data:
            if self.adb_connection.pgrep_common(args="alsa_arecord"):
                self.step_data = False
        return self.step_data

