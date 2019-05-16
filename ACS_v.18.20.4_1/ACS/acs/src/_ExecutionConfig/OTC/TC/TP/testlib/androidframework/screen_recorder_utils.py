import signal,os,subprocess
from testlib.androidframework.adb_utils import AdbUtils
from testlib.androidframework.shell_utils import ShellUtils
from testlib.androidframework.dut_manager import dut_manager
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)

class ScreenRecorder(object):
    force_record = False
    max_rec_size = 15000000L  # 15MB max file size

    def __init__(self, bitrate, time_limit, delay):
        self.rec_path = os.path.dirname(os.path.realpath(__file__))
        self.temp_dut_file = "/sdcard/test_screen_rec.mp4"
        self.rec_shell_cmd_template = "sleep $DELAY$ ;" \
                                      " adb -s $DUTSERIAL$ shell screenrecord --bit-rate $BITRATE$ --time-limit $TIME_LIMIT$ "
        self.bitrate = bitrate
        self.time_limit = time_limit
        self.delay = delay
        self.is_recording = False
        self.recording_proc = None

    def start_recording(self):
        self.rec_shell_cmd = self.rec_shell_cmd_template.replace('$BITRATE$', str(self.bitrate))\
                                 .replace("$DUTSERIAL$", dut_manager.active_uiautomator_device_serial)\
                                 .replace('$TIME_LIMIT$', str(self.time_limit))\
                                 .replace('$DELAY$', str(self.delay)) + self.temp_dut_file
        try:
            rec_proc = subprocess.Popen(self.rec_shell_cmd, shell=True)
            self.is_recording = True
            self.recording_proc = rec_proc
        except:
            LOG.info("failed to start screen recording", self)

    def stop_recording(self):
        if self.is_recording:
            try:
                self.recording_proc.terminate()
                os.kill(self.recording_proc.pid, signal.SIGKILL)
                ShellUtils.kill_processes_containing_string("screenrecord")
            except:
                LOG.info("error while trying to stop recording", self)

    def retrieve_recording(self, local_recording_name):
        local_path = os.path.join(self.rec_path, local_recording_name)
        try:
            AdbUtils.pull(self.temp_dut_file, local_path)
            rec_stat = os.stat(local_path)
            if rec_stat.st_size > ScreenRecorder.max_rec_size:
                os.remove(local_path)
            return local_path
        except:
            LOG.info("error while pulling screen recording from DUT", self)


class ScreenRecorderManager(object):
    recording_env_var_name = "RECORD_PYUIAPITESTS"
    save_all_recordings_env_var_name = "SAVE_ALL_RECS_PYUIAPITESTS"
    start_recording_delay_var_name = "RECORDING_DELAY_PYUIAPITESTS"

    def __init__(self, bitrate=1000000, time_limit=180, delay=1):
        if ScreenRecorderManager.start_recording_delay_var_name in os.environ:
            delay = int(os.environ[ScreenRecorderManager.start_recording_delay_var_name])
        self.screen_recorder = ScreenRecorder(bitrate, time_limit, delay)

    @staticmethod
    def activate_recording(save_only_failed_tests_recording=True, recording_delay=0):
        os.environ[ScreenRecorderManager.recording_env_var_name] = "SET"
        os.environ[ScreenRecorderManager.start_recording_delay_var_name] = str(recording_delay)
        if not save_only_failed_tests_recording:
            os.environ[ScreenRecorderManager.save_all_recordings_env_var_name] = "True"

    @staticmethod
    def deactivate_recording():
        if os.getenv(ScreenRecorderManager.recording_env_var_name) is not None:
            del os.environ[ScreenRecorderManager.recording_env_var_name]
        if os.getenv(ScreenRecorderManager.start_recording_delay_var_name) is not None:
            del os.environ[ScreenRecorderManager.start_recording_delay_var_name]
        if os.getenv(ScreenRecorderManager.save_all_recordings_env_var_name) is not None:
            del os.environ[ScreenRecorderManager.save_all_recordings_env_var_name]

    def begin_recording(self):
        if ScreenRecorderManager.recording_env_var_name in os.environ:
            LOG.info("starting screen recording", self)
            self.screen_recorder.start_recording()

    def end_recording(self, pull_recording_from_dut, save_rec_name):
        if self.screen_recorder.is_recording:
            LOG.info("stopping screen recording", self)
            self.screen_recorder.stop_recording()
            if pull_recording_from_dut or ScreenRecorderManager.save_all_recordings_env_var_name in os.environ:
                LOG.info("retrieving screen recording from DUT", self)
                return self.screen_recorder.retrieve_recording(save_rec_name)
