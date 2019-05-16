import time
import os

class NohupProcess(object):

    def __init__(self, test_device, script_path, device_exec_dir, string_args):
        self.test_device = test_device
        self.script_name = os.path.basename(script_path)
        self.device_exec_dir = device_exec_dir
        self.string_args = string_args
        self.process_running = False
        if not os.path.isabs(script_path):
            script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_path)
        assert os.path.isfile(script_path), "Script not exists"
        cmd = "push %s %s" % (script_path, device_exec_dir)
        self.test_device.adb_cmd_common(cmd)

    def start(self):
        if not self.process_running:
            cmd = "'cd %s; nohup sh %s %s' &" % (self.device_exec_dir, self.script_name, self.string_args)
            self.test_device.adb_cmd(cmd)
            self.process_running = True

    def stop(self):
        # needs script to save its pid to pid.txt
        if self.process_running:
            pid_txt = os.path.join(self.device_exec_dir, "pid.txt")
            cmd = "'kill `cat %s`'" % pid_txt
            pid = self.test_device.adb_cmd_capture_msg(cmd)
            self.process_running = False

