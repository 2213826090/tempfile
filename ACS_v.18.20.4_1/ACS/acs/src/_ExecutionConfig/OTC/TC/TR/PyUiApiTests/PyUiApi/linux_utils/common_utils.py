import psutil
from PyUiApi.common.shell_utils import *


class CommonLinuxUtils(object):

    @staticmethod
    def get_running_procs(name_pattern):
        result = []
        for proc in psutil.process_iter():
            if name_pattern in proc.name():
                result.append(proc)
        return result

    @staticmethod
    def is_proc_running(name):
        mtp_gvfs_procs = CommonLinuxUtils.get_running_procs(name)
        for proc in mtp_gvfs_procs:
            if name in proc.name():
                return True
        return False

    @staticmethod
    def disable_running_proc(proc_name):
        procs = CommonLinuxUtils.get_running_procs(proc_name)
        for proc in procs:
            LOG.info("killed process: " + proc.name())
            proc.kill()
        for i in range(10):
            procs = CommonLinuxUtils.get_running_procs(proc_name)
            if len(procs) == 0:
                return True
            time.sleep(1)
        return False

    @staticmethod
    def enable_proc(proc_path, proc_name):
        if not CommonLinuxUtils.is_proc_running(proc_name):
            subprocess.Popen(['nohup', proc_path], preexec_fn=os.setpgrp)
            time.sleep(1)
        for i in range(5):
            procs = CommonLinuxUtils.get_running_procs(proc_name)
            if len(procs) > 0:
                return True
            time.sleep(1)
        return False
