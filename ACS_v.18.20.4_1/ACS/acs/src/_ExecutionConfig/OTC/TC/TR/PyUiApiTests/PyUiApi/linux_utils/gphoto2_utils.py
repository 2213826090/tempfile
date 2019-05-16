from PyUiApi.common.shell_utils import *
from PyUiApi.common.environment_utils import *
import psutil


class Gphoto2Utils(object):
    gphoto2 = "gphoto2"
    gphoto2_get_all_files_cmd = "gphoto2 --get-all-files"
    gphoto2_get_file_cmd = Template("gphoto2 --get-file $beginrange-$endrange")
    gphoto2_list_files_cmd = "gphoto2 --list-files"
    gphoto2_gvfs_monitor_path = os.path.normpath("/usr/lib/gvfs/gvfs-gphoto2-volume-monitor")
    gphoto2_gvfs_monitor_name = "gvfs-gphoto2-volume-monitor"
    gvfs_copy_cmd_template = Template("gvfs-copy $src $dest")
    gphoto2_working_dir = Environment.tmp_dir_path
    previous_working_dir = None

    @staticmethod
    def import_all_files():
        return ShellUtils.run_shell_cmd(Gphoto2Utils.gphoto2_get_all_files_cmd)

    @staticmethod
    def get_files_info():
        files_info = []
        list_files_output = ShellUtils.run_shell_cmd(Gphoto2Utils.gphoto2_list_files_cmd)
        info_pices = re.findall(u'#(\d+)\s+(\S+)\s+\S+\s+(\d+).+\s(\S+)\n', list_files_output)
        for info in info_pices:
            file_info = GphotoFileInfo(*info)
            files_info.append(file_info)
        return files_info

    @staticmethod
    def import_file(index):
        import_file_cmd = Gphoto2Utils.gphoto2_get_file_cmd.substitute(beginrange=index, endrange=index)
        ShellUtils.run_shell_cmd(import_file_cmd)

    @staticmethod
    def gvfs_copy_file(host_file_path, host_ptp_file_path):
        gvfs_copy_cmd = Gphoto2Utils.gvfs_copy_cmd_template.substitute(src=host_file_path, dest=host_ptp_file_path)
        ShellUtils.run_shell_cmd(gvfs_copy_cmd)

    @staticmethod
    def go_to_working_dir():
        Gphoto2Utils.previous_working_dir = os.getcwd()
        os.chdir(Gphoto2Utils.gphoto2_working_dir)

    @staticmethod
    def restore_working_dir():
        if Gphoto2Utils.previous_working_dir is not None:
            os.chdir(Gphoto2Utils.previous_working_dir)

    @staticmethod
    def clean_working_dir():
        ShellUtils.clean_local_dir(Gphoto2Utils.gphoto2_working_dir)

    @staticmethod
    def get_running_gphoto2_procs():
        result = []
        for proc in psutil.process_iter():
            if Gphoto2Utils.gphoto2 in proc.name():
                result.append(proc)
        return result

    @staticmethod
    def is_gvfs_monitor_running():
        gphoto2_procs = Gphoto2Utils.get_running_gphoto2_procs()
        for proc in gphoto2_procs:
            if Gphoto2Utils.gphoto2_gvfs_monitor_name in proc.name():
                return True
        return False

    @staticmethod
    def disable_gvfs_monitor():
        gphoto2_procs = Gphoto2Utils.get_running_gphoto2_procs()
        for proc in gphoto2_procs:
            LOG.info("killed process: " + proc.name())
            proc.kill()
        gphoto2_procs = Gphoto2Utils.get_running_gphoto2_procs()
        for i in range(10):
            if len(gphoto2_procs) == 0:
                return True
            time.sleep(1)
        return False

    @staticmethod
    def enable_gvfs_monitor():
        if not Gphoto2Utils.is_gvfs_monitor_running():
            subprocess.Popen(['nohup', Gphoto2Utils.gphoto2_gvfs_monitor_path], preexec_fn=os.setpgrp)
            time.sleep(2)


class GphotoFileInfo(object):
    def __init__(self, index, name, size, mime):
        self.index = int(index)
        self.name = name
        self.size = int(size)
        self.mime = mime

    def __str__(self):
        return "File index %s, name %s, size %s, mime %s" % (self.index, self.name, self.size, self.mime)
