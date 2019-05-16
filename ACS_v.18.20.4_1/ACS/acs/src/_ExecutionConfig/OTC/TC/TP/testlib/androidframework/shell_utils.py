from string import Template
import re,os,time
import shutil,shlex
import subprocess
from testlib.util.log import Logger
from testlib.androidframework.dut_manager import dut_manager

LOG = Logger.getlogger(__name__)

class ShellUtils(object):
    fallocate_cmd_template = Template("fallocate -l $size $filepath")
    ps_ef_grep_cmd_template = Template("ps -ef | grep $procname")

    @staticmethod
    def get_current_dut_time():
        date = ShellUtils.run_shell_cmd("adb -s %s shell date" % dut_manager.active_uiautomator_device_serial)
        time_val = re.findall("(\d+):(\d+):(\d+)", date)
        hour = time_val[0][0]
        minute = time_val[0][1]
        second = time_val[0][2]
        return hour, minute, second

    @staticmethod
    def clean_local_dir(dir_path):
        ShellUtils.run_shell_cmd("rm -rf " + dir_path + "/*")

    @staticmethod
    def get_running_processes_by_name(proc_name):
        cmd = ShellUtils.ps_ef_grep_cmd_template.substitute(procname=proc_name)
        return ShellUtils.run_shell_cmd(cmd)

    @staticmethod
    def delete_file_or_folder(absolute_path):
        if os.path.isfile(absolute_path):
            os.remove(absolute_path)
        elif os.path.isdir(absolute_path):
            shutil.rmtree(absolute_path)

    @staticmethod
    def fallocate_file(size, file_path):
        cmd = ShellUtils.fallocate_cmd_template.substitute(size=size, filepath=file_path)
        return ShellUtils.run_shell_cmd(cmd)

    @staticmethod
    def kill_processes_containing_string(string):
        procs = subprocess.check_output("ps -ef | grep " + string, shell=True)
        procs = procs.splitlines()
        for proc_line in procs:
            infos = proc_line.split()
            pid = infos[1]
            subprocess.call("kill -9 " + pid, shell=True)
        time.sleep(2)

    @staticmethod
    def run_cmd_in_process(cmd_string):
        args = shlex.split(cmd_string)
        proc = subprocess.Popen(args, stdout=subprocess.PIPE)
        return proc

    @staticmethod
    def run_shell_cmd(cmd_string):
        LOG.info(cmd_string)
        output = subprocess.check_output(cmd_string, shell=True)
        return output

    @staticmethod
    def copy_file(source_file, destination_file):
        copy_cmd = Template("cp -r '$src' '$dest'")
        out = ShellUtils.run_shell_cmd(copy_cmd.substitute(src=source_file, dest=destination_file))
        return out

    @staticmethod
    def gvfs_copy_file(source_file, destination_file):
        copy_cmd = Template("gvfs-copy '$src' '$dest'")
        out = ShellUtils.run_shell_cmd(copy_cmd.substitute(src=source_file, dest=destination_file))
        return out

    @staticmethod
    def get_props_from_cmd_output(command):
        LOG.info("running command: " + str(command))
        output = ShellUtils.run_shell_cmd("adb -s %s shell " % dut_manager.active_uiautomator_device_serial + command)
        prop_regex = r"\[([a-zA-Z\.\d_]+)\].+ \[([a-zA-Z\.\d_=>: ]+)\]"
        output_lines = output.splitlines()
        LOG.info("command result: " + str(output_lines))
        result = {}
        for line in output_lines:
            properties = re.findall(prop_regex, line)
            for prop in properties:
                result[prop[0]] = prop[1]
        return result
