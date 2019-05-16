#!/usr/bin/env python

########################################################################
#
# @filename:    hal_steps.py
# @description: HAL autodetect test steps
# @author:      alexandrux.n.branciog@intel.com
#
########################################################################

from hal_step import step as hal_step
from testlib.base import base_utils
import filecmp
import hal_utils
import time
import os

class halctl(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl"
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        return filecmp.cmp(self.outfile + ".ref", self.outfile + ".current")

class halctl_list(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl -l"
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        ref = hal_utils.getHALBindings(self.outfile + ".ref")
        current = hal_utils.getHALBindings(self.outfile + ".current")
        return hal_utils.cmp_bindings(ref, current, 20)

class halctl_filter_gralloc(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl -i gralloc"
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        ref = hal_utils.getHALBindings(self.outfile + ".ref")
        current = hal_utils.getHALBindings(self.outfile + ".current")
        return hal_utils.cmp_bindings(ref, current)

class halctl_filter_fallback(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl -i type=fallback"
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        ref = hal_utils.getHALBindings(self.outfile + ".ref")
        current = hal_utils.getHALBindings(self.outfile + ".current")
        return hal_utils.cmp_bindings(ref, current)

class halctl_filter_unsupported(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl -i type=unsupported"
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        ref = hal_utils.getHALBindings(self.outfile + ".ref")
        current = hal_utils.getHALBindings(self.outfile + ".current")
        return hal_utils.cmp_bindings(ref, current)

class halctl_get_module(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl -g {0}".format(self.module)
        self.outfile += "_{0}".format(self.module)
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(self.module)
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        return filecmp.cmp(self.outfile + ".ref", self.outfile + ".current")

class halctl_check_bindings(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl -i {0}".format(self.module)
        # twick to handle both madalias and hal_id
        self.outfile += "_{0}".format(self.module.split("=")[-1].\
                                                    replace("*",""))
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(self.module)
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        ref = hal_utils.getHALBindings(self.outfile + ".ref")
        current = hal_utils.getHALBindings(self.outfile + ".current")
        return hal_utils.cmp_bindings(ref, current)

class halctl_add(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl -a {0}".format(self.module['modalias'])
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(eval(self.module['modalias']))
        self.adb_connection.run_cmd(cmd)
        self.module["refcount"] = str(int(self.module["refcount"])+1)
        cmd = "halctl -i {0}".format(self.module['modalias'])
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        ref = self.module
        current = hal_utils.getHALBindings(self.outfile + ".current")
        return hal_utils.check_binding(current, ref)

class halctl_suppress(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "halctl -s {0}".format(self.module['modalias'])
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(eval(self.module['modalias']))
        self.adb_connection.run_cmd(cmd)
        if int(self.module["refcount"]) > 0:
            self.module["refcount"] = str(int(self.module["refcount"])-1)
        if self.module["refcount"] == 0:
            try:
                self.module.pop("devpath")
            except KeyError: pass
        cmd = "halctl -i {0}".format(self.module['modalias'])
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        ref = self.module
        current = hal_utils.getHALBindings(self.outfile + ".current")
        return hal_utils.check_binding(current, ref)

class kmod(hal_step):
    def do(self):
        super(self.__class__, self).do()
        cmd = "kmod"
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        return filecmp.cmp(self.outfile + ".ref", self.outfile + ".current")

class kmod_list(hal_step):
    def do(self):
        super(self.__class__, self).do()
        try:
            self.kmodule = self.kmodule[self.platform]
        except TypeError: pass
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(self.kmodule)
        self.outfile += "_{0}".format(self.kmodule)
        cmd = "kmod -n {0}".format(self.kmodule)
        self.adb_connection.run_cmd(cmd, soutfile = self.outfile + '.current')

    def check_condition(self):
        return filecmp.cmp(self.outfile + ".ref", self.outfile + ".current")

class kmod_insert(hal_step):
    def do(self):
        super(self.__class__, self).do()
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(self.kmodule['name'])
        cmd = "kmod -i {0}".format(self.kmodule['name'])
        self.adb_connection.run_cmd(cmd, timeout = 3)

    def check_condition(self):
        output = self.adb_connection.parse_cmd_output("lsmod",
                        grep_for = self.kmodule['grep_for'])
        new_line = "\r\n" if "\r\n" in output else "\n"
        return output.split(new_line) == self.kmodule['lsmod_output']

class kmod_remove(hal_step):
    def do(self):
        super(self.__class__, self).do()
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(self.kmodule['name'])
        cmd = "kmod -r {0}".format(self.kmodule['name'])
        self.adb_connection.run_cmd(cmd, timeout = 5)

    def check_condition(self):
        output = self.adb_connection.parse_cmd_output("lsmod",
                    grep_for = self.kmodule['grep_for'])
        return output == ""

class bind_mount_check(hal_step):
    entries = None
    def do(self):
        super(self.__class__, self).do()
        self.entries = self.adb_connection.parse_cmd_output("mount",
            grep_for = self.mount_point['name']).split("\r\n")

    def check_condition(self):
        return hal_utils.check_mount_entries(self.entries,
                            self.mount_point['entries'])

class halctl_add_suppress(hal_step):
    def do(self):
        super(self.__class__, self).do()
        module = self.module[self.platform]
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(eval(module['modalias']))
        halctl_add(print_error = " Could not add module",
                module = module,
                media_path = self.media_path)()
        halctl_suppress(print_error = " Could not suppress module",
                module = module,
                media_path = self.media_path)()

class kmod_remove_insert(hal_step):
    def do(self):
        super(self.__class__, self).do()
        kmodule = self.kmodule[self.platform]
        self.passm =  self.__class__.__name__ + \
                        " {0} - [PASSED]".format(kmodule['name'])
        kmod_remove(print_error = " Could not remove module",
                kmodule = kmodule,
                media_path = self.media_path)()
        kmod_insert(print_error = " Could not insert module",
                kmodule = kmodule,
                media_path = self.media_path)()

class platform_device(hal_step):
    def do(self):
        super(self.__class__, self).do()
        module = self.module[self.platform]
        if self.mount_point is not None:
            self.mount_point = self.mount_point[self.platform]
        if module is not None:
            halctl_check_bindings(
                print_error = " Incorrect bindings list",
                module = hal_utils.get_string_for_filter(module),
                media_path = self.media_path)()
            if module.has_key('hal_id'):
                halctl_get_module(
                    print_error = " Could not get module",
                    module = eval(module["hal_id"]),
                    media_path = self.media_path)()
            if module.has_key('modalias'):
                halctl_add_suppress(print_error = " Add/Suppress unsuccessful",
                     module = self.module)()
            if module.has_key('kmod'):
                kmodule = self.kmodule[self.platform]
                kmod_list(print_error = " Incorrect output",
                    kmodule = eval(module['kmod']),
                    media_path = self.media_path)()
                kmod_remove_insert(print_error = " Remove/Insert unsuccessful",
                    kmodule = self.kmodule)()
        if self.mount_point is not None:
            bind_mount_check(
                print_error = " Incorrect bind mount",
                mount_point = self.mount_point,
                media_path = self.media_path)()

class boot_time_impact(hal_step):
    bootchart_file = None
    hald_time = None
    total_time = None
    def __init__(self, **kwargs):
        hal_step.__init__(self, **kwargs)
        if kwargs.has_key('bootchart_file'):
            self.bootchart_file = kwargs['bootchart_file']

    def do(self):
        super(self.__class__, self).do()
        if self.bootchart_file == None:
            string_total = self.adb_connection.parse_file(\
                "/data/local/bootchart.svg")
        else:
            self.bootchart_file = os.path.join(self.file_path,
                                               self.bootchart_file)
            with open(self.bootchart_file, "r") as b_file:
                string_total = b_file.read()
        string_hald = base_utils.parse_string(string_total,
                                                        grep_for = "hald")
        self.hald_time = hal_utils.get_runtime(string_hald)
        self.total_time = hal_utils.get_runtime(string_total)

    def check_condition(self):
        if self.verbose:
            print "Total boot time: {0}".format(self.total_time)
            print "hald boot impact time: {0}".format(self.hald_time)
        return self.hald_time < 0.03*self.total_time

class boot_error_check(hal_step):
    dmesg_file = None
    logcat_file = None
    ignore_errors = {"dmesg":[], "elogcat":[], "ilogcat":[]}
    def __init__(self, **kwargs):
        hal_step.__init__(self, **kwargs)
        if kwargs.has_key('dmesg_file'):
            self.dmesg_file = kwargs['dmesg_file']
        if kwargs.has_key('logcat_file'):
            self.logcat_file = kwargs['logcat_file']
        if kwargs.has_key('ignore_errors'):
            self.ignore_errors = kwargs['ignore_errors']

    def do(self):
        super(self.__class__, self).do()
        self.ignore_errors = self.ignore_errors[self.platform]
        if self.dmesg_file == None:
            hal_dmesg = self.adb_connection.parse_dmesg(grep_for = "HAL")
            hal_elogcat = self.adb_connection.parse_logcat(grep_for = "E/HAL")
            hal_ilogcat = self.adb_connection.parse_logcat(grep_for = "I/HAL")
        else:
            self.dmesg_file = os.path.join(self.file_path, self.dmesg_file)
            self.logcat_file = os.path.join(self.file_path, self.logcat_file)
            with open(self.dmesg_file, "r") as d_file:
                hal_dmesg = base_utils.parse_string(
                                d_file.read(), grep_for = "HAL")
            with open(self.logcat_file, "r") as l_file:
                hal_elogcat = base_utils.parse_string(
                                l_file.read(), grep_for = "E/HAL")
                hal_ilogcat = base_utils.parse_string(
                                l_file.read(), grep_for = "I/HAL")
        if "\r\n" in hal_dmesg:
            dmesg_new_line = "\r\n"
        else:
            dmesg_new_line = "\n"
        if "\r\n" in hal_elogcat:
            logcat_new_line = "\r\n"
        else:
            logcat_new_line = "\n"
        self.hal_dmesg_lines = hal_dmesg.split(dmesg_new_line)
        self.hal_elogcat_lines = hal_elogcat.split(logcat_new_line)
        self.hal_ilogcat_lines = hal_ilogcat.split(logcat_new_line)

    def check_condition(self):
        return  hal_utils.check_boot_errors(self.hal_dmesg_lines,\
                        self.ignore_errors['dmesg'], verbose = self.verbose)\
                and \
                hal_utils.check_boot_errors(self.hal_elogcat_lines,\
                        self.ignore_errors['elogcat'], verbose = self.verbose)\
                and \
                hal_utils.check_boot_errors(self.hal_ilogcat_lines,\
                        self.ignore_errors['ilogcat'], verbose = self.verbose)

class check_hald_owner(hal_step):
    def do(self):
        super(self.__class__, self).do()

    def check_condition(self):
        string = self.adb_connection.parse_cmd_output("ps",
                    grep_for = 'hald')
        return string.split()[0] == 'hal'

class kill_hald(hal_step):
    pid_hald = None
    def do(self):
        super(self.__class__, self).do()
        self.pid_hald = self.adb_connection.get_pid("hald")
        cmd = "kill -9 {0}".format(self.pid_hald)
        try:
            self.adb_connection.run_cmd(cmd)
        except Exception, e:
            if self.verbose:
                print e
        self.new_pid = self.adb_connection.get_pid("hald")

    def check_condition(self):
        return self.pid_hald != None and self.new_pid != None and \
                self.new_pid != self.pid_hald

class check_hald_recovery(hal_step):
    def do(self):
        kill_hald(print_error = "Error when trying to kill hald process",
            media_path = self.media_path)()
        time.sleep(3)
        halctl_list(
            print_error = "Bindings should be the same after recovery",
            media_path = self.media_path)()
