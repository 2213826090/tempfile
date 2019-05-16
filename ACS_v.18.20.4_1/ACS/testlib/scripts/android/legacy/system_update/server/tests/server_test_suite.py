#!/usr/bin/env python

#######################################################################
#
# @filename:    test_01_check_verisions.py
# @description: Checks update versions external stage against internal
#               one
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.base.base_step import step as base_step
from testlib.scripts.connections.local import local_steps
from testlib.scripts.system_update import system_update_utils

import sys

internal_url = system_update_utils.get_internal_url()
external_url = system_update_utils.get_external_url()
version_path = system_update_utils.get_version_path()
'''
version = local_steps.curl(url = internal_url + version_path,
                           args = "--noproxy sp.jf.intel.com --insecure",
                           blocking = False)()

local_steps.curl(url = external_url + version_path,
                 args = "--noproxy sp.jf.intel.com --insecure",
                 grep_for = version,
                 critical = False)()
'''
version = "1000"
files_to_check = [
    "Manifest-MoM-delta-from-MASK250",
    "Manifest-MoM-delta-from-MASK250.signed",
    "Manifest-base-delta-from-MASK250",
    "Manifest-base-delta-from-MASK250.signed",
    "Manifest.MoM",
    "Manifest.MoM.signed",
    "Manifest.MoM.tar",
    "Manifest.MoM.tar.signed",
    "Manifest.base",
    "Manifest.base.signed",
    "Manifest.base.tar",
    "Manifest.base.tar.signed",
    "Manifest.esp",
    "Manifest.esp.signed",
    "Manifest.esp.tar",
    "Manifest.esp.tar.signed",
    "Newfiles",
    "pack-base-from-MASK100.tar",
    "pack-base-from-MASK100.tar.signed",
]

urls = [
    #internal_url,
    external_url,
]

class check_file(base_step):

    def __init__(self, url, args, file_name, version, delta = 0,
            success_mess = "OK", **kwargs):
        self.url = url
        self.args = args
        self.file_name = file_name
        self.version = version
        self.delta = delta
        self.ok = success_mess
        base_step.__init__(self, **kwargs)

    def do(self):
        self.no_files = 0
        if delta == 0:
            self.step_data = local_steps.curl(url = self.url
                                                  + self.version
                                                  + "/"
                                                  + self.file_name,
                                              args = self.args,
                                              grep_for = self.ok)()
        else:
            ver = int(self.version) / 10
            for build_number in range(ver - self.delta, ver):
                file_name = self.file_name.replace("MASK" + str(self.delta), str(build_number))
                self.step_data = local_steps.curl(url = self.url
                                                      + self.version
                                                      + "/"
                                                      +file_name,
                                                  args = self.args,
                                                  grep_for = self.ok)()
                if self.ok in self.step_data:
                    self.no_files += 1

    def check_condition(self):
        return self.no_files == self.delta

for url in urls:
    for file_name in files_to_check:
        delta = 0
        if "MASK250" in file_name:
            delta = 25
        elif "MASK100" in file_name:
            delta = 10
        check_file(url = url,
                   version = version,
                   args = "--noproxy sp.jf.intel.com --insecure -s --head",
                   file_name = file_name,
                   delta = delta,
                   success_mess = "OK",
                   critical = False)()
