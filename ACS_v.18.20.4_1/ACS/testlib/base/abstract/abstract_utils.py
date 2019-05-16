#!/usr/bin/env python

##############################################################################
#
# @filename:    abstract_step_util.py
# @description: Defines the utils for astract step
# @author:      aurel.constantin@intel.com
#
##############################################################################
import os
import re
import types
import sys
from testlib.base import base_utils
from testlib.base.base_step import step as base_step
from testlib.base.abstract.mock_step import mock_step
from testlib.scripts.connections.local import local_steps


MODULES = {
            "ap_module": {"ddwrt": "ddwrt_steps",
                          "ddwrt_atheros": "ddwrt_atheros_steps",
                          #"routeros": "routeros_steps",
                          "mock": "mock",
                         },

            "wifi_module": {"wifi_ui": "wifi_steps",
                            "mock": "mock",
                           },
          }

MODULE_PATHS = {"ddwrt_steps": "testlib.scripts.ddwrt.ddwrt_steps",
                "ddwrt_atheros_steps": "testlib.scripts.ddwrt.ddwrt_atheros_steps",
                "wifi_steps": "testlib.scripts.wireless.wifi.wifi_steps",
                "fastboot_steps": "testlib.scripts.android.fastboot.fastboot_steps",
                "kb_steps": "testlib.scripts.kb.kb_steps",
                "telephony_steps": "testlib.scripts.telephony.telephony_steps",
                "brillo_steps": "testlib.scripts.brillo.brillo_steps",
                "relay_steps": "testlib.scripts.relay.relay_steps",
                "ssh_steps": "testlib.scripts.connections.ssh.ssh_steps",
                "local_steps": "testlib.scripts.connections.local.local_steps",
                "bluetooth_steps": "testlib.scripts.wireless.bluetooth.bluetooth_steps",
                "browser_steps": "testlib.scripts.android.ui.browser.browser_steps",
                "ui_steps": "testlib.scripts.android.ui.ui_steps",
                "adb_steps": "testlib.scripts.android.adb.adb_steps",
                "logcat_steps": "testlib.scripts.android.logcat.logcat_steps",
                "file_steps": "testlib.scripts.file.file_steps",
                }

def get_module(mod_type):
    if mod_type in MODULES.keys():
        mod_param = [s for s in sys.argv if mod_type in s]
        if mod_param:
            return MODULES[mod_type][mod_param[0].split("=")[-1]]
    return None


def import_module(module_name):
    """ Method for geting the desired step class by importing it's
    module and returning the class object from inside it. """

    if module_name == 'mock':
        new_mock_step = mock_step
        new_mock_step.__name__ = step_class
        return new_mock_step

    module_path_to_import = MODULE_PATHS[module_name]

    return __import__(name = module_path_to_import,
                     globals = globals(),
                     locals = locals(),
                     fromlist = ["*"])


def get_obj(module, step_class):
    try:
        obj = getattr(module, step_class)
    except:
        raise Exception("Module {0} does not implement class {1}.".format(module, step_class))

    if obj:
        if (not isinstance(obj, (type, types.ClassType))):
            raise Exception("Target {0} found, but not of type Class".format(step_class))
        if( not issubclass(obj, base_step)):
            raise Exception("Target {0} found, but subclass is not a 'step'".format(step_class))
    return obj
