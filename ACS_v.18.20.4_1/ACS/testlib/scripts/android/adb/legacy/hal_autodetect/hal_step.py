#!/usr/bin/env python

########################################################################
#
# @filename:    hal_step.py
# @description: HAL autodetect basic test step
# @author:      alexandrux.n.branciog@intel.com
#
########################################################################

from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.utils.connections.adb import Adb as connection_adb
import atexit
import os

scripts_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
module_dir = os.path.dirname(scripts_dir)

class step(adb_step):
    file_path = None
    adb_connection = None
    module = None
    kmodule = None
    mount_point = None

    def __init__(self, **kwargs):
        adb_step.__init__(self, **kwargs)
        if kwargs.has_key('module'):
            self.module = kwargs['module']
        if kwargs.has_key('kmodule'):
            self.kmodule = kwargs['kmodule']
        if kwargs.has_key('mount_point'):
            self.mount_point = kwargs['mount_point']
        self.blocking = False
        self.critical = False

    def do(self):
        self.adb_connection.open_connection()
        self.adb_connection.adb_root()
        if self.platform == None:
            product_model = self.adb_connection.get_prop("ro.product.device")
            if "t100" in product_model.lower():
                self.platform = "t100"
            elif "xps" in product_model.lower():
                self.platform = "xps12"
            elif "cruise7_coho" in product_model.lower():
                self.platform = "ecs_e7"
            else:
                self.platform = ""
        if self.media_path == None:
            self.media_path = os.path.join(module_dir,
                                           "resources/hal_autodetect/")
        self.file_path = os.path.join(self.media_path, self.platform)
        self.outfile = os.path.join(self.file_path, self.__class__.__name__)

        atexit.register(self.cleanup)

    def cleanup(self):
        #TODO - kill hald process
        pass
