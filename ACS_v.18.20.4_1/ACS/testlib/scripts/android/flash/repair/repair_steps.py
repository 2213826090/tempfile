#!/usr/bin/env python

##############################################################################
#
# @filename:    steps.py
# @description: Flashing test steps
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.base.base_step import step as base_step
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.connections.local.local_step import step as local_step
from testlib.scripts.android.ui.ui_step import step as ui_step

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.connections.ssh import ssh_steps
from testlib.scripts.connections.local import local_steps

import time

class flash_device(adb_step):

    """ description:
            flashes the device with <ip> using repair -wR with the
                rootfs image from <image_path> location
            it checks the <build_number> against /etc/buildstamp

        usage:
            flash_device(ip = "10.237.100.213", image_path =
                         "/path/to/starpeak-repair-rootfs-940.img",
                         build_number = "940")

        tags:
            adb, command, flash, repair, wipe, rootfs
    """

    buildstamp_ok = False

    def __init__(self, ip, image_path, build_number, **kwargs):
        self.ip = ip
        self.image_path = image_path
        self.build_number = build_number
        adb_step.__init__(self, **kwargs)
        self.set_errorm("", "The device " + self.ip + " could not flashed")

    def do(self):
        adb_steps.root_connect_device(serial = self.serial)()
        device_prepair_for_repair(serial = self.serial,
                                  image_path = self.image_path,
                                  blocking = True)()

        adb_steps.reboot(serial = self.serial,
                         blocking = True,
                         reboot_timeout = 600,
                         command = "recovery")()
        ssh_repair(ip = self.ip, blocking = True)()

        adb_steps.kill_server(serial = self.serial,
                              blocking = True)()
        adb_steps.connect_device(serial = self.serial,
                                 blocking = True,
                                 timeout = 50)()
        adb_steps.root_connect_device(serial = self.serial,
                                      blocking = True)()

    def check_condition(self):
        return adb_steps.check_buildstamp(serial = self.serial,
                                          blocking = True,
                                          build_number =  self.build_number)()


class device_prepair_for_repair(adb_step):

    """ description:
            prepares the device for repair operation by pushing the
                rootfs image from <image_path> into /cache/repair
                folder, after deleting the content of that folder
            it checks the presence of the rootfs file at that location

        usage:
            device_prepair_for_repair(image_path =
                            "/path/to/starpeak-repair-rootfs-940.img")

        tags:
            adb, command, copy, repair, rootfs
    """

    def __init__(self, image_path, **kwargs):
        self.image_path = image_path
        adb_step.__init__(self, **kwargs)
        self.set_errorm("", "The iamge " + self.image_path
                        + " could not be pushed on the device "
                        + self.adb_connection.serial)

    def do(self):
        adb_steps.delete_folder_content(serial = self.serial,
                                        folder = "/cache/repair")()
        adb_steps.push_file(serial = self.serial,
                            blocking = True,
                            local = self.image_path,
                            remote = "/cache/repair/", timeout = 600)()

    def check_condition(self):
        image_name = self.image_path.split('/')[-1]
        return image_name in \
               self.adb_connection.parse_cmd_output(cmd = "ls /cache/repair",
                                                    grep_for = image_name)


class ssh_repair(local_step):

    """ description:
            executes the repair operation over ssh using repair -wR and
                than reboots (the rootfs should be firstly push in the
                /cache/repair folder)
            it checks that the device can be pinged

        usage:
            ssh_repair(ip = "10.237.100.213")

        tags:
            ssh, command, repair, wipe, rootfs
    """

    def __init__(self, ip, **kwargs):
        self.ip = ip
        local_step.__init__(self, **kwargs)
        self.set_errorm("", "The repair -wR command could not be performed")

    def do(self):
        ssh_steps.command(blocking = True, command = "repair -wR",
                          timeout = 600, host = self.ip, user = "root",
                          password = "", sftp_enabled = False)()
        ssh_steps.command(blocking = True, command = "reboot", timeout = 200,
                          host = self.ip, user = "root", password = "",
                          sftp_enabled = False)()
        time.sleep(10)
        local_steps.wait_for_ping(blocking = True, timeout = 100, ip = self.ip)()

    def check_connection(self):
        return self.local_connection.check_ping(self.ip)


class after_repair_operations(base_step):

    """ description:
        executes the after repair operations on the <ip> device:
                - sets the sqlite db entries <db_prop_list>
                - sets the props entries <env_prop_list>
                - accepts the "telemetry" ("Allow telemtry button")
                - pushes the 2 "OK" buttons from the first boot
                (homescreen, applications page)


        usage:
            my_db_prop_list = [
    {
    "db": "/data/data/com.android.providers.settings/databases/settings.db",
    "table": "secure",
    "columns": ["name", "value"],
    "values": ["user_setup_complete", 1],
    },
    {
    "db": "/data/data/com.android.providers.settings/databases/settings.db",
    "table": "system",
    "columns": ["name", "value"],
    "values": ["volume_system", 0],
    }
            ]

            my_env_prop_list = [
                ("persist.sys.utility_iface", "eth0"),
                ("persist.intel.update.auto", "false"),
            ]

            after_repair_operations(db_prop_list = my_db_prop_list,
                env_prop_list =my_env_prop_list, ip = "10.237.100.213")

        tags:
            adb, ui, sqlite, setprop, telemetry, first_boot
    """

    def __init__(self, db_prop_list, env_prop_list, ip, **kwargs):
        self.db_prop_list = db_prop_list
        self.env_prop_list = env_prop_list
        self.ip = ip
        self.set_errorm("",
                        "After flash operations could not be performed")
        base_step.__init__(self, **kwargs)

    def do(self):
        after_wipe_prepare_device(serial = self.serial,
                                  db_prop_list = self.db_prop_list,
                                  env_prop_list = self.env_prop_list)()
        adb_steps.reboot(serial = self.serial)()
        time.sleep(10)
        local_steps.wait_for_ping(blocking = True, timeout = 100, ip = self.ip)()
        after_wipe_prepare_ui(serial = self.serial)()


class after_wipe_prepare_device(base_step):

    """ description:
            sets the entries in the android db and sets the props after
                first boot

        usage:
            after_wipe_prepare_device(db_prop_listip = "10.237.100.213")

        tags:
            ssh, command, repair, wipe, rootfs
    """

    def __init__(self, db_prop_list, env_prop_list, **kwargs):
        self.queries = db_prop_list
        self.props = env_prop_list
        base_step.__init__(self, **kwargs)

    def do(self):
        for query in self.queries:
            where_columns = []
            where_columns.append(query["columns"][0])
            where_values = []
            where_values.append(query["values"][0])
            adb_steps.sqlite_replace_query(serial = self.serial,
                                           db = query["db"],
                                           table = query["table"],
                                           columns = query["columns"],
                                           values = query["values"],
                                           where_columns = where_columns,
                                           where_values = where_values)()

        for key, value in self.props:
            adb_steps.set_prop(serial = self.serial,
                               key = key,
                               value = value)()


class after_wipe_prepare_ui(ui_step):

    """ description:
            executes the repair operation over ssh using repair -wR and
                than reboots (the rootfs should be firstly push in the
                /cache/repair folder)
            it checks that the device can be pinged

        usage:
            ssh_repair(ip = "10.237.100.213")

        tags:
            ssh, command, repair, wipe, rootfs
    """

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              print_error =
                              "Telemetry button could not be pressed",
                              view_to_find = {"resourceId":
                              "com.intel.TelemetrySetup:id/button_allow"},
                              view_to_check = {"text": "OK"})()
        ui_steps.click_button(serial = self.serial,
                              print_error =
                              "Homescreen <OK> button could not be pressed",
                              view_to_find = {"text": "OK"},
                              view_to_check = {"description": "Apps"})()
        ui_steps.click_button(serial = self.serial,
                              print_error =
                              "All application view could not be opened",
                              view_to_find = {"description": "Apps"},
                              view_to_check = {"text": "OK"})()
        ui_steps.click_button(serial = self.serial,
                              print_error =
                              "All aplication <OK> button could not be pressed",
                              view_to_find = {"text": "OK"})()

    def check_condition(self):
        return self.uidevice(text = "Clock").exists
