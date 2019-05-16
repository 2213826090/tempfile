#! /usr/bin/env python

from testlib.scripts.android.flash.repair import repair_steps
from testlib.scripts.android.adb import adb_steps
import multiprocessing
import os
for dirname, dirnames, filenames in \
    os.walk("/home/sys_spjenkins/upstream-artifacts/"):
    for filename in filenames:
        if "starpeak-repair-rootfs" in filename:
            SP_IMG_PATH = os.path.join(dirname, filename)


def flash_a_device(device, db_props, env_props):

    build_number = SP_IMG_PATH.split("-")[-1].split(".")[0]
    image_path = SP_IMG_PATH

    adb_steps.connect_device(serial = device["ip"] + ":5555",
                             port = device["adb_port"])()

    repair_steps.flash_device(print_error = "Device could not be flashed!",
                              ip = device["ip"],
                              image_path = image_path,
                              build_number = build_number)()

    repair_steps.after_repair_operations(db_prop_list = db_props,
                                         env_prop_list = env_props,
                                         ip = device["ip"])()


db_props = [{
    "db": "/data/data/com.android.providers.settings/databases/settings.db",
    "table": "secure",
    "columns": ["name", "value"],
    "values": ["user_setup_complete", 1],
    }, {
    "db": "/data/data/com.android.providers.settings/databases/settings.db",
    "table": "system",
    "columns": ["name", "value"],
    "values": ["volume_system", 0],
    }, {
    "db": "/data/data/com.android.providers.settings/databases/settings.db",
    "table": "global",
    "columns": ["name", "value"],
    "values": ["package_verifier_enable", 1],
    }, {
    "db": "/data/system/locksettings.db",
    "table": "locksettings",
   "columns": ["name", "user", "value"],
    "values": ["lockscreen.disabled", 0, 1],
    },
]

env_props = [
    ("persist.sys.utility_iface", "eth0"),
    ("persist.intel.update.auto", "false"),
]

devices = eval(open('dut.ini', 'r').read())
pool = multiprocessing.Pool(processes = len(devices))
flash_processes = []
for device in devices:

    new_process = pool.apply_async(flash_a_device,
                                    [device,
                                     db_props,
                                     env_props]
                                   )
    flash_processes.append(new_process)

    #flash_a_device(device, db_props, env_props)
pool.close()
pool.join()
