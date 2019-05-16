#!/usr/bin/env python

from testlib.utils.connections.adb import Adb
from testlib.utils.ui import uiandroid
from testlib.scripts.android.adb import adb_utils

def count_alarms(db = "/data/data/com.google.android.deskclock/databases/alarms.db",
                table = "alarm_instances",
                where = "1"):
    return int(adb_utils.sqlite_count_query(
                                    db = db,
                                    table = table,
                                    where = where))
