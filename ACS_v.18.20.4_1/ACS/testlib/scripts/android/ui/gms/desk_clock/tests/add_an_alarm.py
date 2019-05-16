import sys

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.gms.desk_clock import desk_clock_steps



################################################################################
# Fetch parameters passed to the script
################################################################################
globals().update(vars(get_args(sys.argv)))
adb_steps.root_connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ALARM_LABEL = "New alarm label"
desk_clock_steps.add_an_alarm(serial = serial, 
                            alarm_label = ALARM_LABEL,
                            hours=17, 
                            minutes=34)()
desk_clock_steps.delete_an_alarm(serial = serial, 
                            alarm_label = ALARM_LABEL,
                            hours=17, 
                            minutes=34)()
