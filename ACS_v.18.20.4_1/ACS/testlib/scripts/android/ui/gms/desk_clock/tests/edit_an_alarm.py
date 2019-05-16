import sys

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.gms.desk_clock import desk_clock_steps
from testlib.scripts.gms.desk_clock import desk_clock_utils



################################################################################
# Fetch parameters passed to the script
################################################################################
globals().update(vars(get_args(sys.argv)))
adb_steps.root_connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ALARM_LABEL = "New Alarm Name"
UPDATED_LABEL = "Updated Alarm Name"

# Add an alarm
desk_clock_steps.add_an_alarm(serial = serial,
                            alarm_label= ALARM_LABEL)()

# At this point there will always be an enabled alarm with the needed label.
desk_clock_steps.change_alarm_label(serial = serial,
                            alarm_label= ALARM_LABEL,
                            new_alarm_label = UPDATED_LABEL)()

# Remove the alarm
desk_clock_steps.delete_an_alarm(serial = serial,
                                alarm_label = UPDATED_LABEL)()
