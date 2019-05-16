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

ALARM_LABEL = "To be activated"

# Count all alarms with needed label.
alarms_with_label = desk_clock_utils.count_alarms(table = "alarm_templates",
                            where = 'label="{0}"'.format(ALARM_LABEL))
# Count all alarms with needed label and which are disabled.
existing_alarms_number = desk_clock_utils.count_alarms(
                                table = "alarm_templates",
                                where = 'enabled="0" and label="{0}"'.format(ALARM_LABEL))

# If no alarms exist, create one and ensure it's disabled
if alarms_with_label == 0:
    desk_clock_steps.add_an_alarm(serial = serial,
                            alarm_label=ALARM_LABEL,
                            hours=13,
                            minutes=35)()
    desk_clock_steps.change_alarm_state(serial = serial,
                            alarm_label= ALARM_LABEL,
                            alarm_state = 0)()
# If an alarm does exist but it is enabled, disable it
elif existing_alarms_number == 0:
    desk_clock_steps.change_alarm_state(serial = serial,
                            alarm_label= ALARM_LABEL,
                            alarm_state = 0)()

# At this point there will always be a disabled alarm with the needed label.
# Enable it.
desk_clock_steps.change_alarm_state(serial = serial,
                            alarm_label= ALARM_LABEL,
                            alarm_state = 1)()

#delete alarm
desk_clock_steps.delete_an_alarm(serial = serial,
                            alarm_label=ALARM_LABEL)()
