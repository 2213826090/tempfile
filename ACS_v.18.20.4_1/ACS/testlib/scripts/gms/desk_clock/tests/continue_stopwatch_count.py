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

# Start stopwatch
desk_clock_steps.stopwatch_action(serial = serial,action = "start")()

# Stop stopwatch
desk_clock_steps.stopwatch_action(serial = serial,action = "stop")()

# Start stopwatch
desk_clock_steps.stopwatch_action(serial = serial,action = "start")()

# Stop stopwatch
desk_clock_steps.stopwatch_action(serial = serial,action = "stop")()

# Reset stopwatch
desk_clock_steps.stopwatch_action(serial = serial,action = "reset")()
