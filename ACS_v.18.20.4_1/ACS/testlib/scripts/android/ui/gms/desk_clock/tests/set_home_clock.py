import sys

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.desk_clock import desk_clock_steps
from testlib.scripts.gms.desk_clock import desk_clock_utils



################################################################################
# Fetch parameters passed to the script
################################################################################
globals().update(vars(get_args(sys.argv)))
adb_steps.root_connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})



# set automatic home clock to on
desk_clock_steps.set_automatic_home_clock(serial = serial, ahc = "on")()

# This test makes sense if the system TZ and home TZ are different.
# Therefore, it has two main steps:
#   - set system time zone to a predefined one
#   - set home time zone to a different one

# Set system TZ
ui_steps.set_timezone_from_settings(serial = serial, timezone = "London")()

# Set home TZ different than system TZ
desk_clock_steps.set_home_timezone(serial = serial, timezone = "Baghdad")()

# Check if home clock @ world clock exists. Expecting it to exist.
desk_clock_steps.check_home_clock_exists(serial = serial, expected = True)()

# Set home TZ to system TZ
desk_clock_steps.set_home_timezone(serial = serial, timezone = "London")()

# Check if home clock @ world clock exists. Expecting it to not exist.
desk_clock_steps.check_home_clock_exists(expected = False)()

