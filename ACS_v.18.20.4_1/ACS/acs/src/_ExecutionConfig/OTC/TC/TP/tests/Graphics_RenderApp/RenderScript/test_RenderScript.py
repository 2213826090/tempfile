from testlib.gps.common import GPS_Common


if GPS_Common().check_android_version() == "M":
    from test_RenderScript_M import *
else:
    from test_RenderScript_N import *
