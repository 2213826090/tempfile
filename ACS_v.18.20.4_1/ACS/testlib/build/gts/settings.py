# Import this file for run variables
import os
HOME = os.environ['HOME']

# Devices information

# Batterry level -  if battery level is lower for any device script
#                   will put all devices to sleep until
#                   the charging level is greater
battery_ready_level = 10

# The list of devices - serials and relay information
all_devices = [
    {'serial': 'DD2715EA',},
]

# IMAGE information
# The absolute path for the builds
#  build number, target and "gts" string will be appended
build_root_path = os.path.join(HOME, "jenkins")

# The xml file used to flash the image with Phone Flash Tool
# values:
#   - flash.xml
#   - update.xml
#   - update_automation.xml (custom xml file to flash userdata for adb debugging)
#   - flash.json (for sofia like platforms)
flash_xml_file = "flash.json"

# The image type
# values:
#   - usersigned (user signed build)
#   - user (automation build)
#   - userdebug
user_build_variant = "usersigned"

# Android version
build_version = "M"


# REPORTING
report_recipients = [
    'ion-horia.petrisor@intel.com',
]

# GTS
# GTS version
gts_version = "gts-2.1_r2"
# path for GTS
gts_base_path = os.path.join(HOME, "Work/Compliance/GTS/gts-3.0/android-xts")
# GTS plan
gts_plan_name = "XTS_min"
# Number of loops - how many times will the failed test cases be rerun
gts_run_loops_no = 1
# set this to True in order the failing tests to appear in the report
gts_mail_report_failing_tests = True

# Access point information
# SSID
gts_ap_name = "CTS1"
# password
gts_ap_pass = "cts12345"
# encryption
gts_ap_encryption = "WPA2"
gts_ap_802_1x_EAP_method = None
gts_ap_802_1x_phase_2_auth = None
gts_ap_802_1x_user_certificate = None
gts_ap_802_1x_identity = None
gts_ap_802_1x_anonymous_identity = None

# After run operations
# set shutdown_to_charge to True if devices
# need to be shutdown instead of put to sleep after gts run
shutdown_to_charge = False
