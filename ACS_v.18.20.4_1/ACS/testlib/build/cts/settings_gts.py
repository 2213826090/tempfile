"""
Description: Settings file for the ACAS runner.

These parameters are imported in the runner as global vars.
This file is just an example! Please update parameters to match local
configuration before ACAS run.

Configuration: GTS
For more information, check the ACAS runner wiki.
"""
import os
HOME = os.environ['HOME']

# Devices information

# Batterry level -  if battery level is lower for any device script
#                   will put all devices to sleep until
#                   the charging level is greater
battery_ready_level = 10

# The list of devices - serials and relay information
# The relay information is optional. It should only be used if the devices
# have issues with maintaining adb or fastboot connectivity.
all_devices = [
       {'serial': 'DAF57EC2'},
       {'serial': 'AGND3R18BC480C'},
]

# IMAGE information
# The absolute path for the builds
# build number, target and cts abi will be appended
build_root_path = os.path.join(HOME, 'acas')

# The xml file used to flash the image with Phone Flash Tool
# values:
#   - flash.xml
#   - update.xml
#   - update_automation.xml (custom xml file to flash userdata for adb
#                            debugging)
#   - flash.json (for sofia like platforms)
flash_xml_file = 'flash.json'

# The image type
# values:
#   - usersigned (user signed or unsigned build)
#   - user (automation build)
#   - userdebug
user_build_variant = 'usersigned'

# Android version
build_version = 'M'

# Reporting
report_recipients = [
    'email_address@intel.com',
]
# Email report can be sent via RELAY (default) or SSH
send_email_type = 'RELAY'
email_machine = None
email_machine_user = None
email_machine_password = None

# After run operations
# set shutdown_to_charge to True if devices
# need to be shutdown instead of put to sleep after cts run
shutdown_to_charge = False

# Parameters for the Sofia flashing step
# Location of the FlsTool on the local host
fls_tool_location = os.path.join(HOME, 'acas/DownloadTool')
# The size of the userdata image in the current build.
# If you don't have this information use 4569694208
fake_userdata_size = "4569694208"


# GTS only information

gts_version = '3.0_r3'
gts_base_path = os.path.join(HOME, 'acas/compliance/gts', 'gts-3.0_r3', 'android-xts')
gts_plan_name = 'XTS'
# Number of loops - how many times will the failed test cases be rerun
gts_run_loops_no = 1
# set this to True to use --disable-reboot for gts run
gts_disable_reboot = True
# set this to True in order the failing tests to appear in the report
gts_mail_report_failing_tests = True

# Access point information
# SSID
gts_ap_name = 'SSID'
# Password
gts_ap_pass = 'AP_PASSSWORD'
# Encryption details
gts_ap_encryption = 'WPA/WPA2'
gts_ap_802_1x_EAP_method = ''
gts_ap_802_1x_phase_2_auth = ''
gts_ap_802_1x_user_certificate = ''
gts_ap_802_1x_identity = ''
gts_ap_802_1x_anonymous_identity = ''

# set this to False for AOSP runs, images with Chrome app missing
browser_init = True
