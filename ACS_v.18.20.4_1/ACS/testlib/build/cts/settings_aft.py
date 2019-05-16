"""
Description: Settings file for the ACAS runner.

These parameters are imported in the runner as global vars.
This file is just an example! Please update parameters to match local
configuration before ACAS run.

Configuration: AFT
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
# The relay information is optional.It should only be used if the devices have
# issues with maintaining adb or fastboot connectivity.
all_devices = [{'serial': 'DAF57EDF'},]

# IMAGE information
# The absolute path for the builds
# build number, target and cts abi will be appended
build_root_path = os.path.join(HOME, "acas")

# The xml file used to flash the image with Phone Flash Tool
# values:
#   - flash.xml
#   - update.xml
#   - update_automation.xml (custom xml file to flash userdata for adb debugging)
#   - flash.json (for sofia like platforms)
flash_xml_file = "flash.json"

# The image type
# values:
#   - usersigned (user signed or unsigned build)
#   - user (automation build)
#   - userdebug
user_build_variant = "userdebug"

# Android version
build_version = "M"

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