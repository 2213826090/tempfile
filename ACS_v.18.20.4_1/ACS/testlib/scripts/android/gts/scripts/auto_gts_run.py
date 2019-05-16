from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.flash import flash_steps
from testlib.scripts.android.gts import gts_steps
from testlib.scripts.connections.local import local_utils
from testlib.scripts.connections.local import local_steps

from testlib.base.base_utils import get_args

from serial.serialutil import SerialException
import sys
import time
import traceback
import atexit
import os

"""
How to run GTS automation script

python auto_gts_run.py --script-args
                            dut-config-file=dut_config_trekstor.ini
                            image-config-file=image_config.ini
                            gts-config-file=gts_config.ini
                            report-config-file=report_config.ini

"""

def remove_gts_lock():
    try:
        os.remove(os.path.join(os.getenv("HOME"), "lock_run"))
    except: pass
    try:
        os.remove(os.path.join(os.getenv("HOME"), "lock_run_" + build_platform))
    except: pass


atexit.register(remove_gts_lock)


globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

build_no = args['build-no']
target = args['target']
ready_level = 60
if "min-battery-level" in args:
    ready_level = int(args["min-battery-level"])
shutdown_to_charge = False
if "shutdown-to-charge" in args:
    shutdown_to_charge = args["shutdown-to-charge"] == 'True'
oem_unlock = False
if "oem-unlock" in args:
    oem_unlock  = args['oem-unlock']=='True'
no_flash = False
if "no-flash" in args:
    no_flash  = args['no-flash']=='True'
no_prepare = False
if "no-ui-prepare" in args:
    no_prepare  = args['no-ui-prepare']=='True'


dut_file = args['dut-config-file']
device = eval(open(dut_file, 'r').read())

image_file = args['image-config-file']
image = eval(open(image_file, 'r').read())
build_prop_file = image['prop-file']
build_platform = image['platform']
user_build_variant = image['variant']
build_version = image['version']
if user_build_variant == "usersigned":
        user_signed = True
        user_build = True
        resources_folder = image["usersigned-data-folder"]
elif user_build_variant == "user":
    user_build = True
    user_signed = False
else:
    user_build = False
    user_signed = False
build_root_path  = image['path']

gts_config_file = args['gts-config-file']
gts_config = eval(open(gts_config_file, 'r').read())

build_path = build_root_path + build_no + '/' + target + '/'

flash_xml_file = "flash.xml"
if 'xml-file' in image:
    flash_xml_file = image['xml-file']
update = "update" in flash_xml_file

########################################################################
# FLASH
########################################################################

print "Check for battery level on the device"
not_charged = True
rate_per_minute = 0.25
while not_charged:
    not_charged = False
    time_sleep = 0
    s = device["serial"]
    try:
        b_level = adb_utils.get_battery_level(serial = s)
    except:
        #TODO Use relay to reboot device
        # workaroud is to assume is charged
        print "ADB connection error for ", str(s)
        b_level = ready_level
    print "Battery level for " + s + " is: " + str (b_level)
    if int(b_level) < ready_level:
        new_sleep = int((ready_level - int(b_level))/rate_per_minute)
        not_charged = True
        time_sleep = time_sleep if time_sleep > new_sleep else new_sleep
        print "... device should charge for " + str(new_sleep) + " minutes."
    else:
        print "... ok"
    if time_sleep > 0:
        for s in serials:
            print str(s) + ": Put device to sleep"
            try:
                ui_steps.put_device_into_sleep_mode(serial = s)()
                print str(s) + ": Sleep - Done!"
            except: pass
    print "Sleep for " + str(time_sleep) + " minutes....."
    time.sleep(time_sleep * 60)
    print "Done -  checking battery level"
print "Battery level is ok"

ok = True
if not no_flash:
    print "Preparing the device for flash"
    if oem_unlock:
        gts_steps.prepare_device_for_flash(intent = True,
                                            parallel = True,
                                            device = device,
                                            platform = build_platform)()
    print "Done preparing the device for flash!"
    print "Preparing image for flash"
    device_prop = flash_steps.prepare_image_for_flash(path_to_image = build_path,
                                                      platform = build_platform,
                                                      prop_file = build_prop_file,
                                                      user_signed = user_signed,
                                                      resources_folder = resources_folder,
                                                      device_prop = "ro.build.version.incremental")()
    print "DONE preparing image!"
    print "Start flashing for CTS"
    ok = gts_steps.flash_device_for_gts(device = device,
                                        user_build = user_build,
                                        flash_xml_path = build_path + "/" + build_platform + "/" + flash_xml_file,
                                        build_no = device_prop,
                                        usb_debugging = True,
                                        version = build_version,
                                        platform = build_platform,
                                        user_signed = user_signed,
                                        timeout = 900,
                                        update = update)()
    print "DONE flashing the device!"

bios = adb_utils.get_bios_version(serial = device["serial"]).strip()
print str(s) + " device has bios version: " + bios

########################################################################

########################################################################
# PREPARE
########################################################################
print "Start preparing for GTS"
gts_config_file = args['gts-config-file']
gts_config = eval(open(gts_config_file, 'r').read())
gts_version =  gts_config['version']
gts_plan_name =  gts_config['plan']
gts_base_path = gts_config['base-path']
gts_binary_dir = gts_config['binary-dir']
gts_tcs_dir = gts_config['testcases-dir']
gts_plans_dir = gts_config['plans-dir']
gts_results_dir = gts_config['results-dir']
gts_logs_dir = gts_config['logs-dir']
gts_binary = gts_config['binary']
gts_run_loops_no = gts_config['loops-no']
gts_list_results_command = gts_config['list-results-command']
gts_exit_command = gts_config['exit-command']
gts_screen_name = gts_config['screen-name']

gts_ap_name = "CTS"
gts_ap_pass = "cts12345"
if "ap-name" in gts_config:
    gts_ap_name = gts_config['ap-name']
if 'ap-pass' in gts_config:
    gts_ap_pass = gts_config['ap-pass']

if no_prepare == False:
    return_data = gts_steps.prepare_device_for_gts(serial = device["serial"],
                                  gts_base_path = gts_base_path,
                                  gts_tcs_dir = gts_tcs_dir,
                                  platform = build_platform,
                                  update = update,
                                  hidden_ap = True,
                                  ap_name = gts_ap_name,
                                  ap_pass = gts_ap_pass,
                                  intent = True)()
    #TODOO
    adb_steps.reboot(serial = device["serial"],
                     ip_enabled = False,
             enable_uiautomator = False)()
########################################################################

########################################################################
# GTS RUN
########################################################################
print "Start with the GTS"
report_config_file = args['report-config-file']
report_config = eval(open(report_config_file, 'r').read())
gts_mail_report_path = report_config['mail-report-path']
gts_mail_report_template = report_config['mail-report-template']
report_recipients = report_config['report-recipients']
debug_recipients = report_config['debug-recipients']
email_sender = report_config['email-sender']
email_host = report_config['email-sender-host']
email_user = report_config['email-sender-user']
email_pass = report_config['email-sender-pass']
sender_report_path = report_config['email-sender-report-path']

gts_steps.run_gts_plan(screen_name = gts_screen_name,
                       device = device,
                       gts_plan_name = gts_plan_name,
                       gts_base_path = gts_base_path,
                       gts_binary_dir = gts_binary_dir,
                       gts_tcs_dir = gts_tcs_dir,
                       gts_plans_dir = gts_plans_dir,
                       gts_results_dir = gts_results_dir,
                       gts_logs_dir = gts_logs_dir,
                       gts_binary = gts_binary,
                       gts_version = gts_version,
                       loops_no = gts_run_loops_no,
                       list_results_command = gts_list_results_command,
                       exit_command = gts_exit_command,
                       gts_mail_report_path = gts_mail_report_path,
                       gts_mail_report_template = gts_mail_report_template,
                       recipients = report_recipients,
                       debug_recipients = debug_recipients,
                       build_no = build_no,
                       bios = bios,
                       platform = build_platform,
                       target = target,
                       user_build_variant = user_build_variant,
                       email_sender = email_sender,
                       host = email_host,
                       user = email_user,
                       password = email_pass,
                       remote_path = sender_report_path,
                      )()
print "GTS run Done!"
########################################################################

########################################################################
# PREPARE FOR NEXT RUN
########################################################################

s = device["serial"]
print "Prepare device " + str(s) + " for next build."
if local_utils.has_adb_serial(serial = s):
    adb_steps.enable_uiautomator_service(serial = s, timeout = 120)()
    if ui_utils.is_view_displayed(serial = s,
                                  view_to_find = {"textContains":
                            "To start Android, enter your"}):
        ############
        # workaround pin/pass bug
        ############
        print "Workaround: To enter Android, input pin/pass on " + str(s)
        try:
            adb_steps.reboot(serial = s, command = "fastboot", ip_enabled = False)()
            local_steps.command(command = "fastboot -s " + s + " oem unlock")()
            time.sleep(3)
            my_relay = Relayed_device(relay_port = device["relay"]["tty"],
                                    power_port = device["relay"]["power_port"],
                                    v_up_port = device["relay"]["v_up_port"],
                                    v_down_port = device["relay"]["v_down_port"])
            my_relay.press_volume_up()
            my_relay.press_power()
            my_relay.close()
            local_steps.command(command = "fastboot -s " + s + " oem verified")()
            local_steps.command(command = "fastboot -s " + s + " oem lock")()

            time.sleep(3)
            my_relay = Relayed_device(relay_port = device["relay"]["tty"],
                                    power_port = device["relay"]["power_port"],
                                    v_up_port = device["relay"]["v_up_port"],
                                    v_down_port = device["relay"]["v_down_port"])
            my_relay.press_volume_up()
            my_relay.press_power()
            my_relay.close()
            local_steps.command(command = "fastboot -s " + s + " reboot")()
            local_steps.wait_for_adb(serial = s, timeout = 500)()
            local_steps.wait_for_ui(serial = s, timeout = 900)()
        except:
            print "Workaround could not be performed for " + str(s)
            print traceback.format_exc()
    if shutdown_to_charge:
        print str(s) + ": Shutdown device to charge"
        local_steps.command(command = "adb -s " + str(s) + " shell reboot -p")()
        time.sleep(5)
        print str(s) + ": Shutdown - Done!"
    else:
        print str(s) + ": Put device to sleep"
        adb_steps.put_device_into_sleep_mode(serial = s)()
        print str(s) + ": Sleep - Done!"
else:
    print str(s) + ": does not have adb connection!!!"
########################################################################
