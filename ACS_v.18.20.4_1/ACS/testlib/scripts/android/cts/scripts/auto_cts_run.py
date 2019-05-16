from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.flash import flash_steps
from testlib.scripts.android.cts import cts_steps
from testlib.scripts.connections.local import local_utils
from testlib.scripts.connections.local import local_steps

from testlib.base.base_utils import get_args
import sys
import time
import os
from serial.serialutil import SerialException
import atexit
import traceback
from testlib.utils.relay import Relayed_device


def lock_release():
    try:
        os.rmdir("/tmp/lock")
    except: pass

def remove_cts_lock():
    try:
        os.remove(os.path.join(os.getenv("HOME"), "jenkins/lock_run"))
    except: pass
    try:
        os.remove(os.path.join(os.getenv("HOME"), "jenkins/lock_run_" + cts_platform))
    except: pass

cts_platform = ""

atexit.register(lock_release)
atexit.register(remove_cts_lock)


"""
How to run CTS automation script

python auto_cts_run.py --script-args
                            dut-config-file=dut_config_trekstor.ini
                            image-config-file=image_config.ini
                            cts-config-file=cts_config.ini
                            report-config-file=report_config.ini

"""

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

build_no = args['build-no']
target = args['target']
ready_level = 50
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
devices = eval(open(dut_file, 'r').read())
serials = []
for device in devices['list']:
    serials.append(device['serial'])

image_file = args['image-config-file']
image = eval(open(image_file, 'r').read())
build_prop_file = image['prop-file']
build_platform = image['platform']
user_build_variant = image['variant']
build_version = image['version']
resources_folder = None
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

cts_config_file = args['cts-config-file']
cts_config = eval(open(cts_config_file, 'r').read())
cts_platform = cts_config['platform']
report_config_file = args['report-config-file']
report_config = eval(open(report_config_file, 'r').read())
debug_recipients = report_config['debug-recipients']
report_recipients = report_config['report-recipients']

build_path = build_root_path + build_no + '/' + target + '/' + cts_platform + '/'

flash_xml_file = "flash.xml"
if 'xml-file' in image:
    flash_xml_file = image['xml-file']
update = "update" in flash_xml_file


########################################################################
# FLASH
########################################################################

print "Preparing devices for flash"
if oem_unlock:
    cts_steps.prepare_devices_for_flash(intent = True,
                                        parallel = True,
                                        devices = devices["list"],
                                        platform = build_platform)()
print "Done preparing devices for flash!"

print "Check for battery level on devices"
not_charged = True
rate_per_minute = 0.25
while not_charged:
    not_charged = False
    time_sleep = 0
    for s in serials:
        try:
            b_level = adb_utils.get_battery_level(serial = s)
        except:
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

if not no_flash:
    print "Preparing image for flash"
    device_prop = flash_steps.prepare_image_for_flash(path_to_image = build_path,
                                                      platform = build_platform,
                                                      prop_file = build_prop_file,
                                                      user_signed = user_signed,
                                                      resources_folder = resources_folder,
                                                      device_prop = "ro.build.version.incremental")()
    print "DONE preparing image!"
    print "Start flashing for CTS"
    cts_steps.flash_devices_for_cts(devices = devices,
                                    user_build = user_build,
                                    flash_xml_path = build_path + "/" + build_platform + "/" + flash_xml_file,
                                    build_no = device_prop,
                                    usb_debugging = True,
                                    version = build_version,
                                    platform = build_platform,
                                    timeout = 900,
                                    parallel = True,
                                    update = update,
                                    user_signed = user_signed,
                                    blocking = False,
                                    critical = False)()
    print "DONE flashing devices!"

    print "Check serials for availability after flash"
    for s in serials:
        if not local_utils.has_adb_serial(serial = s):
            serials.remove(s)
            for d in devices["list"]:
                if d["serial"] == s:
                    devices["list"].remove(d)
    print "Serials still available after flash: " + str(serials) + " - " + str(devices)
    cts_steps.send_report(debug = True,
            recipients = report_recipients,
            subject = str(len(serials)) + " shards available after flash for " + build_platform +\
                        " - " + cts_platform + "  with " + str(device_prop),
            objective = "Serials: " + str(serials))()

bios = None
for d in devices["list"]:
    try:
        s = d["serial"]
        s_bios = adb_utils.get_bios_version(serial = s).strip()
        if bios:
            if bios != s_bios:
                print "ERROR: " + str(s) + " has BIOS version: " + s_bios + \
                    " and " + bios + " was expected"
        else:
            bios = s_bios
        print str(s) + " device has bios version: " + s_bios
    except:
        pass
#
#print "Serials still available after bios listing: " + str(serials) + " - " + str(devices)
########################################################################

########################################################################
# PREPARE
########################################################################
print "Start preparing for CTS"
cts_version =  cts_config['version']
cts_plan_name =  cts_config['plan']
cts_media_path = cts_config['media-path']
cts_base_path = cts_config['base-path']
cts_binary_dir = cts_config['binary-dir']
cts_tcs_dir = cts_config['testcases-dir']
cts_plans_dir = cts_config['plans-dir']
cts_results_dir = cts_config['results-dir']
cts_logs_dir = cts_config['logs-dir']
cts_binary = cts_config['binary']
cts_admin_apk = cts_config['admin-apk']
cts_run_loops_no = cts_config['loops-no']
cts_list_results_command = cts_config['list-results-command']
cts_exit_command = cts_config['exit-command']
cts_screen_name = cts_config['screen-name']

cts_ap_name = "CTS1"
cts_ap_pass = "cts12345"

if "ap-name" in cts_config:
    cts_ap_name = cts_config['ap-name']
if 'ap-pass' in cts_config:
    cts_ap_pass = cts_config['ap-pass']

if no_prepare == False:
    return_data = cts_steps.prepare_devices_for_cts(devices = devices["list"],
                                  cts_media_path = cts_media_path,
                                  cts_base_path = cts_base_path,
                                  cts_tcs_dir = cts_tcs_dir,
                                  cts_admin_apk = cts_admin_apk,
                                  platform = build_platform,
                                  #hidden_ap = False,
                                  #ap_name = "sp_gpt",
                                  #ap_pass = "Starpeakqwe123!",
                                  hidden_ap = True,
                                  ap_name = cts_ap_name,
                                  ap_pass = cts_ap_pass,
                                  intent = True,
                                  blocking = False,
                                  critical = False)()

    print "Check serials for availability after prepare"
    for s in serials:
        if not return_data[s] or not local_utils.has_adb_serial(serial = s):
            serials.remove(s)
            for d in devices["list"]:
                if d["serial"] == s:
                    devices["list"].remove(d)
    print "Serials still available after prepare: " + str(serials)
    print "Serials still available after prepare from devices list: "
    print [d["serial"] for d in  devices["list"]]
    time.sleep(1)

    cts_steps.reboot_devices(devices = devices["list"], parallel = True,
                                blocking = True)()
    print "Serials still available after reboot: "
    print [d["serial"] for d in  devices["list"]]


serials = [d["serial"] for d in devices["list"]]


#timeout = 60
#while timeout > 0:
#    connected = 0
#    for s in serials:
#        if local_utils.has_adb_serial(serial = s):
#            connected += 1
#    if connected == len(serials):
#        break
#    time.sleep(2)
#    timeout -= 2
#print "DONE preparing devices!"

########################################################################

########################################################################
# CTS RUN
########################################################################
print "Start with the CTS"
cts_mail_report_path = report_config['mail-report-path']
cts_mail_report_template = report_config['mail-report-template']
email_sender = report_config['email-sender']
email_host = report_config['email-sender-host']
email_user = report_config['email-sender-user']
email_pass = report_config['email-sender-pass']
sender_report_path = report_config['email-sender-report-path']

cts_steps.run_cts_plan(screen_name = cts_screen_name,
                       serials = serials,
                       devices = devices["list"],
                       cts_plan_name = cts_plan_name,
                       cts_base_path = cts_base_path,
                       cts_binary_dir = cts_binary_dir,
                       cts_tcs_dir = cts_tcs_dir,
                       cts_plans_dir = cts_plans_dir,
                       cts_results_dir = cts_results_dir,
                       cts_logs_dir = cts_logs_dir,
                       cts_binary = cts_binary,
                       cts_admin_apk = cts_admin_apk,
                       cts_version = cts_version,
                       loops_no = cts_run_loops_no,
                       list_results_command = cts_list_results_command,
                       exit_command = cts_exit_command,
                       cts_mail_report_path = cts_mail_report_path,
                       cts_mail_report_template = cts_mail_report_template,
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
print "CTS run Done!"
########################################################################

########################################################################
# PREPARE FOR NEXT RUN
########################################################################
devices = eval(open(dut_file, 'r').read())
for d in devices["list"]:
    s = d["serial"]
    print "Prepare device " + str(s) + " for next build."
    print "Will try to use adb connection to shutdown/put to sleep the device"
    if local_utils.has_adb_serial(serial = s):
        adb_steps.enable_uiautomator_service(serial = s, timeout = 120)()
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
        print str(s) + ": Try to use relay to shutdown the devices"
        try:
            my_relay = Relayed_device(relay_port = d["relay"]["tty"],
                                    power_port = d["relay"]["power_port"],
                                    v_up_port = d["relay"]["v_up_port"],
                                    v_down_port = d["relay"]["v_down_port"])
            my_relay.power_on()
            my_relay.power_off()
            my_relay.close()
        except Exception, e:
            print "NO relay!!!"
            print e.message
########################################################################
