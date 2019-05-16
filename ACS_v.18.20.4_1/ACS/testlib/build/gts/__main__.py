import sys
import time
import os
from serial.serialutil import SerialException
import atexit
import traceback
from optparse import OptionParser

gts_mail_report_failing_tests = True
gts_ap_802_1x_EAP_method = None
gts_ap_802_1x_phase_2_auth = None
gts_ap_802_1x_user_certificate = None
gts_ap_802_1x_identity = None
gts_ap_802_1x_anonymous_identity = None

from settings import *
from testlib.utils.logger import log_stdout_stderr

# log stdout and stderr to a file
if not os.path.exists("logs_gts"):
    os.mkdir("logs_gts")
dirfmt = "%4d-%02d-%02d_%02d-%02d-%02d"
log_path = os.path.join("logs_gts", dirfmt % time.localtime()[0:6])
os.mkdir(log_path)
os.environ["LOG_PATH"] = log_path
log_out_err_file = log_stdout_stderr(os.path.join(log_path, "all.log"))

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.flash import flash_steps
from testlib.scripts.android.cts import cts_steps
from testlib.scripts.android.gts import gts_steps
from testlib.scripts.connections.local import local_utils
from testlib.scripts.connections.local import local_steps
from testlib.utils.statics.android import statics
from testlib.utils.relay import Relayed_device

version = "0.3.1"

def lock_release():
    try:
        os.rmdir("/tmp/lock")
    except: pass

def remove_gts_lock():
    if not gts_is_running:
        try:
            os.remove(lock_file)
        except: pass

def remove_tmp_files():
    try:
        for root, dirs, files in os.walk(".", topdown=False):
            for name in files:
                if "tmp_" in name:
                    os.remove(name)
    except: pass

gts_is_running = False
if 'gts_mail_report_failing_tests' not in globals():
    gts_mail_report_failing_tests = False

atexit.register(lock_release)
atexit.register(remove_gts_lock)
atexit.register(remove_tmp_files)


def check_battery_level():
    global devices
    print "Ensure battery level on the devices is at least {0}...%".format(battery_ready_level)
    serials = [device["serial"] for device in devices]
    not_charged = True
    rate_per_minute = 0.25
    while not_charged:
        not_charged = False
        time_sleep = 0
        for serial in serials:
            try:
                battery_path = statics.Device(serial = devices[0]["serial"],
                                              platform = build_platform,
                                              dessert = build_version).battery_path
                b_level = adb_utils.get_battery_level(serial = serial, path = battery_path)
            except:
                print "[ {0} ]: ADB connection error".format(serial)
                b_level = battery_ready_level
            print "[ {0} ]: Battery level is: {1}%".format(serial, b_level)
            if int(b_level) < battery_ready_level:
                new_sleep = int((battery_ready_level - int(b_level))/rate_per_minute)
                not_charged = True
                time_sleep = time_sleep if time_sleep > new_sleep else new_sleep
                print "[ {0} ]: Device should charge for {0} minutes.".format(serial, new_sleep)
            else:
                print "[ {0} ]: Battery level is ok.".format(serial)
        if time_sleep > 0:
            for serial in serials:
                print "[ {0} ]: Put device into sleep mode".format(serial)
                try:
                    ui_steps.put_device_into_sleep_mode(serial = serial)()
                    print "[ {0} ]: Put device into sleep mode - Done!".format(serial)
                except:
                    print "[ {0} ]: Error putting device intro sleep mode - {1}".format(serial, traceback.format_exc())
            print "[ {0} ]: Sleeping for {1} minutes.....".format(serial, time_sleep)
            time.sleep(time_sleep * 60)
    print "Done -  checking battery level"


def flash_devices():
    global devices
    global device_prop
    print "Preparing image for flash"
    flash_xml_path = "{0}{1}".format(os.path.join(build_path, build_platform), "gts")
    flash_cfg_file = os.path.join(flash_xml_path, flash_xml_file)
    resource_subfolder = statics.Device(serial = devices[0]["serial"],
                                        platform = build_platform,
                                        dessert = build_version).resource_folder
    has_multiple_flashfiles = statics.Device(serial = devices[0]["serial"],
                                        platform = build_platform,
                                        dessert = build_version).has_multiple_flashfiles
    edit_flash_file = statics.Device(serial = devices[0]["serial"],
                                     platform = build_platform,
                                     dessert = build_version).edit_flash_file
    device_prop = flash_steps.prepare_image_for_flash(path_to_image = build_path,
                                                      platform = build_platform,
                                                      prop_file = "build.prop",
                                                      cts_abi = "gts",
                                                      edit_flash_file = edit_flash_file,
                                                      user_signed = user_signed,
                                                      resources_subfolder = resource_subfolder,
                                                      device_prop = "ro.build.version.incremental",
                                                      flash_xml_path = flash_cfg_file)()
    print "DONE preparing image!"
    print "Start flashing for GTS"
    cts_steps.flash_devices_for_cts(devices = devices,
                                    user_build = user_build,
                                    flash_xml_path = flash_cfg_file,
                                    build_no = device_prop,
                                    usb_debugging = True,
                                    version = build_version,
                                    platform = build_platform,
                                    timeout = 1200,
                                    update = update,
                                    user_signed = user_signed,
                                    blocking = False,
                                    critical = False)()
    print "DONE flashing devices!"
    check_devices(previous_step = "flash",
                  send_mail = True)


def check_devices(previous_step = "prepare for gts",
                  send_mail = False,
                  return_data = None):
    global devices
    print "Check serials for availability after {0} step".format(previous_step)
    device_to_remove = []
    for device in devices:
        serial = device["serial"]
        previous_check_fail = return_data is not None and not return_data[serial]
        if previous_check_fail or not local_utils.has_adb_serial(serial = serial):
            devices.remove(device)
    for device in device_to_remove:
        devices.remove(device)
    serials = [device["serial"] for device in devices]
    print "Serials still available after {0} step: {1}".format(previous_step, str(serials))
    if send_mail:
        cts_steps.send_report(debug = True,
                critical = False,
                recipients = report_recipients,
                subject = "{0} shards available after {1} for {2} with {3}".format(len(serials),
                            previous_step, build_platform, str(build_no)),
                objective = "Serials: {0}".format(serials))()


def check_bios_version():
    global bios
    bios = None
    for device in devices:
        try:
            serial = device["serial"]
            s_bios = adb_utils.get_bios_version(serial = serial).strip()
            if bios:
                if bios != s_bios:
                    print "[ {0} ]: Device has wrong BIOS version: {1} and {2} was expected".format(serial, s_bios, bios)
            else:
                bios = s_bios
            print "[ {0} ]: Device has bios version: {1}".format(serial, s_bios)
        except:
            print "[ {0} ]: Error getting BIOS version - {1}".format(serial, traceback.format_exc())


def prepare_for_gts():
    global devices
    return_data = gts_steps.prepare_devices_for_gts(devices = devices,
                                  gts_base_path = gts_base_path,
                                  gts_tcs_dir = "repository/testcases/",
                                  recipients = report_recipients,
                                  platform = build_platform,
                                  hidden_ap = True,
                                  ap_name = gts_ap_name,
                                  ap_pass = gts_ap_pass,
                                  ap_encryption = gts_ap_encryption,
                                  ap_802_1x_EAP_method = gts_ap_802_1x_EAP_method,
                                  ap_802_1x_phase_2_auth = gts_ap_802_1x_phase_2_auth,
                                  ap_802_1x_user_certificate = gts_ap_802_1x_user_certificate,
                                  ap_802_1x_identity = gts_ap_802_1x_identity,
                                  ap_802_1x_anonymous_identity = gts_ap_802_1x_anonymous_identity,
                                  intent = True,
                                  blocking = False,
                                  critical = False,
                                  parallel = True)()

    check_devices(previous_step = "prepare",
                  send_mail = False,
                  return_data = return_data)
    time.sleep(1)


def reboot_devices():
    global devices
    cts_steps.reboot_devices(devices = devices, parallel = True,
                                blocking = True)()
    check_devices(previous_step = "reboot", send_mail = False)


def run_gts():
    global devices
    print "Start with the GTS"
    if len(devices) > 0:
        gts_steps.run_gts_plan(screen_name = gts_screen_name,
                       devices = devices,
                       gts_plan_name = gts_plan_name,
                       gts_base_path = gts_base_path,
                       gts_binary_dir = "tools/",
                       gts_tcs_dir = "repository/testcases/",
                       gts_plans_dir = "repository/plans/",
                       gts_results_dir = "repository/results/",
                       gts_logs_dir = "repository/logs/",
                       gts_binary = "xts-tradefed",
                       gts_version = gts_version,
                       loops_no = gts_run_loops_no,
                       list_results_command = "list results",
                       exit_command = "exit",
                       gts_mail_report_template = "gts_mail_report_template.html",
                       gts_mail_report_failing_tests = gts_mail_report_failing_tests,
                       recipients = report_recipients,
                       debug_recipients = [],
                       build_no = build_no,
                       bios = bios,
                       platform = build_platform,
                       target = target,
                       user_build_variant = user_build_variant,
                       email_sender = "SSH",
                       host = "10.237.112.149",
                       user = "regression",
                       password = "regressioncts",
                       remote_path = "/opt/regression/mail/",
                      )()
    else:
        cts_steps.send_report(debug = True,
                              critical = False,
                              recipients = report_recipients,
                              subject = "GTS cannot run on 0 devices!!!",
                              objective = "No device is available for running GTS!!!!")()

    print "GTS run Done!"


def prepare_for_next_run():
    devices = [device for device in all_devices]
    print "Start preparing for the next GTS run"
    if len(devices) > 0:
        for device in devices:
            serial = device["serial"]
            print "Prepare device {0} for next build.".format(device)
            print "Will try to use adb connection to shutdown/put the device into sleep mode"
            if local_utils.has_adb_serial(serial = serial):
                if shutdown_to_charge:
                    print "[ {0} ]: Shutdown device to charge".format(serial)
                    local_steps.command(command = "adb -s {0} shell reboot -p".format(serial))()
                    time.sleep(5)
                    print "[ {0} ]: Shutdown - Done!".format(serial)
                else:
                    print "[ {0} ]: Put device into sleep mode".format(serial)
                    adb_steps.put_device_into_sleep_mode(serial = serial)()
                    print "[ {0} ]: Sleep - Done!".format(serial)
            else:
                print "[ {0} ]: does not have adb connection!!!".format(serial)
                print "[ {0} ]: Try to use relay to shutdown the devices".format(serial)
                try:
                    my_relay = Relayed_device(relay_port = device["relay"]["tty"],
                                            power_port = device["relay"]["power_port"],
                                            v_up_port = device["relay"]["v_up_port"],
                                            v_down_port = device["relay"]["v_down_port"])
                    my_relay.power_on()
                    my_relay.power_off()
                    my_relay.close()
                except Exception, e:
                    print "{0} devivice has no relay - {1}!!!".format(serial, e.message)
    else:
        print "No devices available for the next run!!!"


if __name__ == "__main__":
    global devices

    parser = OptionParser()
    parser.add_option("-v", "--version",
                      action = "store_true",
                      dest = "version")
    parser.add_option("-p", "--platform",
                      dest = "build_platform",
                      help = "The target platform")
    parser.add_option("-b", "--build-number",
                      dest = "build_no",
                      help = "The build number")
    parser.add_option("-t", "--target",
                      dest = "target",
                      help = "The target, coho / cohol / s3gr10m6s / slti20mr6 / oars7 / cht_ffd / ...")
    parser.add_option("-F", "--no-flash",
                      dest = "no_flash",
                      default = False,
                      action = "store_true",
                      help = "Use this option to skip flash")
    parser.add_option("-U", "--no-gts-prepare",
                      dest = "no_gts_prepare",
                      default = False,
                      action = "store_true",
                      help = "Use this option to skip gts prepare")
    parser.add_option("-R", "--no-gts-run",
                      dest = "no_gts_run",
                      default = False,
                      action = "store_true",
                      help = "Use this option to skip gts run")
    parser.add_option("-o", "--oem-unlock",
                      dest = "oem_unlock",
                      default = False,
                      action = "store_true",
                      help = "Use this option to enable oem unlock from Developer Options")
    parser.add_option("-f", "--force",
                      dest = "force_run",
                      default = False,
                      action = "store_true",
                      help = "Use this option to run GTS even if a run lock exists")

    (options, args) = parser.parse_args()
    if options.version:
        print "GTS runner version {0}".format(version)
        sys.exit(0)

    globals().update(vars(options))
    if build_platform is None or build_no is None or target is None:
        print "Please provide target, platform and build number information!!!"
        parser.print_help()
        sys.exit(1)

    devices = [d for d in all_devices]

    # check if GTS is already running
    # if no create the lock
    lock_dir = os.path.join(os.getenv("HOME"), "jenkins")
    if not os.path.exists(lock_dir):
        os.mkdir(lock_dir)
    lock_file = os.path.join(lock_dir, "lock_run_GTS_{0}".format(build_platform))
    if os.path.exists(lock_file) and not force_run:
        print "GTS already running on {0}!!!".format(build_platform)
        print "Add -f(--force) option to remove the lock. Make sure GTS is not running anymore!"
        gts_is_running = True
        log_out_err_file.close()
        sys.exit(1)
    with open(lock_file, "w"): pass

    build_path = os.path.join(build_root_path, build_no, target)
    os.environ["RESOURCES_FOLDER"] = os.path.join(build_root_path, "resources")
    os.environ["PLATFORM"] = build_platform
    os.environ["TARGET"] = target
    if user_build_variant == "usersigned":
        user_signed = True
        user_build = True
    elif user_build_variant == "user":
        user_build = True
        user_signed = False
    else:
        user_build = False
        user_signed = False
    update = "update" in flash_xml_file

    # check battery level
    #check_battery_level()

    if no_flash:
        print "Skipping flash!!!"
    else:
        print "Flashing"
        if oem_unlock:
            print "Enable OEM unlock from Developer Options"
            cts_steps.prepare_devices_for_flash(intent = True,
                                                parallel = True,
                                                devices = devices,
                                                platform = build_platform)()
        flash_devices()

    has_bios = statics.Device(serial = all_devices[0]["serial"],
                              platform = build_platform,
                              dessert = build_version).has_bios
    if has_bios:
        check_bios_version()
    else:
        bios = "N/A"

    if no_gts_prepare:
        print "Skipping prepare for GTS step!!!"
    else:
        prepare_for_gts()
        reboot_devices()

    if no_gts_run:
        print "Skipping GTS run!!!"
    else:
        gts_screen_name = "gts-{0}-run".format(build_platform)
        run_gts()

    if no_gts_prepare and no_gts_run:
        print "Nothing else to do"
    else:
        prepare_for_next_run()
