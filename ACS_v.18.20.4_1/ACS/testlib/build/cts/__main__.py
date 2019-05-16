import sys
import time
import os
import atexit
import traceback
from optparse import OptionParser

from testlib.utils.logger import log_stdout_stderr
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


version = "0.3.4"

def lock_release():
    try:
        """
            This lock is used in the flash logic if usb debugging is not
            enabled (legacy)
        """
        os.rmdir("/tmp/lock")
    except: pass

def remove_suite_lock():
    if not suite_is_running:
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
def remove_environ_vars():
    try:
        del os.environ["LOG_PATH"]
        del os.environ["TESTLIB_UI_HANDLERS_GROUP"]
        del os.environ["RESOURCES_FOLDER"]
        del os.environ["PLATFORM"]
        del os.environ["TARGET"]
    except: pass

suite_is_running = False
if 'suite_mail_report_failing_tests' not in globals():
    suite_mail_report_failing_tests = False
if 'gts_version' not in globals():
    gts_version = None
if 'cts_version' not in globals():
    cts_version = None

atexit.register(lock_release)
atexit.register(remove_suite_lock)
atexit.register(remove_tmp_files)
atexit.register(remove_environ_vars)

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

def flash_devices(suite, FLASH_STATICS, SUITE_NAME_STATICS):
    global devices
    global device_prop
    print "Preparing image for flash"
    flash_xml_path = os.path.join(build_path, build_platform)
    flash_xml_path = "{0}{1}".format(flash_xml_path, FLASH_STATICS[suite])
    flash_cfg_file = os.path.join(flash_xml_path, flash_xml_file)
    resource_subfolder = statics.Device(serial = devices[0]["serial"],
                                        platform = build_platform,
                                        dessert = build_version).resource_folder
    edit_flash_file = statics.Device(serial = devices[0]["serial"],
                                     platform = build_platform,
                                     dessert = build_version).edit_flash_file
    repackage_userdata = statics.Device(serial = devices[0]["serial"],
                                     platform = build_platform,
                                     dessert = build_version).repackage_userdata_on_flash
    device_prop = flash_steps.prepare_image_for_flash(path_to_image = build_path,
                                                      platform = build_platform,
                                                      cts_abi = FLASH_STATICS[suite],
                                                      prop_file = "build.prop",
                                                      user_signed = user_signed,
                                                      resources_subfolder = resource_subfolder,
                                                      device_prop = "ro.build.version.incremental",
                                                      edit_flash_file = edit_flash_file,
                                                      repackage_userdata = repackage_userdata,
                                                      flash_xml_path = flash_cfg_file,
                                                      fls_tool_location = fls_tool_location)()
    print "DONE preparing image!"
    print "Start flashing for CTS"
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
                  send_mail = True,
                  suite = suite,
                  SUITE_NAME_STATICS = SUITE_NAME_STATICS)


def check_devices(previous_step = "prepare for cts",
                  send_mail = False,
                  return_data = None,
                  suite = 'cts',
                  SUITE_NAME_STATICS = {}):
    global devices
    print "Check serials for availability after {0} step: {1}".format(previous_step, return_data)
    device_to_remove = []
    for device in devices:
        serial = device["serial"]
        previous_check_fail = return_data is not None and not return_data[serial]
        if previous_check_fail or not local_utils.has_adb_serial(serial = serial):
            device_to_remove.append(device)
    for device in device_to_remove:
        devices.remove(device)
    serials = [device["serial"] for device in devices]
    print "Serials still available after {0} step: {1}".format(previous_step, str(serials))
    if send_mail:
        if send_email_type == "SSH":
            cts_steps.send_report(debug = True,
                    recipients = report_recipients,
                    subject = "{0}: {1} shards available after {2} for {3} with {4}".format(SUITE_NAME_STATICS[suite],
                                                                                              len(serials),
                                                                                              previous_step,
                                                                                              build_platform,
                                                                                              build_no),
                    objective = "Serials: {0}".format(serials),
                    email_sender = "SSH",
                    host = email_machine,
                    user = email_machine_user,
                    passwd = email_machine_password,
                    critical = False)()
        else:
            if send_email_type == "SSH":
                cts_steps.send_report(debug = True,
                        recipients = report_recipients,
                        subject = "{0}: {1} shards available after {2} for {3} with {4}".format(SUITE_NAME_STATICS[suite],
                                                                                              len(serials),
                                                                                              previous_step,
                                                                                              build_platform,
                                                                                              build_no),
                        objective = "Serials: {0}".format(serials),
                        email_sender = "SSH",
                        host = email_machine,
                        user = email_machine_user,
                        passwd = email_machine_password,
                        critical = False)()
            else:
                cts_steps.send_report(debug = True,
                        recipients = report_recipients,
                        subject = "{0}: {1} shards available after {2} for {3} with {4}".format(SUITE_NAME_STATICS[suite],
                                                                                              len(serials),
                                                                                              previous_step,
                                                                                              build_platform,
                                                                                              build_no),
                        objective = "Serials: {0}".format(serials),
                        critical = False)()

    if len(serials) == 0:
        print("This run has reached 0 devices ready for the next step. Terminating!")
        sys.exit(1)

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

def prepare_for_suite(no_media_files, suite):

    global devices

    if suite == "cts":
        kwargs = {}
        kwargs['devices'] = devices
        kwargs['cts_media_path'] = cts_media_path
        kwargs['cts_base_path'] = cts_base_path
        kwargs['cts_tcs_dir'] = "repository/testcases/"
        kwargs['cts_admin_apk'] = "CtsDeviceAdmin.apk"
        kwargs['recipients'] = report_recipients
        kwargs['platform'] = build_platform
        kwargs['email_sender'] = send_email_type
        kwargs['email_machine'] = email_machine
        kwargs['email_machine_user'] = email_machine_user
        kwargs['email_machine_password'] = email_machine_password
        kwargs['dessert'] = build_version
        kwargs['hidden_ap'] = True
        kwargs['ap_name'] = cts_ap_name
        kwargs['ap_pass'] = cts_ap_pass
        kwargs['ap_encryption'] = cts_ap_encryption
        kwargs['ap_802_1x_EAP_method'] = cts_ap_802_1x_EAP_method
        kwargs['ap_802_1x_phase_2_auth'] = cts_ap_802_1x_phase_2_auth
        kwargs['ap_802_1x_user_certificate'] = cts_ap_802_1x_user_certificate
        kwargs['ap_802_1x_identity'] = cts_ap_802_1x_identity
        kwargs['ap_802_1x_anonymous_identity'] = cts_ap_802_1x_anonymous_identity
        kwargs['intent'] = True
        kwargs['browser_init'] = browser_init
        kwargs['blocking'] = False
        kwargs['critical'] = False
        kwargs['parallel'] = True
        kwargs['copy_media_files'] = not no_media_files
    elif suite == "gts":
        kwargs = {}
        kwargs['devices'] = devices
        kwargs['gts_base_path'] = gts_base_path
        kwargs['gts_tcs_dir'] = "repository/testcases/"
        kwargs['recipients'] = report_recipients
        kwargs['platform'] = build_platform
        kwargs['email_sender'] = send_email_type
        kwargs['email_machine'] = email_machine
        kwargs['email_machine_user'] = email_machine_user
        kwargs['email_machine_password'] = email_machine_password
        kwargs['hidden_ap'] = True
        kwargs['ap_name'] = gts_ap_name
        kwargs['ap_pass'] = gts_ap_pass
        kwargs['ap_encryption'] = gts_ap_encryption
        kwargs['ap_802_1x_EAP_method'] = gts_ap_802_1x_EAP_method
        kwargs['ap_802_1x_phase_2_auth'] = gts_ap_802_1x_phase_2_auth
        kwargs['ap_802_1x_user_certificate'] = gts_ap_802_1x_user_certificate
        kwargs['ap_802_1x_identity'] = gts_ap_802_1x_identity
        kwargs['ap_802_1x_anonymous_identity'] = gts_ap_802_1x_anonymous_identity
        kwargs['intent'] = True
        kwargs['blocking'] = False
        kwargs['critical'] = False
        kwargs['parallel'] = True
    elif suite == "aft":
        kwargs = {}
        kwargs['devices'] = devices
        kwargs['recipients'] = report_recipients
        kwargs['platform'] = build_platform
        kwargs['email_sender'] = send_email_type
        kwargs['email_machine'] = email_machine
        kwargs['email_machine_user'] = email_machine_user
        kwargs['email_machine_password'] = email_machine_password
        kwargs['intent'] = True
        kwargs['blocking'] = False
        kwargs['critical'] = False
        kwargs['parallel'] = True

    if suite in ['aft', 'cts']:
        target_library = "testlib.scripts.android.cts.cts_steps"
    elif suite == 'gts':
        target_library = "testlib.scripts.android.gts.gts_steps"
    library = sys.modules[target_library]

    return_data = getattr(library, "prepare_devices_for_{0}".format(suite))(**kwargs)()

    check_devices(previous_step = "prepare",
                  send_mail = False,
                  return_data = return_data)
    time.sleep(1)

def reboot_devices():
    global devices
    cts_steps.reboot_devices(devices = devices,
                             parallel = True,
                             blocking = True)()
    check_devices(previous_step = "reboot", send_mail = False)

def run_suite(suite, SUITE_NAME_STATICS):

    global devices

    print "Starting the {0} suite".format(suite)
    if suite == "cts":
        kwargs = {}
        kwargs['screen_name'] = suite_screen_name
        kwargs['serials'] = [d["serial"] for d in devices]
        kwargs['devices'] = devices
        kwargs['cts_abi'] = cts_abi
        kwargs['email_sender'] = send_email_type
        kwargs['email_machine'] = email_machine
        kwargs['email_machine_user'] = email_machine_user
        kwargs['email_machine_password'] = email_machine_password
        kwargs['dessert'] = build_version
        kwargs['cts_plan_name'] = cts_plan_name
        kwargs['cts_module_name'] = cts_module_name
        kwargs['cts_base_path'] = cts_base_path
        kwargs['cts_binary_dir'] = "tools/"
        kwargs['cts_tcs_dir'] = "repository/testcases/"
        kwargs['cts_plans_dir'] = "repository/plans/"
        kwargs['cts_results_dir'] = "repository/results/"
        kwargs['cts_logs_dir'] = "repository/logs/"
        kwargs['cts_binary'] = "cts-tradefed"
        kwargs['cts_version'] = cts_version
        kwargs['cts_disable_reboot'] = cts_disable_reboot
        kwargs['cts_skip_preconditions'] = cts_skip_preconditions
        kwargs['cts_mail_report_failing_tests'] = cts_mail_report_failing_tests
        kwargs['loops_no'] = cts_run_loops_no
        kwargs['list_results_command'] = "list results"
        kwargs['exit_command'] = "exit"
        kwargs['cts_mail_report_template'] = "cts_mail_report_template.html"
        kwargs['recipients'] = report_recipients
        kwargs['debug_recipients'] = []
        kwargs['build_no'] = build_no
        kwargs['bios'] = bios
        kwargs['platform'] = build_platform
        kwargs['target'] = target
        kwargs['user_build_variant'] = user_build_variant
        kwargs['remote_path'] = "/opt/regression/mail/"
    elif suite == "gts":
        kwargs = {}
        kwargs['screen_name'] = suite_screen_name
        kwargs['devices'] = devices
        kwargs['gts_plan_name'] = gts_plan_name
        kwargs['gts_base_path'] = gts_base_path
        kwargs['email_sender'] = send_email_type
        kwargs['email_machine'] = email_machine
        kwargs['email_machine_user'] = email_machine_user
        kwargs['email_machine_password'] = email_machine_password
        kwargs['gts_binary_dir'] = "tools/"
        kwargs['gts_tcs_dir'] = "repository/testcases/"
        kwargs['gts_plans_dir'] = "repository/plans/"
        kwargs['gts_results_dir'] = "repository/results/"
        kwargs['gts_logs_dir'] = "repository/logs/"
        kwargs['gts_binary'] = "xts-tradefed"
        kwargs['gts_version'] = gts_version
        kwargs['loops_no'] = gts_run_loops_no
        kwargs['list_results_command'] = "list results"
        kwargs['exit_command'] = "exit"
        kwargs['gts_mail_report_template'] = "gts_mail_report_template.html"
        kwargs['gts_mail_report_failing_tests'] = gts_mail_report_failing_tests
        kwargs['recipients'] = report_recipients
        kwargs['debug_recipients'] = []
        kwargs['build_no'] = build_no
        kwargs['bios'] = bios
        kwargs['platform'] = build_platform
        kwargs['target'] = target
        kwargs['user_build_variant'] = user_build_variant
        kwargs['remote_path'] = "/opt/regression/mail/"

    if suite == 'cts':
        target_library = "testlib.scripts.android.cts.cts_steps"
    elif suite == 'gts':
        target_library = "testlib.scripts.android.gts.gts_steps"
    library = sys.modules[target_library]

    if len(devices) > 0:
        return_data = getattr(library, "run_{0}_plan".format(suite))(**kwargs)()

    else:
        cts_steps.send_report(debug = True,
                             recipients = report_recipients,
                             subject = "{0}: {1} cannot run on 0 devices on {2} with {3}!!!".format(SUITE_NAME_STATICS[suite],
                                                                                                    suite,
                                                                                                    build_platform,
                                                                                                    build_no),
                             objective = "No device is available for running {0}!!!!".format(suite.upper()),
                             critical = False)()

    print "{0} run Done!".format(suite.upper())

def prepare_for_next_run(suite):
    devices = [device for device in all_devices]
    print "Start preparing for the next {0} run".format(suite.upper())
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


def validate_global_var(settings_file = 'settings', var_name = None, var_type = None):
    # check if variable is defined
    if var_name not in globals():
        print "{0}.py - settings file error:".format(settings_file)
        print "{0} not found".format(var_name)
        sys.exit(1)

    # check for correct type
    # if variable is None, no further checks are performed
    if globals()[var_name]:
        # special validation for paths
        if var_type == "path":
            if not os.path.exists(globals()[var_name]):
                print "{0}.py - settings file error:".format(settings_file)
                print "{0} is not a valid path".format(var_name)
                sys.exit(1)
        elif not isinstance(globals()[var_name], var_type):
            print "{0}.py - settings file error:".format(settings_file)
            print "{0} is defined as {1}. It should be {2}".format(var_name,
                                                                   type(globals()[var_name]),
                                                                   var_type)
            sys.exit(1)


def validate_settings_vars(settings_file, suite):
    # list of vars to check for in settings.py
    # tuple of (var_name, var_type)
    common_vars = [("HOME", "path"),
                   ("battery_ready_level", int),
                   ("all_devices", list),
                   ("build_root_path", "path"),
                   ("flash_xml_file", basestring),
                   ("user_build_variant", basestring),
                   ("build_version", basestring),
                   ("report_recipients", list),
                   ("send_email_type", basestring),
                   ("email_machine", basestring),
                   ("email_machine_user", basestring),
                   ("email_machine_password", basestring),
                   ("shutdown_to_charge", bool),
                   ("fls_tool_location", "path"),
                   ("fake_userdata_size", basestring)]

    cts_vars = [("browser_init", bool),
                ("cts_skip_preconditions", bool),
                ("cts_version", basestring),
                ("cts_abi", basestring),
                ("cts_media_path", "path"),
                ("cts_base_path", "path"),
                ("cts_plan_name", basestring),
                ("cts_run_loops_no", int),
                ("cts_disable_reboot", bool),
                ("cts_mail_report_failing_tests", bool),
                ("cts_ap_name", basestring),
                ("cts_ap_pass", basestring),
                ("cts_ap_encryption", basestring),
                ("cts_ap_802_1x_EAP_method", basestring),
                ("cts_ap_802_1x_phase_2_auth", basestring),
                ("cts_ap_802_1x_user_certificate", basestring),
                ("cts_ap_802_1x_identity", basestring),
                ("cts_ap_802_1x_anonymous_identity", basestring)]

    gts_vars = [("browser_init", bool),
                ("gts_version", basestring),
                ("gts_plan_name", basestring),
                ("gts_run_loops_no", int),
                ("gts_disable_reboot", bool),
                ("gts_mail_report_failing_tests", bool),
                ("gts_ap_name", basestring),
                ("gts_ap_pass", basestring),
                ("gts_ap_encryption", basestring),
                ("gts_ap_802_1x_EAP_method", basestring),
                ("gts_ap_802_1x_phase_2_auth", basestring),
                ("gts_ap_802_1x_user_certificate", basestring),
                ("gts_ap_802_1x_identity", basestring),
                ("gts_ap_802_1x_anonymous_identity", basestring)]

    for var_tuple in common_vars:
        validate_global_var(settings_file = settings_file,
                            var_name = var_tuple[0],
                            var_type = var_tuple[1])
    if suite == "cts":
        for var_tuple in cts_vars:
            validate_global_var(settings_file = settings_file,
                                var_name = var_tuple[0],
                                var_type = var_tuple[1])
    elif suite == "gts":
        for var_tuple in gts_vars:
            validate_global_var(settings_file = settings_file,
                                var_name = var_tuple[0],
                                var_type = var_tuple[1])


if __name__ == "__main__":
    global devices

    parser = OptionParser()
    parser.add_option("-v", "--version",
                      action = "store_true",
                      dest = "version")
    parser.add_option("-s", "--suite",
                      dest = "suite",
                      help = "Use this option to select the suite that should be run. Available options are cts, gts and aft")
    parser.add_option("-a", "--abi",
                      dest = "abi",
                      help = "Use this option to select the abi for a CTS run. Available options are x86 and arm. It will be ignored for other suites.",
                      default = None)
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
    parser.add_option("-U", "--no-suite-prepare",
                      dest = "no_suite_prepare",
                      default = False,
                      action = "store_true",
                      help = "Use this option to skip suite prepare step (the setup of the devices)")
    parser.add_option("-R", "--no-suite-run",
                      dest = "no_suite_run",
                      default = False,
                      action = "store_true",
                      help = "Use this option to skip the suite run")
    parser.add_option("-M", "--no-media-files",
                      dest = "no_media_files",
                      default = False,
                      action="store_true",
                      help = "Use this option to skip cts media files copying (Not applicable for GTS or AFT)")
    parser.add_option("-o", "--oem-unlock",
                      dest="oem_unlock",
                      default = False,
                      action = "store_true",
                      help = "Use this option to enable oem unlock from Developer Options")
    parser.add_option("-f", "--force",
                      dest = "force_run",
                      default = False,
                      action = "store_true",
                      help = "Use this option to run the suite even if a run lock exists")

    (options, args) = parser.parse_args()
    if options.version:
        print "Android OS Compliance suite runner version {0}".format(version)
        #log_out_err_file.close()
        sys.exit(0)

    ##default values for new settings options
    fls_tool_location = os.path.join(os.environ['HOME'], "acas/DownloadTool")
    fake_userdata_size = "4569694208"
    cts_module_name = ''

    globals().update(vars(options))
    if build_platform is None or build_no is None or target is None:
        print "Please provide target, platform and build number information!!!"
        parser.print_help()
        #log_out_err_file.close()
        sys.exit(1)
    if abi:
        settings_file = "settings_{0}_{1}".format(suite, abi)
    else:
        settings_file = "settings_{0}".format(suite)
    module = __import__(settings_file, globals(), locals(), [], -1)
    globals().update(module.__dict__)

    validate_settings_vars(settings_file, suite)

    cts_abi = abi

    LOCK_STATICS = {
        'gts': suite,
        'cts': "{0}-{1}".format(suite, cts_abi),
        'aft': suite
    }

    #create log files for curent run
    abi_for_logs = ""
    if cts_abi:
        abi_for_logs = "_{0}".format(cts_abi)
    log_root = "logs_{0}{1}".format(suite, abi_for_logs)
    if not os.path.exists(log_root):
        os.mkdir(log_root)
    dirfmt = "%4d-%02d-%02d_%02d-%02d-%02d"
    log_path = os.path.join(log_root, dirfmt % time.localtime()[0:6])
    os.mkdir(log_path)
    os.environ["LOG_PATH"] = log_path
    log_out_err_file = log_stdout_stderr(os.path.join(log_path, "all.log"))

    devices = [d for d in all_devices]

    # check if the suite is already running
    # if no create the lock
    lock_dir = os.path.join(build_root_path)
    if not os.path.exists(lock_dir):
        os.mkdir(lock_dir)

    lock_file = os.path.join(lock_dir, "lock%run%{0}%{1}".format(LOCK_STATICS[suite], build_platform))
    if os.path.exists(lock_file) and not force_run:
        print "{0} already running on {1}!!!".format(LOCK_STATICS[suite], build_platform)
        print "Add -f(--force) option to remove the lock. Make sure the selected suite is not running anymore!"
        suite_is_running = True
        log_out_err_file.close()
        sys.exit(0)
    with open(lock_file, "w"): pass

    os.environ["TESTLIB_UI_HANDLERS_GROUP"] = "cts"

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

    FLASH_STATICS = {
        'gts': suite,
        'cts': cts_abi,
        'aft': suite,
    }

    SUITE_NAME_STATICS = {
        'gts': gts_version,
        'cts': "{0}-{1}".format(cts_version, cts_abi),
        'aft': "AFT"
    }


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
        flash_devices(suite, FLASH_STATICS, SUITE_NAME_STATICS)

    has_bios = statics.Device(serial = all_devices[0]["serial"],
                              platform = build_platform,
                              dessert = build_version).has_bios
    if has_bios:
        check_bios_version()
    else:
        bios = "N/A"

    if no_suite_prepare:
        print "Skipping the prepare for suite step!!!"
    else:
        if suite in ["aft", "gts"]:
            no_media_files = True
        print ("Starting to prepare the suite setup")
        prepare_for_suite(no_media_files, suite)
        reboot_devices()

    if suite == "aft":
        no_suite_run = True
    if no_suite_run:
        print "Skipping the suite run!!!"
    else:
        suite_screen_name = "{0}-{1}-run".format(SUITE_NAME_STATICS[suite], build_platform)
        run_suite(suite, SUITE_NAME_STATICS)

    if no_suite_prepare and no_suite_run:
        print "Nothing else to do"
    # else:
        # if suite != 'aft':
            # print ("prepare for next suite")
            #prepare_for_next_run(suite)
