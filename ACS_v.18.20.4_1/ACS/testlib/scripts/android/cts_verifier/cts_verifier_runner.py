from testlib.scripts.android.cts_verifier import cts_verifier_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.logcat import logcat_steps
from testlib.utils.connections.adb import Adb as connection_adb
import argparse
import os
import sys

DEVICE = {"TREKSTOR" : "DE8867CC",
          "CHT-FFD" : "INMO50700094",
          "ECS2-10A" : "8BE68170",
          "ECS2-8A" : "D388556B",
          "ECS27B" : "8BDC9270",
          "MALATA8_LOW" : "58902EA7",
          "SOFIA3GR" : "0123456789012345"}

INSTRUMENTATION = ["android.hardware.cts.SensorBatchingTests",
                   "android.hardware.cts.SensorIntegrationTests",
                   "android.hardware.cts.SensorTest",
                   "android.hardware.cts.SingleSensorTests"]

RESULTS_PATH = "/home/sensors/compliance/CTS/cts-5.1_r2/x86"

def run_prepare(platform):

    serial = DEVICE[platform]

    print "Pushing compass.conf"
    adb_steps.root_connect_device(serial = serial)()
    adb_steps.push_file(local = "/home/sensors/CTS/x86/android-cts/resource/compass.conf" , remote = "/data/compass.conf", serial = serial)()

    print "Unlocking device"
    adb_steps.wake_up_device(serial = serial)()
    adb_steps.menu_to_unlock(serial = serial)()

    print "Set location to False"
    cts_verifier_steps.set_location(serial = serial)()

    print "Set Adaptive brightness to False"
    cts_verifier_steps.set_display(serial = serial)()

    print "Set Stay Awake to False"
    adb_steps.enable_developer_options(serial = serial)()
    ui_steps.disable_options_from_developer_options(serial = serial,
                                                    enabled = True,
                                                    developer_options =
                                                    ["Stay awake"])()

    print "Set airplane mode to True"
    wifi_steps.set_airplane_mode(state = "ON", serial = serial)()
    ui_steps.press_home(serial = serial)()

    print "Set Auto rotate to False"
    cts_verifier_steps.set_screen_rotation(serial = serial)()
    ui_steps.press_home(serial = serial)()
#    ui_steps.open_quick_settings(serial = serial)()
#    ui_steps.click_button_if_exists(serial = serial,
#                                    view_to_find = {"text": "Auto-rotate"})()
#    ui_steps.press_home(serial = serial)()

def run_instrumentation(platform, build_number):

    platform_results_path = os.path.join(RESULTS_PATH, platform, "android-cts/repository/results")

    if not os.path.exists(platform_results_path):
        os.makedirs(platform_results_path)

    f = open(os.path.join(platform_results_path, "{0}_{1}_instrumentation_results.html".format(build_number, platform), "w")
    f.write("<HTML><BODY>")

    adb_steps.install_apk(apk_path = os.path.join(os.environ["HOME"], "compliance/resources/CtsHardwareTestCases.apk"),
                        serial=DEVICE[platform],
                        logger = None)()

    for item in INSTRUMENTATION:
        print "Running {0}...".format(item)
        logcat_steps.clear_logcat(serial=DEVICE[platform])()
        results = cts_verifier_steps.run_instrumentation(argument_list = ["-e class {0}".format(item)],
                                                        script = "com.android.cts.hardware/android.support.test.runner.AndroidJUnitRunner",
                                                        pass_message = "OK",
                                                        serial=DEVICE[platform],
                                                        logger = None)()

        log = open(os.path.join(platform_results_path, "{0}_{1}_LOGCAT_{2}.txt".format(build_number, platform, item )), "w")
        log.write(logcat_steps.get_logcat(serial = DEVICE[platform])())
        log.close()

        log = open(os.path.join(platform_results_path,"{0}_{1}_DMESG_{2}.txt".format(build_number, platform, item )), "w")
        log.write(logcat_steps.get_dmesg(serial = DEVICE[platform])())
        log.close()

        f.write("Results for instrumentation: " + item + "<br \>")
        f.write("Running Time : " + "<br \>")
        f.write(results.replace("\n","<br \>\n"))
        f.write("-------------------------------------------------<br \>")

    f.write("</BODY></HMTL>")
    f.close()

def main():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='Instrumentation Runner')
    parser.add_argument('--build-number', required=True, default="",
                        help="""the build number from which the \
                                artifacts will be downloaded""")
    parser.add_argument('--platform', required=True, default="",
                        help="""the platform for which the testing \
                                will be made""")

    args = parser.parse_args()

    adb_conn = connection_adb(serial = DEVICE[args.platform])
    build_number_on_device = adb_conn.get_prop(prop = "ro.build.version.incremental").strip()
    if args.build_number not in build_number_on_device:
        print "The image on the device has a different build number: expected {0} and found {1} on device {2}".format(args.build_number,  build_number_on_device, DEVICE[args.platform])
        sys.exit(1)

    run_prepare(args.platform)
    run_instrumentation(args.platform, args.build_number)

if __name__ == '__main__':
    main()

