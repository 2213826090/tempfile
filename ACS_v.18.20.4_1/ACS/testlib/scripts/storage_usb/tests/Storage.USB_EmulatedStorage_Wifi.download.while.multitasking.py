import sys, os
from testlib.scripts.storage_usb import storage_usb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.utils.connections.adb import Adb
from testlib.base.base_utils import get_args
from testlib.utils.ui.uiandroid import UIDevice as ui_device
from testlib.external.uiautomator import AutomatorDeviceUiObject as t
from testlib.utils.ui.uiandroid import UIDevice as ui_device
from time import sleep


from testlib.utils.defaults import wifi_defaults
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps
from testlib.base.base_utils import get_args
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.adb import adb_steps
from testlib.utils.statics.android import statics

from testlib.scripts.android.android_step import step

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
encryption = None
if "encryption" in args.keys():
    encryption = args["encryption"]
ddwrt_ap_pass = None
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]
conf_security = None
if "conf_security" in args.keys():
    conf_security = args["conf_security"]
pairwise_cipher = None
if "pairwise_cipher" in args.keys():
    pairwise_cipher = args["pairwise_cipher"]
airplane_mode = "ON"
if "airplane_mode" in args.keys():
    airplane_mode = args["airplane_mode"]
compare_method = "cmp"
if "compare_method" in args.keys():
    compare_method = args["compare_method"]

radius_ip = None
if "radius_ip" in args.keys():
    radius_ip = args["radius_ip"]
radius_secret = None
if "radius_secret" in args.keys():
    radius_secret = args["radius_secret"]
radius_identity = None
if "radius_identity" in args.keys():
    radius_identity = args["radius_identity"]
EAP_method = None
if "EAP_method" in args.keys():
    EAP_method = args["EAP_method"]
phase_2_auth = None
if "phase_2_auth" in args.keys():
    phase_2_auth = args["phase_2_auth"]
user_certificate = None
if "user_certificate" in args.keys():
    user_certificate = args["user_certificate"]
channel_bw = None
if "channel_bw" in args.keys():
    channel_bw = args["channel_bw"]

##### test start #####
platform = statics.Device(serial=serial)

adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# airplane mode on, if configured so
#if airplane_mode == "ON":
#    wifi_generic_steps.set_airplane_mode(state = "ON", serial = serial)()

# wifi ON
wifi_generic_steps.set_wifi(state="ON", serial = serial)()

# configure ap with the first settings set
'''ap_steps.setup(mode, security,
               new_ssid = "SSG_LAB_VAL_S3",
               encryption = encryption,
               wifi_password = "ssg_ba_otc_val_linksyss3",
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               channel_bw = channel_bw,
               serial = serial)()
'''
# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# add the Wi-Fi network
wifi_generic_steps.add_network(ssid = "SSG_LAB_VAL_S3",
                               security = "WPA2",
                               password = "ssg_ba_otc_val_linksyss3",
                               #identity = radius_identity,
                               #EAP_method = EAP_method,
                               #phase_2_auth = phase_2_auth,
                               #user_certificate = user_certificate,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()


# check we are connected to the correct network
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = "SSG_LAB_VAL_S3",
                                        state='CONNECTED/CONNECTED',
                                        Security="WPA_PSK")()
                                        #pairwise_cipher=pairwise_cipher)()

#Download from external network SED network


uidevice = ui_device(serial=serial)
dut = Adb(serial=serial)
#Download a big file over wifi.

download = "am start -a android.intent.action.VIEW -d ftp://speedtest.tele2.net/100MB.zip"
#dut.run_cmd(download)
if uidevice(packageName="com.estrongs.android.pop").exists:
    uidevice(packageName="com.estrongs.android.pop", text="Hide").click()

def cam_landing_page():
    btn = ['Allow','NEXT']
    for b in btn:
        ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": b})()
        #sleep(2)
    return True

#Take a picture.
cmd_pic = "am start -a android.media.action.IMAGE_CAPTURE"
dut.run_cmd(cmd_pic)
cam_landing_page()
uidevice(resourceId = "com.android.camera2:id/shutter_button", description ="Shutter").click()
sleep(1)
done_btn = uidevice(resourceId = "com.android.camera2:id/done_button", description ="Done")
if done_btn.exists:
    done_btn.click()

#Take a video.
sleep(1)
cmd_vid = "am start -a android.media.action.VIDEO_CAPTURE"
dut.run_cmd(cmd_vid)
cam_landing_page()
uidevice(packageName="com.android.camera2", description ="Shutter").click()
sleep(10)
uidevice(packageName="com.android.camera2", description ="Shutter").click()
sleep(1)
if done_btn.exists:
    done_btn.click()

launch_photo = "am start com.google.android.apps.photos"
dut.run_cmd(launch_photo)
if uidevice(className="android.widget.Switch").exists :
    ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": "ON"})()
    ui_steps.press_back(serial=serial,view_to_check = {"text": "Keep off"})()
    ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": "Keep off"})()

img = uidevice(className="android.view.ViewGroup", descriptionContains="Photo")
vid = uidevice(className="android.view.ViewGroup", descriptionContains="Video")
count_img = img.count
count_vid = vid.count

#open image
img[count_img-1].click()
uidevice(packageName="com.google.android.apps.photos", descriptionContains="Edit").click.wait()
editor = uidevice(packageName="com.google.android.apps.photos", descriptionContains="filter")
from random import randint
editor[randint(1,editor.count)].click.wait()
uidevice(packageName="com.google.android.apps.photos", text="SAVE").click.wait()
uidevice(packageName="com.google.android.apps.photos", descriptionContains="Delete").click.wait()
uidevice(packageName="com.google.android.apps.photos", text="Delete").click.wait()
uidevice.press.back()

vid[count_vid-1].click()
uidevice(packageName="com.google.android.apps.photos").click()
sleep(5)
uidevice(packageName="com.google.android.apps.photos", descriptionContains="Pause").click.wait()
uidevice(packageName="com.google.android.apps.photos", descriptionContains="Delete").click.wait()
uidevice(packageName="com.google.android.apps.photos", text="Delete").click()

ui_steps.open_notifications_menu(serial=serial)
ui_steps.wait_for_view(view_to_find = {"text":"Download finished"}, timeout =200,serial = serial)()
