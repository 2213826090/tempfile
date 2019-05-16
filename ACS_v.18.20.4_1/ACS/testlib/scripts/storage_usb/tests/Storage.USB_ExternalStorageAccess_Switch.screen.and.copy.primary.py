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


globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

#stdout =  adb_steps.command(serial=serial, command= "ls", mode = "async",stdout_grep= 'proc')()
#adb_steps.open_users_settings(serial=serial)
uidevice = ui_device(serial=serial)
dut = Adb(serial=serial)

fileManager_pkg = 'com.estrongs.android.pop'
app_launch = 'com.estrongs.android.pop/.app.openscreenad.NewSplashActivity'
check_box = 'android.widget.CheckBox'
test_file = 'CS.dat'

#Create and transfer 1G of file to Download
create_file = "fallocate -l 1G {0}".format(test_file)
print os.getcwd()

os.system(create_file)
dut.put_file(test_file,'/sdcard/Download/',timeout=200)
os.system("rm -rf %s"%test_file)

#Launch ES file explorer
dut.run_cmd('am start %s' %app_launch)
ui_steps.swipe

def pass_through_guidlines():
    uidevice = ui_device(serial=serial)
    app = uidevice(className='android.widget.FrameLayout')
    x_coord = app.info['bounds']['right']
    y_coord = app.info['bounds']['left']
    print x_coord
    print y_coord
    start_btn = uidevice.exists(className='android.widget.Button',textContains= 'START NOW')
    while not (start_btn):
        app.swipe('left')
        if start_btn:
            ui_steps.click_button_if_exists(serial=serial,view_to_find = {"text": "START NOW"})()
            ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": "OK"})()
            sleep(1)
        if uidevice.exists(className="android.widget.TextView", textContains = "Internal Storage"):
            break
        clipboard = uidevice.exists(className='android.widget.Button',textContains= 'Clipboard')
        if clipboard:
            print "Clipboard Found"

#if launching for the first time
if not uidevice.exists(className="android.widget.TextView", textContains = "Internal Storage"):
    pass_through_guidlines()
ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": "Internal Storage"})()
ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": "Internal Storage"}, view_to_check ={"text": "Download"})()
ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": "Download"}, view_to_check ={"text": test_file})()

#long press a 2G file and copy to other directory
f = uidevice(className="android.widget.TextView", textContains=test_file)
fx = f.info['bounds']['right']
fy = f.info['bounds']['bottom']
uidevice.long_click(fx,fy)

ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": "Copy"})()

paste_btn = uidevice.exists(className='android.widget.Button', textContains = "Paste")
steps = ("New", "OK", "Folder", "Paste")
while paste_btn :
    for s in steps:
        ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": s })()
        sleep(1)
    break

copying  = uidevice.exists(className='android.widget.TextView', textContains = "Copying")
if copying:
    print "Copying in Progress"

progress = uidevice.exists(className='android.widget.TextView', textContains = "Progress")
while progress:
    print "Progress bar found"
    for i in range(3, -1, -1):
        # switch screen between landscape/portrait mode when the file is copying
        cmd = "settings put system user_rotation"+ " "+ str(i)
        print cmd
        dut.run_cmd(cmd)
        sleep(2)
    progress = False

bar = uidevice.exists(className='android.widget.TextView', textContains = "Progress")
print "Waiting to complete the file copy"
while bar:
    print " .. "
    ui_steps.wait_for_view(serial=serial, view_to_find={"text": test_file})
    if uidevice.exists(textContains=test_file):
        bar = False
        sleep(2)
        print " . "

uidevice(resourceId = "com.estrongs.android.pop:id/address_bar").click()

#Deleting the folder
f = uidevice(className="android.widget.TextView",textContains = 'Folder')
fx = f.info['bounds']['right']
fy = f.info['bounds']['bottom']
uidevice.long_click(fx,fy)
print "Folder Selected to Delete"
del_steps = ["Delete", "Move to recycle bin", "OK"]
for d in del_steps:
    ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": d })()
    sleep(3)