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
from threading import Thread
from subprocess import Popen, PIPE, STDOUT
import Queue


globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val


uidevice = ui_device(serial=serial)
dut = Adb(serial=serial)

list_vol = "sm list-volumes"
test_file = "test_adb_copy.dat"

dut.adb_root()
uuid=""
def check_sdcard():
    global uuid
    p = dut.run_cmd('blkid')
    sdcard = False
    for line in p.stdout.read().split('\n'):
        if 'mmcblk0p1' in line:
            sdcard = True
            uuid = line.split(' ')[1].split('=')[1].strip('"')
            break

    if sdcard:
        m = dut.run_cmd(list_vol)
        for ln in m.stdout.read().strip().split('\n'):
            if uuid in ln:
                #returns mounted or unmounted
                return ln.split()[-2]
    else:
        return "SD Card Not Found"


def goto_Storage_Settings():
    launch = "com.android.settings"
    dut.run_cmd('pm clear %s' % launch)
    dut.run_cmd('am start %s' % launch)
    ui_steps.click_button_if_exists(serial=serial, view_to_find={"text": "Storage & USB"},view_to_check ={"text": "Portable storage"})()
    #dut.run_cmd('pm clear %s' % launch)

#Transfer file to device from host
def adb_file_transfer(destination):
    print "File Transferring to: " ,destination
    create_file = "fallocate -l 500M {0}".format(test_file)

    try:
        print "Creating a file: ", test_file
        os.system(create_file)
        cmd ='adb -s {0} push {1} {2}'.format(serial,test_file,destination)
        proc = Popen(cmd, shell=True, stdout=PIPE, stderr=STDOUT)
        if 'failed to copy' in proc.stdout.read():
            print "Expected: 'failed to copy' - [PASSED]"

        else:
            print "Unexpected ",proc.stdout.read()

    except Exception as e:
        print e.message


def unmount_sdcard():
    goto_Storage_Settings()
    #ui_steps.open_users_settings(serial=serial)
    sleep(1)
    ui_steps.click_button_if_exists(serial=serial, view_to_find={"descriptionContains": "Eject"})()


def remount_sdcard():
    goto_Storage_Settings()
    ui_steps.click_button_if_exists(serial=serial,view_to_find={"textContains": "Ejected"},view_to_check={"text": "Mount"})()
    ui_steps.click_button_if_exists(serial=serial,view_to_find={"textContains": "Mount"})()

if check_sdcard() == 'unmounted':
    remount_sdcard()
    print "SD Card is %s" %check_sdcard()

if check_sdcard() == 'mounted':
    try:

        dest = '/storage/'+uuid
        que = Queue.Queue()
        t = Thread(target=lambda q, arg1: q.put(adb_file_transfer(arg1)), args=(que, dest))
        t.start()
        sleep(15)
        unmount_sdcard()
        sleep(5)
        print "SD Card is %s" % check_sdcard()
        t.join()
        result = que.get()
        print result

    except Exception as e:
        print e.message

    finally:
        print "Deleted the test_file created in host machine"
        os.system("rm -rf %s" % test_file)
        if check_sdcard() == 'unmounted':
            print "Remounting the SD card"
            remount_sdcard()
            print "Closing the android settings"
            dut.run_cmd('pm clear com.android.settings')
        else:
            assert False, "UNEXPECTED: 'Failed to copy error not found' - [FAILED]"

else:
    raise "SD card not found"
    sys.exit(1)
