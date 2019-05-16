from time import sleep
import os.path
from subprocess import Popen, PIPE
import sys
import random

from adb_helper.adb_utils import *
from test_utils import *

if os.getenv('ACS_EXECUTION_CONFIG_PATH') is not None:
    sys.path.insert(0, (os.getenv('ACS_EXECUTION_CONFIG_PATH') + '/TC/TP'))
else:
    sys.path.insert(0,(os.getcwd() + '/_ExecutionConfig/OTC/TC/TP'))
from testlib.util.common import g_common_obj
from testlib.androidframework.common import *


class StorageUSBFillTests(unittest.TestCase):
    '''
    Description: Check for the internal memory and perform write operation to fill
    the memory and perform test cases after finding the storage full notification.

    Usage: Functionality checking when device memory is full

    To do: AdbUtil has to be changed to g_common_obj implementation
    '''

    TAG = "StorageUSBFillTests"

    def setUp(self):
        print self.TAG, "setUp"
        self.d = g_common_obj.get_device()
        self.file_name = 'test_data_mem_' + str(random.randint(1000,9999))

    def tearDown(self):
        print self.TAG, "tearDown"


    def test_fill_internal_memory_check_notification(self):
        #self.d = g_common_obj.get_device()
        print self.d.info

        #Check for internal memory free space
        free_memory = 'df /sdcard'
        output = AdbUtils.run_adb_cmd(free_memory)
        #output = g_common_obj.adb_cmd_capture_msg(free_memory)
        print output

        delta = 50 * 1024 * 1024
        if 'sdcard' in output:
            f_space = output.split('\n')[1].split()[-2]
            if 'G' in f_space:
                print 'G'
                f_space = f_space[0:-1]
                f_space =  float(f_space) * (1024*1024*1024)
                f_space =f_space - delta
                self.fill_data(f_space)
            elif 'M' in f_space:
                f_space = f_space[0:-1]
                if float(f_space) > 50:
                    f_space = float(f_space) * (1024 * 1024)
                    f_space = f_space - delta
                    self.fill_data(f_space)
                else:
                    print "Memory is low already. Check for notification " \
                          "exists"
            else:
                self.assertFalse('Could not read the free space')
                exit(1)
        else:
            self.assertFalse('Failed to find the free_Space')
            exit(1)

        check_space = AdbUtils.run_adb_cmd(free_memory)
        print "After Filling \n" ,check_space

        #open notification
        self.d.wait.update()
        self.d.open.notification()
        sleep(3)

        #check for Storage space running out notification
        if not (self.d.exists(text="Storage space running out")):
            self.assertFalse("Storage space running out: Notification not found")

    def test_fill_internal_memory_install_apk(self):
        self.test_fill_internal_memory_check_notification()
        free_memory = 'df /sdcard'
        output = AdbUtils.run_adb_cmd(free_memory)

        print "Going to Install an APK"
        apk_file = 'AnTuTu-5.6.apk'
        nav_dir = os.path.join(os.path.expanduser("~/"),
                               ".acs/Artifacts/BENCHMARKS/ANTUTU/")
        cur_dir = os.getcwd()

        # changing directory to apk file location
        os.chdir(nav_dir)
        if os.path.exists(apk_file):
            print "APK file found"
            output = Popen(['adb', 'install', apk_file], stdin=PIPE,
                           stdout=PIPE, stderr=PIPE)
            out, err = output.communicate()
            if not 'No space left on device' in err:
                self.assertFalse(
                    "APK got installed though Storage space running out")
            else:
                print "APK was not installed due to low internal memory"

            # changing back the directory to script location
            os.chdir(cur_dir)

    def test_clear_internal_memory_full_notification(self):
        self.d.open.notification()

        if (self.d.exists(text="Storage space running out")):
            output = AdbUtils.run_adb_cmd("ls /sdcard/Download/test_data_mem_*")
            if 'No such file or directory' not in output:
                print "File found"
                remove_file = "rm -rf /sdcard/Download/test_data_mem_*"
                AdbUtils.run_adb_cmd(remove_file)
            else:
                print "{0} file not found".format(self.file_name)

        free_memory = 'df /sdcard'
        output = AdbUtils.run_adb_cmd(free_memory)
        print output

        self.d.press.home()

        self.d.open.notification()
        if (self.d.exists(text="Storage space running out")):
            sleep(60)
            if (self.d.exists(text="Storage space running out")):
                self.assertFalse("NOTIFICATION FOUND AFTER CLEARING DATA: Storage space running out")
        self.d.press.home()

    def fill_data(self,f_space):
        print "Filling the Internal Memory, Takes more time to write"
        #using dd command to fill data
        dd_cmd = "dd if=/dev/urandom of=/sdcard/Download/{0}.dat " \
                 "bs=65535 count={1}".format(self.file_name,
            int(round(f_space/65535)))
        print dd_cmd
        output_dd = AdbUtils.run_adb_cmd(dd_cmd)
        print output_dd
        print "Filling data completed.."

    def test_open_all_apps(self):
        #List the current packages available in the device
        cmd = 'pm list packages'
        output_pkg = AdbUtils.run_adb_cmd(cmd)
        pkg = output_pkg.replace('package:', '').split('\n')

        #commands used for starting the application
        cmd2 = "shell pm dump {0} |grep -A 1 'filter' | head -n 1 | cut -d ' ' -f 12"
        cmd3 = "am start {0}"
        cmd4 = "pm clear {0}"
        cmd5 = "shell pm dump {0} | grep -A 1 MAIN"
        cmd6 = "am start {0} -a android.intent.action.MAIN -c android.intent.category.LAUNCHER"

        #Apps that works without activity manager
        f_apps = []  #collect actual package name
        self.d.press.home()
        apps = ['youtube', 'camera', 'deskclock', 'music', 'docs', 'chrome', 'videos', 'photos', 'calendar', 'settings', 'talk', 'magazines']
        for i in apps:
            for p in pkg:
                if i in p:
                    f_apps.append(p)
                    break
        print f_apps
        #launch the apps and check for crash message
        for t in f_apps:
            m = AdbUtils.run_adb_cmd(cmd3.format(t.strip()))
            #self.d.press.home()
            #sleep(1)
            if SystemPopupsAndDialogs.is_app_crash_popup_visible():
                self.assertFalse("App Crashed for ",t)

        #To Launch the apps based on the activity manager
        act_apps = []
        apps_2 = ['googlequicksearchbox', 'contacts', 'gm', 'maps', 'calculator']
        for app in apps_2:
            for p in pkg:
                if app in p:
                    act_apps.append(p)
                    break
        print act_apps
        #Launch the apps with activity manager
        for item in act_apps:
            print item,
            n = AdbUtils.run_adb_cmd(cmd2.format(item.strip()),adb_shell=False)
            if n == '':
                m = AdbUtils.run_adb_cmd(cmd5.format(item.strip()),adb_shell=False)
                n = m.split('\n')[1].strip().split()[-1]
            #Launching the app using am start
            AdbUtils.run_adb_cmd(cmd6.format(n))
            if SystemPopupsAndDialogs.is_app_crash_popup_visible():
                self.assertFalse("App Crashed for ",item)

        #Close all the apps that are opened
        for p in f_apps+act_apps:
            print "Clearing App: ",p
            n = AdbUtils.run_adb_cmd(cmd4.format(p.strip()))
            sleep(2)
            if 'Success' not in n:
                self.assertFalse('App failed to close: ',p.strip())

if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(StorageUSBFillTests,"test_fill_internal_memory")
    print test_result.wasSuccessful()
