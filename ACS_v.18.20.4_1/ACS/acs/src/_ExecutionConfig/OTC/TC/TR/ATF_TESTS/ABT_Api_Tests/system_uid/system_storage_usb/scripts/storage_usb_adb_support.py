from adb_helper.adb_utils import *
from test_utils import *
import os.path
import sys

if os.getenv('ACS_EXECUTION_CONFIG_PATH') is not None:
    sys.path.insert(0, (os.getenv('ACS_EXECUTION_CONFIG_PATH') + '/TC/TP'))
else:
    sys.path.insert(0,(os.getcwd() + '/_ExecutionConfig/OTC/TC/TP'))
from testlib.androidframework.common import *
from testlib.androidframework.shell_utils import ShellUtils as term

class StorageUSBAdbTests(unittest.TestCase):
    TAG = "ADB Support"

    def setUp(self):
        print self.TAG, "setUp"

    def tearDown(self):
        print self.TAG, "tearDown"

    def test_adb_secure(self):
        #get-serialno and checking for working of adb shell cmd
        serial_num = AdbUtils.run_adb_cmd("getprop | grep ro.boot.serialno")
        serial_num = serial_num.split(':')[-1].strip().replace('[','').replace(']','')
        print "Serial-Number: ", serial_num
        #To find which line device is listed in adb devices
        adb_devices = term.run_shell_cmd('adb devices')
        ln = 0
        for lines in adb_devices.split('\n'):
            if 'device' in lines:
                if serial_num in lines:
                    print "Line = ",ln
                    break
                ln = ln + 1
        #To find the address of adkkey for device
        cmd = '''awk '{0}print ${1}{2}' <~/.android/adbkey.pub | openssl base64 -A -d -a | openssl md5 -c'''
        output = term.run_shell_cmd(cmd.format('{',ln,'}'))
        print "Secure adb public/private key: ", output
        #validate key
        if 'stdin' in output:
            adb_key = output.split('=')[-1].strip()
            import re
            k = re.match(r'^([\w]{2}[:-]){15}([\w]{2})$',adb_key)
            if k:
                print "Valid key: ",k.group()
            else:
                assert("Invalid key format")
        else:
            assert('Invalid ADB secure key')


    def test_sdcardfs_support(self):
        sdcardfs = AdbUtils.run_adb_cmd("mount | grep sdcardfs")
        #Find the partition mount point
        if len(sdcardfs.strip()) == 0:
            assert("sdcardfs Not found in mount")
        for lines in sdcardfs.strip().split('\n'):
            if '/storage/emulated' in lines:
                print 'sdcardfs supported'
        #Create a file
        from random import randint
        file_name = 'test_file' + '_' + str(randint(1,99))
        cmd = 'touch /storage/emulated/0/{0}'
        AdbUtils.run_adb_cmd(cmd.format(file_name))

        #Check File Permission
        cmd_1 = 'ls -l /storage/emulated/0/{0}'
        before_perm = AdbUtils.run_adb_cmd(cmd_1.format(file_name))
        before_perm =before_perm.split()[0]
        print "File Permission: ",before_perm

        #change permission
        cmd_2 = 'chmod 777 /storage/emulated/0/{0}'
        AdbUtils.run_adb_cmd(cmd_2.format(file_name))

        after_perm = AdbUtils.run_adb_cmd(cmd_1.format(file_name))
        after_perm = after_perm.split()[0]
        print "File Permission after chmod: ",after_perm

        #compare permission
        if before_perm != after_perm:
            assert("File Permission Changed")

if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(StorageUSBAdbTests, "test_adb_secure")
    print test_result.wasSuccessful()