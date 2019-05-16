#!/usr/bin/env python

##### imports #####
import os
import sys
import time
import filecmp
import pexpect
from testlib.scripts.android.ui.security import security_utils
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args

##### initialization #####
globals().update(vars(get_args(sys.argv)))

##### test start #####
platform_name = security_utils.get_platform_name()
file_name = None
if platform_name == "bxtp_abl": file_name = "M.txt"
if platform_name == "gordon_peak": file_name = "O.txt"

adb_steps.connect_device(serial = serial)()
adb_steps.root_connect_device(serial = serial)()
time.sleep(5)

os.system("mkdir -p ./temp/files")

# child = pexpect.spawn("scp -o \"StrictHostKeyChecking=no\" android-prcqa@10.239.97.117:/home/www/PRCQA/Auto_Test_Res/security/files/" + file_name + " ./temp/files")
# child.expect("android-prcqa@10.239.97.117's password:")
# child.sendline("123@456")
# child.interact()
os.system("curl -o ./temp/files/" + file_name + " \"https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/Security/files/" + file_name + "\" -k  > /dev/null 2>&1")

os.system("touch ./temp/files/file.txt")
os.system("adb shell cat /system/etc/security/cacerts/* > ./temp/files/file.txt")

result = filecmp.cmp("./temp/files/file.txt", "./temp/files/" + file_name)

os.system("sudo rm -rf ./temp")

if not result:
    raise Exception("The test result did not achieve the desired results")
##### test end #####