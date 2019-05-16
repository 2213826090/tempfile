#Copyright (C) 2014  Lan, SamX <samx.lan@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

'''
@summary: test basic GOTA update
@since: 11/07/2014
@author: Sam Lan(samx.lan@intel.com)
'''
import os
import string
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl
from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl
from testlib.util.common import g_common_obj
from testlib.dut_init.dut_init_impl import Function
from testlib.common.common import g_common_obj2
from testlib.util.device import TestDevice
from testlib.AfW.api_impl import ApiImpl
from testlib.AfW.entity import Remote

class CheckBasicInfoBeforeGOTA(UIATestBase):
    """
    @summary: Test basic GOTA update
    """

    def setUp(self):
        super(CheckBasicInfoBeforeGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), 'tests.tablet.gota.conf')
        globle_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), 'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota = gotaImpl(self.cfg)
        self.system = SystemImpl(self.cfg)
        self.apk = ApkInstallUninstallImpl(self.cfg)
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")
        self.d = g_common_obj.get_device()
        self.locator = ApkInstallUninstallImpl.Locator(self.d)
        self.serial = g_common_obj2.getSerialNumber()
        self.api = ApiImpl()
        self.remote = Remote()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckBasicInfoBeforeGOTA, self).tearDown()
        self.cfg = None

    def testCheckBasicInfoBeforeGOTA(self):
        """
        This test case is to test basic GOTA update function

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. test GOTA update

        Expect Result:
        1. DUT can GOTA update successfully

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()

        base=self.cfg.get("base")
        target=self.cfg.get("target")
        base_build=self.cfg.get("base_build")
        gota_test_run_by_testplan = self.cfg.get("gota_test_run_by_testplan")
        self.gota.download_gota_content_artifactory('/GOTA/','gota.zip')
        print "finish download"
        base_bios=self.cfg.get("base_bios")
        host_file=self.cfg.get("host_file")
        client_file=self.cfg.get("client_file")
        base_fingerprint=self.cfg.get("base_fingerprint")
        google_account=self.cfg.get("google_account")
        google_password=self.cfg.get("google_password")
        apk_name=self.cfg.get("apk_name")
        apk_file=self.cfg.get("apk_file")
        apk_website=self.cfg.get("apk_website")
        host_mp3_file=self.cfg.get("host_mp3_file")
        client_mp3_file=self.cfg.get("client_mp3_file")
        sdcard_file = self.cfg.get("sdcard_file")
        oem_file = self.cfg.get("oem_file")
        timezone = self.cfg.get("timezone")

        g_common_obj.root_on_device()
        self.gota.mount_device()
        self.gota.unlock_screen()
        print "[INFO]connect wifi ....."
        print "self.ssid: %s" %self.ssid
        print "self.passwd: %s" %self.passwd
        if self.ssid==None:
            print "using default ap setting"
            self.gota.connect_AP("shz13-otc-irda102","qweasdzxc")
        else:
            print "using config ap setting"
            self.gota.connect_AP(self.ssid,self.passwd)
        self.d.press.home()
        #add for file operation start
        exist = os.path.exists("/tmp/GOTA_test")
        if False == exist:
            os.mkdir("/tmp/GOTA_test")
        date = os.popen("date").read()
        filename=date.replace(" ","",6).strip('\n')
        gota_record="/tmp/GOTA_test/"+filename+".log"
        os.mknod(gota_record)
        fp=open(gota_record,'a')
        fp.write("latest GOTA test:"+gota_record+"\n")
        fp.close()
        self.gota.update_info(gota_record)
        self.gota.insert_info("GOTA status","before GOTA")

        #add for file operation end
        cache_before_gota=os.popen("adb -s %s shell df |grep -E '/cache '" %self.serial).read()
        fplist=cache_before_gota.split()
        cache_used=fplist[2].encode('utf-8')
        self.gota.insert_info("Cache patition used before GOTA",cache_used)
        data_before_gota=os.popen("adb -s %s shell df |grep -E '/data '" %self.serial).read()
        fplist=data_before_gota.split()
        data_used=fplist[2].encode('utf-8')
        self.gota.insert_info("Data patition used before GOTA",data_used)

        #1.check the bios version before gota
        BIOS_before_GOTA = os.popen("adb -s %s shell getprop ro.bootloader" %self.serial).read()
        BIOS_before_GOTA=BIOS_before_GOTA.replace("\n","",1)
        self.gota.insert_info("BIOS before GOTA",BIOS_before_GOTA)
        print "[INFO]: 1. check the bios version before gota is: %s" %BIOS_before_GOTA
        #2.push sample.txt file into dut before gota
        if host_file.count("http")!=0:
            file_path="/tmp/resource_ota"
            self.gota.file_download(host_file,file_path)
            host_file="%s/%s" %(file_path, host_file.split("/")[-1])
            print "[INFO]: The host file is: %s" %host_file
        self.system.adb_push_file(host_file, client_file)
        exist = self.system.file_exists(client_file, "DEVICE")
        print "[INFO]: 2. check the push file before gota is: %s" %client_file
        #3.check the fingerprint before gota
        Fingerprint_before_GOTA = os.popen("adb -s %s shell getprop ro.build.fingerprint" %self.serial).read()
        Fingerprint_before_GOTA=Fingerprint_before_GOTA.replace("\n","",1)
        self.gota.insert_info("Fingerprint before GOTA",Fingerprint_before_GOTA)
        print "[INFO]: 3. check the fingerprint before gota is: %s" %Fingerprint_before_GOTA
        #4.install apk before gota
        self.gota.unlock_screen()
        time.sleep(2)
        self.locator.btn_apps.click()
        print "*************************"
        print apk_name
        print "*************************"
        resource = "/tmp/resource_apk/" + apk_file
        if not self.apk.apk_check(apk_name):
            cmd = "adb -s %s install %s"%(self.serial, resource)
            print cmd
            pipe=os.popen(cmd).read()
            print pipe
        print "[INFO]: 4. check the installed apk before gota is: %s" %apk_name
        #5.push mp3 file into dut before gota
        if host_mp3_file.count("http")!=0:
            file_path="/tmp/resource_ota"
            self.gota.file_download(host_mp3_file,file_path)
            host_mp3_file="%s/%s" %(file_path, host_mp3_file.split("/")[-1])
            print "[INFO]: The host file is: %s" %host_mp3_file
        self.system.adb_push_file(host_mp3_file, client_mp3_file)
        assert self.system.file_exists(client_mp3_file, "DEVICE"), "[ERROR]: The client file %s does not exist" % client_mp3_file
        print "[INFO]: 5. check the mp3 before gota is: %s" %client_mp3_file
        self.system.adb_push_file(host_file, sdcard_file)
        exist=self.system.file_exists(sdcard_file, "DEVICE")
        print "[INFO]: 6. check the sdcard file before gota is: %s" %sdcard_file
        assert self.system.file_exists(oem_file, "DEVICE"), "[ERROR]: The oem file %s does not exist before gota" % oem_file
        print "[INFO]: 7. check the oem file before gota is: %s" %oem_file
        OStime_before_GOTA = os.popen("adb -s %s shell date +%%s" %self.serial).read()
        print ("%s" %OStime_before_GOTA )

        cmd="OStime_before_GOTA:"+OStime_before_GOTA
        OStime_before_GOTA=OStime_before_GOTA.replace("\n","",1)
        self.gota.insert_info("OStime before GOTA",OStime_before_GOTA)
        print "[haley]: 8. OStime_before_GOTA is: %s" % OStime_before_GOTA
        self.gota.unlock_screen()
        time.sleep(1)
        status=self.gota.check_encryption_status()
        print "[INFO]: 9. check encryption status before gota is: %s" %status
        self.gota.insert_info("Encryption status before GOTA",status)
        self.gota.update_info(gota_record)
        exist=self.system.file_exists(client_file, "DEVICE")
        print "[INFO]: 10. check the emmc push file before gota is: %s" %client_file
        if g_common_obj2.getAndroidVersion()=='L':
            mdm_apk = self.api.download_file_from_artifactory(self.remote.sample_mdm['sub_path'],self.remote.sample_mdm['name'])
        else:
            mdm_apk = self.api.download_file_from_artifactory(self.remote.sample_mdm['sub_path'],self.remote.sample_mdm['name_m'])    
        old_dir = os.getcwd()
        os.chdir(os.path.split(mdm_apk)[0])
        if g_common_obj2.getAndroidVersion()=='L':
            self.api.install_apps(self.remote.sample_mdm['pkg_name'],self.remote.sample_mdm['name'].split('.apk')[0])
        else:
            self.api.install_apps(self.remote.sample_mdm['pkg_name'],self.remote.sample_mdm['name_m'].split('.apk')[0])
        os.chdir(old_dir)
        self.gota.check_AFW_profile_work_before_GOTA()
        self.gota.check_AFW_data_in_managed_profile_before_GOTA()
        self.gota.check_AFW_data_in_personal_profile_before_GOTA()
        #self.gota.check_AFW_chrome_restriction_before_GOTA()
        print "[Info] --- Profile work Sample MDM is successful before gota"

