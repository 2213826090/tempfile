#Copyright (C) 2014  Chen mei <meix.chen@intel.com@intel.com>
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

import os
import time
from testlib.util.common import g_common_obj


class FTPSet:
    """
    FTPSetting definition
    """

    def __init__(self, cfg):
        """Get value from config"""

        self.name = cfg.get("name")
        self.passwd = cfg.get("passwd")
        self.ssid = cfg.get("ssid")
        self.security = cfg.get("security")
        self.identity = cfg.get("identity")
        self.eap = cfg.get("eap")
        self.phase = cfg.get("phase")
        self.ca = cfg.get("ca")
        self.usr = cfg.get("usr")
        self.conf = cfg.get("conf")
        self.dconf = cfg.get("dconf")
        self.type = cfg.get("type")
        self.setname = cfg.get("setname")
        self.pin = cfg.get("pin")
        self.aconf = cfg.get("aconf")
        self.apchangetime = cfg.get("apchangetime")
        self.enable = cfg.get("enable")
        self.times = cfg.get("times")
        self.number = cfg.get("num")

        self.type = cfg.get("type")
        # self.timeout = cfg.get("timeout")
        # self.size = cfg.get("size")
        self.ftpfile = cfg.get("ftpfile")


class FTPSettingImpl:
    """
    Implements AndFTP Setting UI actions.
    """

    ftp_package = "lysesoft.andftp"
    ftp_activity = ".SettingsActivity"
    es_package = "com.estrongs.android.pop"
    es_activity = ".view.FileExplorerActivity"

    class Locator(object):
        """
        Helper class to locate UI object on screen
        """

        def __init__(self, device):
            """Init environment"""
            self.d = device

        @property
        def btn_connect(self):
            """ UI button Connect """
            return self.d(text="Connect")

        @property
        def btn_close(self):
            """ UI button Close """
            return self.d(text="Close")

        @property
        def btn_ok(self):
            """ UI button OK """
            if self.d(text="Ok").exists:
                return self.d(text="Ok")
            elif self.d(text="OK").exists:
                return self.d(text="OK")

        @property
        def btn_save(self):
            """ UI button Save """
            return self.d(text="Save")

        @property
        def btn_download(self):
            """ UI button Download """
            if self.d(text="download").exists:
                return self.d(text="download")
            elif self.d(text="Download").exists:
                return self.d(text="Download")
            elif self.d(description="download"):
                return self.d(description="download")
            elif self.d(description="Download").exists:
                return self.d(description="Download")

        @property
        def btn_property(self):
            """ UI button Properties """
            return self.d(text="Properties")

        @property
        def btn_browser(self):
            """ UI button ftp file browser """
            return self.d(description="Device file browser")

        @property
        def btn_upload(self):
            """ UI button upload """
            if self.d(text="upload").exists:
                return self.d(text="upload")
            elif self.d(text="Upload").exists:
                return self.d(text="Upload")
            elif self.d(description="upload"):
                return self.d(description="upload")
            elif self.d(description="Upload").exists:
                return self.d(description="Upload")

        @property
        def btn_delete(self):
            """ UI button Delete """
            return self.d(text="Delete")

        @property
        def btn_detail(self):
            """ UI button Details """
            return self.d(text="Details")

        @property
        def ui_scroll(self):
            """ UI scroll """
            return self.d(scrollable=True)

        @property
        def more_options(self):
            """ UI scroll """
            return self.d(description="More options")

        @property
        def ftp_host(self):
            """UI text ftp host"""
            return self.d(resourceId="lysesoft.andftp:id/ftp_host")

        @property
        def ftp_port(self):
            """UI text ftp port"""
            return self.d(resourceId="lysesoft.andftp:id/ftp_port")

        @property
        def ftp_user(self):
            """UI text ftp username"""
            return self.d(resourceId="lysesoft.andftp:id/ftp_username")

        @property
        def ftp_passwd(self):
            """UI text ftp password"""
            return self.d(resourceId="lysesoft.andftp:id/ftp_password")

        @property
        def ftp_local(self):
            """UI text ftp store path"""
            return self.d(resourceId="lysesoft.andftp:id/ftp_local_dir")

    def __init__(self, cfg):
        """Init environment"""
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = FTPSettingImpl.Locator(self.d)

    def launch_from_am(self):
        """Launch AndFTP from am"""
        print "[INFO] Launch AndFTP from am"
        g_common_obj.launch_app_am(self.ftp_package, self.ftp_activity)
        time.sleep(2)
        assert self.d(packageName=self.ftp_package)

    def stop_from_am(self, packageName):
        """Stop app from am"""
        print "[INFO] Stop app %s from am" % packageName
        g_common_obj.stop_app_am(packageName)

    def init_ftp(self, host, user, passwd, store):
        """Set up the ftp environment"""
        if self.d(text="Disable tips").exists:
            self.d(text="Disable tips").click()
            self._locator.btn_close.click()
        time.sleep(2)
        self.d(text="Add").click()
        time.sleep(2)
        self._locator.ftp_host.set_text(host)
        self._locator.ftp_port.click()
        self._locator.ftp_port.set_text("21")
        self._locator.ftp_user.click()
        self._locator.ftp_user.set_text(user)
        self._locator.ftp_passwd.click()
        self._locator.ftp_passwd.set_text(passwd)
        self._locator.ftp_local.clear_text()
        self._locator.ftp_local.set_text(store)
        time.sleep(1)

        self.d(text="Advanced").click.wait()
        self.d(text="Prompt").click.wait()
        self.d(text="Overwrite/Resume").click()
        self._locator.btn_save.click()

        if self.d(text="Name FTP settings to be saved:").exists:
            self._locator.btn_ok.click()
        flag = "FTP server settings saved successfully"
        if self.d(textContains=flag).exists:
            self._locator.btn_ok.click()

    def connect(self):
        """Connect AndFTP to host"""
        print "[INFO] Connect AndFTP"
        ftp_host = self.cfg.get("hostname")
        ftp_user = self.cfg.get("username")
        ftp_passwd = self.cfg.get("password")
        ftp_store = self.cfg.get("localdir")

        if self._locator.btn_browser.exists:
            return True

        if not self.d(text=ftp_host).exists:
            self.init_ftp(ftp_host, ftp_user, ftp_passwd, ftp_store)

        self._locator.btn_connect.click()
        time.sleep(4)
        assert self._locator.btn_browser.exists

    def download(self, ftpset):
        """Download file from AndFTP"""
        file_name = ftpset.ftpfile
        print "[INFO] Download file from AndFTP %s" % file_name
        cmdstr = "rm -rf /sdcard/Download/%s" % file_name
        g_common_obj.adb_cmd(cmdstr)
        if self._locator.ui_scroll.exists:
            self._locator.ui_scroll.scroll.to(text=file_name)

        self.d(text=file_name).click()
        self.d(description="Download").click()
        self.d(text="OK").click()
        timeout = int(self.cfg.get("%s_timeout" % file_name.lower()))
        for i in range(timeout):
            if self.d(textContains="Download completed").exists:
                self.d(text="OK").click()
                break
            time.sleep(1)

    def download_check(self, ftpset):
        """Check file is downloaded successfully"""
        file_name = ftpset.ftpfile
        print "[INFO]: Check success to download"
        g_common_obj.launch_app_am(self.es_package, self.es_activity)
        time.sleep(4)
        self._locator.btn_download.click()
        if self._locator.ui_scroll.exists:
            self._locator.ui_scroll.scroll.to(text=file_name)
        self.d(text=file_name).drag.to(text=file_name, steps=50)
        time.sleep(2)
        self.d.press.menu()
        self._locator.btn_property.click()
        size = self.cfg.get("%s_dl_size" % file_name.lower())
        assert self.d(textContains=size).exists
        self.stop_from_am(self.es_package)

    def upload(self, ftpset):
        """Upload file to FTP"""
        file_name = ftpset.ftpfile
        print "[INFO] Upload file to Andftp %s" % file_name
        if not self.d(text="Current: /upload").exists:
            self._locator.btn_upload.click()
        time.sleep(2)
        if self.d(text=file_name).exists:
            self.d(text=file_name).click()
            self._locator.more_options.click()
            self._locator.btn_delete.click()
            self._locator.btn_ok.click()
        time.sleep(2)
        self._locator.btn_browser.click()
        self.d(text="[Go up a folder]").click()

        self._locator.btn_upload.click()
        self.d(text=file_name).click.wait()
        self._locator.btn_upload.click.wait()
        self._locator.btn_ok.click()

        timeout = int(self.cfg.get("%s_timeout" % file_name.lower()))
        for i in range(timeout):
            if self.d(textContains="Upload completed").exists:
                self._locator.btn_ok.click()
                break
            time.sleep(1)
        for i in range(3):
            self.d.press.back()
        time.sleep(3)
        assert self.d(text=file_name).exists

    def upload_check(self, ftpset):
        """Check success to upload file"""
        file_name = ftpset.ftpfile
        print "[INFO] Check success to upload %s" % file_name
        self.d(text=file_name).click()
        self._locator.more_options.click()
        self._locator.btn_detail.click()
        size = self.cfg.get("%s_up_size" % file_name.lower())
        assert self.d(textContains=size).exists
        self.stop_from_am(self.ftp_package)
