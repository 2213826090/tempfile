#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
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

"""
@summary: Class for manipulate AndFTP application
@since: 07/10/2014
@author: yongzoux (yongx.zou@intel.com)
"""

import os
import time

from tests.utils.common import g_common_obj
from tests.wifi_reliability.conf import cfg
from tests.acs_aligned_mtbf.BROWSING.BROWSING_impl.BROWSING_impl import BrowsingImpl

class FtpOption(object):
    ''' Constant class for FTP setting option value
    Overwrite options, Retry options
    '''

    # option of Overwrite
    Overwrite_Resume = "Overwrite/Resume"
    Overwrite_Skip = "Skip"
    Overwrite_Prompt = "Prompt"

    # option of Retry:
    Retry_Disable = "Disabled"
    Retry_3x = "3 attempts (starts 20s after failure)"
    Retry_5x = "5 attempts (starts 20s after failure)"
    Retry_10x = "10 attempts (starts 20s after failure)"


class FtpSetting(object):
    ''' data mode for store ftp config
    '''

    def __init__(self, dic):
        self._dic = dic

    def __getattr__(self, name):
        return self._dic[name].strip()

class AddFtpServer(object):
    ''' class for manipulate 'Add/Edit FTP server settings' pop-up UI,
    after click Add or Edit on Application Home screen
    '''


    def __init__(self, device):
        self.d = device

    #---------Begin UI of 'GENERAL' Tab -----------------
    @property
    def __hostname(self):
        """ UI Hostname: """
        return self.d(resourceId="lysesoft.andftp:id/ftp_host")

    @property
    def __type(self):
        """ UI selected type """
        return self.d(resourceId="android:id/text1")

    @property
    def __type_list(self):
        """ UI a small triangle at the right-bottom corner of Type:
        use it to trigger pop-up available types
        """
        return self.d(resourceId="lysesoft.andftp:id/ftp_type")

    def __type_option(self, ftptype):
        """ UI type in pop-up available types """
        return self.d(className="android.widget.CheckedTextView", textStartsWith=ftptype)

    @property
    def __port(self):
        """ UI Port: """
        return self.d(resourceId="lysesoft.andftp:id/ftp_port")

    @property
    def __username(self):
        """ UI Username: """
        return self.d(resourceId="lysesoft.andftp:id/ftp_username")

    @property
    def __password(self):
        """ UI Password: """
        return self.d(resourceId="lysesoft.andftp:id/ftp_password")

    @property
    def __local_dir(self):
        """ UI Local dir: """
        return self.d(resourceId="lysesoft.andftp:id/ftp_local_dir")

    @property
    def __remote_dir(self):
        """ UI Remote dir: """
        return self.d(resourceId="lysesoft.andftp:id/ftp_remote_dir")

    @property
    def __resume(self):
        """ UI checkbox Enable resume support """
        return self.d(resourceId="lysesoft.andftp:id/ftp_resume")

    #---------End UI of 'GENERAL' Tab -----------------

    #---------Begin UI of 'ADVANCED' Tab -----------------
    @property
    def __tab_advanced(self):
        """ UI Tab Advanced """
        return self.d(text="Advanced", resourceId="android:id/title")

    @property
    def __overwrite(self):
        """ UI Overwrite: """
        return self.d(resourceId="lysesoft.andftp:id/ftp_overwrite")

    def __overwrite_option(self, option):
        """ UI option of dropdown Overwrite: """
        return self.d(text=option, className="android.widget.CheckedTextView")

    @property
    def __retry(self):
        """ UI Retry: """
        return self.d(resourceId="lysesoft.andftp:id/ftp_retry")

    def __retry_option(self, option):
        """ UI option of dropdown Overwrite: """
        return self.d(text=option, className="android.widget.CheckedTextView")

    #---------End UI of 'ADVANCED' Tab -----------------

    @property
    def __btn_save(self):
        """ UI button Save """
        return self.d(text="Save")

    @property
    def __name_to_be_saved(self):
        """ UI label 'Name FTP settings to be saved:'
        after click button Save, pop-up a prompt for setting saved name
        """
        return self.d(text="Name FTP settings to be saved:")

    @property
    def __save_success_msg(self):
        """ UI label 'FTP server settings saved successfully into your device.' """
        return self.d(text="FTP server settings saved successfully into your device.")

    @property
    def __btn_ok(self):
        """ UI button Ok
        after click button Save, pop-up a prompt for setting saved name
        the button Ok appear on the pop-up
        """
        return self.d(text='Ok')

    def add_ftp_setting(self, setting):
        """ Add FTP server setting """
        self.__fill_general_setting(setting)
        self.__fill_advance_setting_with_default()
        self.__btn_save.click()
        self.__name_to_be_saved.wait.exists(timeout=10*1000)
        self.__btn_ok.click()
        self.__save_success_msg.wait.exists(timeout=10*1000)
        self.__btn_ok.click()

    def __fill_advance_setting_with_default(self):
        """ fill FTP server Advance setting with default value """
        self.__tab_advanced.click()
        self.__overwrite.click()
        self.__overwrite_option(FtpOption.Overwrite_Resume).click()
        time.sleep(2)
        self.__retry.click()
        self.__retry_option(FtpOption.Retry_10x).click()
        time.sleep(2)

    def __fill_general_setting(self, setting):
        """ fill FTP server General setting """
        self.__hostname.wait.exists(timeout=10*1000)
        self.__hostname.clear_text()
        self.__hostname.set_text(setting.hostname)
        self.__type_list.click()
        self.__type_option(setting.type.upper()).click()
        self.__port.set_text(setting.port)
        self.__username.clear_text()
        self.__username.set_text(setting.username)
        self.__password.clear_text()
        self.__password.set_text(setting.password)
        self.__local_dir.clear_text()
        self.__local_dir.set_text(setting.local_dir)
        self.__remote_dir.clear_text()
        self.__remote_dir.set_text(setting.remote_dir)

    def edit_ftp_setting(self, setting):
        """ Edit FTP server setting """
        self.__fill_general_setting(setting)
        self.__btn_save.click()
        self.__name_to_be_saved.wait.exists(timeout=10*1000)
        self.__btn_ok.click()
        self.__save_success_msg.wait.exists(timeout=10*1000)
        self.__btn_ok.click()

class AndFTPImpl(object):

    packagename = "lysesoft.andftp"
    activityname = ".SplashActivity"

    def __init__(self, device):
        self.d = device
        self.ftpsetting = None
        self.notifyPanelImpl = None

    @property
    def __btn_add(self):
        """ UI button Add """
        return self.d(text="Add")

    @property
    def __btn_connect(self):
        """ UI button Connect """
        return self.d(text="Connect")

    @property
    def __ftp_server_list(self):
        """ UI click it can pop up added ftp servers """
        return self.d(resourceId="lysesoft.andftp:id/settings_server_list")

    @property
    def __ftp_server_first_option(self):
        """ UI first item listed out in the pop-up after click selected ftp server """
        return self.d(className="android.widget.CheckedTextView", instance=0)

    def __ftp_server_option(self, server):
        """ UI item listed out in the pop-up after click selected ftp server """
        return self.d(className="android.widget.CheckedTextView", text=server)

    @property
    def __selected_server(self):
        """ UI selected ftp server on AndFTP Home screen, Under 'Select your FTP server:' """
        return self.d(className="android.widget.TextView", resourceId="android:id/text1")

    @property
    def __browser_list(self):
        """ UI the file / folder list """
        return self.d(resourceId="lysesoft.andftp:id/browser_list", className="android.widget.ListView")

    def __browser_item(self, name):
        """ UI file or folder under current path """
        return self.d(resourceId="lysesoft.andftp:id/browser_item_name", text=name)

    @property
    def __current_path(self):
        """ UI display the current path, above button [Go up a folder] """
        return self.d(resourceId="lysesoft.andftp:id/browser_title", textStartsWith="Current:")

    @property
    def __btn_home(self):
        """ UI image button Home for jump to AndFTP Home screen """
        return self.d(resourceId="android:id/home")

    @property
    def __go_upfolder(self):
        """ UI [Go up a folder] """
        self.__browser_list.\
            child_by_text("[Go up a folder]", allow_scroll_search=True)
        return self.d(text="[Go up a folder]")

    @property
    def __btn_download(self):
        """ UI image button download """
        return self.d(description="Download")

    @property
    def __btn_upload(self):
        """ UI image button upload """
        return self.d(description="Upload")

    @property
    def __btn_browser_device_file(self):
        """ UI image button 'Device file browser' """
        return self.d(description="Device file browser")

    @property
    def __btn_browser_ftp_file(self):
        """ UI image button 'Device file browser' """
        return self.d(description="FTP file browser")

    @property
    def __btn_more_options(self):
        """ UI image button 'More options' """
        return self.d(description='More options')

    @property
    def __btn_refresh(self):
        """ UI button 'Refresh' in 'More options' """
        return self.d(text='Refresh')

    @property
    def __msg_remove_ftpserver_prompt(self):
        """ UI label 'Are you sure you want to remove ....'
        after you click button Remove to remove added ftp server
        """
        return self.d(textStartsWith="Are you sure you want to remove")

    @property
    def __msg_download_prompt(self):
        """ UI label '...download from FTP server to.....'
        appear on download prompt window, after click download button
        """
        return self.d(textContains="download from FTP server to")

    @property
    def __msg_download_complete(self):
        """ UI label 'Download completed (x files)' x is a number
        """
        return self.d(textContains="Download completed")

    @property
    def __msg_upload_prompt(self):
        """ UI label '...upload from device to.....'
        appear on upload prompt window, after click upload button
        """
        return self.d(textContains="upload from device to")

    @property
    def __msg_upload_complete(self):
        """ UI label 'Upload completed (x files)' x is a number
        """
        return self.d(textContains="Upload completed")

    @property
    def __btn_ok(self):
        """ UI button OK
        appear on below window: remove ftp server setting/download file prompt/download complete/upload file prompt/
        upload complete
        """
        return self.d(text="OK")

    @property
    def __btn_cancel(self):
        """ UI button Cancel
        appear on below window: remove ftp server setting/download file prompt/download complete/upload file prompt/
        upload complete
        """
        return self.d(text="Cancel")

    def launch_from_ui(self):
        """ Launch AndFTP from UI
        """
        print "[INFO] Launch AndFTP from UI"
        g_common_obj.launch_app_from_home_sc("AndFTP", "Apps", None)
        self.__btn_home.wait.exists(timeout=20*1000)
        assert self.__btn_home.exists, "[ERROR] Launch AndFTP from UI fail"
        print "[INFO] Launch success"

    def launch_from_am(self):
        """ Launch AndFTP from adb am
        """
        print "[INFO] Launch AndFTP from adb am"
        g_common_obj.launch_app_am(AndFTPImpl.packagename, AndFTPImpl.activityname)
        self.__btn_home.wait.exists(timeout=20*1000)
        assert self.__btn_home.exists, "[ERROR] Launch AndFTP from adb am fail"
        print "[INFO] Launch success"

    def stop_from_am(self):
        """ Stop AndFTP from am
        """
        print "[INFO] Stop AndFTP from am"
        g_common_obj.stop_app_am(AndFTPImpl.packagename)

    def connect_to(self, ftpsetting=None):
        """ Connect to Ftp server
        please call launch_from_ui or launch_from_am before this function, to start AndFtp application
        @param ftpsetting: (instance of FtpSetting), 
        if None, read ftp setting from config file, add and connect to ftp server from this ftp setting
        """
        self.ftpsetting = ftpsetting
        if self.ftpsetting is None:
            self.ftpsetting = cfg.ftp_setting

        hostname = self.ftpsetting.hostname
        if not hostname or not hostname.strip():
            raise Exception("[ERROR] Please set host name in config file")
        print "[INFO] Connect to FTP Server: %s" % hostname
        # There is no any FTP Server, add a FTP Server
        if self.__selected_server.exists is False:
            self.add_ftp_server(self.ftpsetting)

        self.__ftp_server_list.click()
        self.__ftp_server_option(hostname).wait.exists(timeout=3*1000)
        # if your FTP Server's host name not in the added list
        if self.__ftp_server_option(hostname).exists is False:
            self.__ftp_server_first_option.click()
            self.add_ftp_server(self.ftpsetting)
        else:
            self.__ftp_server_option(hostname).click()
        self.__btn_connect.click()
        # wait connect success
        self.__current_path.wait.exists(timeout=60*1000)
        assert self.__current_path.exists, "Connect to FTP Server: %s fail" % hostname
        print "[INFO] Connect success"

    def add_ftp_server(self, setting):
        """ Add FTP Server """
        print "[INFO] Add FTP Server"
        self.ftpsetting = setting
        self.__btn_add.click()
        addftpserver = AddFtpServer(self.d)
        addftpserver.add_ftp_setting(self.ftpsetting)
        self.__selected_server.wait.exists(timeout=10*1000)
        self.__selected_server.click()
        self.__ftp_server_option(setting.hostname).wait.exists(timeout=5*1000)
        if self.__ftp_server_option(setting.hostname).exists is False:
            raise Exception("[ERROR] Add FTP Server Fail")
        self.__ftp_server_option(setting.hostname).click()
        print "[INFO] Add success"

    def get_download_url(self, download_file):
        """ Get download url """
        print "[INFO] Get download url"
        if self.ftpsetting is None:
            self.ftpsetting = cfg.ftp_setting
        username = self.ftpsetting.username
        passord = self.ftpsetting.password
        hostname = self.ftpsetting.hostname
        port = self.ftpsetting.port

        if download_file:
            if download_file.startswith(os.sep):
                download_file = download_file[1:]
    
            if download_file.endswith(os.sep):
                download_file = download_file[0:-1]

        download_url = "ftp://%s:%s@%s:%s/%s" \
            % (username, passord, hostname, port, download_file)
        print "[INFO] %s " % download_url
        return download_url

    def __find_browser_item(self, name):
        """ Find item under browser list
        if not appear in current screen, will scroll to find it
        """
        return self.__browser_list.\
            child_by_text(name, allow_scroll_search=True, \
                          resourceId="lysesoft.andftp:id/browser_item_name")

    def __select_browser_item(self, name):
        """ Select browser item (file or folder)
        """
        print "[INFO] Select browser item (file or folder)"
        self.__find_browser_item(name)
        self.__browser_item(name).drag.to(
            resourceId="lysesoft.andftp:id/browser_item_name", text=name)
        time.sleep(2)
        print "[INFO] '%s' selected" % name

    def __open_folder(self, folder_path):
        """ Open folder recursively
        @param folder_path: a relative path, relatived to remote/local home folder
        remote home folder = remote dir of ftp setting
        local home folder = local dir of ftp setting
        """
        # Back to Home folder
        self.__back_to_home_folder()
        if not folder_path:
            return
        print "[INFO] Open folder: %s" % folder_path
        folder_path = folder_path.replace('/', os.sep)
        if folder_path.startswith(os.sep):
            folder_path = folder_path[1:]

        if folder_path.endswith(os.sep):
            folder_path = folder_path[0:-1]
        folders = folder_path.split(os.sep)
        for folder in folders:
            if folder:
                self.__find_browser_item(folder).click()
                time.sleep(1)
        print "[INFO] Open folder complete"

    def __get_current_path(self):
        """ Get the path of current opened folder
        """
        return self.__current_path.info['text'].split(':')[1].strip()

    def __back_to_home_folder(self):
        """ Back to home folder
        Click [Go up a folder] unitl enter home folder
        """
        home_folder_path = self.ftpsetting.local_dir.strip()
        if self.__btn_browser_device_file.exists:
            home_folder_path = self.ftpsetting.remote_dir.strip()
        if cmp(self.__get_current_path(), home_folder_path) != 0:
            print "[INFO] Back to home folder"
        while (cmp(self.__get_current_path(), home_folder_path) != 0):
            self.__go_upfolder.click()
            time.sleep(1)

    def download(self, filepath, timeout=30*60):
        """ Download file from FTP server to device
        please call connect_to before this function
        @param filepath: the path of the file/folder you want to download, 
        It's a relative path to FTP remote dir
        @param timeout: (seconds) timeout of download file, default value is 30 mins 
        """
        print "[INFO] Download file:%s from FTP Server to device" % filepath
        # if current view is not browser ftp server file, switch to browser ftp server file
        if self.__btn_upload.exists:
            self.__btn_browser_ftp_file.click()
        # click Refresh button
        self.__btn_more_options.click()
        time.sleep(2)
        self.__btn_refresh.click()
        parent_path, itemname = os.path.split(filepath)
        self.__open_folder(parent_path)
        self.__select_browser_item(itemname)
        self.__btn_download.click()
        self.__msg_download_prompt.wait.exists(timeout=10*1000)
        assert self.__msg_download_prompt.exists
        self.__btn_ok.click()
        self.__wait_download_upload_complete(timeout, True)
        assert self.__msg_download_complete.exists, "[ERROR] Download not complete or fail"
        self.__btn_ok.click()
        print "[INFO] Download %s to device success" % filepath

    def upload(self, filepath, timeout=30*60):
        """ Upload file from device to FTP server
        please call connect_to before this function
        @param filepath: (string) the path of the file/folder you want to upload, 
        It's a relative path to FTP local dir
        @param timeout: (int) seconds timeout of upload file, default value is 30 mins
        """
        print "[INFO] Upload from device to FTP Server"
        # if current view is not browser device file, switch to browser device file
        if self.__btn_download.exists:
            self.__btn_browser_device_file.click()
        # click Refresh button
        self.__btn_more_options.click()
        time.sleep(2)
        self.__btn_refresh.click()
        parent_path, itemname = os.path.split(filepath)
        self.__open_folder(parent_path)
        self.__select_browser_item(itemname)
        self.__btn_upload.click()
        self.__msg_upload_prompt.wait.exists(timeout=10*1000)
        assert self.__msg_upload_prompt.exists
        self.__btn_ok.click()
        self.__wait_download_upload_complete(timeout, False)
        assert self.__msg_upload_complete.exists, "[ERROR] Upload not complete or fail"
        self.__btn_ok.click()
        print "[INFO] Upload %s to FTP Server success" % filepath

    def sync_download(self, filepath):
        """ Download file from FTP server to device, but wait download complete sync
        please call connect_to before this function
        @param filepath: the path of the file/folder you want to download, 
        It's a relative path to FTP remote dir
        """
        print "[INFO] Sync download file from FTP Server to device"
        # if current view is not browser ftp server file, switch to browser ftp server file
        if self.__btn_upload.exists:
            self.__btn_browser_ftp_file.click()
        # click Refresh button
        self.__btn_more_options.click()
        time.sleep(2)
        self.__btn_refresh.click()
        parent_path, itemname = os.path.split(filepath)
        self.__open_folder(parent_path)
        self.__select_browser_item(itemname)
        self.__btn_download.click()
        self.__msg_download_prompt.wait.exists(timeout=10*1000)
        assert self.__msg_download_prompt.exists
        self.__btn_ok.click()

    def sync_upload(self, filepath):
        """ Upload file from device to FTP serve, but wait upload complete sync
         please call connect_to before this function
        @param filepath: the path of the file/folder you want to download, 
        It's a relative path to FTP local dir
        """
        if self.__btn_download.exists:
            self.__btn_browser_device_file.click()
        # click Refresh button
        self.__btn_more_options.click()
        time.sleep(2)
        self.__btn_refresh.click()
        parent_path, itemname = os.path.split(filepath)
        self.__open_folder(parent_path)
        self.__select_browser_item(itemname)
        self.__btn_upload.click()
        self.__msg_upload_prompt.wait.exists(timeout=10*1000)
        assert self.__msg_upload_prompt.exists
        self.__btn_ok.click()

    def sync_wait_upload_complete(self, filepath, _timeout=30*60):
        """ Sync wait upload complete prompt appear
        @param filepath: the path of the file/folder you want to upload,
        @param _timeout: (int) seconds timeout of upload file, default value is 30 mins
        """
        self.__wait_download_upload_complete(_timeout, False)
        assert self.__msg_upload_prompt.exists, \
            "[ERROR] Upload not complete or fail"
        self.__btn_ok.click()
        print "[INFO] Upload %s to FTP server success" % filepath

    def sync_wait_download_complete(self, filepath, _timeout=30*60):
        """ Sync wait download complete prompt appear
        @param filepath: the path of the file/folder you want to download,
        @param _timeout: (int) seconds timeout of download file, default value is 30 mins
        """
        self.__wait_download_upload_complete(_timeout, True)
        assert self.__msg_download_complete.exists, \
            "[ERROR] Download not complete or fail"
        self.__btn_ok.click()
        print "[INFO] Download %s to device success" % filepath

    def download_from_browser(self, downfile):
        """ Download file on FTP server from browser """
        print "[INFO] Download file on FTP server from browser"
        download_url =  self.get_download_url(downfile)
        browsingImpl = BrowsingImpl(self.d)
        browsingImpl.launch_by_am()
        browsingImpl.browser_openwebsite(download_url)

    def __check_download_upload_complete_in_andftp(self, isDownload):
        """ Check Download complete prompt appear in AndFT
        @param isDownload: True = check download complete, False = check upload complete
        @return True = 'Download complete' appear, False = 'Download complete' not appear
        """
        if isDownload:
            return self.__msg_download_complete.exists
        else:
            return self.__msg_upload_complete.exists

    def __wait_download_upload_complete(self, timeout, isDownload):
        """ Long time wait download/upload complete
        Due to UIAutomator python wrapper wait.exists and wait.gong not support timeout > 90s
        So add this method as a workaround
        @param timeout: seconds of wait timeout
        @param isDownload: True = wait download complete, False = wait upload complete
        """
        print "[INFO] %s" %(isDownload and "Downloading ..." or "Uploading ...")
        _timeout = timeout
        while timeout > 0:
            if self.__check_download_upload_complete_in_andftp(isDownload):
                print "[INFO] %s complete" %(isDownload and "Download" or "Upload")
                return
            time.sleep(10)
            timeout -= 10
        print "[ERROR] %s not complete within %d seconds" %(isDownload and "Download" or "Upload", _timeout)

    def del_device_file(self, filepath):
        """ Delete file/folder on device
        @param filepath: (string) a relative path to FTP local dir
        """
        if self.ftpsetting is None:
            self.ftpsetting = cfg.ftp_setting
        fpath = os.path.join(self.ftpsetting.local_dir, filepath)
        print "[INFO] Delete device file: %s" % fpath
        # delete file
        cmd = 'rm -rf "%s"; echo $?' % fpath
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        ret = int(res_list[-1])
        assert ret == 0, "[ERROR] Delete fail"
        # check file not exists
        cmd = 'ls "%s"; echo $?' % fpath
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        ret = int(res_list[-1])
        assert ret == 0  and 'No such file or directory' in ''.join(res_list), \
            "[ERROR] Delete fail"
        print "[INFO] Delete success"

    def create_device_file(self, fsize, fname):
        """ Create file on device under FTP local dir with specified size
        @param fsize:(int) bytes of file size
        @param fname:(string) file name
        """
        if self.ftpsetting is None:
            self.ftpsetting = cfg.ftp_setting
        fpath = os.path.join(self.ftpsetting.local_dir, fname)
        print "[INFO] Create file on device: %s" % fpath
        cmd = "dd if=/dev/null of=%s bs=1 seek=%d; echo $?" % (fpath, fsize)
        res_list = g_common_obj.adb_cmd_capture_msg(cmd)
        ret = int(res_list[-1])
        assert ret == 0, "[ERROR] Create fail"
        print "[INFO] Create success"

    def __push_busybox(self):
        """ Push busybox to device: /data/local/tmp """
        base_path = os.path.split(os.path.realpath(__file__))[0]
        src_path = os.path.join(base_path, "busybox")
        dst_path = os.path.join("/data/local/tmp", "busybox")

        #check busybox exist on device and has execute permission
        cmd = '%s; echo $?' % dst_path
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        ret = int(res_list[-1])
        if ret == 0 and "not found" not in ''.join(res_list) and \
            "Permission denied" not in ''.join(res_list):
            print "[INFO] busybox exist on device"
            return
        cmd = 'adb push %s %s; echo $?' % (src_path, dst_path)
        deviceid = g_common_obj.get_test_device().serial
        if deviceid:
            cmd = 'adb -s %s push %s %s; echo $?' % (deviceid, src_path, dst_path)
        res_list = os.popen(cmd).readlines()
        ret = int(res_list[-1])
        assert ret == 0, "[ERROR] Push busybox to device fail"
        print "[INFO] Push busybox success"
