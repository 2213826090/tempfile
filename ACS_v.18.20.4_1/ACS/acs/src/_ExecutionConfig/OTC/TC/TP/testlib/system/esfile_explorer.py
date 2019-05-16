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
@summary: ES File Explorer UI Operation Class
@since: 06/06/2014
@author: yongzoux (yongx.zou@intel.com)
"""
import os
import time
from nose.tools import assert_equals

from testlib.util.common import g_common_obj

class ESButton(object):
    New = "New"
    Copy = "Copy"
    Cut = "Cut"
    Delete = "Delete"
    Paste = "Paste"
    Rename = "Rename"
    More = "More"
    Exit = "Exit"
    Cancel = "Cancel"
    Retry = "Retry"

class ESNavigateMenu(object):
    """
        ES Navigate Menu
    """
    Local = "Local"
    Home = "Home"
    Device = "Device"

class ESFileExplorer(object):
    '''
    ES File Explorer UI Operation Class
    '''

#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_gridview(self):
            """ UI button ok """
            return self.d(\
                resourceId="com.estrongs.android.pop:id/gridview", \
                scrollable="true")

        @property
        def btn_fast_access(self):
            """ UI button fast access """
            return self.d(text="Fast Access", \
                resourceId="com.estrongs.android.pop:id/title")

        @property
        def btn_fast_access_tool(self):
            """ UI button fast access tool """
            return self.d(\
                resourceId="com.estrongs.android.pop:id/tool_fast_access")

        @property
        def btn_copy_fail(self):
            """ UI button copy fail"""
            return self.d(text="Copy Task failed", \
                resourceId="com.estrongs.android.pop:id/title_common_dialog")

        @property
        def btn_copying(self):
            """ UI button copying"""
            return self.d(text="Copying")

        def build_button(self, btn_name):
            """ return ES File Explorer Buttion UI object """
            return self.d(text=btn_name, className="android.widget.Button")

        def build_folder_file(self, folder_file_name):
            """
            return folder or file UI object under a folder
            in ES File Explorer
            """
            return self.d(text=folder_file_name, \
                resourceId="com.estrongs.android.pop:id/message")

        def build_menu(self, menu):
            """ return menu UI object at left panel in ES File Explorer """
            return self.d(text=menu, \
                resourceId="com.estrongs.android.pop:id/label")

    pkg_name = "com.estrongs.android.pop"
    activity_name = ".view.FileExplorerActivity"
    app_name = "ES File Explorer"

    def __init__ (self):
        self.d = g_common_obj.get_device()
        self._locator = ESFileExplorer.Locator(self.d)

    @staticmethod
    def launch_by_am():
        """ launch ES File Explorer by adb am command """
        g_common_obj.launch_app_am(\
            ESFileExplorer.pkg_name, ESFileExplorer.activity_name)
        time.sleep(6)

    @staticmethod
    def launch_from_app_gallery():
        """ launch ES File Explorer from UI """
        g_common_obj.launch_app_from_home_sc(\
            ESFileExplorer.app_name, "Apps", "New")
        time.sleep(6)

    def check_item_exists_in_folder(self, item):
        """ check if item exists in folder """
        iffind = False
        for _ in range(5):
            if self.d(text=item).exists:
                iffind = True
                break
            if self._locator.btn_gridview.exists:
                iffind = True
                self._locator.btn_gridview.scroll.to(text=item)
                break
            time.sleep(2)
        return iffind

    def open_folder(self, folder_path):
        """ open the folder in ES """
        if not folder_path.startswith("/"):
            print "The folder path not start with /"
            return False
        _parts = folder_path.split(os.sep)

        # if left navigate panel not expand
        if not self._locator.btn_fast_access.exists:
            self._locator.btn_fast_access_tool.click()
        # if menu Local not expand
        if not self._locator.build_menu(ESNavigateMenu.Device).exists:
            self._locator.build_menu(ESNavigateMenu.Local).click()
        # click "/ Device" under Local
        self._locator.build_menu(ESNavigateMenu.Device).click()
        # iterative open each layer in folder_path
        opened_folders = []
        for folder in _parts:
            if folder.strip():
                self._locator.build_folder_file(folder).click()
                opened_folders.append(folder)
                print "Open folder success: %s " % "/".join(opened_folders)

    def copy(self, src_folder, copyone, dest_folder):
        """ copy file or folder to target folder """
        print "Start copy %s from %s to %s" % (copyone, src_folder, dest_folder)
        if src_folder.endswith(os.sep):
            src_folder = src_folder[:-1]
        # open parent folder
        self.open_folder(src_folder)
        # select the folder/file copying
        self.select_item_in_folder(copyone)
        # click Copy button
        self._locator.build_button(ESButton.Copy).click()
        # open target folder
        if dest_folder.endswith(os.sep):
            dest_folder = dest_folder[:-1]
        self.open_folder(dest_folder)
        # click Paste button
        self._locator.build_button(ESButton.Paste).click()
        # check copy success
        self.check_copy_success(src_folder, copyone, dest_folder)
        print "Copy %s from %s to %s success" % \
        (copyone, src_folder, dest_folder)

    def select_item_in_folder(self, name):
        """ long click on item to let itself become selected """
        # Do not use click.wait() or long_click()
        #they can not archive the effect you wanted
        # below drag.to(itself) is a workaround
        print "Select item: %s" % name
        self._locator.build_folder_file(name).drag.to(text=name)
        time.sleep(10)

    def check_copy_success(self, src_folder, copyone, dest_folder, timeout=0):
        """ check copy success or not """
        print "Checking copy success or not"
        if timeout == 0:
            timeout = 30
        # wait 'Copying' prompt disappear
        self._locator.btn_copying.wait("gone", 10*60*1000)
        # check if 'Copy Task failed' prompt appear
        msg_fail = "Copy %s from %s to %s fail" % \
        (copyone, src_folder, dest_folder)
        while(timeout > 0):
            time.sleep(1)
            copyfail = self.copy_task_failed_appear()
            assert_equals(copyfail, False, msg_fail)
            timeout -= 1
        # check if copy one under target folder
        copyone_exist = self._locator.build_folder_file(copyone).exists
        assert_equals(copyone_exist, True, msg_fail)
        # compare the files name under the two folder
        #to make sure all files copied
        scr_path = os.path.join(src_folder, copyone)
        dest_path = os.path.join(dest_folder, copyone)
        names_of_src_folder = set(self.\
            get_all_itemnames_of_folder_on_device(scr_path))
        names_of_dest_folder = set(self.\
            get_all_itemnames_of_folder_on_device(dest_path))
        all_copy = names_of_src_folder.issubset(names_of_dest_folder)
        msg_fail = "Not all files copied"
        assert_equals(all_copy, True, msg_fail)

    def get_all_itemnames_of_folder_on_device(self, folder_path):
        """
        get all items name of one folder on
        device, not include items of sub folder
        """
        cmd = "ls %s; echo $?" % folder_path
        result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        msg_fail = "Get all items' name under folder: %s fail" % folder_path
        if len(result_list)>0:
            ret = result_list[-1].rstrip()
            del result_list[-1]
            assert_equals(int(ret), 0, msg_fail)
            #print "%s: %s" % (folder_path, result_list)
            print "Get all items name under folder: %s success" % folder_path
            return filter(lambda x: x.strip(), result_list)
        else:
            assert_equals(False, True, msg_fail)

    def copy_task_failed_appear(self, retrytimes=0):
        """ check 'Copy Task failed' prompt appear or not """
        appear = self._locator.btn_copy_fail.exists
        if appear:
            if retrytimes == 0:
                self._locator.build_button(ESButton.Cancel).click()
            else:
                for _ in range(0, retrytimes):
                    self._locator.build_button(ESButton.Retry).click()
                    time.sleep(2)
                    appear = self._locator.btn_copy_fail.exists
                    if not appear:
                        break
        return appear
