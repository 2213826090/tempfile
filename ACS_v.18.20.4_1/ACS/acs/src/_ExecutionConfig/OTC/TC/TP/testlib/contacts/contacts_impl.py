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

import time
import os
import random
from nose.tools import assert_equals
from testlib.util.common import g_common_obj



class ContactsImpl:
    """
    Implements Contact app actions.
    """

#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_select_account(self):
            """ UI button select account """
            return self.d(resourceId="com.android.contacts:id/account_list")

        @property
        def btn_account(self):
            """ UI button select account """
            return self.d(\
                className="android.widget.LinearLayout", index="0")

        @property
        def btn_account_sync(self):
            """ UI button account sync """
            return self.d(textContains="Your new contact will be synchronized")

        @property
        def btn_ok(self):
            """ UI button ok """
            return self.d(textContains="OK")

        def btn_contact_view(self, name):
            """ UI button contact """
            return self.d(text=name, \
                resourceId="com.android.contacts:id/cliv_name_textview")


        def btn_contact_duplicate(self, name):
            """ UI button contact """
            return self.d(text=name, \
                resourceId="com.android.contacts:id/cliv_name_textview", \
                instance="1")

        @property
        def contact_list(self):
            """ UI contact list at left side """
            return self.d(resourceId="android:id/list")

        def contact(self, name):
            """ UI one contact of contact list """
            try:
                if self.contact_list.child_by_text(name, \
                    allow_scroll_search=True, \
                    resourceId="com.android.contacts:id/cliv_name_textview")\
                .exists:
                    return self.d(text=name, \
                        resourceId="com.android.contacts:id/cliv_name_textview")
            # if can't find the UI object
            except:
                return None

        @property
        def btn_contact_menu(self):
            """ UI button menu """
            return self.d(resourceId="com.android.contacts:id/menu_edit")

        @property
        def btn_contact_option(self):
            """ UI button account """
            return self.d(description="More options")

        @property
        def btn_contact_delete(self):
            """ UI button account """
            return self.d(resourceId="android:id/title", text="Delete")

        @property
        def btn_contact_name(self):
            """ UI button account """
            return self.d(resourceId="com.android.contacts:id/editors")

        @property
        def btn_contact_add(self):
            """ UI button account """
            return self.d(resourceId="com.android.contacts:id/floating_action_button")

        @property
        def btn_contact_firstname(self):
            """ UI button account """
            return self.d(className="android.widget.EditText", \
                text="Name")

        @property
        def btn_account_done(self):
            """ UI button select account """
            return self.d(resourceId="com.android.contacts:id/save_menu_item")

        @property
        def btn_account_done2(self):
            return self.d(description="Navigate up")

    contact_pkg_name = "com.android.contacts"
    contact_activity_name = ".activities.PeopleActivity"
    vcf_file_name = ""

    def __init__ (self, wifi_cf={}):
        self.d = g_common_obj.get_device()
        self._locator = ContactsImpl.Locator(self.d)

    # launch contact app by am
    def launch_by_am(self):
        g_common_obj.launch_app_am(ContactsImpl.contact_pkg_name, \
            ContactsImpl.contact_activity_name)
        print "[INFO] launch contact app successfully"


    # jump add account screen, if it appears in the first launching contact app
    def jump_add_account_screen(self):
        time.sleep(2)
        if self.d(text="Not now").exists:
            self.d(text="Not now").click()
            print "[INFO] jump add add account screen"
        time.sleep(3)


    # add a new contact in the contact app
    def add_a_new_contact(self, contactName, account=None):
        self.d(description="add new contact").click()
        if self.d(text="You can synchronize your new contact with one of the following accounts. Which do you want to use?").exists:
            if account!=None:
                print "[INFO]: select account %s" % account
                self.d(text=account).click()
        if self.d(text="Keep local").exists:
            self.d(text="Keep local").click()
        if self.d(text="OK").exists:
            self.d(text="OK").click()
        self.d(text="Name").set_text(contactName)
        if self.d(description="Done").exists:
            self.d(description="Done").click()
        else:
            self.d(description="Navigate up").click()
        time.sleep(3)
        self.d.press.back()
        print "[INFO] add new contact %s successfully" % contactName

    # export contact list to a vcf file in internal storage
    def export_contact_list(self):
        self.d(description="More options").click()
        self.d(text="Import/export").click()
        self.d(text="Export to storage").click()

        # get the export vcf file name
        self.vcf_file_name = self.d(resourceId="android:id/message").info["text"].split(": ")[1].replace("vcf.", "vcf").split("/")[-1]
        print "[INFO] exported vcf file name is %s" % self.vcf_file_name
        self.d(text="OK").click()
        time.sleep(5)
        print "[INFO] export contact list successfully"
        return self.vcf_file_name
    # delete all the contacts in the contact app
    def delete_all_contacts(self):
        while self.d(resourceId="android:id/list").child(className="android.view.View").exists:
            self.d(resourceId="android:id/list").child(className="android.view.View").child(className="android.widget.TextView").click()
            time.sleep(2)
            self.d(description="Edit").click()
            time.sleep(3)
            if self.d(resourceId="com.android.contacts:id/menu_delete").exists:
                self.d(resourceId="com.android.contacts:id/menu_delete").click()
                time.sleep(2)
                if self.d(text="OK").exists:
                    self.d(text="OK").click()
            else:
                self.d(description="More options").click()
                self.d(text="Delete").click()
                self.d(text="OK").click()
            time.sleep(1)
        print "[INFO] delete all the contacts successfully"

    #check if contact list is empty
    def contact_list_is_empty(self):
        if self.d(resourceId="android:id/list").child(className="android.view.View").exists:
            return True
        else:
            return False
    # import the contact list from the vcf file in the internal storage (which one is just exported)
    def import_contact_list(self, contactfile, account=None):
        if self.d(text="Import contacts").exists:
            self.d(text="Import contacts").click()
        if self.d(description="More options").exists:
            self.d(description="More options").click()
        if self.d(text="Import/export").exists:
            self.d(text="Import/export").click()
        #self.d(text="Import contacts").click()
        if self.d(text="Import from storage").exists:
            self.d(text="Import from storage").click()
        time.sleep(2)
        if self.d(text="Create contact under account").exists:
            if account!=None:
                print "[INFO]: Select account %s" % account
                self.d(text=account).click()
        if self.d(text="OK").exists:
            self.d(text="OK").click()
        if self.d(text="OK").exists:
            self.d(textContains=contactfile).click()
        if self.d(text="OK").exists:
            self.d(text="OK").click()
        time.sleep(2)
        print "[INFO] contact list %s is imported...." % contactfile

    def import_contact_list_successfully(self, contactName):
        if self.d(text=contactName).count > 0:
            print "[INFO] contact list is imported successfully"
        else:
            assert "[INFO] contact list is imported failed"

    # delete a contact
    def delete_a_contact(self, contactName):
        added_index = self.d(text=contactName).count - 1
        self.d(text=contactName, instance=added_index).click()
        time.sleep(2)
        self.d.click(200, 200)
        time.sleep(2)
        self.d(description="Edit").click()
        if self.d(description="Delete").exists:
            self.d(description="Delete").click()
        else:
            self.d(description="More options").click()
            self.d(text="Delete").click()
        self.d(text="OK").click()
        time.sleep(2)
        print "[INFO] delete the added contact %s successfully" % contactName

    # delete the vcf contact file
    def delete_the_exported_vcf_file(self, contactfile):
        g_common_obj.adb_cmd("rm /sdcard/%s" % contactfile)
        print "[INFO] exprted vcf file %s is deleted successfully" % contactfile

    def import_contact_from_local_vcf_file(self, base_path, vcf_file_name):
        """ Import contacts into device from vcf """
        print "[INFO] Pre-load contacts from vcf"
        vcf_file_path = os.path.join(base_path, vcf_file_name)
        cmd = 'adb push %s /sdcard/%s; echo $?' % (vcf_file_path, vcf_file_name)
        print "%s" % cmd
        res_list = os.popen(cmd).readlines()
        ret = int(res_list[-1].strip())
        assert ret == 0, "Push %s to device fail" % vcf_file_path
        # clear logcat
        print """ Clear logcat """
        cmd = 'logcat -c'
        g_common_obj.adb_cmd_capture_msg(cmd)

        #import_cmd = 'am start -t text/x-vcard -d file:///sdcard/%s -a android.intent.action.VIEW com.android.contacts; echo $?' % vcf_file_name
        #res_list = g_common_obj.adb_cmd_capture_msg(import_cmd).split('\r\n')
        #ret = int(res_list[-1].strip())
        #assert ret == 0, "Import contacts fail"

    def import_contact_from_local_vcf_file_successfully(self, timeout=0):
        """ Wait import contact complete """
        print "[INFO] Wait import complete"
        cmd = 'logcat -d | grep "Successfully finished importing one vCard file"; echo $?'
        if timeout == 0:
            timeout = 60
        starttime = time.time()
        while timeout > 0:
            reslist = g_common_obj.adb_cmd_capture_msg(cmd)
            ret = int(reslist[-1].strip())
            if ret == 0:
                break
            time.sleep(3)
            timeout -= 3
        endtime = time.time()
        print "[INFO] Import elapse %.3fs" % (endtime-starttime)
        time.sleep(2)
        print "[INFO] Import contacts from vcf successfully"

    def contact_add(self, contact):
        """
        @summary: add contact
        """

        self.launch_by_am()
        if self._locator.contact(contact) != None:
            print "[INFO] The contact added is existed.Skip this"
            time.sleep(2)
            while self._locator.btn_contact_duplicate(contact).exists:
                self._locator.btn_contact_duplicate(contact).click()
                self._locator.btn_contact_menu.click()
                self._locator.btn_contact_option.click()
                self._locator.btn_contact_delete.click()
                self._locator.btn_ok.click()
            return True
        count = 0
        while not self._locator.btn_contact_name.exists:
            if count >= 5:
                break
            if self._locator.btn_account_sync.exists:
                self._locator.btn_ok.click()
            if self._locator.btn_select_account.exists:
                self._locator.btn_account.click()
            if self._locator.btn_contact_add.exists:
                self._locator.btn_contact_add.click()
            count += 1
        assert self._locator.btn_contact_firstname.exists, \
        "ERROR: Add contact error!"
        self._locator.btn_contact_firstname.clear_text()
        self._locator.btn_contact_firstname.set_text(contact)
        time.sleep(1)
        if self._locator.btn_account_done.exists:
            self._locator.btn_account_done.click()
        elif self._locator.btn_account_done2.exists:
            self._locator.btn_account_done2.click()
        else:
            assert False, "failed to find save account button!"
