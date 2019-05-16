#!/usr/bin/env python
#-*- encoding: utf-8 -*-

import os
import time
from nose.tools import assert_equals
from testlib.util.common import g_common_obj


class ContactImpl:
    """
    Implements Contact Setting UI actions.

    """
    def __init__(self):
        self.d = g_common_obj.get_device()

    def launch_contact(self):
        '''
        Launch People.
        '''
        print "[Info] ---Launch People."
        g_common_obj.launch_app_am("com.android.contacts", ".activities.PeopleActivity")
        for i in range(10):
            if self.d(text="Contacts").exists:
                break
            time.sleep(1)
        assert_equals(self.d(text="Contacts").exists, True)

    def go_to_all_contacts(self):
        '''
        After launch People,choose All contacts.
        '''
        print "[Info] ---Choose All contact."
        self.d(text="All contacts",className="android.widget.TextView").click.wait()
        assert_equals(self.d(text="Me").exists,True)

    def display_all_contacts(self):
        '''
        Display all contacts.
        '''
        print "[Info] ---Display all contact"
        self.d.press.menu()
        self.d(text="Contacts to display").click.wait()
        self.d(text="All contacts").click.wait()
        assert_equals(self.d(text="All contacts", resourceId="com.android.contacts:id/accountType").exists, False)

    def set_sync(self, status=True):
        '''
        Set sync ON or OFF.
        1.status: value is True or False. True to set sync status ON, False to set sync OFF.
        '''
        print "[Info] ---Set sync %s." % status
        self.d.press.menu()
        self.d(text="Accounts").click.wait()
        self.d.press.menu()
        if self.d(resourceId="android:id/checkbox").checked != status:
            self.d(resourceId="android:id/checkbox").click.wait()
            self.d(text="OK").click.wait()
        else:
            self.d.press.back()
        self.d.press.back()

    def add_contact(self):
        '''
        Click button of add to add contact.
        '''
        print "[Info] ---Click button of add to edit contact."
        while self.d(description="add new contact").exists:
            self.d(description="add new contact").click.wait()
        if self.d(text="OK").exists:
            self.d(text="OK").click.wait()
        if self.d(text="Google").exists:
            self.d(text="Google").click.wait()
        assert_equals(self.d(text="Add new contact", resourceId="com.android.contacts:id/title").exists, True)

    def add_name(self,contact):
        '''
        Add name.
        1. contact: The contact to be added.
        '''
        print '[Info] ---Add contact name: %s.' % contact.name
        name = contact.name.split(' ')
        if 1== len(name):
            first_name = name[0]
            last_name = None
        elif 2 == len(name):
            first_name = name[0]
            last_name = name[1]
        if first_name != None:
            self.d(text="Name").set_text(first_name)
            assert_equals(self.d(text=first_name, className="android.widget.EditText").exists, True)
            self.d(text=first_name, className="android.widget.EditText").click.wait()
        if last_name != None:
            self.d(description="Expand or collapse name fields").click.wait()
            time.sleep(1)
            self.d(text="Last name").set_text(last_name)
            assert_equals(self.d(text=last_name, className="android.widget.EditText").exists, True)
        if self.d(text="Name prefix",className="android.widget.EditText").exists:
            self.d(description="Expand or collapse name fields").click.wait()

    def finish_add_edit_contact(self,contact):
        '''
        After add or edit contact ,save it.
        1. contact: The contact to be viewed.
        '''
        print "[Info] ---Save contact info after adding or editing contact."
        self.d(description="Done").click.wait()
        time.sleep(2)
        assert_equals(self.d(text=contact.name, resourceId="com.android.contacts:id/large_title").exists, True)
        self.d.press.back()

    def delete_contact(self, contactName):
        '''
        Delete a contact in app People.
        1. contactName: The contact name to be deleted.
        '''
        print "[Info] ---Delete a contact in app People."
        self.d(resourceId="com.android.contacts:id/menu_search").click.wait()
        self.d(resourceId="android:id/search_src_text").set_text(contactName)
        while self.d(text=contactName, resourceId="com.android.contacts:id/cliv_name_textview").exists:
            self.d(text=contactName, resourceId="com.android.contacts:id/cliv_name_textview").click()
            self.d.press.menu()
            self.d(text="Delete").click.wait()
            self.d(text="OK").click.wait()
        assert_equals(self.d(text=contactName, resourceId="com.android.contacts:id/cliv_name_textview").exists, False)
        self.d(resourceId="android:id/home").click.wait()

    def delete_contact_by_key_word(self, key_word):
        '''
        Delete contact whose name contains certain word in app People.
        1. key_word: The contact name to be deleted.
        '''
        print "[Info] ---Delete contact by key word in app People."
        self.d(resourceId="com.android.contacts:id/menu_search").click.wait()
        self.d(resourceId="com.android.contacts:id/search_view").set_text(key_word)
        while self.d(textStartsWith=key_word, resourceId="com.android.contacts:id/cliv_name_textview").exists:
            self.d(textStartsWith=key_word, resourceId="com.android.contacts:id/cliv_name_textview").click()
            self.d(description="Edit").click.wait()
            self.d.press.menu()
            self.d(text="Delete").click.wait()
            self.d(text="OK").click.wait()
        assert_equals(self.d(textStartsWith=key_word, resourceId="com.android.contacts:id/cliv_name_textview").exists, False)
        self.d(description="stop searching").click.wait()

    def delete_multi_contacts(self, num):
        '''
        Delete multi contact in app People.
        1. num: The num of contacts to be deleted.
        '''
        print "[Info] ---Delete multi contacts in app People."
        i = 0
        while i<num:
            for j in range(5):
                if self.d(resourceId="com.android.contacts:id/cliv_name_textview").exists:
                    break
                time.sleep(1)
            if self.d(resourceId="com.android.contacts:id/cliv_name_textview").exists:
                self.d(resourceId="com.android.contacts:id/cliv_name_textview").click.wait()
                self.d(description="Edit").click.wait()
                self.d.press.menu()
                self.d(text="Delete").click.wait()
                self.d(text="OK").click.wait()
                i+=1
                print "[Info] --- Deleted contacts num: %s." % i
            else:
                break
        assert_equals(i, num)

    def make_vcf_file(self, num_per_file = 1, num_of_files = 1):
        '''
        Make vcf files.
        1. num_per_file: The num of contacts in ech file.
        2. num_of_files: The num of vcf files.
        '''
        print "[Info] ---Make vcf file."
        from testlib.common.base import getTmpDir
        tmp_path = getTmpDir()
        vcf_file = tmp_path + os.sep + "test.vcf"
        if os.path.isfile(vcf_file):
            os.remove(vcf_file)
        file_obj = open(vcf_file, 'w')
        for i in range(num_per_file):
            vcf_contact = "BEGIN:VCARD\nVERSION:2.1\nN:;contact%03d;;;\nFN:contact%03d\nEND:VCARD\n" % (i, i)
            file_obj.write(vcf_contact)
        file_obj.close()
        assert os.path.isfile(vcf_file)
        os.system("adb shell rm /sdcard/*.vcf >/dev/null 2>&1")
        push_cmd = "adb push %s /sdcard/ >/dev/null 2>&1" % vcf_file
        os.system(push_cmd)
        os.remove(vcf_file)
        for i in range(num_of_files):
            copy_cmd = "adb shell cp /sdcard/test.vcf /sdcard/test%04d.vcf" % (i + 1)
            os.system(copy_cmd)
        os.system("adb shell rm /sdcard/test.vcf >/dev/null 2>&1")
        os.system("rm -rf %s >/dev/null 2>&1" % tmp_path)

    def import_vcf_file(self, choose_file = "one"):
        '''
        Import vcf file.
        1. num: The num of contacts.
        '''
        print "[Info] ---Import vcf file."
        self.d.open.notification()
        time.sleep(1)
        if self.d(description="Clear all notifications.").exists:
            self.d(description="Clear all notifications.").click.wait()
        else:
            self.d.press.back()
        self.d.press.menu()
        self.d(text="Import/export").click.wait()
        self.d(text="Import from storage").click.wait()
        if self.d(text="Google").exists:
            self.d(text="Google").click.wait()
        if self.d(text="Choose vCard file").exists:
            if choose_file == "one":
                self.d(text="Import one vCard file").click.wait()
                self.d(text="OK").click.wait()
                assert_equals(self.d(textStartsWith="test0001.vcf").exists, True)
                self.d(textStartsWith="test0001.vcf").click.wait()
                self.d(text="OK").click.wait()
            else:
                self.d(text="Import all vCard files").click.wait()
                self.d(text="OK").click.wait()
        for i in range(30):
            if self.d(text="Contacts").exists:
                break
            time.sleep(1)
        self.d.open.notification()
        time.sleep(1)
        for i in range(30):
            if self.d(textStartsWith="Finished importing vCard").exists:
                break
            time.sleep(1)
        assert_equals(self.d(textStartsWith="Finished importing vCard").exists, True)
        self.d.press.back()

    def export_vcf_file(self):
        '''
        Export vcf file.
        '''
        print "[Info] ---Export vcf file."
        os.system("adb shell rm /sdcard/00001.vcf >/dev/null 2>&1")
        self.d.open.notification()
        time.sleep(1)
        if self.d(description="Clear all notifications.").exists:
            self.d(description="Clear all notifications.").click.wait()
        else:
            self.d.press.back()
        self.d.press.menu()
        self.d(text="Import/export").click.wait()
        self.d(text="Export to storage").click.wait()
        self.d(text="OK").click.wait()
        self.d.open.notification()
        time.sleep(1)
        for i in range(30):
            if self.d(textStartsWith="Finished exporting").exists:
                break
            time.sleep(1)
        assert_equals(self.d(textStartsWith="Finished exporting").exists, True)
        self.d.press.back()

    def clear_contacts_app_data(self):
        '''
        clear the data of contacts app.
        '''
        print "[Info] ---Clear the data of contacts app."
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        time.sleep(2)
        self.d(text="Apps").click.wait()
        i = 0
        while not self.d(text = "All",clickable = False).exists and i<=1:
            self.d(className = "android.widget.FrameLayout").swipe.left()
            i = i + 1
        i = 0
        while (not self.d(text = "All",clickable = False).exists) and i<=1:
            self.d(className = "android.widget.FrameLayout").swipe.right()
            i = i + 1
        self.d(resourceId = "android:id/list").scroll.vert.to(text="Contacts Storage")
        self.d(text = "Contacts Storage").click.wait()
        self.d(text = "Clear data").click.wait()
        if self.d(text = "OK").exists:
            self.d(text = "OK").click.wait()
        self.d.press.back()


class Contact(object):

    def __init__(self, data):
        self._data = data

    def __getattr__(self, name):
        if self._data.has_key(name) is False:
            return None
        return self._data[name]


class getConfValue(object):

    @classmethod
    def getStrValue(cls, section, option):
        import ConfigParser
        configFile = os.path.dirname(__file__) + os.sep + "contact.conf"
        cf = ConfigParser.ConfigParser()
        cf.read(configFile)
        return cf.get(section, option)
