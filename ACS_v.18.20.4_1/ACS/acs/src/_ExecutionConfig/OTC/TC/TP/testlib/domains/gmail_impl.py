# coding: UTF-8
import os
import time
from testlib.util.common import g_common_obj

class GmailImpl:
    """
    Implements Gmail app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self.serial = self.d.server.adb.device_serial()

    def launch_gmail(self):
        '''
        Launch Gmail app.
        '''
        print "[Info] ---Launch Gmail app."
        g_common_obj.launch_app_am("com.google.android.gm", ".ConversationListActivityGmail")
        time.sleep(2)
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()
        if self.d(text="Take me to Gmail").exists:
            self.d(text="Take me to Gmail").click.wait()
        assert self.d(description="Open navigation drawer").exists

    def add_personal_account(self, account, passwd):
        '''
        Add personal account.
        '''
        while not self.d(resourceId="android:id/list").exists:
            self.d(description="Open navigation drawer").click.wait()
        if self.d(description="Show accounts").exists:
            self.d(description="Show accounts").click.wait()
        if not self.d(text=account).exists:
            print "[Info] ---Add personal account %s." % account
            self.d(text="Add account").click.wait()
            self.d(text="Personal (IMAP/POP)").click.wait()
            self.d(text="OK").click.wait()
            self.d(description="Email address").set_text(account)
            self.d(text="Next").click.wait()
            self.d(text="Personal (POP3)").click.wait()
            self.d(resourceId="com.google.android.gm:id/regular_password").set_text(passwd)
            self.d(text="Next").click.wait()
            self.d(resourceId="com.google.android.gm:id/account_server").clear_text()
            self.d(resourceId="com.google.android.gm:id/account_server").set_text("pop3.163.com")
            self.d(text="Next").click.wait()
            for i in range(20):
                time.sleep(3)
                if self.d(text="Outgoing server settings").exists:
                    break
            assert self.d(text="Outgoing server settings").exists
            self.d(resourceId="com.google.android.gm:id/account_server").clear_text()
            self.d(resourceId="com.google.android.gm:id/account_server").set_text("smtp.163.com")
            self.d(resourceId="com.google.android.gm:id/account_port").clear_text()
            self.d(resourceId="com.google.android.gm:id/account_port").set_text("25")
            self.d(text="Next").click.wait()
            for i in range(20):
                time.sleep(3)
                if self.d(text="Account options").exists:
                    break
            assert self.d(text="Account options").exists
            self.d(text="Next").click.wait()
            time.sleep(5)
            while self.d(text="Next").exists:
                self.d(text="Next").click.wait()
            assert self.d(text=account).exists

    def select_account(self, account):
        '''
        Select account.
        '''
        print "[Info] ---Select account %s." % account
        while not self.d(resourceId="android:id/list").exists:
            self.d(description="Open navigation drawer").click.wait()
        if self.d(description="Show accounts").exists:
            self.d(description="Show accounts").click.wait()
        assert self.d(text=account).exists
        if self.d(description="Switch to %s" % account).exists:
            self.d(description="Switch to %s" % account).click.wait()
        self.d(description="Hide accounts").click.wait()
        assert self.d(text=account).exists
        self.d(description="Close navigation drawer").click.wait()

    def delete_mail(self, subject):
        '''
        Delete mail.
        '''
        print "[Info] ---Delete mail."
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.vert.toBeginning()
        while self.d(descriptionContains=subject).exists:
            while not self.d(description="Delete").exists:
                self.d(descriptionContains=subject).long_click()
            self.d(description="Delete").click.wait()
        assert not self.d(descriptionContains=subject).exists

    def write_mail(self):
        '''
        Write a mail.
        '''
        print "[Info] ---Write a mail."
        self.d(description="Compose").click.wait()
        time.sleep(1)
        assert self.d(text="Compose").exists

    def add_receiver(self, addr_list):
        '''
        Add receiver.
        '''
        print "[Info] ---Add receiver."
        addrs = ",".join(addr_list) + ","
        self.d(resourceId="com.google.android.gm:id/to").set_text(addrs)

    def add_subject(self, subject):
        '''
        Add subject.
        '''
        print "[Info] ---Add subject."
        self.d(resourceId="com.google.android.gm:id/subject").set_text(subject)
        assert self.d(text=subject).exists

    def add_attachment(self, num):
        '''
        Add attachment.
        '''
        print "[Info] ---Add attachment."
        if num < 1:
            return
        os.system("adb -s %s shell rm /sdcard/attach_* > /dev/null 2>&1" % self.serial)
        os.system("adb -s %s shell rm /sdcard/Download/attach_* > /dev/null 2>&1" % self.serial)
        for i in range(num):
            os.system("adb -s %s shell 'echo 1234 > /sdcard/attach_%02d.txt'" % (self.serial, i))
        self.d(description="Attach file").click.wait()
        self.d(text="Attach file").click.wait()
        time.sleep(3)
        if self.d(text="Open from").exists:
            if not self.d(text="Internal storage").exists:
                self.d(text="Recent").click.wait()
                self.d.press.menu()
                self.d(text="Show SD card").click.wait()
                self.d(description="Show roots").click.wait()
            self.d(text="Internal storage").click.wait()
        if not self.d(text="Internal storage").exists:
            self.d(description="Show roots").click.wait()
            self.d(description="Show roots").click.wait()
            self.d(text="Internal storage").click.wait()
        self.d(scrollable=True).scroll.to(text="attach_00.txt")
        assert self.d(text="attach_00.txt").exists
        self.d(text="attach_00.txt").long_click()
        for i in range(1, num):
            self.d(scrollable=True).scroll.to(text="attach_%02d.txt" % i)
            self.d(text="attach_%02d.txt" % i).click.wait()
        self.d(description="More options").click.wait()
        time.sleep(2)
        assert self.d(resourceId="com.google.android.gm:id/attachment_name").exists

    def send_mail(self):
        '''
        Send mail.
        '''
        print "[Info] ---Send mail."
        self.d(description="Send").click.wait()
        if self.d(text="Send").exists:
            self.d(text="Send").click.wait()
        assert not self.d(description="Send").exists

    def receive_mail(self, subject):
        '''
        Receive mail.
        '''
        print "[Info] ---Receive mail."
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.vert.toBeginning()
        for i in range(20):
            if self.d(descriptionContains=subject).exists:
                return
            self.d.swipe(400, 300, 400, 800, 10)
            time.sleep(3)
        assert self.d(descriptionContains=subject).exists

    def open_mail(self, subject):
        '''
        Open mail.
        '''
        print "[Info] ---Open mail."
        self.d(descriptionContains=subject).click.wait()
        for i in range(10):
            if self.d(textStartsWith=subject).exists:
                break
            time.sleep(2)
        assert self.d(textStartsWith=subject).exists

    def view_and_save_attachment(self, num):
        '''
        View and save attachment.
        '''
        print "[Info] ---View and save attachment."
        self.d.server.adb.cmd("shell rm /sdcard/Download/attach_*")
        for i in range(num):
            if self.d(scrollable=True).exists:
                self.d(scrollable=True).scroll.to(text="attach_%02d.txt" % i)
            for j in range(20):
                if not self.d(resourceId="com.google.android.gm:id/attachment_progress").exists:
                    break
                time.sleep(3)
            self.d(text="attach_%02d.txt" % i).click.wait()
            time.sleep(1)
            assert self.d(text="HTMLViewer").exists
            self.d.press.back()
            time.sleep(1)
            self.d(text="attach_%02d.txt" % i).right(description="More options").click.wait()
            self.d(text="Save").click.wait()

    def switch_account_randomly(self):
        '''
        Switch account randomly.
        '''
        import random
        i = random.randint(1, 4)
        print "[Info] ---Switch account index %d." % i
        self.d(resourceId="com.google.android.gm:id/clickable_area", index=i).click.wait()

