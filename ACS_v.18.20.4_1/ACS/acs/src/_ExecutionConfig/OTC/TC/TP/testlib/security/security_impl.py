# -*- coding: utf-8 -*-
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.log import Logger
from security_common import SecurityCommon
import time
import os
import filecmp
import commands
import threading
import subprocess

d = g_common_obj.get_device()
dsn = d.server.adb.device_serial()
NOSERUNNER = "NOSERUNNER"

class SecurityImpl(SecurityCommon):

    def __init__(self):
        SecurityCommon.__init__(self)

    def get_config_value(self, tag, name):
        cfg = TestConfig().read(self.cfg_file, tag)
        return cfg.get(name)

    def get_state(self):
        state = g_common_obj.adb_cmd_common("get-state")
        print "[info]--- device state:", state
        if 'device' in state:
            return True
        else:
            return False

    def _execute_command_with_popen(self, cmd, t_shell=True):
        return subprocess.Popen(cmd, shell=t_shell, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)

    def _execute_command_with_call(self, cmd, t_shell=True):
        return subprocess.call(cmd, shell=t_shell, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)

    def action_screen_lock_security(self, action_lock, wait_time = 1):
        product_name = self.check_product()
        if action_lock == "Unlock":
            if "androidia" in product_name or "celadon" in product_name:
                self.screen_turn_off_option_AIA("Sleep")
            else:
                self.press_power_button()
            time.sleep(wait_time)
            self.press_power_button()
            time.sleep(10)
        elif action_lock == "Reboot":
            self.reboot_devices()
        else:
            return

    def set_screen_lock_type_all(self, lock_type, set_passwd=None, set_pin=None):
        if lock_type == "None":
            self.choose_screen_lock(lock_type)
        elif lock_type == "Swipe":
            self.choose_screen_lock(lock_type)
        elif lock_type == "Password":
            if self.set_screen_lock_password(set_passwd) is False:
                if self.check_set_screen_lock_success() is False:
                    if self.set_screen_lock_password(set_passwd) is False:
                        self.remove_screen_lock_password(set_passwd)
                        assert False, "[Debug]---Set screen lock password is Failed"
        elif lock_type == "PIN":
            if self.set_screen_lock_pin(set_pin) is False:
                if self.check_set_screen_lock_success() is False:
                    if self.set_screen_lock_pin(set_pin) is False:
                        self.remove_screen_lock_pin(set_pin)
                        assert False, "[Debug]---Set screen lock PIN is Failed"
        elif lock_type == "Pattern":
            if self.set_screen_lock_pattern() is False:
                if self.check_set_screen_lock_success() is False:
                    if self.set_screen_lock_pattern() is False:
                        self.remove_screen_lock_pattern()
                        assert False, "[Debug]---Set screen lock Pattern is Failed"
        time.sleep(3)

    def check_set_screen_lock_success(self):
        print "[Verify]------Verify Set up screen lock---"
        set_passwd_pin_reId = "com.android.settings:id/password_entry"
        set_pattern_reId = "com.android.settings:id/lockPattern"
        self.open_security_in_settings()
        d(text="Screen lock").click()
        time.sleep(2)
        if d(resourceId=set_passwd_pin_reId).exists or d(resourceId=set_pattern_reId).exists:
            print "[Verify]------Verify Set up screen lock < Success >"
            return True
        else:
            return False

    def minimium_requirement_set_screen_lock_password_pin(self, lock_type, set_passwd_pin=None):
        set_passwd_pin_reId = "com.android.settings:id/password_entry"
        set_pattern_reId = "com.android.settings:id/password_entry"
        self.choose_screen_lock(lock_type)
        time.sleep(2)
        print "[Info]------ Set screen lock %s : %s" % (lock_type, set_passwd_pin)
        if lock_type == "Password":
            d(resourceId="com.android.settings:id/password_entry").set_text(set_passwd_pin)
            d(resourceId="com.android.settings:id/next_button").click()
            assert d(textContains="Must be at least 4 characters").exists
        elif lock_type == "PIN":
            d(resourceId="com.android.settings:id/password_entry").set_text(set_passwd_pin)
            d(resourceId="com.android.settings:id/next_button").click()
            assert d(textContains="PIN must be at least 4 digits").exists
        elif lock_type == "Pattern":
            self.push_send_event_script("draw_pattern_lock.sh")
            self.draw_pattern_lock_incorrect(set_passwd_pin, 5)
            assert d(textContains="Connect at least 4 dots").exists
        time.sleep(2)

    def minimium_requirement_set_screen_lock_type(self, lock_type, set_passwd=None, set_pin=None):
        if lock_type == "Password":
            self.minimium_requirement_set_screen_lock_password_pin(lock_type, set_passwd)
        elif lock_type == "PIN":
            self.minimium_requirement_set_screen_lock_password_pin(lock_type, set_pin)
        elif lock_type == "Pattern":
            self.minimium_requirement_set_screen_lock_password_pin(lock_type, set_passwd)
        time.sleep(3)

    def check_unlock_no_passwd_pin_pattern(self):
        mos_ui_passwd = "com.android.systemui:id/passwordEntry"
        mos_ui_pin = "com.android.systemui:id/pinEntry"
        mos_ui_pattern = "com.android.systemui:id/lockPatternView"
        if d(resourceId=mos_ui_pin).exists or d(resourceId=mos_ui_passwd).exists or d(resourceId=mos_ui_pattern).exists:
            assert False, "[Debug]------No password test, passwd or pin or pattern exists"

    def unlock_screen_lock_type_all(self, lock_type, try_input_num, set_passwd=None, incorrect_passwd=None,
                                    set_pin=None, incorrect_pin=None):
        if lock_type == "None":
            self.check_home_ui_user_safely_O()
        elif lock_type == "Swipe":
            self.boot_up_completed_skip_boot_ui()
        elif lock_type == "Password":
            if self.check_screen_unlock_need_reboot_at_boot(lock_type, try_input_num, incorrect_passwd) is True:
                if d(textContains="START DRIVING").exists:
                    d(textContains="START DRIVING").click.wait()
                    self.unlock_screen_lock_passwd_pin_mos(try_input_num, set_passwd)
                if d(resourceId="com.android.settings:id/passwordEntry").exists:
                    self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
            else:
                print "[Retry]------ Unlock screen password"
                self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
                self.remove_screen_lock_password(set_passwd)
                assert False, "[Debug]---Unlock screen lock password is Failed"
        elif lock_type == "PIN":
            if self.check_screen_unlock_need_reboot_at_boot(lock_type, try_input_num, set_passwd, incorrect_pin) is True:
                if d(textContains="START DRIVING").exists:
                    d(textContains="START DRIVING").click.wait()
                    self.unlock_screen_lock_passwd_pin_mos(try_input_num, set_pin)
                if d(resourceId="com.android.settings:id/passwordEntry").exists:
                    self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
            else:
                print "[Retry]------ Unlock screen PIN"
                self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
                self.remove_screen_lock_pin(set_pin)
                assert False, "[Debug]---Unlock screen lock PIN is Failed"
        elif lock_type == "Pattern":
            if set_passwd == None:
                if self.unlock_screen_lock_pattern(try_input_num) is False:
                    print "[Retry]------ Unlock screen Pattern"
                    self.reboot_devices()
                    self.unlock_screen_lock_pattern(try_input_num)
                    self.remove_screen_lock_pattern()
                    assert False, "[Debug]---Unlock screen lock Pattern is Failed"
                else:
                    time.sleep(23)
                    print "[Completed]------ Unlock screen lock pattern completed"
            else:
                print "incorrect----------------"
                if self.unlock_screen_lock_incorrect_pattern(try_input_num) is True:
                    self.reboot_unlock_reusme_unlock_pattern()
                else:
                    print "[Retry]------ Unlock screen Pattern"
                    self.reboot_unlock_reusme_unlock_pattern()
                    self.remove_screen_lock_pattern()
                    assert False, "[Debug]---Unlock screen lock Pattern is Failed"

    def resume_unlock_screen_lock_type_all(self, lock_type, try_input_num, set_passwd=None, incorrect_passwd=None,
                                    set_pin=None, incorrect_pin=None, check_more_than_30_status=None):
        if lock_type == "Password":
            if self.check_resume_screen_unlock_need_unlock_type(lock_type, try_input_num, incorrect_passwd) is True:
                if d(resourceId="com.android.systemui:id/passwordEntry").exists:
                    if check_more_than_30_status == None:
                        self.unlock_screen_lock_password(5, set_passwd)
                    else:
                        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
                        self.unlock_screen_lock_password(5, set_passwd)
                        if self.check_resume_reboot_last_input_5_times_reset(lock_type, 5, incorrect_passwd, incorrect_pin) is False:
                            print "[Retry]------ Unlock last_input_5_times_reset Failed"
                            self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
                            self.unlock_screen_lock_password(5, set_passwd)
                            self.remove_resume_screen_lock_all(set_passwd)
                            assert False, "Unlock last_input_5_times_reset Failed"
                        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
                        self.unlock_screen_lock_password(5, set_passwd)
            else:
                print "[Retry]------ Unlock screen password"
                self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
                self.unlock_screen_lock_password(5, set_passwd)
                self.remove_resume_screen_lock_type_all(lock_type, set_passwd)
                assert False, "[Debug]---Unlock screen lock password is Failed"
        elif lock_type == "PIN":
            if self.check_resume_screen_unlock_need_unlock_type(lock_type, try_input_num, set_passwd, incorrect_pin) is True:
                if d(resourceId="com.android.systemui:id/pinEntry").exists:
                    if check_more_than_30_status == None:
                        self.unlock_screen_lock_pin(5, set_pin)
                    else:
                        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
                        self.unlock_screen_lock_pin(5, set_pin)
                        if self.check_resume_reboot_last_input_5_times_reset(lock_type, 5, incorrect_passwd,incorrect_pin) is False:
                            print "[Retry]------ Unlock last_input_5_times_reset Failed"
                            self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
                            self.unlock_screen_lock_pin(5, set_pin)
                            self.remove_resume_screen_lock_type_all(lock_type, set_passwd, set_pin)
                            assert False, "Unlock last_input_5_times_reset Failed"
                        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
                        self.unlock_screen_lock_pin(5, set_pin)
            else:
                print "[Retry]------ Unlock screen PIN"
                self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
                self.unlock_screen_lock_pin(5, set_pin)
                self.remove_resume_screen_lock_type_all(lock_type, set_passwd, set_pin)
                assert False, "[Debug]---Unlock screen lock PIN is Failed"
        elif lock_type == "Pattern":
            if set_passwd == None:
                if self.unlock_screen_lock_pattern(try_input_num) is False:
                    print "[Retry]------ Unlock screen Pattern"
                    self.reboot_unlock_reusme_unlock_pattern()
                    self.remove_screen_lock_pattern()
                    assert False, "[Debug]---Unlock screen lock Pattern is Failed"
                else:
                    time.sleep(23)
                    print "[Completed]------ Unlock screen lock pattern completed"
            else:
                print "incorrect----------------"
                if self.check_resume_screen_unlock_need_unlock_type(lock_type, try_input_num) is True:
                    if incorrect_passwd ==None:
                        self.unlock_screen_lock_pattern(5)
                    else:
                        self.reboot_unlock_reusme_unlock_pattern()
                        if self.check_resume_reboot_last_input_5_times_reset(lock_type, 5) is False:
                            print "[Retry]------ Unlock last_input_5_times_reset Failed"
                            self.reboot_unlock_reusme_unlock_pattern()
                            self.remove_resume_screen_lock_all()
                            assert False, "Unlock last_input_5_times_reset Failed"
                        self.reboot_unlock_reusme_unlock_pattern()
                else:
                    print "[Retry]------ Unlock screen Pattern"
                    self.reboot_unlock_reusme_unlock_pattern()
                    self.remove_resume_screen_lock_type_all(lock_type)
                    assert False, "[Debug]---Unlock screen lock Pattern is Failed"

    def reboot_unlock_reusme_unlock_pattern(self):
        self.reboot_devices()
        self.unlock_screen_lock_pattern(5)
        if d(resourceId="android:id/message").exists:
            self.check_screen_unlock_need_unlock_at_resume()
        time.sleep(30)
        self.unlock_screen_lock_pattern(5)
        if d(resourceId="android:id/message").exists:
            self.check_screen_unlock_need_unlock_at_resume()
            time.sleep(30)

    def check_remove_screen_lock_success(self):
        print "[Verify]------Verify Remove screen lock---"
        self.open_security_in_settings()
        time.sleep(2)
        if d(text="Screen lock").exists and d(textContains="Swipe").exists:
            print "[Verify]------Verify Remove screen lock < Success >"
            return True
        else:
            print "[Verify]------Verify Remove screen lock < Fail >, Need redo remove action"
            return False

    def remove_resume_screen_lock_type_all(self, lock_type, set_passwd=None, set_pin=None):
        if lock_type == "Password":
            if self.remove_resume_screen_lock_all(set_passwd) is False:
                if self.check_remove_screen_lock_success() is False:
                    if self.remove_resume_screen_lock_all(set_passwd) is False:
                        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
                        self.unlock_screen_lock_password(5, set_passwd)
                        self.remove_resume_screen_lock_all(set_passwd)
                        assert False, "[Debug]---Remove screen lock password is Failed"
        elif lock_type == "PIN":
            if self.remove_resume_screen_lock_all(set_pin) is False:
                if self.check_remove_screen_lock_success() is False:
                    if self.remove_resume_screen_lock_all(set_pin) is False:
                        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
                        self.unlock_screen_lock_pin(5, set_pin)
                        self.remove_resume_screen_lock_all(set_pin)
                        assert False, "[Debug]---Remove screen lock PIN is Failed"
        elif lock_type == "Pattern":
            if self.remove_resume_screen_lock_all() is False:
                if self.check_remove_screen_lock_success() is False:
                    if self.remove_resume_screen_lock_all() is False:
                        self.reboot_unlock_reusme_unlock_pattern()
                        self.remove_resume_screen_lock_all()
                        assert False, "[Debug]---Remove screen lock Pattern is Failed"

    def remove_screen_lock_type_all(self, lock_type, set_passwd=None, set_pin=None):
        if lock_type == "None":
            self.check_home_ui_user_safely_O()
        elif lock_type == "Swipe":
            self.boot_up_completed_skip_boot_ui()
        elif lock_type == "Password":
            if self.remove_screen_lock_password(set_passwd) is False:
                if self.check_remove_screen_lock_success() is False:
                    if self.remove_screen_lock_password(set_passwd) is False:
                        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
                        self.unlock_screen_lock_password(5, set_passwd)
                        self.remove_screen_lock_password(set_passwd)
                        assert False, "[Debug]---Remove screen lock password is Failed"
        elif lock_type == "PIN":
            if self.remove_screen_lock_pin(set_pin) is False:
                if self.check_remove_screen_lock_success() is False:
                    if self.remove_screen_lock_pin(set_pin) is False:
                        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
                        self.unlock_screen_lock_pin(5, set_pin)
                        self.remove_screen_lock_pin(set_pin)
                        assert False, "[Debug]---Remove screen lock PIN is Failed"
        elif lock_type == "Pattern":
            if self.remove_screen_lock_pattern() is False:
                if self.check_remove_screen_lock_success() is False:
                    if self.remove_screen_lock_pattern() is False:
                        self.reboot_unlock_reusme_unlock_pattern()
                        self.remove_screen_lock_pattern()
                        assert False, "[Debug]---Remove screen lock Pattern is Failed"

    def check_update_return_var(self, out_return):
        time.sleep(2)
        var_return_1 = out_return.split()[0]
        assert var_return_1 == "200"
        var_return_pass =out_return.split()[-1]
        assert var_return_pass == "0"

    def update_lock_passwd_with_None_type(self, set_passwd):
        self.adb_root()
        cmd_passwd = "vdc cryptfs changepw password %s" %set_passwd
        ver_passwd = "vdc cryptfs verifypw %s" %set_passwd
        set_out = g_common_obj.adb_cmd_capture_msg(cmd_passwd)
        self.check_update_return_var(set_out)
        ver_out = g_common_obj.adb_cmd_capture_msg(ver_passwd)
        self.check_update_return_var(ver_out)
        print "[Info]------verify passwd start to AOS"
        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_passwd)
        output = g_common_obj.adb_cmd_capture_msg(ver_passwd)
        print "[Verify]------check return result: %s " %output
        self.check_update_return_var(output)

    def update_lock_pin_with_None_type(self, set_pin):
        self.adb_root()
        cmd_passwd = "vdc cryptfs changepw pin %s" %set_pin
        ver_passwd = "vdc cryptfs verifypw %s" %set_pin
        set_out = g_common_obj.adb_cmd_capture_msg(cmd_passwd)
        self.check_update_return_var(set_out)
        ver_out = g_common_obj.adb_cmd_capture_msg(ver_passwd)
        self.check_update_return_var(ver_out)
        print "[Info]------verify PIN start to AOS"
        self.reboot_require_unlock_screen_passwd_pin_to_mos(set_pin)
        output = g_common_obj.adb_cmd_capture_msg(ver_passwd)
        print "[Verify]------check return result: %s " %output
        self.check_update_return_var(output)

    def update_lock_pattern_with_None_type(self, set_pattern, update_set=None):
        self.adb_root()
        cmd_passwd = "vdc cryptfs changepw pattern %s" % set_pattern
        ver_passwd = "vdc cryptfs verifypw %s" % set_pattern
        set_out = g_common_obj.adb_cmd_capture_msg(cmd_passwd)
        self.check_update_return_var(set_out)
        ver_out = g_common_obj.adb_cmd_capture_msg(ver_passwd)
        self.check_update_return_var(ver_out)
        print "[Info]------verify pattern start to AOS"
        self.reboot_devices()
        self.push_send_event_script("draw_pattern_lock.sh")
        if d(resourceId="com.android.settings:id/lockPattern").exists:
            for i in range(5):
                if d(resourceId="com.android.settings:id/lockPattern").exists:
                    if update_set == None:
                        self.draw_pattern_lock()
                    else:
                        self.draw_pattern_lock_incorrect()
        else:
            assert False, "[Debug]------Pattern not exists"
        time.sleep(40)
        self.adb_root()
        output = g_common_obj.adb_cmd_capture_msg(ver_passwd)
        print "[Verify]------check return result: %s " % output
        self.check_update_return_var(output)

    def upate_lock_type_with_None_type_all(self, lock_type, set_passwd=None, set_pin=None, set_pattern =None):
        if lock_type == "Password":
            self.update_lock_passwd_with_None_type(set_passwd)
        elif lock_type == "PIN":
            self.update_lock_pin_with_None_type(set_pin)
        elif lock_type == "Pattern":
            self.update_lock_pattern_with_None_type(set_pattern)
        else:
            return

    def upate_lock_type_with_lock_type_all(self, lock_type, update_passwd=None, update_pin=None, update_pattern =None):
        if lock_type == "Password":
            print "[Info]------ Update screen lock password: %s" %update_passwd
            self.update_lock_passwd_with_None_type(update_passwd)
        elif lock_type == "PIN":
            print "[Info]------ Update screen lock PIN: %s" % update_pin
            self.update_lock_pin_with_None_type(update_pin)
        elif lock_type == "Pattern":
            print "[Info]------ Update screen lock Pattern: %s" % update_pattern
            self.update_lock_pattern_with_None_type(update_pattern, "update")
        else:
            return

    def check_gatekeeperd_running(self):
        self.adb_root()
        cmd_print_m = "ps |grep gatekeeperd"
        cmd_pid_m = "ps |grep gatekeeperd |grep [0-9]| awk '{print $2}' "
        cmd_print_o = "ps -A |grep gatekeeperd"
        cmd_pid_o = "ps -A |grep gatekeeperd |grep [0-9]| awk '{print $2}' "
        product = self.check_product()
        if "gordon_peak" in product:
            print g_common_obj.adb_cmd_capture_msg(cmd_print_o)
            pre_pid = g_common_obj.adb_cmd_capture_msg(cmd_pid_o)
            kill_cmd = "kill %s" % pre_pid
            g_common_obj.adb_cmd_capture_msg(kill_cmd)
            time.sleep(2)
            print g_common_obj.adb_cmd_capture_msg(cmd_print_o)
            post_pid = g_common_obj.adb_cmd_capture_msg(cmd_pid_o)
            print pre_pid, post_pid
            assert int(post_pid) != int(pre_pid)
        else:
            print g_common_obj.adb_cmd_capture_msg(cmd_print_m)
            pre_pid = g_common_obj.adb_cmd_capture_msg(cmd_pid_m)
            kill_cmd = "kill %s" % pre_pid
            g_common_obj.adb_cmd_capture_msg(kill_cmd)
            time.sleep(2)
            print g_common_obj.adb_cmd_capture_msg(cmd_print_m)
            post_pid = g_common_obj.adb_cmd_capture_msg(cmd_pid_m)
            print pre_pid, post_pid
            assert int(post_pid) != int(pre_pid)

    def check_vold_decrypt_status(self, action_do):
        # HP desc: need use userdebug image
        self.adb_root()
        cmd_status = "getprop vold.decrypt"
        vold_status = g_common_obj.adb_cmd_capture_msg(cmd_status)
        if "Unlock" in action_do:
            if "trigger_restart_framework" in vold_status:
                print "[Check]------vold_status is: %s" %vold_status
            else:
                assert False, "[Error]------vold_status is: %s" %vold_status
        elif "Reboot" in action_do:
            if "trigger_restart_min_framework" in vold_status:
                print "[Check]------vold_status is: %s" % vold_status
            else:
                assert False, "[Error]------vold_status is: %s" %vold_status
        else:
            return

    def check_getprop_encryption_state(self):
        cmd = 'getprop ro.crypto.state'
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "encrypted" == msg:
            print "[Check]------crypto.state: %s" % msg
        else:
            assert False, "[Error]------crypto.state: %s" % msg

    def check_getprop_encryption_type(self):
        cmd = 'getprop ro.crypto.type'
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "file" == msg:
            print "[Check]------crypto.type: %s" % msg
        else:
            assert False, "[Error]------crypto.type: %s" % msg

    def file_encryption_flag_check(self):
        self.adb_root()
        self.check_getprop_encryption_state()
        self.check_getprop_encryption_type()

    def fbe_enable_check_for_encrypted_file(self, lock=None):
        self.adb_root()
        cmd = "ls /data/data/"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if lock == None:
            if "com.android." in msg or "com.google." in msg:
                print "[Check]------/data/data/: com.android.* exists, no screen lock, file doesn't encrypted"
        else:
            if "com.android." in msg or "com.google." in msg:
                assert False, "[Check]------/data/data/: screen locked, file doesn't encrypted "
            else:
                print "[Check]------/data/data/: screen locked, com.android.* not exists, file has been encrypted"

    def disk_encryption_flag_check(self):
        # HP desc: need use userdebug image
        result = False
        self.adb_root()
        self.check_getprop_encryption_state()

        cmd_data = "cat /fstab.* | grep forceencrypt"
        cmd_data_O = "cat /vendor/etc/fstab.* | grep forceencrypt"
        msg_data = g_common_obj.adb_cmd_capture_msg(cmd_data)
        if msg_data == '':
            msg_data = g_common_obj.adb_cmd_capture_msg(cmd_data_O)
        if "noatime,nosuid,nodev,discard,noauto_da_alloc,errors=panic" in msg_data:
            print "[Check]------crypto data: %s" % msg_data
        else:
            assert False, "[Error]------crypto data: %s" % msg_data

        cmd_flag = "mount | grep /data | grep /dev/block/"
        msg_flag = g_common_obj.adb_cmd_capture_msg(cmd_flag)
        if "rw,seclabel,nosuid,nodev,noatime,discard,noauto_da_alloc,errors=panic,data=ordered" in msg_flag:
            print "[Check]------crypto data flag: %s" % msg_flag
        else:
            assert False, "[Error]------crypto data flag: %s" % msg_flag

    def google_verified_boot_boot_state_green(self):
        cmd = "getprop ro.boot.verifiedbootstate"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "green" in msg:
            print "[Check]------google_verified_boot_boot_state_green: %s" % msg
        else:
            assert False, "[Error]------google_verified_boot_boot_state_green: %s" % msg

    def hardware_backed_keystore(self):
        # HP desc: need use user image
        self.open_security_in_settings()
        product = self.check_product()
        if "gordon_peak" in product or "androidia" in product or "celadon" in product:
            self.scrollable_select_text("credentials")
            if d(textContains="credentials").exists:
                d(textContains="credentials").click()
            self.scrollable_select_text("Storage type")
        else:
            self.scrollable_select_text("Storage type")
        if d(textContains="Storage type").exists:
            time.sleep(2)
            if d(textContains="Hardware-backed").exists:
                print "[Check]------hardware_backed_keystore: Hardware-backed"
            else:
                assert False, "[Error]------hardware_backed_keystore: Hardware-backed not Exists"

    def hardware_backed_keystore_libary_check(self):
        # HP desc: need use userdebug image
        #M: /system/lib/hw/,/system/lib64/hw/ -- keystore.<platform>.so, keystore.gmin.so
        #O MR1: /vendor/lib/hw/ -- keystore.trusty.so
        build_release = self.check_build_release()
        print "[Info]Build release: {}".format(build_release)
        self.adb_root()
        cmd1 = "ls /system/lib/hw/ |grep keystore"
        msg1 = g_common_obj.adb_cmd_capture_msg(cmd1)
        if "keystore.gmin.so" in msg1:
            result1 = True
            print "[Check]------/system/lib/hw/: \n{}".format(msg1)
        else:
            result1 = False
        cmd2 = "ls /system/lib64/hw/ |grep keystore"
        msg2 = g_common_obj.adb_cmd_capture_msg(cmd2)
        if "keystore.gmin.so" in msg2:
            result2 = True
            print "[Check]------/system/lib64/hw/: \n{}".format(msg2)
        else:
            result2 = False
        cmd3 = "ls /vendor/lib/hw/ |grep keystore"
        msg3 = g_common_obj.adb_cmd_capture_msg(cmd3)
        if "keystore.trusty.so" in msg3:
            result3 = True
            print "[Check]------/vendor/lib/hw/: \n{}".format(msg3)
        else:
            result3 = False
        if result1 == False and result2 == False and result3 == False:
            assert False, "keystore_libary Check: %s \n %s \n %s" % (msg1, msg2, msg3)

    def keymaster_version_check(self):
        # HP desc: need use userdebug image
        # O MR1: adb logcat -d |grep TrustyKeymaster -- TrustyKeymaster
        cmd1 = "logcat -d |grep keymaster"
        msg1 = g_common_obj.adb_cmd_common(cmd1)
        if msg1 == '':
            self.reboot_devices()
        msg1 = g_common_obj.adb_cmd_common(cmd1)
        if "keymaster module" in msg1 or "keymaster1 module" in msg1 or "android.hardware.keymaster@3.0" in msg1:
            result1 = True
            print "[Check]------Logcat keymaster:\n {}".format(msg1)
        else:
            result1 = False
        cmd2 = "logcat -d |grep TrustyKM"
        msg2 = g_common_obj.adb_cmd_common(cmd2)
        if "TrustyKM1: Trusty-km connectting" in msg2 or "TrustyKM1: Connected to device successfully" in msg2:
            result2 = True
            print "[Check]------Logcat TrustyKM:\n {}".format(msg2)
            return
        else:
            result2 = False
        cmd3 = "logcat -d |grep TrustyKeymaster"
        msg3 = g_common_obj.adb_cmd_common(cmd3)
        if msg3 == '':
            self.reboot_devices()
        msg3 = g_common_obj.adb_cmd_common(cmd3)
        if "TrustyKeymaster: Device received" in msg3:
            result3 = True
            print "[Check]------Logcat TrustyKeymaster:\n {}".format(msg3)
        else:
            result3 = False
        if result1 == False and result2 == False and result3 == False:
            assert False, "keymaster_version: %s \n %s \n %s" %(msg1, msg2, msg3)

    def selinux_mode_check(self):
        # HP desc: need use user image
        cmd = "getenforce"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "Enforcing" in msg:
            print "[Check]------selinux_mode_check: %s" % msg
        else:
            assert False, "[Error]------selinux_mode_check: %s" % msg

    def selinux_mode_switch(self):
        #0 -- Permissive
        #1 -- Enforcing
        for k in range(3):
            g_common_obj.adb_cmd_common("unroot")
            time.sleep(1)
            info = commands.getoutput('adb shell setenforce 0')
            if "Permission denied" in info:
                print "[Info]------no root, {}".format(info)
                result_1 = True
                break
            else:
                result_1 = False
        for m in range(3):
            self.adb_root()
            g_common_obj.adb_cmd_capture_msg("setenforce 0")
            perm_info = g_common_obj.adb_cmd_capture_msg("getenforce")
            if "Permissive" in perm_info:
                print "[Info]------getenforce 0(Permissive): {}".format(perm_info)
                result_2 = True
            else:
                result_2 = False
            time.sleep(3)
            g_common_obj.adb_cmd_capture_msg("setenforce 1")
            perm_info = g_common_obj.adb_cmd_capture_msg("getenforce")
            if "Enforcing" in perm_info:
                print "[Info]------getenforce 1(Enforcing): {}".format(perm_info)
                result_3 = True
            else:
                result_3 = False
            if (result_2 == True and result_3 == True):
                break
        if result_1 == False:
            assert False, "[Info]------Check root fail,  Permission denied not exist, {}".format(info)
        if result_2 == False or result_3 == False:
            assert False, "Fail to selinux mode switch"

    def system_mount_are_read_only(self):
        # HP desc: need use user image
        cmd = "mount | grep /system | grep rw"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        assert "" == msg

    def vmx_flag_for_trusty_os(self):
        # HP desc: need use user image
        cmd = "cat /proc/cpuinfo"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "vmx" in msg:
            assert False, "vmx flag should not be cleared"

    def check_adb_usb_debugging_status(self):
        print

    def disable_usb_debugging(self):
        g_common_obj.stop_app_am("com.android.settings")
        devoption_cmd = "am start -a android.settings.APPLICATION_DEVELOPMENT_SETTINGS"
        g_common_obj.adb_cmd_capture_msg(devoption_cmd)
        self.scrollable_select_text("USB debugging")
        coordinates = self.bounds_icon_coordinates("USB debugging")
        x = coordinates[0]
        y = coordinates[1]
        print x, y
        self.push_send_event_script("widget_enable_nohup.sh")
        cmd = "nohup sh /mnt/user/0/primary/widget_enable_nohup.sh %s %s > /dev/null 2>&1" %(x, y)
        g_common_obj.adb_cmd_capture_msg(cmd)

        #g_common_obj.adb_cmd_capture_msg("input tap %d %d" % (x, y))

    def auto_lock_timer(self, lock_time):
        if d(text="Automatically lock").exists:
            d(text="Automatically lock").click.wait()
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=lock_time)
        for i in range(5):
            if d(text=lock_time).exists:
                d(text=lock_time).click()
                break
            time.sleep(2)

    def enable_power_button_instantly_lock(self, enable = True):
        # HP desc: need use user image
        product = self.check_product()
        if "gordon_peak" in product or "androidia" in product or "celadon" in product:
            if d(text="Screen lock").exists:
                d(resourceId="com.android.settings:id/settings_button").click.wait()
            self.auto_lock_timer("15 seconds")
            if d(textContains="Power button instantly locks").exists:
                self.setting_ON_OFF("Power button instantly locks", enable)
            else:
                self.open_security_in_settings()
                d(resourceId="com.android.settings:id/settings_button").click.wait()
                self.auto_lock_timer("15 seconds")
                self.setting_ON_OFF("Power button instantly locks", enable)
        else:
            if d(textContains="Power button instantly locks").exists:
                self.auto_lock_timer("15 seconds")
                self.setting_ON_OFF("Power button instantly locks", enable)
            else:
                self.open_security_in_settings()
                self.auto_lock_timer("15 seconds")
                self.setting_ON_OFF("Power button instantly locks", enable)

    def smart_lock_access(self, lock_type, passwd_pin=None):
        # before must be setup password or pin or pattern
        print "[Check]------Smart Lock"
        if d(text="Smart Lock").exists:
            d(text="Smart Lock").click.wait()
        else:
            self.open_security_in_settings()
            d(text="Smart Lock").click.wait()
        if lock_type == "Pattern":
            self.draw_pattern_lock()
            time.sleep(3)
            if d(text="GOT IT").exists:
                d(text="GOT IT").click.wait()
            time.sleep(2)
            assert d(textContains="Trusted").exists or d(textContains="On-body").exists or d(textContains="detection").exists
        else:
            d(resourceId="com.android.settings:id/password_entry").set_text(passwd_pin)
            d.press.enter()
            time.sleep(3)
            if d(text="GOT IT").exists:
                d(text="GOT IT").click.wait()
            time.sleep(2)
            assert d(textContains="Trusted").exists or d(textContains="On-body").exists or d(textContains="detection").exists

    def car_ble_trust_agent_for_o(self, lock_type, passwd_pin=None):
        print "[Check]------Car Ble Trust Agent"
        if d(text="Car Ble Trust Agent").exists:
            d(text="Car Ble Trust Agent").click.wait()
        else:
            self.open_security_in_settings()
            d(text="Car Ble Trust Agent").click.wait()
        if lock_type == "Pattern":
            self.draw_pattern_lock()
            time.sleep(5)
            assert d(textContains="CarBleTrustAgent").exists
        else:
            d(resourceId="com.android.settings:id/password_entry").set_text(passwd_pin)
            d.press.enter()
            time.sleep(5)
            assert d(textContains="CarBleTrustAgent").exists

    def smart_lock_car_ble_trust_agent(self, lock_type, passwd_pin=None):
        self.open_security_in_settings()
        if d(textContains="Screen lock").exists and d(text="Car Ble Trust Agent").exists:
            self.car_ble_trust_agent_for_o(lock_type, passwd_pin)
        elif d(textContains="Screen lock").exists and d(text="Smart Lock").exists:
            self.smart_lock_access(lock_type, passwd_pin)
        else:
            print

    def oem_unlocking_enable_or_password_lock(self, enable=False, lock_type = None, passwd_pin=None):
        if d(text="OEM unlocking").exists:
            print "[Check]------OEM unlocking is exist"
        else:
            g_common_obj.stop_app_am("com.android.settings")
            devoption_cmd = "am start -a android.settings.APPLICATION_DEVELOPMENT_SETTINGS"
            g_common_obj.adb_cmd_capture_msg(devoption_cmd)
        time.sleep(2)
        pre_status = d(textContains="OEM unlocking").right(className="android.widget.Switch").checked
        if enable == pre_status:
            return
        d(text="OEM unlocking").click.wait()
        time.sleep(2)
        if lock_type == None:
            print "[Info] ------No password, Enable(%s) oem unlocking" % enable
        elif lock_type == "Pattern":
            self.draw_pattern_lock()
        else:
            if d(resourceId="com.android.settings:id/password_entry").exists:
                d(resourceId="com.android.settings:id/password_entry").set_text(passwd_pin)
                d.press.enter()
        time.sleep(2)
        if d(text="Enable").exists:
            d(text="Enable").click.wait()
        if d(resourceId="android:id/button1").exists:
            d(resourceId="android:id/button1").click.wait()
        time.sleep(2)
        post_status = d(textContains="OEM unlocking").right(className="android.widget.Switch").checked
        assert enable == post_status

    def check_CA_credentials_comm(self):
        product_name = self.check_product()
        if "gordon_peak" in product_name or "androidia" in product_name or "celadon" in product_name:
            print "[Info]------product_name: {}".format(product_name)
            omr1_refer_file = self.download_artifactory_content("omr1_ca_refer_file")
            path_dut_f = os.path.dirname(omr1_refer_file)
            dut_file = os.path.join(path_dut_f, "omr1_dut.txt")
            cmd = " cat /system/etc/security/cacerts/* > {}".format(dut_file)
            g_common_obj.adb_cmd_capture_msg(cmd)
            os.path.exists(dut_file)
            result = filecmp.cmp(omr1_refer_file, dut_file)
        else:
            print "[Info]------product_name: {}".format(product_name)
            m_refer_file = self.download_artifactory_content("m_ca_refer_file")
            path_dut_f = os.path.dirname(m_refer_file)
            dut_file = os.path.join(path_dut_f, "m_dut.txt")
            cmd = " cat /system/etc/security/cacerts/* > {}".format(dut_file)
            g_common_obj.adb_cmd_capture_msg(cmd)
            os.path.exists(dut_file)
            result = filecmp.cmp(m_refer_file, dut_file)
        if not result:
            raise Exception("The test result did not achieve the desired results")

    def push_to_dut_file_add_permission(self, tar_file, tar_dut_path):
        file_path = None
        self.adb_root()
        for i in range(3):
            file_path = self.push_artifactory_resource(tar_file, tar_dut_path)
            g_common_obj.adb_cmd_capture_msg("chmod 777 {}".format(file_path))
            file_perm = g_common_obj.adb_cmd_capture_msg("ls -l {} ".format(file_path))
            if "-rwxrwxrwx" in file_perm:
                print file_perm
                break
        return file_path

    def run_tipc_test(self, file_path):
        #13-2(burst) tipc test name as below
        # tipc-test -t burst_write -r 10
        # tipc-test -t burst_write -b 2 -r 1000
        tipc_name = ["connect", "connect_foo", "echo", "select", "closer1", "closer2", "closer3"]
        for i in range(len(tipc_name)):
            print
            print "[Info]------tipc name test case: {} ---{}----------".format(tipc_name[i], i+1)
            print
            time.sleep(2)
            cmd = file_path + ' -t ' + tipc_name[i] + ' -r 10 '
            info = g_common_obj.adb_cmd_capture_msg(cmd).strip()
            time.sleep(2)
            for k in range(10):
                if "result:PASS at repeat {}".format(k) in info:
                    print "Test Case {} Test result:PASS at repeat {}".format(tipc_name[i], k)
                else:
                    time.sleep(2)
                    info = g_common_obj.adb_cmd_capture_msg(cmd).strip()
                    if "result:PASS at repeat {}".format(k) in info:
                        print "Test Case {} Test result:PASS at repeat {}".format(tipc_name[i], k)
                    else:
                        assert False, "Test Case {} Test result:Fail at repeat {}".format(tipc_name[i], k)
        #dev-uuid, ta-access, connect -m 64 -b 64, blocked_read
        cmd_uuid = file_path + ' -t dev-uuid'
        cmd_ta = file_path + ' -t ta-access'
        cmd_conn = file_path + ' -t connect -m 64 -b 64'
        info_uuid = g_common_obj.adb_cmd_capture_msg(cmd_uuid).strip()
        info_ta = g_common_obj.adb_cmd_capture_msg(cmd_ta).strip()
        info_conn = g_common_obj.adb_cmd_capture_msg(cmd_conn).strip()
        for info in (info_uuid, info_ta, info_conn):
            if "result:PASS at repeat 0" in info:
                print "Test Case: {}".format(info.split("Test case")[-1])
            else:
                assert False, "Test Case: {}".format(info.split("Test case")[-1])
        #adb shell ps -A | grep tipc-test32
        #adb shell kill -2 5465
        get_log = self.block_read(file_path).strip()
        if "blocked_read_test result:PASS" in get_log:
            print "Test Case:----------\n {}".format(get_log)
        else:
            assert False, "Test Case----------:\n {}".format(get_log)

    def block_read(self, file_path):
        cmd_block = 'adb shell ' + file_path + ' -t blocked_read'
        print cmd_block
        block = self._execute_command_with_popen(cmd_block, True)
        time.sleep(2)
        self.kill_ps_tipc()
        block_log = block.stdout.read()
        return block_log

    def kill_ps_tipc(self):
        cmd = "adb shell ps -A | grep tipc-test32 |awk '{print $2}'"
        cmd_1 = "adb shell ps | grep tipc-test32 |awk '{print $2}'"
        p_pid = self._execute_command_with_popen(cmd, True)
        p_pid.wait()
        pid = p_pid.stdout.read()
        if pid == '' or pid == None:
            p_pid = self._execute_command_with_popen(cmd_1, True)
            p_pid.wait()
            pid = p_pid.stdout.read()
        print "[Info] Kill ps tipc test process PID: {}".format(pid)
        cmd_kill = "adb shell kill -2 {}".format(pid)
        self._execute_command_with_call(cmd_kill, True)

    def run_negative_test(self, file_path):
        negative_name = ["neg_scene_ca_send_msg_no_space","neg_scene_ta_send_msg_large_size","neg_api_create_port", "neg_api_set_cookie",\
                         "neg_api_accept","neg_api_wait", "neg_api_close", "neg_api_get_msg", "neg_api_read_msg",\
                         "neg_api_put_msg","neg_scene_ta_only", "neg_scene_get_msg_without_wait","neg_scene_read_retired_msg",\
                         "neg_scene_put_retired_msg", "neg_scene_ta_calls_after_ca_close_chan"]
        for i in range(len(negative_name)):
            print
            print "[Info]------negative name test case: {} ---{}----------".format(negative_name[i], i+1)
            print
            time.sleep(2)
            cmd = file_path + ' ' + negative_name[i]
            info = g_common_obj.adb_cmd_capture_msg(cmd).strip()
            time.sleep(2)
            if "test Pass" in info:
                print "Test Case: {}".format(info.split("done")[-1])
            else:
                self.reboot_devices()
                time.sleep(5)
                info_2 = g_common_obj.adb_cmd_capture_msg(cmd).strip()
                if "test Pass" in info_2:
                    print "Test Case: {}".format(info_2.split("done")[-1])
                else:
                    assert False, "Test Case: {}".format(info_2.split("done")[-1])
        #15 + 2 --"neg_api_send_msg", "neg_scene_ta_calls_after_ta_close_chan"
        cmd_send_msg = file_path + ' ' + 'neg_api_send_msg'
        cmd_close_chan = file_path + ' ' + 'neg_scene_ta_calls_after_ta_close_chan'
        result_mesg = False
        for k in range(3):
            time.sleep(3)
            g_common_obj.adb_cmd_capture_msg(cmd_close_chan)
            time.sleep(3)
            g_common_obj.adb_cmd_capture_msg(cmd_send_msg)
            close_chan_set = "SET_COOKIE_after_chan_close_scene_test"
            close_chan_get = "GET_MSG_after_chan_close_scene_test"
            close_chan_send = "SEND_MSG_after_chan_close_scene_test"
            close_chan_close = "CLOSE_after_chan_close_scene_test"
            mesg_cha_set = g_common_obj.adb_cmd_capture_msg(" dmesg |grep {}".format(close_chan_set))
            mesg_cha_get = g_common_obj.adb_cmd_capture_msg(" dmesg |grep {}".format(close_chan_get))
            mesg_cha_send = g_common_obj.adb_cmd_capture_msg(" dmesg |grep {}".format(close_chan_send))
            mesg_cha_close = g_common_obj.adb_cmd_capture_msg(" dmesg |grep {}".format(close_chan_close))
            mesg_send = g_common_obj.adb_cmd_capture_msg(" dmesg |grep neg_api_send_msg")
            for info_mesg in (mesg_cha_set, mesg_cha_get, mesg_cha_send, mesg_cha_close, mesg_send):
                if "test Pass" in info_mesg:
                    print "Test Case: {}".format(info_mesg.split(":")[-1])
                    result_mesg = True
                else:
                    result_mesg = False
            if result_mesg == True:
                break
            else:
                self.reboot_devices()
        assert result_mesg, "Test Case: {} \n {} \n {}".format(mesg_cha_set.split(":")[-1],mesg_cha_get.split(":")[-1],\
                                    mesg_cha_send.split(":")[-1],mesg_cha_close.split(":")[-1], mesg_send.split(":")[-1])

    def check_eb_image_file_exists(self, remote_dir, res_file):
        cmd = "'ls %s 2>/dev/null'| grep %s" % (remote_dir, res_file)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if msg != "":
            print "[info]---res file %s already existed, don't need to flash image..." % res_file
            return True
        else:
            return False

    def check_secure_storage_unit_test(self):
        self.adb_root()
        cmd = 'dmesg |grep unittest'
        info = g_common_obj.adb_cmd_capture_msg(cmd)
        test_result = True
        if info == '':
            self.reboot_devices()
            info = g_common_obj.adb_cmd_capture_msg(cmd)
            time.sleep(10)
            if info == '':
                test_result = False
        if "PASSED" not in info:
            test_result = False
        if "FAILED" in info or "Fail" in info:
            test_result = False
        assert test_result, "[Check]------The unit test secure_storage case is Failed"

    def kill_minicom_process_PID(self):
        result = os.popen("ps -aux |grep minicom| awk '{print $2}'").readlines()
        print result
        for pid in result:
            pid = pid.strip("\r\n")
            pid = pid.strip()
            if os.environ.get(NOSERUNNER) is None:
                os.system("sudo kill " + pid)
            else:
                os.system("echo '123456' | sudo -S  kill " + pid)

    def check_uart_minicom_serial_string_get_log(self, string):
        from uart_ioc_log import UartIOC
        mini_ser = UartIOC("serial")
        mini_ser.setDaemon(True)
        mini_ser.start()
        #hwcrypto_unittest,  boot image
        print "[Info]------Start IOC port, get uart log during reboot DUT..."
        product_name = self.check_product()
        if "gordon_peak" in product_name:
            print "[Info]------product_name: {}".format(product_name)
            mini_ser.ioc_reboot()
        else:
            print "[Info]------product_name: {}".format(product_name)
            mini_ser.ioc_reboot_quickly()
        log = mini_ser.check_string_before_get_key_log(string)
        mini_ser.stop_reading()
        #mini_ser.join() # need't to wait for child thread
        print "[Info]------Stopping subprocess UART log..."
        time.sleep(10)
        mini_ser.stop_reading()
        self.boot_up_completed_no_root(10)
        return log

    def check_hwcrypto_unit_test(self):
        #Total 10 cases
        #M: hwkey_derive_repeatable: FAILED     hwkey_derive_different: FAILED
        log = self.check_uart_minicom_serial_string_get_log("boot completed")
        if log == []:
            log = self.check_uart_minicom_serial_string_get_log("boot image")
            if log == []:
                assert False, "[Info]------test case Failed: {}".format(log)
        check_str = []
        for line in log:
            if "hwcrypto_unittest" in line:
                check_str.append(line)
                if "PASSED" in line:
                    print "[Info]------test case: {}".format(line)
                if "FAILED" in line:
                    if "hwkey_derive_repeatable" in line or "hwkey_derive_different" in line:
                        print "[Info]------test case EB patch code changed, developer tag case is PASSED:\n{}".format(line)
                    else:
                        assert False, "[Info]------test case Failed: {}".format(line)
        if check_str == []:
            assert False, "[Info]------test case Failed, No hwcrypto_unittest test keyword"

    def check_ALSR_variable_address(self):
        '''
        ==========<ASLR_TA test> start=====
        local_variable:   427dae60
        global_varibale:  417ff480
        function:         417e60d0
        ==========<ASLR_TA test> end========
        '''
        get_log = []
        get_log_after_reboot = []
        log = self.check_uart_minicom_serial_string_get_log("<ASLR_TA test> end")
        if log == []:
            log = self.check_uart_minicom_serial_string_get_log("<ASLR_TA test> end")
            if log == []:
                assert False, "[Info]------test case Failed: {}".format(log)
        for line in log:
            if "<ASLR_TA test> start" in line:
                get_log.append(line.strip())
            if "local_variable" in line:
                get_log.append(line.strip())
            if "global_varibale" in line:
                get_log.append(line.strip())
            if "function" in line:
                get_log.append(line.strip())
            if "<ASLR_TA test> end" in line:
                get_log.append(line.strip())
        print "[LOG_1]---{}".format(get_log)
        if get_log == []:
            assert False, "[Info]------test case Failed, No ALSR_variable_address test keyword"
        time.sleep(10)
        log_reboot = self.check_uart_minicom_serial_string_get_log("<ASLR_TA test> end")
        if log_reboot == []:
            log_reboot = self.check_uart_minicom_serial_string_get_log("<ASLR_TA test> end")
            if log_reboot == []:
                assert False, "[Info]------test case Failed: {}".format(log_reboot)
        for line in log_reboot:
            if "<ASLR_TA test> start" in line:
                get_log_after_reboot.append(line.strip())
            if "local_variable" in line:
                get_log_after_reboot.append(line.strip())
            if "global_varibale" in line:
                get_log_after_reboot.append(line.strip())
            if "function" in line:
                get_log_after_reboot.append(line.strip())
            if "<ASLR_TA test> end" in line:
                get_log_after_reboot.append(line.strip())
        print "[LOG_2]---{}".format(get_log_after_reboot)
        if get_log_after_reboot == []:
            assert False, "[Info]------test case Failed, No ALSR_variable_address test keyword"
        if get_log == get_log_after_reboot:
            assert False, "[LOG]------Compare log's address Failed, \n {} \n {}".format(get_log, get_log_after_reboot)

    def threading_comm(self, func1, func2=None):
        threads = []
        t1 = threading.Thread(target=func1)
        threads.append(t1)
        if func2:
            t2 = threading.Thread(target=func2)
            threads.append(t2)
        for t in threads:
            t.setDaemon(True)
            t.start()
        for t in threads:
            t.join()
        print
