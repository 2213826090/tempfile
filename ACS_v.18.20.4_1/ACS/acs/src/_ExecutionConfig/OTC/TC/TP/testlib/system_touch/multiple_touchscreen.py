# -*- coding: utf-8 -*-
from testlib.util.common import g_common_obj
from system_touch import SystemTouch
import time
import os
import re


d = g_common_obj.get_device()
dsn = d.server.adb.device_serial()

class MultiTouchScreen(SystemTouch):

    def __init__(self):
        SystemTouch.__init__(self)
        self.official_tsn = """ getevent -p |grep '{}' |wc -l """.format(self.official_key)
        self.td_tsn = """ getevent -p |grep '{}' |grep 'CoolTouchR' |wc -l """.format(self.td_key)

    '''
    tsn : touch screen number

    official panel: GP display Kit-GL116FHD_CTA
    support max point: 4 fingers

    big screen: TD 2230
    support max point: 10 fingers

    EDP screen: edp screen
    support max point: don't kown
    '''

    def check_car_mode_exists(self, timeout=20):
        self.adb_root()
        cmd = "pm disable android.car.cluster.sample"
        cmd_user = "pm disable-user android.car.cluster.sample"
        product_name = self.check_product()
        if "gordon_peak" in product_name:
            g_common_obj.adb_cmd_capture_msg(cmd)
            g_common_obj.adb_cmd_capture_msg(cmd_user)
            time.sleep(timeout)
        elif "bxt" in product_name:
            if d(packageName="com.android.launcher3").exists:
                return
            else:
                time.sleep(5)
        else:
            g_common_obj.adb_cmd_capture_msg(cmd)
            g_common_obj.adb_cmd_capture_msg(cmd_user)
            time.sleep(timeout)

    def get_multi_touch_sp_connected_num(self):
        connect_offic_ts_num = g_common_obj.adb_cmd_capture_msg(self.official_tsn)
        connect_big_ts_num = g_common_obj.adb_cmd_capture_msg(self.td_tsn)
        conn_num = int(connect_offic_ts_num) + int(connect_big_ts_num)
        return conn_num

    def check_multi_touch_sp_connected_num(self, sp_target_num = 1):
        conn_num = self.get_multi_touch_sp_connected_num()
        if conn_num == sp_target_num == 1:
            print "[INFO]Devices is connecting 1 touch panel"
            print "[INFO]One touch screen case test start..."
        elif conn_num == sp_target_num == 2:
            print "[INFO]Devices is connecting 2 touch panel"
            print "[INFO]Two touch screen case test start..."
        elif conn_num == sp_target_num == 3:
            print "[INFO]Devices is connecting 3 touch panel"
            print "[INFO]Three touch screen case test start..."
        else:
            raise EnvironmentError, "[INFO]Devices is connecting target <{}> Panel Error".format(conn_num)

    def enter_to_mosaic_mode_display_touch_work_comm(self):
        build_release = self.check_build_release()
        print "[INFO]Build release: %s" % build_release
        #M: 6.0, N: 7.0, O: 8.0
        if "6." in build_release:
            self.extended_desktop_mosaic_mode_display_work()
            self.extended_desktop_mosaic_mode_touch_work()
        elif "8." in build_release:
            self.extended_desktop_mosaic_mode_display_work_for_O()
        else:
            print

    def exit_to_mosaic_mode_display_touch_work_comm(self):
        build_release = self.check_build_release()
        print "[INFO]Build release: %s" % build_release
        if "6." in build_release:
            self.exit_extended_desktop_mosaic_mode_display_work()
            self.exit_extended_desktop_mosaic_mode_touch_work()
        elif "8." in build_release:
            self.exit_extended_desktop_mosaic_mode_display_work_for_O()
        else:
            print

    def extended_desktop_mosaic_mode_display_work(self):
        self.adb_root()
        g_common_obj.adb_cmd_capture_msg("'cat /cache/hwc.reg'")
        dmconfig_cmd = """ 'echo "option.dmconfig=[SF:0 MOSAIC 3840x1080 PANEL (P:0 3 0,0 1920x1080 0,0 1920x0@0)\
                (P:2 3 1920,0 1920x1080 0,0 1920x0@0)]" >> /cache/hwc.reg' """
        proxyenabled_cmd = """ 'echo "option.proxyenabled=0" >> /cache/hwc.reg' """
        g_common_obj.adb_cmd_capture_msg(dmconfig_cmd)
        g_common_obj.adb_cmd_capture_msg(proxyenabled_cmd)
        g_common_obj.adb_cmd_capture_msg("stop")
        g_common_obj.adb_cmd_capture_msg("start")
        check_mosaic_setup = g_common_obj.adb_cmd_capture_msg("'cat /cache/hwc.reg'")
        if "option.dmconfig" and "option.proxyenabled=0" in check_mosaic_setup:
            print "[INFO]Mosaic mode display work"
        else:
            assert False, "[INFO]Mosaic mode display function Setup failed"

    def extended_desktop_mosaic_mode_touch_work(self):
        self.adb_root()
        g_common_obj.adb_cmd_capture_msg("' setprop persist.sys.touch.mosaic.mode 1 '")
        g_common_obj.adb_cmd_capture_msg(""" 'setprop persist.sys.touch.mosaic.0 "usb-0000:00:15.0-2.3/input0"' """)
        g_common_obj.adb_cmd_capture_msg(""" 'setprop persist.sys.touch.mosaic.1 "usb-0000:00:15.0-3/input0"' """)
        self.reboot_devices()
        self.reboot_devices()
        check_mosaic_touch = g_common_obj.adb_cmd_capture_msg("' getprop persist.sys.touch.mosaic.mode '")
        print "[INFO]check_mosaic_touch_mode: %s" % check_mosaic_touch
        if "1" in check_mosaic_touch:
            print "[INFO]Mosaic mode touch function work"
        else:
            assert False, "[INFO]Mosaic mode touch function Setup failed"

    def exit_extended_desktop_mosaic_mode_display_work(self):
        self.adb_root()
        g_common_obj.adb_cmd_capture_msg("'cat /cache/hwc.reg > /cache/hwc.reg'")
        g_common_obj.adb_cmd_capture_msg("stop")
        g_common_obj.adb_cmd_capture_msg("start")
        check_mosaic_setup = g_common_obj.adb_cmd_capture_msg("'cat /cache/hwc.reg'")
        if "option.dmconfig" and "option.proxyenabled=0" not in check_mosaic_setup:
            print "[INFO]Exit Mosaic mode display function Passed"
        else:
            assert False, "[INFO]Exit Mosaic mode display function failed"

    def exit_extended_desktop_mosaic_mode_touch_work(self):
        self.adb_root()
        g_common_obj.adb_cmd_capture_msg("'setprop persist.sys.touch.mosaic.mode 0'")
        self.reboot_devices()
        check_mosaic_touch = g_common_obj.adb_cmd_capture_msg("' getprop persist.sys.touch.mosaic.mode '")
        if "0" in check_mosaic_touch:
            print "[INFO]Exit Mosaic mode touch function Passed"
        else:
            assert False, "[INFO]Exit Mosaic mode touch function failed"

    def multi_touch_screen_mode_setup(self):
        self.adb_root()
        self.check_adb_remount()
        g_common_obj.adb_cmd_capture_msg("mount -o rw,remount /")

    def extended_desktop_mosaic_mode_display_work_for_O(self):
        '''
        1. adb pull /vendor/etc/hwc_display.ini
            save Default setting file
        2. edit the file:
            MOSIAC="true"
            CLONE="False"
        3. adb push hwc_display.ini /vendor/etc/hwc_display.ini
        4. adb reboot
        '''
        self.check_car_mode_exists()
        self.multi_touch_screen_mode_setup()
        file_path = "./temp/files"
        file_name = "hwc_display.ini"
        os.system("mkdir -p {}".format(file_path))
        g_common_obj.adb_cmd_common("pull /vendor/etc/hwc_display.ini {}".format(file_path))
        file_path_name = os.path.join(file_path, file_name)
        bak_file = os.path.join(file_path, "hwc_display.ini_bak")
        os.system("cp {} {}".format(file_path_name, bak_file))
        cmd_1 = """ sed -i 's/MOSAIC="false"/MOSAIC="true"/g' {} """.format(file_path_name)
        cmd_2 = """ sed -i 's/CLONE="true"/CLONE="false"/g' {} """.format(file_path_name)
        os.system(cmd_1)
        os.system(cmd_2)
        time.sleep(2)
        g_common_obj.adb_cmd_common("push {} /vendor/etc/hwc_display.ini".format(file_path_name))
        self.reboot_devices()
        g_common_obj.adb_cmd_capture_msg("setenforce 0")
        self.extended_desktop_mosaic_mode_touch_work()

    def exit_extended_desktop_mosaic_mode_display_work_for_O(self):
        self.multi_touch_screen_mode_setup()
        file_name = "./temp/files/hwc_display.ini_bak"
        g_common_obj.adb_cmd_capture_msg("setenforce 1")
        g_common_obj.adb_cmd_common("push {} /vendor/etc/hwc_display.ini".format(file_name))
        g_common_obj.adb_cmd_capture_msg("'setprop persist.sys.touch.mosaic.mode 0'")
        self.reboot_devices()
        self.check_car_mode_exists()
        os.system("rm -rf ./temp")

    def check_mosaic_single_touch_sendevent_screen_comm(self, sp_num =None):
        self.push_send_event_script("touch_click_sendevent.sh")
        if not hasattr(self, "event_info"):
            self.event_info = self.get_event_info()
        if type(self.event_info) == tuple:
            print "[Screen_Event_Info]---Screen event info type is Tuple, connect to > 1 TS"
            print "<<<Multi_TS>>> Test Event Info: {}".format(self.event_info[0])
            self.check_sendevent_touch_screen_response(self.event_info[0], sp_num)
            print "<<<Multi_TS>>> Test Event Info: {}".format(self.event_info[1])
            self.check_sendevent_touch_screen_Security(self.event_info[1])

    def check_touch_gesture_swipe_vertical_down_up_comm(self, swipe_down = None):
        self.push_send_event_script("touch_gesture_swipe_vertical_down.sh")
        if not hasattr(self, "event_info"):
            self.event_info = self.get_event_info()
        if type(self.event_info) == dict:
            print "[Screen_Event_Info]---Screen event info type is Dict, connect to 1 TS"
            self.touch_gesture_swipe_vertical_down_up(self.event_info, swipe_down)
        if type(self.event_info) == tuple:
            print "[Screen_Event_Info]---Screen event info type is Tuple, connect to > 1 TS"
            for event_info in self.event_info:
                print "<<<Multi_TS>>> Test Event Info: {}".format(event_info)
                self.touch_gesture_swipe_vertical_down_up(event_info, swipe_down)

    def touch_gesture_swipe_vertical_down_up(self, event_info, def_cmd = None):
        #gesture swipe developer option, default swipe down
        scale_x = int(event_info["scale_x"])
        scale_y = int(event_info["scale_y"])
        self.launch_developer_option()
        time.sleep(2)
        if d(className="android.widget.ListView").exists:
            bounds = d(className="android.widget.ListView").bounds
        else:
            bounds = d(resourceId="com.android.settings:id/content_parent").bounds
        bar_on = d(resourceId="com.android.settings:id/switch_bar").bounds
        bar_on_top = (bar_on["bottom"] - bar_on["top"] + 6) * 2
        x_top = ((bounds["right"]-bounds["left"]) / 2) * scale_x
        y_top = (bounds["top"] + bar_on_top) * scale_y
        x_bottom = ((bounds["right"] - bounds["left"]) / 2) * scale_x
        y_bottom = (bounds["bottom"] - bounds["top"]) * scale_y
        assert d(text="Stay awake").exists
        if def_cmd == None:
            cmd_down = self.cmd_touch_gesture_swipe_vertical_down(event_info["dev_num"], x_top, y_top, x_bottom, y_bottom)
            g_common_obj.adb_cmd_capture_msg(cmd_down)
            g_common_obj.adb_cmd_capture_msg(cmd_down)
            assert not d(text="Stay awake").exists
        else:
            self.push_send_event_script("touch_gesture_swipe_vertical_up.sh")
            d().swipe.up(steps=2)
            time.sleep(2)
            max_cmd = self.cmd_touch_gesture_swipe_vertical_up(event_info["dev_num"], x_top, y_top, x_bottom, y_bottom)
            while not d(text="Stay awake").exists:
                g_common_obj.adb_cmd_capture_msg(max_cmd)
            assert d(text="Stay awake").exists

    def check_touch_gesture_swipe_horizontal(self):
        self.push_send_event_script("touch_gesture_swipe_horizontal_sendevent.sh")
        g_common_obj.launch_app_am("com.google.android.deskclock", "com.android.deskclock.DeskClock")
        time.sleep(1)
        if not hasattr(self, "event_info"):
            self.event_info = self.get_event_info()
        if type(self.event_info) == dict:
            print "[Screen_Event_Info]---Screen event info type is Dict, connect to 1 TS"
            self.touch_gesture_swipe_horizontal(self.event_info)
        if type(self.event_info) == tuple:
            print "[Screen_Event_Info]---Screen event info type is Tuple, connect to > 1 TS"
            for event_info in self.event_info:
                print "<<<Multi_TS>>> Test Event Info: {}".format(event_info)
                self.touch_gesture_swipe_horizontal(event_info)

    def touch_gesture_swipe_horizontal(self, event_info):
        scale_x = int(event_info["scale_x"])
        scale_y = int(event_info["scale_y"])
        time.sleep(2)
        product_name = self.check_product()
        print "[INFO]product_name: %s" % product_name
        if "androidia" in product_name:
            text_clock = "CLOCK"
        else:
            text_clock = "Clock"
        d(description=text_clock).click()
        assert d(description=text_clock).selected
        info = d.info
        x_R = (info["displayWidth"] - 100) * scale_x
        y_R = (info["displayHeight"] / 2) * scale_y
        x_L = 100 * scale_x
        cmd_r = self.cmd_touch_gesture_swipe_horizontal_sendevent(event_info["dev_num"], x_R, y_R, x_L, y_R)
        cmd_l = self.cmd_touch_gesture_swipe_horizontal_sendevent(event_info["dev_num"], x_L, y_R, x_R, y_R)
        g_common_obj.adb_cmd_capture_msg(cmd_r)
        time.sleep(2)
        g_common_obj.adb_cmd_capture_msg(cmd_r)
        assert not d(description=text_clock).selected
        # swipe to left
        time.sleep(2)
        d(description=text_clock).click()
        assert d(description=text_clock).selected
        g_common_obj.adb_cmd_capture_msg(cmd_l)
        time.sleep(2)
        g_common_obj.adb_cmd_capture_msg(cmd_l)
        assert not d(description=text_clock).selected

