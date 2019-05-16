#!/usr/bin/env python
#-*- encoding: utf-8 -*-

import re
import os
import time
import subprocess
from nose.tools import assert_equals
from testlib.util.device import TestDevice


class BluetoothSettingImpl:
    """
    Implements Bluetooth Setting UI actions.

    """

    def __init__(self, cfg={}):
        """Initialize the environment"""
        self.cfg = cfg

    def init_pair(self, serialdut, serialtest, unpair=False):
        """Pair with another device"""
        d = TestDevice(serialdut).get_device()
        t = TestDevice(serialtest).get_device()
        if unpair:
            self.unpair_all(serialdut)
        if d(description="Device settings").exists:
            dut_name = d(description="Device settings").left(className="android.widget.TextView").info["text"]
            if t(description="Device settings").exists:
                test_name =t(description="Device settings").left(className="android.widget.TextView").info["text"]
                if dut_name == serialtest and test_name == serialdut:
                    print "[INFO]:  %s Already paired with %s " % (serialdut, serialtest)
        else:
            self.scan(serialdut, serialtest)
            self.pair(serialdut, serialtest)
            self.click_pair(serialtest)
            self.click_pair(serialdut)
        self.check_paired(serialdut, serialtest)

    def execute_cmd(self, command):
        lines = os.popen(command).readlines()
        return lines

    def execute_no_end_cmd(self, command):
        child = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        time.sleep(20)
        child.kill()
        return child.stdout.readlines()

    def set_orientation(self, serial):
        """Set the screen to portrait."""
        d = TestDevice(serial).get_device()
        d.orientation = "n"

    def return_home(self, serial):
        """Return to home screen"""
        g_device = TestDevice(serial)
        g_device.back_home()

    def drag_widget(self, serial, appname, appgallery="Apps", inspection=None):
        """Drag widget to home screen"""
        d = TestDevice(serial).get_device()
        iffind = False
        d.press.home()
        d(packageName="com.google.android.googlequicksearchbox").long_click()
        d(text="Widgets").click()
        for i in range(40):
            if d(text=appname).exists:
                d(text=appname).drag.to(text=appname, steps=100)
                iffind = True
                break
            d(scrollable=True).scroll.horiz()
        assert_equals(iffind, True)
        time.sleep(3)
        if (inspection is not None):
            assert d(textMatches=inspection)

    def setup_connection(self, serial):
        """Set up the connection"""
        d = TestDevice(serial).get_device()
        value = False
        for i in range(5):
            if d.server.alive:
                value = True
                break
            else:
                d.server.stop()
                d.server.start()
        if value:
            print "[INFO]: %s --- Success to set up connection" % serial
        else:
            print "[INFO]: %s --- Fail to set up connection" % serial
        assert_equals(value, True)

    def setUp(self, devices):
        for serial in devices:
            self.set_orientation(serial)
            self.return_home(serial)
            self.setup_connection(serial)

    def tearDown_apps(self, devices):
        for serial in devices:
            g_device = TestDevice(serial)
            g_device.close_background_apps()
            self.return_home(serial)

    def tearDown(self, devices):
        for serial in devices:
            self.return_home(serial)

    def launch_from_settings(self, serial):
        """Launch bluetooth from Settings"""
        print "[INFO]: %s --- Launch bluetooth from Settings" % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        for i in range(3):
            d.press.home()
            g_device.launch_app_from_home_sc("Settings")
            d(text="Bluetooth").click.wait()
            if d(description="More options").exists:
                break
            time.sleep(2)
        assert_equals(d(description="More options").exists, True)

    def launch_from_quick_settings(self, serial):
        """Launch bluetooth from Quick Settings"""
        print "[INFO]: %s --- Launch bluetooth from Quick Settings" % serial
        d = TestDevice(serial).get_device()
        d.open.quick_settings()
        time.sleep(1)
        d.open.quick_settings()
        time.sleep(2)
        d(textContains="Bluetooth").click.wait()
        time.sleep(2)
        assert_equals(d(description="More options").exists, True)

    def on(self, serial):
        """Turn on bluetooth"""
        print "[INFO]: %s --- Turn on bluetooth" % serial
        d = TestDevice(serial).get_device()
        if d(text="OFF", className="android.widget.Switch").exists:
            d(text="OFF", className="android.widget.Switch").click()
            time.sleep(8)
        assert_equals(d(className="android.widget.Switch").info['text'], "ON")

    def off(self, serial):
        """Turn off bluetooth"""
        print "[INFO]: %s --- Turn off bluetooth" % serial
        d = TestDevice(serial).get_device()
        if d(text="ON", className="android.widget.Switch").exists:
            d(text="ON", className="android.widget.Switch").click()
            time.sleep(8)
        assert_equals(d(className="android.widget.Switch").info['text'], "OFF")

    def toggle(self, serial):
        """Turn on/off bluetooth"""
        print "[INFO]: %s --- Toggle bluetooth" % serial
        d = TestDevice(serial).get_device()
        d(className="android.widget.Switch").click()
        time.sleep(3)

    def on_by_quick(self, serial):
        """Turn on bluetooth from quick settings"""
        print "[INFO]: %s --- Turn on bluetooth from quick setting" % serial
        d = TestDevice(serial).get_device()
        d.open.quick_settings()
        time.sleep(1)
        d.open.quick_settings()
        time.sleep(2)
        if d(description="Bluetooth off.").exists:
            d(description="Bluetooth off.").click()
            time.sleep(7)
        assert_equals(d(description="Bluetooth on.").exists, True)

    def off_by_quick(self, serial):
        """Turn off bluetooth from quick settings"""
        print "[INFO]: %s --- Turn off Bluetooth from quick setting" % serial
        d = TestDevice(serial).get_device()
        d.open.quick_settings()
        time.sleep(1)
        d.open.quick_settings()
        time.sleep(2)
        if d(description="Bluetooth on.").exists:
            d(description="Bluetooth on.").click()
            time.sleep(7)
        assert_equals(d(description="Bluetooth off.").exists, True)

    def check_bt(self, serial, status):
        """Check bluetooth is on/off"""
        print "[INFO]: %s --- Check bluetooth is %s" % (serial, status)
        d = TestDevice(serial).get_device()
        btn_text = d(className="android.widget.Switch").info['text']
        assert_equals(btn_text, status)

    def rename(self, serial, rename):
        """Rename bluetooth"""
        print "[INFO]: %s --- Rename bluetooth to %s" % (serial, rename)
        d = TestDevice(serial).get_device()
        d(description="More options").click.wait()
        time.sleep(2)
        d(text="Rename this device").click.wait()
        time.sleep(2)
        while d(className="android.widget.EditText").text != "":
            d(className="android.widget.EditText").clear_text()
        d(className="android.widget.EditText").set_text(rename)
        d(text="Rename").click.wait()
        time.sleep(2)
        rename_flag = "%s is visible to nearby devices" % rename
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.toEnd()
            value = d(textStartsWith=rename_flag).exists
            d(scrollable=True).scroll.toBegging()
        else:
            value = d(textStartsWith=rename_flag).exists
        assert_equals(value, True)

    def scan(self, serial, btName):
        """Scan nearby bluetooth devices"""
        print "[INFO]: %s --- Scan bluetooth device %s" % (serial, btName)
        d = TestDevice(serial).get_device()
        for i in range(15):
            flag = d(resourceId="com.android.settings:id/scanning_progress")
            if not flag.exists:
                break
            time.sleep(5)
        d(description="More options").click.wait()
        d(text="Refresh").click()
        time.sleep(2)
        for i in range(15):
            flag = d(resourceId="com.android.settings:id/scanning_progress")
            if not flag.exists:
                break
            time.sleep(5)
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=btName)
        assert_equals(d(text=btName).exists, True)

    def click_scan(self, serial):
        """Click scan button"""
        print "[INFO]: %s --- Click scan button" % serial
        d = TestDevice(serial).get_device()
        for i in range(15):
            flag = d(resourceId="com.android.settings:id/scanning_progress")
            if not flag.exists:
                break
            time.sleep(5)
        d(description="More options").click.wait()
        d(text="Refresh").click()
        time.sleep(2)
        assert_equals(flag.exists, True)

    def pair(self, serial, btName):
        """Pair with another device"""
        print "[INFO]: %s --- Pair with %s" % (serial, btName)
        d = TestDevice(serial).get_device()
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=btName)
        d(text=btName).click.wait()
        for i in range(5):
            if d(text="Bluetooth pairing request").exists:
                break
            time.sleep(2)
        assert_equals(d(text="Bluetooth pairing request").exists, True)

    def get_pair_dialog(self, serial):
        """Get pair dialog from notification"""
        print "[INFO]: %s --- Get pair dialog" % serial
        d = TestDevice(serial).get_device()
        if not d(text="Bluetooth pairing request").exists:
            d.open.quick_settings()
            d(text="Pairing request").click()
        time.sleep(2)
        assert_equals(d(text="Bluetooth pairing request").exists, True)

    def click_pair(self, serial):
        """Click pair button in dialog"""
        print "[INFO]: %s --- Click pair button in dialog" % serial
        d = TestDevice(serial).get_device()
        d(text="Pair").click.wait()
        assert_equals(d(text="Pair").exists, False)

    def click_cancel(self, serial):
        """Click cancel button in dialog"""
        print "[INFO]: %s --- Click cancel button in dialog" % serial
        d = TestDevice(serial).get_device()
        d(text="Cancel").click.wait()
        assert_equals(d(text="Cancel").exists, False)

    def check_paired(self, serial, btName, expect=True):
        """Check paired/not"""
        info_tuple = (serial, btName, expect)
        print "[INFO]: %s --- Check paired of %s is %s" % info_tuple
        d = TestDevice(serial).get_device()
        for i in range(5):
            value = d(text=btName).right(description="Device settings")
            if str(value is not None) == str(expect):
                break
            time.sleep(3)
        assert_equals(str(value is not None), str(expect))

    def open_paired(self, serial, btName):
        """Open paired settings"""
        print "[INFO]: %s --- Open paired settings of %s" % (serial, btName)
        d = TestDevice(serial).get_device()
        time.sleep(2)
        d(text=btName).right(description="Device settings").click()
        time.sleep(2)
        assert_equals(d(text="Paired devices").exists, True)

    def check_rejected(self, serial, btName, expect=True):
        """Check rejected dialog"""
        print "[INFO]: %s --- Check rejected dialog is %s" % (serial, expect)
        d = TestDevice(serial).get_device()
        name = "Couldn't pair with %s because of" % btName
        value = str(d(textContains=name).exists)
        if value == "True":
            d(text="OK").click()
        assert_equals(value, str(expect))

    def unpair(self, serial, btName):
        """Unpair bluetooth device"""
        print "[INFO]: %s --- Unpair device %s" % (serial, btName)
        d = TestDevice(serial).get_device()
        target = d(text=btName).right(description="Device settings")
        target.click.wait()
        time.sleep(2)
        d(text="FORGET").click.wait()
        time.sleep(2)
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=btName)
        value = d(text=btName).right(description="Device settings")
        assert_equals(value is None, True)

    def unpair_all(self, serial):
        """Unpair all paired device"""
        print "[INFO]: %s --- Unpair all paired device " % serial
        d = TestDevice(serial).get_device()
        number = d(description="Device settings").count
        while (number != 0):
            d(description="Device settings").click()
            time.sleep(2)
            d(text="FORGET").click.wait()
            time.sleep(2)
            number = d(description="Device settings").count
        assert_equals(number, 0)

    def toggle_by_widget(self, serial):
        """Toggle bluetooth from widget"""
        d = TestDevice(serial).get_device()
        self.return_home(serial)
        resource = "com.aiteam.bluetoothtogglewidget:id/widget_icon"
        target = d(resourceId=resource)
        if not target.exists:
            print "[INFO]: %s --- Drag widget to screen" % serial
            name = "Bluetooth Toggle Widget"
            self.drag_widget(serial, name)
            time.sleep(4)
        print "[INFO]: %s --- Toggle bluetooth from widget" % serial
        target.click()
        time.sleep(5)

    def launch_btchat(self, serial):
        """Launch bluetooth chat"""
        print "[INFO]: %s --- Launch bluetooth chat" % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        d.press.home()
        g_device.launch_app_from_home_sc("Bluetooth Chat")
        open_flag1 = d(text="An app wants to turn on Bluetooth.").exists
        open_flag2 = d(packageName="droideplace.BluetoothChat").exists
        assert_equals(open_flag1 or open_flag2, True)

    def on_by_btchat(self, serial):
        """Turn on bluetooth in bluetooth chat"""
        print "[INFO]: %s --- Turn on bluetooth in bluetooth chat" % serial
        d = TestDevice(serial).get_device()
        time.sleep(2)
        d(text="Allow").click()
        for i in range(5):
            if d(packageName="droideplace.BluetoothChat").exists:
                break
            time.sleep(3)
        assert_equals(d(packageName="droideplace.BluetoothChat").exists, True)

    def visible_in_btchat(self, serial):
        """Make bluetooth is visible to other device"""
        print "[INFO]: %s --- Make bluetooth is visible" % serial
        d = TestDevice(serial).get_device()
        time.sleep(2)
        d.press.menu()
        d(text="Make discoverable").click.wait()
        time.sleep(2)
        d(text="Allow").click()
        assert_equals(d(text="Allow").exists, False)

    def scan_in_btchat(self, serial, btName):
        """Scan bluetooth device in bluetooth chat"""
        print "[INFO]: %s --- Scan %s in bluetooth chat" % (serial, btName)
        d = TestDevice(serial).get_device()
        time.sleep(2)
        d.press.menu()
        d(text="Connect a device").click.wait()
        time.sleep(4)
        d(text="Scan for devices").click()
        for i in range(10):
            if not d(text="scanning for devices...").exists:
                break
            time.sleep(5)
        assert_equals(d(textContains=btName).exists, True)

    def pair_in_btchat(self, serial, btName):
        """Pair bluetooth device in bluetooth chat"""
        print "[INFO]: %s --- Pair %s in bluetooth chat" % (serial, btName)
        d = TestDevice(serial).get_device()
        d(textContains=btName).click.wait()
        for i in range(5):
            if d(text="Bluetooth pairing request").exists:
                break
            time.sleep(2)
        assert_equals(d(text="Bluetooth pairing request").exists, True)

    def connect_in_btchat(self, serial, btName):
        """Connect to a paired device"""
        print "[INFO]: %s --- Connect to a paired device %s" % (serial, btName)
        d = TestDevice(serial).get_device()
        d.press.menu()
        d(text="Connect a device").click.wait()
        time.sleep(2)
        d(textContains=btName).click.wait()
        time.sleep(4)
        assert_equals(d(text="select a device to connect").exists, False)

    def check_connected_in_btchat(self, serial, btName, status=True):
        """Check connected/not in bluetooth chat"""
        info_tuple = (serial, btName, status)
        print "[INFO]: %s --- Check %s connected is %s" % info_tuple
        d = TestDevice(serial).get_device()
        time.sleep(5)
        if str(status) == "True":
            assert_equals(d(text="connected:%s" % btName).exists, True)
        elif str(status) == "False":
            assert_equals(d(text="not connected").exists, True)

    def send_character(self, serial, character):
        """Send character by bluetooth chat"""
        info_tuple = (serial, character)
        print "[INFO]: %s --- Send character %s by bluetooth chat" % info_tuple
        d = TestDevice(serial).get_device()
        while d(className="android.widget.EditText").text != "":
            d(className="android.widget.EditText").clear_text()
        d(className="android.widget.EditText").set_text(character)
        d(text="Send").click.wait()
        time.sleep(2)
        name = "android.widget.TextView"
        assert_equals(d(textContains=character, className=name).exists, True)

    def check_character(self, serial, character, status=True):
        """Check the character send/receive status"""
        info_tuple = (serial, character, status)
        print "[INFO]: %s --- Check character %s status is %s" % info_tuple
        d = TestDevice(serial).get_device()
        name = "android.widget.TextView"
        value = str(d(textContains=character, className=name).exists)
        assert_equals(value, str(status))

    def exit_btchat(self, serial):
        """Exit bluetooth chat app"""
        print "[INFO]: %s --- Exit bluetooth chat" % serial
        d = TestDevice(serial).get_device()
        for i in range(5):
            if not d(packageName="droideplace.BluetoothChat").exists:
                break
            d.press.back()
        time.sleep(2)
        assert_equals(d(packageName="droideplace.BluetoothChat").exists, False)

    def launch_bttest(self, serial):
        """Launch bluetooth test in CTS Verifier"""
        print "[INFO]: %s --- Launch bluetooth test in CTS Verifier" % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        d.press.home()
        g_device.launch_app_from_home_sc("CTS Verifier")
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text="Bluetooth Test")
        d(text="Bluetooth Test").click.wait()
        if d(text="Bluetooth Test", resourceId="android:id/alertTitle").exists:
            d(text="OK").click.wait()
        assert_equals(d(text="Toggle Bluetooth").exists, True)

    def scan_in_bttest(self, serial, btName):
        """Scan nearby device in CTS Verifier"""
        info_tuple = (serial, btName)
        print "[INFO]: %s --- Scan device %s in CTS Verifier" % info_tuple
        d = TestDevice(serial).get_device()
        for i in range(10):
            if not d(resourceId="android:id/progress_circular").exists:
                break
            time.sleep(5)
        time.sleep(2)
        d(text="Scan for Devices").click()
        time.sleep(10)
        name = "com.android.cts.verifier:id/bt_new_devices"
        if d(scrollable=True, resourceId=name).exists:
            d(scrollable=True, resourceId=name).scroll.to(textContains=btName)
        assert_equals(d(textContains=btName).exists, True)

    def check_message(self, serial, btName, ownerName, status=True):
        """Check the connected status"""
        print "[INFO]: %s --- Check connected is %s" % (serial, status)
        d = TestDevice(serial).get_device()
        flag1 = d(textMatches="Message.*to.%s" % btName).exists
        flag2 = d(textMatches="Message.*to %s" % ownerName).exists
        assert_equals(str(flag1 or flag2), str(status))

    def visible_in_bttest(self, serial):
        """Make device visible"""
        print "[INFO]: %s --- Make it visible" % serial
        d = TestDevice(serial).get_device()
        d(text="Make Discoverable").click()
        time.sleep(2)
        d(text="Allow").click()
        assert_equals(d(text="Allow").exists, False)

    def insecure_client(self, serial):
        """Open insecure client in CTS Verifier"""
        print "[INFO]: %s --- Open insecure client in CTS Verifier" % serial
        d = TestDevice(serial).get_device()
        d(text="Insecure Client").click.wait()
        time.sleep(2)
        assert_equals(d(text="Scan for Devices").exists, True)

    def insecure_server(self, serial):
        """Open insecure server in CTS Verifier"""
        print "[INFO]: %s --- Open insecure server in CTS Verifier" % serial
        d = TestDevice(serial).get_device()
        d(text="Insecure Server").click.wait()
        if d(text="Waiting for client...").exists:
            d(text="OK").click.wait()
        time.sleep(2)
        assert_equals(d(text="Make Discoverable").exists, True)

    def insecure_connect(self, serial, btName, ownerName):
        """Connect device in insecure method"""
        info_tuple = (serial, btName)
        print "[INFO]: %s --- Connect %s in insecure method" % info_tuple
        d = TestDevice(serial).get_device()
        name = "com.android.cts.verifier:id/bt_new_devices"
        if d(scrollable=True, resourceId=name).exists:
            d(scrollable=True, resourceId=name).scroll.to(textContains=btName)
        d(textContains=btName).click.wait()
        time.sleep(5)
        flag1 = d(textMatches="Message.*to %s" % btName).exists
        flag2 = d(textMatches="Message.*to %s" % ownerName).exists
        assert_equals(flag1 and flag2, True)

    def secure_client(self, serial):
        """Open secure client in CTS Verifier"""
        print "[INFO]: %s --- Open secure client in CTS Verifier" % serial
        d = TestDevice(serial).get_device()
        d(text="Secure Client").click.wait()
        time.sleep(2)
        assert_equals(d(text="Scan for Devices").exists, True)

    def secure_server(self, serial):
        """Open secure server in CTS Verifier"""
        print "[INFO]: %s --- Open secure server in CTS Verifier" % serial
        d = TestDevice(serial).get_device()
        d(text="Secure Server").click.wait()
        if d(text="Waiting for client...").exists:
            d(text="OK").click.wait()
        time.sleep(2)
        assert_equals(d(text="Make Discoverable").exists, True)

    def secure_connect(self, serial, btName):
        '''
        Click the Bluetooth device name to connect in a secure method.

        1. serial: The serial number of tested DUT.
        2. deviceName: The name of Bluetooth which will be connected.
        '''
        print "[INFO]: %s --- Connect %s in secure method" % (serial, btName)
        d = TestDevice(serial).get_device()
        name = "com.android.cts.verifier:id/bt_new_devices"
        if d(scrollable=True, resourceId=name).exists:
            d(scrollable=True, resourceId=name).scroll.to(textContains=btName)
        d(textContains=btName).click.wait()
        time.sleep(5)
        assert_equals(d(text="Bluetooth pairing request").exists, True)

    def launch_tethering(self, serial):
        """Launch bluetooth tethering"""
        print "[INFO]: %s --- Launch bluetooth tethering" % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        d.press.home()
        g_device.launch_app_from_home_sc("Settings")
        d(textStartsWith="More").click.wait()
        d(text="Bluetooth tethering").click.wait()
        assert_equals(d(text="Bluetooth tethering").exists, True)

    def relaunch_tethering(self, serial):
        """Relaunch bluetooth tethering"""
        print "[INFO]: %s --- Relaunch bluetooth tethering" % serial
        d = TestDevice(serial).get_device()
        d.press.back()
        time.sleep(2)
        d(text="Bluetooth tethering").click.wait()
        assert_equals(d(text="Bluetooth tethering").exists, True)

    def on_tethering(self, serial):
        """Turn on bluetooth tethering"""
        print "[INFO]: %s --- Turn on bluetooth tethering" % serial
        d = TestDevice(serial).get_device()
        flag = d(className="android.widget.Switch")
        if flag.info["text"] == "OFF":
            d(className="android.widget.Switch").click()
            time.sleep(2)
        value = d(className="android.widget.Switch").info["text"]
        assert_equals(value, "ON")

    def off_tethering(self, serial):
        """Turn off bluetooth tethering"""
        print "[INFO]: %s --- Turn off bluetooth tethering" % serial
        d = TestDevice(serial).get_device()
        flag = d(className="android.widget.Switch")
        if flag.info["text"] == "ON":
            d(className="android.widget.Switch").click()
            time.sleep(2)
        value = d(className="android.widget.Switch").info["text"]
        assert_equals(value, "OFF")

    def toggle_tethering(self, serial):
        """Toggle bluetooth tethering button"""
        print "[INFO]: %s --- Toggle bluetooth tethering" % serial
        d = TestDevice(serial).get_device()
        d(className="android.widget.Switch").click()
        time.sleep(2)

    def check_tethering(self, serial, status):
        """Check bluetooth tethering is on/off"""
        print "[INFO]: %s --- Check bt tethering is %s" % (serial, status)
        d = TestDevice(serial).get_device()
        flag = d(className="android.widget.Switch")
        assert_equals(flag.info["text"], status)

    def on_internet_access(self, serial):
        """Turn on internet access"""
        print "[INFO]: %s --- Turn on internet access" % serial
        d = TestDevice(serial).get_device()
        for i in range(5):
            name = "android.widget.CheckBox"
            flag = d(textStartsWith="Internet").left(className=name)
            if not flag.checked:
                flag.click()
                time.sleep(7)
            flag = d(textStartsWith="Internet").left(className=name)
            if flag.checked:
                break
        assert_equals(flag.checked, True)

    def off_internet_access(self, serial):
        """Turn off internet access"""
        print "[INFO]: %s --- Turn off internet access" % serial
        d = TestDevice(serial).get_device()
        for i in range(5):
            name = "android.widget.CheckBox"
            flag = d(textStartsWith="Internet").left(className=name)
            if flag.checked:
                flag.click()
                if d(textStartsWith="Disable profile").exists:
                    d(text="OK").click.wait()
                time.sleep(5)
            flag = d(textStartsWith="Internet").left(className=name)
            if not flag.checked:
                break
        assert_equals(flag.checked, False)

    def check_btpan_ip(self, serial, status=True):
        """Check bt-pan ip"""
        print "[INFO]: %s --- Check bt-pan ip is %s" % (serial, status)
        cmd = "adb -s %s shell netcfg | grep bt-pan" % serial
        lines = self.execute_cmd(cmd)
        result = False
        if len(lines) != 0:
            if lines[0].split()[2] != '0.0.0.0':
                result = True
        assert_equals(str(result), str(status))

    def check_btpan_ipfield(self, serial, other, status=True):
        """Check bt-pan ip field is same"""
        info_tuple = (serial, other, status)
        print "[INFO]: %s --- Check bt-pan ip field with %s is %s" % info_tuple
        cmd = "adb -s %s shell netcfg | grep bt-pan"
        host_ip = self.execute_cmd(cmd % serial)[0].split()[2]
        host_field = host_ip.split('.')[:3]
        client_ip = self.execute_cmd(cmd % other)[0].split()[2]
        client_field = client_ip.split('.')[:3]
        assert_equals(str(host_field == client_field), str(status))

    def clear_notice(self, serial):
        """Clear notification"""
        print "[INFO]: %s --- Clear notification" % serial
        d = TestDevice(serial).get_device()
        for i in range(4):
            name = "com.android.systemui:id/notification_stack_scroller"
            if d(resourceId=name).exists:
                d.press.back()
            else:
                break
        d.open.notification()
        if not d(resourceId=name).exists:
            d.click(1, 0)
            d.open.notification()
        print d(resourceId=name).exists
        time.sleep(3)
        target = d(resourceId="com.android.systemui:id/dismiss_text")
        for i in range(5):
            if target.exists:
                target.click.wait()
                break
            time.sleep(1)
        assert_equals(target.exists, False)

    def lanuch_ES(self, serial):
        """Launch ES File Explorer"""
        print "[INFO]: %s --- Launch ES File Explorer" % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        d.press.home()
        g_device.launch_app_from_home_sc("ES File Explorer")
        time.sleep(3)
        target = d(resourceId="com.estrongs.android.pop:id/long_press")
        if target.exists:
            for i in range(3):
                target.click.wait()
        time.sleep(3)
        assert_equals(d(text="Windows").exists, True)

    def send_file(self, serial, btName, fileName):
        """Send file by ES File Explorer"""
        info_tuple = (serial, fileName, btName)
        print "[INFO]: %s --- Send a file %s to %s via ES" % info_tuple
        d = TestDevice(serial).get_device()
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=fileName)
        d(text=fileName).drag.to(text=fileName, steps=50)

        time.sleep(2)
        d.press.menu()
        name = "com.estrongs.android.pop:id/button"
        d(text="Share", resourceId=name).click()
        time.sleep(2)
        d(resourceId="android:id/resolver_list").swipe.up(steps=15)
        time.sleep(2)
        d(text="Bluetooth", resourceId="android:id/text1").click.wait()
        time.sleep(5)
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=btName)
        d(text=btName).click.wait()

        self.clear_notice(serial)
        time.sleep(2)
        print "[INFO]: %s --- Check send info in notification" % serial
        d.open.quick_settings()
        time.sleep(6)
        target = d(text="Bluetooth share: Sending %s" % fileName)
        assert_equals(target.exists, True)

    def send_contact(self, serial, btName, contactName):
        """Send contact via bluetooth"""
        info_tuple = (serial, contactName, btName)
        print "[INFO]: %s --- Send contact %s to %s" % info_tuple
        d = TestDevice(serial).get_device()
        name = "com.android.contacts:id/cliv_name_textview"
        d(text=contactName, resourceId=name).click()
        d(description="More options").click()
        d(text="Share").click.wait()
        time.sleep(2)
        d(resourceId="android:id/resolver_list").swipe.up(steps=15)
        time.sleep(2)
        d(text="Bluetooth").click.wait()
        time.sleep(3)
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=btName)
        d(text=btName).click.wait()

        self.clear_notice(serial)
        time.sleep(2)
        print "[INFO]: %s --- Check send info in notification" % serial
        d.open.quick_settings()
        time.sleep(6)
        target = d(text="Bluetooth share: Sending %s.vcf" % contactName).exists
        assert_equals(target, True)

    def send_unknown_format(self, serial, btName, fileName):
        """Check unknow format file in notification"""
        print "[INFO]: %s --- Check unknow format file in notice" % serial
        d = TestDevice(serial).get_device()
        self.clear_notice(serial)
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=fileName)
        d(text=fileName).drag.to(text=fileName, steps=50)
        time.sleep(2)
        d.press.menu()
        name = "com.estrongs.android.pop:id/button"
        d(text="Share", resourceId=name).click()
        d(text="Bluetooth", resourceId="android:id/text1").click.wait()
        time.sleep(5)
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=btName)
        d(text=btName).click.wait()
        print "[INFO]: %s --- Check file info in notification" % serial
        d.open.quick_settings()
        time.sleep(5)
        d(text="Bluetooth share: Sent files").click.wait()
        assert_equals(d(text="Unknown file").exists, True)

    def receive(self, serial):
        """Recieve bluetooth file"""
        print "[INFO]: %s --- Recieve opp file" % serial
        d = TestDevice(serial).get_device()
        self.clear_notice(serial)
        time.sleep(2)
        print "[INFO]: %s --- Check file info in notification" % serial
        d.open.quick_settings()
        time.sleep(2)
        for i in range(4):
            if d(text="Bluetooth share: Incoming file").exists:
                break
            time.sleep(3)
        d(text="Bluetooth share: Incoming file").click.wait()
        time.sleep(2)
        d(text="Accept").click.wait()
        d.open.quick_settings()
        flag1 = d(text="Bluetooth share: Received files") is not None
        name1 = "android:id/progress"
        name2 = "com.android.bluetooth"
        flag2 = d(resourceId=name1, packageName=name2) is not None
        assert_equals(flag1 or flag2, True)

    def stop_transfer(self, serial, fileName):
        """Stop file transfer in notification"""
        print "[INFO]: %s --- Stop file transfer of %s" % (serial, fileName)
        d = TestDevice(serial).get_device()
        d.open.quick_settings()
        if d(text="Bluetooth share: Sending %s" % fileName).exists:
            d(text="Bluetooth share: Sending %s" % fileName).click.wait()
        elif d(textContains="Bluetooth share: Receiving").exists:
            d(textContains="Bluetooth share: Receiving").click.wait()
        d(text="Stop").click.wait()

    def reject_receive(self, serial):
        """Reject to receive a file"""
        print "[INFO]: %s --- Reject to receive a file" % serial
        d = TestDevice(serial).get_device()
        self.clear_notice(serial)
        time.sleep(2)
        d.open.quick_settings()
        time.sleep(2)
        for i in range(4):
            if d(text="Bluetooth share: Incoming file").exists:
                break
            time.sleep(3)
        d(text="Bluetooth share: Incoming file").click.wait()
        d(text="Decline").click.wait()
        d.open.quick_settings()
        time.sleep(2)
        assert_equals(d(textContains="0 successful").exists, True)

    def retry_send(self, serial, fileName):
        """Retry to send a failed file"""
        info_tuple = (serial, fileName)
        print "[INFO]: %s --- Retry to send a failed file %s " % info_tuple
        d = TestDevice(serial).get_device()
        d.open.quick_settings()
        time.sleep(2)
        d(text="Bluetooth share: Sent files").click.wait()
        d(text=fileName).click.wait()
        d(text="Try again").click()
        time.sleep(2)
        self.clear_notice(serial)
        print "[INFO]: %s --- Check file info in notification" % serial
        d.open.quick_settings()
        time.sleep(6)
        flag = d(text="Bluetooth share: Sending %s" % fileName).exists
        assert_equals(flag, True)

    def check_finish(self, serial):
        """Check send/receive success"""
        print "[INFO]: %s --- Check success to send/receive a file" % serial
        d = TestDevice(serial).get_device()
        d.open.quick_settings()
        time.sleep(2)
        for i in range(5):
            if d(textContains="1 successful").exists:
                break
            time.sleep(10)
        assert_equals(d(textContains="1 successful").exists, True)

    def check_connect_failed(self, serial, btName, fileName):
        """Check connection failed in notification"""
        print "[INFO]: %s --- Check connection failed in notification" % serial
        d = TestDevice(serial).get_device()
        self.clear_notice(serial)
        d.press.back()
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=fileName)
        d(text=fileName).drag.to(text=fileName, steps=50)
        time.sleep(2)
        d.press.menu()
        name = "com.estrongs.android.pop:id/button"
        d(text="Share", resourceId=name).click()
        if not d(text="Bluetooth").exists:
            d(resourceId="android:id/resolver_list").swipe.up(steps=15)
            time.sleep(2)
        d(text="Bluetooth", resourceId="android:id/text1").click.wait()
        time.sleep(5)
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=btName)
        d(text=btName).click.wait()
        print "[INFO]: %s --- Check file info in notification" % serial
        d.open.quick_settings()
        time.sleep(5)
        d(text="Bluetooth share: Sent files").click.wait()
        assert_equals(d(text="Connection unsuccessful.").exists, True)

    def check_send_failed(self, serial):
        """Check fail to send a file"""
        print "[INFO]: %s --- Check fail to send a file" % serial
        d = TestDevice(serial).get_device()
        d.open.quick_settings()
        time.sleep(2)
        for i in range(5):
            if d(text="Bluetooth share: Sent files").exists:
                break
            time.sleep(3)
        flag1 = d(textContains="1 unsuccessful").exists
        flag2 = d(text="Bluetooth share: Sent files").exists
        assert_equals(flag1 and flag2, True)

    def check_received_failed(self, serial):
        """Check fail to receive a file"""
        print "[INFO]: %s --- Check fail to receive a file" % serial
        d = TestDevice(serial).get_device()
        d.open.quick_settings()
        time.sleep(2)
        for i in range(5):
            if d(text="Bluetooth share: Received files").exists:
                break
            time.sleep(3)
        time.sleep(3)
        flag1 = d(text="Bluetooth share: Received files").exists
        flag2 = d(textContains="1 unsuccessful").exists
        assert_equals(flag1 and flag2, True)

    def check_opp_speed(self, serial, fileName):
        """Check the speed is right"""
        info_tuple = (serial, fileName)
        d = TestDevice(serial).get_device()
        measure_speed = False
        filesize = re.match(r'\d+', fileName).group(0)
        self.clear_notice(serial)
        print "[INFO]: %s --- Check the speed of %s is right" % info_tuple
        d.open.quick_settings()
        time.sleep(2)
        for i in range(4):
            if d(text="Bluetooth share: Incoming file").exists:
                break
            time.sleep(3)
        d(text="Bluetooth share: Incoming file").click.wait()
        d(text="Accept").click.wait()

        starttime = time.time()
        d.open.quick_settings()
        while True:
            flag1 = d(text="Bluetooth share: Received files").exists
            flag2 = d(textContains="1 successful").exists
            if flag1 and flag2:
                endtime = time.time()
                break
        stime = endtime - starttime
        speed = int(filesize) * 1024 / int(stime)
        if speed > 70 and speed < int(filesize)*1024:
            measure_speed = True
        assert_equals(measure_speed, True)

    def reboot_device_to_home_screen(self, serial):
        '''
        Reboot device and wait it enter into home screen.

        1. serial: the serial number of device.
        '''
        print "[INFO]: %s --- Reboot device and wait it enter into home screen" % serial
        g_device = TestDevice(serial)
        cmd = "adb -s %s reboot" % serial
        self.execute_no_end_cmd(cmd)
        print g_device.reboot_device()
        time.sleep(15)
        d = TestDevice(serial).get_device()
        self.setup_connection(serial)
        self.wakeup(serial)
        assert d(packageNameMatches="com.google.android.googlequicksearchbox").exists

    def wakeup(self, serial):
        '''
        Wake up the screen and unlock the lock.(It is better to set the screen to None)
        '''
        print "[INFO]: %s --- Unlock the screen" % serial
        d = TestDevice(serial).get_device()
        d.wakeup()

class WifiSettingImpl:
    """
    Implements Wifi Setting UI actions.

    """
    def __init__(self):
        pass

    def execute_no_end_cmd(self, command):
        child = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        time.sleep(20)
        child.kill()
        return child.stdout.readlines()

    def launch_wifi_from_settings(self, serial):
        '''
        Launch Settings and click the Wi-Fi button to enter into Wi-Fi Setting UI

        1. serial: The serial number of tested DUT.
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Enter into Wi-Fi UI" % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        d.press.home()
        g_device.launch_app_from_home_sc("Settings")
        d(textMatches="Wi.*Fi").click()
        assert_equals(d(description="More options").exists, True)

    def init_wifi_status(self, serial, status):
        '''
        Turn on/off wifi according to the expected status.
        If status is ON, wifi will be turned on. If status is OFF, wifi will be turned off.

        1. serial: The serial number of tested DUT.
        2. status: Expected wifi status. The value is ON or OFF.
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Initialize the wifi status to %s" % (serial, status)
        d = TestDevice(serial).get_device()
        if d(className="android.widget.Switch").info['text'] != str(status):
            d(className="android.widget.Switch").click()
            time.sleep(5)
        assert_equals(d(className="android.widget.Switch").text == str(status), True)

    def wifi_forget_ap_all(self, serial):
        '''
        Forget all the ap.

        1. serial: The serial number of tested DUT.
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Forget all the wifi ap" % serial
        d = TestDevice(serial).get_device()
        time.sleep(2)
        d(description="More options").click()
        if d(text="Saved networks").exists:
            d(text="Saved networks").click()
            number = d(resourceId="android:id/title").count
            while (number != 0):
                if d(resourceId="android:id/title").exists:
                    d(resourceId="android:id/title").click()
                    time.sleep(2)
                    d(text="Forget").click()
                    time.sleep(2)
                number = d(resourceId="android:id/title").count
        else:
            number = 0
        d.press.back()
        time.sleep(7)
        assert_equals(number == 0, True)

    def check_wifi_status(self, serial, status):
        '''
        Check wifi status is on or off.

        1. serial: The serial number of tested DUT.
        2. status: The expected status of wifi. The value is ON or OFF.
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Check the Wi-Fi status is %s" % (serial, status)
        d = TestDevice(serial).get_device()
        assert_equals(d(className="android.widget.Switch").text, status)

    def wifi_connect_to_ap(self, serial, ap, passwd):
        '''
        Connect to a Wi-Fi ap.

        1. serial: The serial number of tested DUT.
        2. ap: The name of Wi-Fi ap.
        4. passwd: Wi-Fi ap password.
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Start to connect to a Wi-Fi ap %s, password is %s" % (serial, ap, passwd)
        d = TestDevice(serial).get_device()
        time.sleep(7)
        d(text=ap).click()
        time.sleep(2)
        d(resourceId="com.android.settings:id/password").set_text(passwd)
        d(text="Connect").click()
        for i in range(10):
            if d(text=ap).down(text="Connected") is not None:
                break
            time.sleep(10)
        print "[INFO]: %s --- Finish to connect to a Wi-Fi ap %s" % (serial, ap)
        assert_equals(d(text=ap).down(text="Connected") is not None, True)

    def get_response_from_network(self, serial):
        '''
        Make sure the device can access to the Internet.

        1. serial: The serial number of tested DUT.
        '''
        print "[INFO]: %s --- Check if success to get response from 192.168.1.200" % serial
        cmd = "adb -s %s shell ping 192.168.1.200" % serial
        lines = self.execute_no_end_cmd(cmd)
        value = "@@@".join(lines)
        assert_equals(value.find("64 bytes from") != -1, True)

    def get_no_response_from_network(self, serial):
        '''
        Make sure the device cannot access to the Internet.

        1. serial: The serial number of tested DUT.
        '''
        print "[INFO]: %s --- Check if unable to get response from 192.168.1.200" % serial
        cmd = "adb -s %s shell ping 192.168.1.200" % serial
        lines = self.execute_no_end_cmd(cmd)
        assert_equals(lines[-1].find("64 bytes from") == -1, True)

class AirPlaneModeImpl:
    """
    Implements Airplane mode Setting UI actions.

    """
    def __init__(self):
        pass

    def launch_airplane_mode_from_settings(self, serial):
        '''
        Launch Settings and enter into Airplane mode Setting UI

        1. serial: The serial number of tested DUT.
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Launch airplane mode from Settings" % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        d.press.home()
        g_device.launch_app_from_home_sc("Settings")
        d(textStartsWith="More").click.wait()
        assert_equals(d(text="Airplane mode").exists, True)

    def init_airplane_mode_status(self, serial, status):
        '''
        Turn on/off Airplane mode according to the expected status.
        If status is ON, Airplane mode will be turned on. If status is OFF, Airplane mode will be turned off.

        1. serial: The serial number of tested DUT.
        2. status: Expected Airplane mode status. The value is ON or OFF (True or False).
        (Work on android-l image)
        '''
        value_dict = {"True": "ON", "False": "OFF"}
        print "[INFO]: %s --- Initialize airplane mode to %s" % (serial, status)
        d = TestDevice(serial).get_device()
        if str(status) in value_dict:
            newstatus = value_dict[str(status)]
        else:
            newstatus = str(status)
        if str(d(text="Airplane mode").right(className="android.widget.Switch").text) != newstatus:
            d(text="Airplane mode").right(className="android.widget.Switch").click()
        assert_equals(str(d(text="Airplane mode").right(className="android.widget.Switch").text), newstatus)

    def turn_on_airplane_mode(self, serial):
        '''
        Turn on Airplane mode.

        1. serial: The serial number of tested DUT.
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Turn on airplane mode" % serial
        d = TestDevice(serial).get_device()
        d(text="Airplane mode").right(className="android.widget.Switch").click()
        time.sleep(2)
        assert_equals(d(text="Airplane mode").right(className="android.widget.Switch").text, "ON")

    def turn_off_airplane_mode(self, serial):
        '''
        Turn off Airplane mode.

        1. serial: The serial number of tested DUT.
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Turn off airplane mode" % serial
        d = TestDevice(serial).get_device()
        d(text="Airplane mode").right(className="android.widget.Switch").click()
        time.sleep(2)
        assert_equals(d(text="Airplane mode").right(className="android.widget.Switch").text, "OFF")

    def check_airplane_mode_status(self, serial, status):
        '''
        Check Airplane mode is ON or OFF.

        1. serial: The serial number of tested DUT.
        2. status: The expected status of Bluetooth. The value is ON or OFF (True or False).
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Check airplane mode is %s" % (serial, status)
        d = TestDevice(serial).get_device()
        value_dict = {"True": "ON", "False": "OFF"}
        if str(status) in value_dict:
            newstatus = value_dict[str(status)]
        else:
            newstatus = str(status)
        assert_equals(d(text="Airplane mode").right(className="android.widget.Switch").text, newstatus)

class ChromeImpl:
    """
    Implements chrome UI actions.

    """
    def __init__(self):
        self.chrome_pkg_name = "com.ksmobile.cb"
        self.chrome_activity_name = ".Main"

    def launch_chrome_by_am(self, serial):
        '''
        Launch Chrome by am command

        1. serial: the serial number of device
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Launch Chrome by am command" % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        d.press.home()
        g_device.launch_app_from_home_sc("CM Browser")
        if d(resourceId="com.ksmobile.cb:id/address_bar_search_logo").exists:
            d.press.back()
        assert_equals(d(packageName="com.ksmobile.cb").exists, True)

    def chrome_input_url(self, serial, url):
        '''
        Open website in Chrome.

        1. serial: the serial number of device
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Open url %s in Chrome" % (serial, url)
        d = TestDevice(serial).get_device()
        d(resourceId="com.ksmobile.cb:id/address_bar_hint").set_text(url)
        d.press("enter")
        time.sleep(10)

    def chrome_close_all_tabs(self, serial):
        '''
        Close all the tabs in Chrome.

        1. serial: the serial number of device
        (Work on android-l image)
        '''
        pass

    def chrome_open_new_tab(self, serial):
        '''
        Open a tab in Chrome. (Work with chrome_close_all_tabs)

        1. serial: the serial number of device
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Clear the input history" % serial
        d = TestDevice(serial).get_device()
        d(resourceId="com.ksmobile.cb:id/toolbar_home").click()
        assert_equals(d(text="Search or type a URL").exists, True)

    def check_wepage_open_status(self, serial, title, status=True):
        '''
        Check the status of web page in Chrome.

        1. serial: the serial number of device
        (Work on android-l image)
        '''
        print "[INFO]: %s --- Check if the status of webpage %s is %s" % (serial, title, status)
        d = TestDevice(serial).get_device()
        for i in range(5):
            if d(resourceId='com.ksmobile.cb:id/stop_refresh_btn').exists:
                break
            time.sleep(4)
        assert_equals(str(d(text=title, resourceId='com.ksmobile.cb:id/address_bar_hint').exists), str(status))

class ContactImpl:
    """
    Implements Contact Setting UI actions.

    """

    def launch_contact(self, serial):
        '''
        Launch People.
        1. serial: The serial number of tested DUT.
        '''
        print "[Info] %s ---Launch People." % serial
        d = TestDevice(serial).get_device()
        g_device = TestDevice(serial)
        d.press.home()
        g_device.launch_app_from_home_sc("Contacts")

    def delete_contact(self, serial, contactName):
        '''
        Delete a contact in app People.
        1. serial: The serial number of tested DUT.
        2. contactName: The contact name to be deleted.
        '''
        print "[Info] %s ---Delete a contact in app People." % serial
        d = TestDevice(serial).get_device()
        d(text="All contacts").click.wait()
        time.sleep(2)
        for i in range(10):
            if not d(text=contactName, resourceId="com.android.contacts:id/cliv_name_textview").exists:
                break
            d(text=contactName).click()
            d(resourceId="com.android.contacts:id/menu_edit").click()
            time.sleep(2)
            d.press.menu()
            d(text="Delete").click.wait()
            d(text="OK").click.wait()
        assert_equals(d(text=contactName, resourceId="com.android.contacts:id/cliv_name_textview").exists, False)

    def create_contact(self, serial, contactName):
        '''
        Create a contact in app People.
        1. serial: The serial number of tested DUT.
        2. contactName: The contact name to be created.
        '''
        print "[Info] %s ---Create a contact in app People." % serial
        d = TestDevice(serial).get_device()
        d(text="All contacts").click.wait()
        time.sleep(2)
        d(description="add new contact").click.wait()
        d(text="Name").set_text(contactName)
        d(description="Done").click.wait()
        if d(resourceId="com.android.contacts:id/menu_edit").exists:
            d.press.back()
            time.sleep(2)
        assert_equals(d(text=contactName, resourceId="com.android.contacts:id/cliv_name_textview").exists, True)

    def import_contact(self, serial, contactName):
        '''
        Import a contact in app People.
        1. serial: The serial number of tested DUT.
        2. contactName: The contact name to be imported.
        '''
        print "[Info] %s ---Import a contact in app People." % serial
        d = TestDevice(serial).get_device()
        d(description="More options").click.wait()
        d(textStartsWith="Import").click.wait()
        d(text="Import from storage").click.wait()
        time.sleep(4)
        if d(text="Choose vCard file").exists:
            d(text="Import one vCard file").click.wait()
            d(text="OK").click.wait()
            time.sleep(2)
            d(textStartsWith=contactName, resourceId="android:id/text1").click.wait()
            d(text="OK").click.wait()
            time.sleep(5)
        assert_equals(d(text=contactName, resourceId="com.android.contacts:id/cliv_name_textview").exists, True)
