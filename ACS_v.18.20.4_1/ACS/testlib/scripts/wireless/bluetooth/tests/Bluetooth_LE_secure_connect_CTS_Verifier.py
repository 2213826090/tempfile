#!/usr/bin/env python

#######################################################################
#
# @description: scenarios for SecureConnect via CTS app
# @usage:       Bluetooth_LE_secure_connect_CTS_Verifier.py
#               scan_timeout=60000 scan_max_attempts=1 timeout_time=60000
# @author:      narasimha.rao.rayala@intel.com
#
#
#######################################################################
import sys
import re
import time
import os.path
import subprocess

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps, bt_utils
from testlib.scripts.wireless.bluetooth.bt_step import Step as BtStep
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils


class OpenCtsVerifier(BtStep):
    """ Description:
            Opens the activity from ctsverifierapp, either from all
            apps menu, or by sending an intent. Call this from the Home
            screen if use_intent=False
        Usage:
            OpenCtsVerifer(serial=serial, use_intent=False, version=version)()
    """
    def __init__(self, use_intent=False, **kwargs):
        """
        :param use_intent: True to open from the home screen, False to use ctsverifier launch intent
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.use_intent = use_intent
        self.step_data = True
        # part of logging message
        if self.version.startswith("8.1"):
            self.use_intent = True
            self.message_str = "with intent"
        else:
            self.use_intent = use_intent
            self.message_str = "from menu"
            self.set_passm("ctsverifier  opened " + self.message_str)

    def do(self):
        try:
            if self.use_intent:
                cmd_launch_cts_verifier = "shell am start -n com.android.cts.verifier/com.android.cts.verifier.CtsVerifierActivity"
                self.adb_connection.cmd(cmd_launch_cts_verifier).wait()
            else:
                self.uidevice.press.home()
                if self.uidevice(description="Apps").wait.exists(timeout=self.timeout):
                    self.uidevice(description="Apps").click()
                elif self.uidevice(className="android.widget.ImageView",resourceId="com.android.launcher3:id/all_apps_handle").wait.exists(timeout=self.timeout):
                   self.uidevice(className="android.widget.ImageView",resourceId="com.android.launcher3:id/all_apps_handle").click()
                else:
                    raise Exception("All apps menu was not opened")
                if not self.uidevice(text="CTS Verifier").wait.exists(timeout=self.timeout):
                    raise Exception("CTS Verifier app not found in All apps menu")
                self.uidevice(text="CTS Verifier").click()
                if not self.uidevice(packageName="com.android.cts.verifier").wait.exists(timeout=self.timeout):
                    raise Exception("CTS Verifier app not opened")
        except Exception, e:
            self.set_errorm("Open " + self.message_str, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if CTS app list was launched, False if not
        """
        if not self.step_data:
            self.set_errorm("Open " + self.message_str, "CTS app was not opened")
            # wait for the CTS activity to open
            self.step_data = self.uidevice(text="CTS Verifier").wait.exists(timeout=self.timeout)
        return self.step_data


class CtsVerifierSecureClient(BtStep):
    """ Description:
            Makes sure that the CTS scanning progress is finished.
        Usage:
            CtsVerifierSecureClient(serial=serial,
                                timeout_appear=5000, time_to_wait=60000, version=version)()
    """

    def __init__(self, timeout_appear=5000, time_to_wait=60000, **kwargs):
        """
        :param timeout_appear: time to wait till the scanning progress bar appears
        :param time_to_wait: time to wait till the scanning progress bar is gone
        :param kwargs: serial, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.timeout_appear = timeout_appear
        self.time_to_wait = time_to_wait
        self.list = self.uidevice(packageName="com.android.cts.verifier")
        self.step_data = True
        self.set_passm("ctsverifier SecureClient scanning ")

    def do(self):
        try:
            if not self.list.wait.exists(timeout=self.timeout_appear):
                raise Exception("cts objects list was not found")
            # scroll here to reveal scanning progressbar
            if not self.list.scroll.to(text="Bluetooth Test"):
                raise Exception("NETWORKING/Bluetooth Test title was not found in BT list")
            self.uidevice(text="Bluetooth Test").click()
            if self.uidevice(text="OK").wait.exists(timeout=self.timeout):
                self.uidevice(text="OK").click()
            if not self.uidevice(text="Secure Client").wait.exists(timeout=self.timeout):
                raise Exception("Secure Client Test object not found in menu list")
            self.uidevice(text="Secure Client").click()
            #if not self.uidevice(text="Scan for Devices").wait.exists(timeout=self.timeout):
            #    raise Exception("Scan for Devices object not found in menu list")
            #self.uidevice(text="Scan for Devices").click()
            if not ui_steps.click_button_common(serial=self.serial,
                                                view_to_find={"textContains": "Scan for Devices"},scroll=False)():
                raise Exception("Scan for Devices object not found in menu list")
        except Exception, e:
            self.set_errorm(" CTS Verifier scanning error", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if scanning progress was finished after timeout reached, False if not
        """
        return self.step_data

class CtsVerifierSecureServer(BtStep):
    """ Description:
            Makes sure that the CTS app in discoverable mode.
        Usage:
            CtsVerifierSecureServer(serial=serial,
                                 timeout_appear=5000, time_to_wait=60000, version=version)()
    """

    def __init__(self, timeout_appear=5000, time_to_wait=60000, **kwargs):
        """
          :param timeout_appear: time to wait till the scanning progress bar appears
          :param time_to_wait: time to wait till the scanning progress bar is gone
          :param kwargs: serial, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.timeout_appear = timeout_appear
        self.time_to_wait = time_to_wait
        self.list = self.uidevice(packageName="com.android.cts.verifier")
        self.step_data = True
        self.set_passm("ctsverifier SecureServer in Discoverable mode ")

    def do(self):
        try:
            if not self.list.wait.exists(timeout=self.timeout_appear):
                raise Exception("cts objects list was not found")
            if not self.list.scroll.to(text="Bluetooth Test"):
                raise Exception("NETWORKING/Bluetooth Test title was not found in BT list")
            self.uidevice(text="Bluetooth Test").click()
            if self.uidevice(text="OK").wait.exists(timeout=self.timeout):
                self.uidevice(text="OK").click()
            if not self.uidevice(text="Secure Server").wait.exists(timeout=self.timeout):
                raise Exception("Secure Server object not found in menu list")
            self.uidevice(text="Secure Server").click()
            if self.uidevice(text="OK").wait.exists(timeout=self.timeout):
                self.uidevice(text="OK").click()
            if not self.uidevice(text="MAKE DISCOVERABLE").wait.exists(timeout=self.timeout):
                raise Exception("MAKE DISCOVERABLE object not found in menu list")
            self.uidevice(text="MAKE DISCOVERABLE").click()
            if self.uidevice(text="ALLOW").wait.exists(timeout=self.timeout):
                self.uidevice(text="ALLOW").click()
        except Exception, e:
            self.set_errorm("Not in discoverable mode", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if device in discoverable mode, False if not
        """
        return self.step_data


class CtsVerifierSecureConnect(BtStep):
    """ Description:
            Makes sure that the CTS connected to dev.
        Usage:
            CtsVerifiersecureConnect(serial=serial, dev_name=PAIRING_DEV_NAME)()
    """
    def __init__(self,serial_dev, dev_name, timeout_appear=5000, time_to_wait=60000, **kwargs):
        """
        :param timeout_appear: time to wait till the scanning progress bar appears
        :param time_to_wait: time to wait till the scanning progress bar is gone
        :param kwargs: serial, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.timeout_appear = timeout_appear
        self.time_to_wait = time_to_wait
        self.list = self.uidevice(packageName="com.android.cts.verifier")
        self.step_data = True
        self.dev_name=dev_name
        self.serial_dev=serial_dev
        self.set_passm("ctsverifier InsecureClient connected to " + self.dev_name)
        # initiator_serial=dut_name
        # self.serial=serial

    def do(self):
        try:
            if not ui_steps.click_button_common(serial=self.serial,
                                                view_to_find={"textContains": "Scan for Devices"}, scroll=False)():
                raise Exception("Scan for Devices object not found in menu list")
            if ui_steps.wait_for_view_common(serial=self.serial, view_to_find={"textContains": self.dev_name })():
                ui_steps.click_button_common(serial=self.serial,view_to_find={"textContains": self.dev_name},
                                             view_to_check={"resourceId":"android:id/alertTitle"})()
                time.sleep(1)
                if not bluetooth_steps.PerformActionPairRequest(serial=self.serial, action="Pair")():
                    raise Exception("Pair button not found in DUT list")
                if not bluetooth_steps.PerformActionPairRequest(serial=self.serial_dev, action="Pair")():
                    raise Exception("Pair button not found in DEV list")
                time.sleep(10)
            else:
                self.step_data=False
        except Exception, e:
            self.set_errorm("CTS Verifier connection error", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if connection was finished, False if not
        """
        return self.step_data


# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
if not script_args:
    raise Exception("--script-args is mandatory")
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
# ### mandatory parameters ###

if "serial2" not in args.keys():
    raise Exception("serial2 parameter is mandatory")
serial_dev = args["serial2"]
#########
# ### optional parameters ###
# default values
action_dut = "Pair"
initiator = "DUT"
action_initiator_first = True
scan_timeout = 60000
scan_max_attempts = 1
timeout_time = 60000
# possible values for optional parameters
action_values = ["Pair"]
initiator_values = ["dut"]
true_values = ["true", "t", "1", "yes", "y"]

# parse optional parameters
if "action_dut" in args.keys():
    if args["action_dut"] in action_values:
        action_dut = args["action_dut"]
    else:
        raise Exception("Possible values for action_dut: " + str(action_values))
if "initiator" in args.keys():
    if args["initiator"].lower() in initiator_values:
        initiator = args["initiator"]
    else:
        raise Exception("Possible values for initiator: " + str(initiator_values))
if "action_initiator_first" in args.keys():
    if args["action_initiator_first"].lower() in true_values:
        action_initiator_first = True
    else:
        action_initiator_first = False
if "scan_timeout" in args.keys():
    scan_timeout = int(args["scan_timeout"])
if "scan_max_attempts" in args.keys():
    scan_max_attempts = int(args["scan_max_attempts"])
if "timeout_time" in args.keys():
    timeout_time = int(args["timeout_time"])
bluetooth_steps.LogInfo("##### INITIALIZE ######")()
# Initialize versions and names
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev, blocking=True)()
DUT_NAME = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()
PAIRING_DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev, blocking=True)()
try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    # DUT: turn on BT
    bluetooth_steps.StopPackage(serial=serial, critical=False, package_name="com.android.cts.verifier")()
    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, blocking=True)()

    # DEV: turn on BT
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False, package_name="com.android.cts.verifier")()
    bluetooth_steps.StopPackage(serial=serial_dev, blocking=True)()
    bluetooth_steps.PressHome(serial=serial_dev, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, blocking=True)()

    # DUT: wait scanning, rename device and remove all paired devices
    bluetooth_steps.WaitBtScanning(serial=serial, scan_timeout=scan_timeout, version=DUT_VERSION, blocking=True)()
    #bluetooth_steps.BtChangeDeviceName(serial=serial,name=DUT_NAME, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # DEV: wait scanning(should be already finished), rename device and remove all paired devices
    bluetooth_steps.WaitBtScanning(serial=serial_dev, timeout_appear=0, scan_timeout=scan_timeout, version=DEV_VERSION,
                                   blocking=True)()
    #bluetooth_steps.BtChangeDeviceName(serial=serial_dev,name=PAIRING_DEV_NAME, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # ############ Actual Test ################
    # #########################################
    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()
    OpenCtsVerifier(serial=serial_dev)()
    OpenCtsVerifier(serial=serial)()
    CtsVerifierSecureClient(serial=serial,version=DUT_VERSION)()
    CtsVerifierSecureServer(serial=serial_dev, version=DEV_VERSION)()
    CtsVerifierSecureConnect(serial=serial,serial_dev=serial_dev, dev_name=PAIRING_DEV_NAME)()
finally:

# ########### Postconditions ##############
# #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()


# DUT: stop settings and turn on BT (if not already)

    bluetooth_steps.StopPackage(serial=serial, critical=False,package_name="com.android.cts.verifier")()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, critical=False)()

    # DEV: stop settings and turn on BT (if not already)

    bluetooth_steps.StopPackage(serial=serial_dev, critical=False, package_name="com.android.cts.verifier")()
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, critical=False)()

    # DUT: remove all paired devices and turn off BT
    bluetooth_steps.WaitBtScanning(serial=serial, scan_timeout=scan_timeout, version=DUT_VERSION, critical=False)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()

    # DEV: remove all paired devices and turn off BT
    bluetooth_steps.WaitBtScanning(serial=serial_dev, timeout_appear=0, scan_timeout=scan_timeout, version=DEV_VERSION,
                                   critical=False)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="OFF", version=DEV_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()

                                                                                                                                
