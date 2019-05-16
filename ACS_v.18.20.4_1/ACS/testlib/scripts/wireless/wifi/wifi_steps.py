#!/usr/bin/env python

#######################################################################
#
# @filename:    wifi_steps.py
# @description: WiFi test steps
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.wireless.wifi.wifi_step import step as wifi_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.utils.ui.uiandroid import UIDevice as ui_device
from testlib.scripts.android.ui.browser import browser_steps
from testlib.scripts.android.ui.browser import browser_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.base.base_step import step as base_step
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.utils.connections.local import Local as connection_local
from testlib.scripts.android.logcat import logcat_steps

from testlib.utils.statics.android import statics
from testlib.utils.defaults import wifi_defaults
from testlib.external import uiautomator

from multiprocessing import Process
import time
import os
import subprocess
from subprocess import *
import re


class set_airplane_mode(wifi_step):
    """ description:
        Enables / disabled Airplane mode from settings
        Possible states:
            ON --> enable
            OFF --> disable

        usage:
            wifi_steps.activate_airplane_mode(state = "ON")()

        tags:
            ui, android, settings, airplane
    """

    def __init__(self, state = "ON", **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.state = state
        self.ro_name = adb_utils.get_product_name(serial=self.serial)
        self.set_passm(self.state)
        self.set_errorm("", self.state)

    def do(self):
        if self.ro_name != "bxtp_abl_car":
            ui_steps.open_settings(serial = self.serial)()
        else:
            ui_steps.open_settings(serial=self.serial, intent=True)()
        if self.device_info.dessert == "O":
            ui_steps.click_button_with_scroll(
                view_to_find={"textContains": "Network & Internet"},
                view_to_check={"text": "Airplane mode"}, serial=self.serial)()
        else:
            ui_steps.click_button_with_scroll(view_to_find={"textContains":"More"},
                 view_to_check={"text":"Airplane mode"}, serial=self.serial)()

        actual_state = self.uidevice(**self.device_info.airplane_mode_switch_id).info["text"]

        if self.state == "ON" and actual_state == "OFF":
                ui_steps.click_switch(
                        view_to_find = self.device_info.airplane_mode_switch_id, serial = self.serial)()
        elif self.state == "OFF" and actual_state == "ON":
                ui_steps.click_switch( state = "OFF",
                        view_to_find = self.device_info.airplane_mode_switch_id, serial = self.serial)()

    def check_condition(self):
        #Sometimes at this point, "TypeError: UIDevice object got multiple values for keyword argument 'text'" always occurs
        self.uidevice(resourceId = self.device_info.airplane_mode_switch_id["resourceId"], text = self.state).wait.exists(timeout=2000)
        actual_state = self.uidevice(**self.device_info.airplane_mode_switch_id).info["text"]
        return self.state == actual_state


class set_am_state(wifi_step):
    """ description:
        uses the commands below to turn airplane mode on/off

            settings put global airplane_mode_on 1
            am broadcast -a android.intent.action.AIRPLANE_MODE --ez state true

            settings put global airplane_mode_on 0
            am broadcast -a android.intent.action.AIRPLANE_MODE --ez state false

        usage:
            set_am_state(state='on', serial = serial)()
    """
    def __init__(self, state = 'on', wait_time = 15, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.state = state
        self.wait_time = wait_time

    def do(self):
        if self.state.lower() == 'on':
            adb_steps.command(serial = self.serial, command = "settings put global airplane_mode_on 1", timeout=self.wait_time)()
            adb_steps.command(serial = self.serial, command = "am broadcast -a android.intent.action.AIRPLANE_MODE --ez state true", stdout_grep='result=0', timeout=self.wait_time)()
        elif self.state.lower() == 'off':
            adb_steps.command(serial = self.serial, command = "settings put global airplane_mode_on 0", timeout=self.wait_time)()
            adb_steps.command(serial = self.serial, command = "am broadcast -a android.intent.action.AIRPLANE_MODE --ez state false", stdout_grep='result=0', timeout=self.wait_time)()

    def check_condition(self):
        if self.state.lower() == 'on':
            return wifi_utils.check_airplane_mode_on(serial = self.serial) and adb_steps.command(serial = self.serial,
                command = "settings get global airplane_mode_on", stdout_grep='1', timeout=self.wait_time)()
        elif self.state.lower() == 'off':
            return ( not wifi_utils.check_airplane_mode_on(serial = self.serial) ) and adb_steps.command(serial = self.serial,
                command = "settings get global airplane_mode_on", stdout_grep='0', timeout=self.wait_time)()

class set_wifi_frequency_band(wifi_step):
    """ description:
        sets the preferred frequency band for DUT

        usage:
            set_wifi_frequency_band(frequency_band=5)()

            frequency_band = 2 / 5 / auto

        optional parameters:
            wait_time -> the amount of time to wait for certain operations
            verify_dumpsys -> checks if the scanned APs list complies with the applied setting
    """
    def __init__(self, frequency_band = 'auto', verify_dumpsys = True, wait_time = 5, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.frequency_band = frequency_band
        self.verify_dumpsys = verify_dumpsys
        self.wait_time = int(wait_time)
        if self.frequency_band == '2':
            self.text = "2.4 G"
        elif self.frequency_band == '5':
            self.text = "5 G"
        elif self.frequency_band == 'auto':
            self.text = "Automatic"

    def do(self):
        if self.frequency_band == '2':
            button_to_click = {"textContains": self.text}
        elif self.frequency_band == '5':
            button_to_click = {"textContains": self.text}
        elif self.frequency_band == 'auto':
            button_to_click = {"textContains": self.text}

        open_wifi_settings(serial = self.serial)()
        # Click "More"
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"descriptionContains": "More"},
            view_to_check = {"textContains": "Advanced"})()
        # Click "Advanced"
        ui_steps.click_button(serial =self.serial,
            view_to_find = {"textContains": "Advanced"},
            view_to_check = {"textContains": u"Advanced Wi\u2011Fi"})()
        # Click "wi-fi frequency band" if available ; option appeared in l-dessert
        self.set_errorm("", "Missing GUI option: DUT does not support 5GHz frequency band / old android version")
        ui_steps.click_button(serial =self.serial,
            view_to_find = {"textContains": u"Wi\u2011Fi frequency band"},
            view_to_check = {"textContains": u"Wi\u2011Fi frequency band", "resourceId":"android:id/alertTitle"})()
        ui_steps.click_button(serial =self.serial,
            view_to_find = button_to_click,
            view_to_check = {"textContains": u"Wi\u2011Fi frequency band", "resourceId":"android:id/title"})()
        ui_steps.click_button(serial =self.serial,
            view_to_find = {"textContains": u"Wi\u2011Fi frequency band"},
            view_to_check = {"textContains": u"Wi\u2011Fi frequency band", "resourceId":"android:id/alertTitle"})()
        ui_steps.click_button(serial =self.serial,
            view_to_find = button_to_click,
            view_to_check = {"textContains": u"Wi\u2011Fi frequency band", "resourceId":"android:id/title"})()

    def check_condition(self):
        #check if the GUI setting updated
        ret_value = ui_steps.wait_for_view(view_to_find = {"textContains":self.text, "resourceId":"android:id/summary"}, timeout = self.wait_time,
                          serial = self.serial)()
        if not ret_value:
            self.set_errorm("", " The setting did not update in the GUI ")
            return ret_value
        #check if the setting was applied
        if self.verify_dumpsys:
            #going back to wifi settings to increase the scan rate
            ui_steps.press_back(serial =self.serial, view_to_check = {"text": u"Wi\u2011Fi"})()
            return check_scanned_ssid_frequencies(serial = self.serial, frequency_band = self.frequency_band, wait_time = self.wait_time)()
        return ret_value

class check_scanned_ssid_frequencies(wifi_step):
    """ description:
        checks if scanned aps belong to specified frequency band

        usage:
            check_scanned_ssid_frequencies(frequency_band = 2, serial = serial)()
            band(s) :   2 - only 2.4 GHz
                        5 - only 5 GHz
                        auto - 2.4 & 5 GHz
        tags:
            wifi, adb, dumpsys
    """
    def __init__(self, frequency_band = 'auto', wait_time = 5, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.frequency_band = frequency_band
        self.wait_time = wait_time

    def do(self):
        pass

    def check_condition(self):
        for counter in range(max(self.wait_time,40)):
            time.sleep(1)
            # check if any 2.4 GHz APs were scanned
            output_2 = self.adb_connection.parse_cmd_output(cmd = "dumpsys wifi", grep_for = ['  2412  ','  2417  ','  2422  ','  2427  ','  2432  ','  2437  ','  2442  ','  2447  ','  2452  ','  2457  ','  2462  ','  2467  ','  2472  '], multiple_grep = "OR", strip = False, dont_split=True, timeout = 60)
            # check if any 5 GHz APs were scanned
            output_5 = self.adb_connection.parse_cmd_output(cmd = "dumpsys wifi", grep_for = ['  5180  ','  5200  ','  5220  ','  5240  ','  5260  ','  5280  ','  5300  ','  5320  ','  5500  ','  5520  ','  5540  ','  5560  ','  5580  ','  5660  ','  5680  ','  5700  ','  5720  ','  5745  ','  5765  ','  5785  ','  5805  ','  5825  '], multiple_grep = "OR", strip = False, dont_split=True, timeout = 60)
            if self.frequency_band == '2':
                if (output_2 <> "") and (output_5 == ""):
                    return True
                elif output_2 == "":
                    self.set_errorm("","no 2.4 GHz APs were found - after setting band to show ONLY 2.4 GHz:\n'"+output_2+"'")
                elif output_5 <> "":
                    self.set_errorm("","5 GHz APs were found - after setting band to show ONLY 2.4 GHz:\n'"+output_5+"'")
            elif self.frequency_band == '5':
                if (output_5 <> "") and (output_2 == ""):
                    return True
                elif output_5 == "":
                    self.set_errorm("","no 5 GHz APs were found - after setting band to show ONLY 5 GHz:\n'"+output_5+"'")
                elif output_2 <> "":
                    self.set_errorm("","2.4 GHz APs were found - after setting band to show ONLY 5 GHz:\n'"+output_2+"'")
            elif self.frequency_band == 'auto':
                if (output_5 <> "") and (output_2 <> ""):
                    return True
                elif output_5 == "":
                    self.set_errorm("","no 5 GHz APs were found - after setting band to show ONLY 5 GHz")
                elif output_2 == "":
                    self.set_errorm("","2.4 GHz APs were found - after setting band to show ONLY 5 GHz")
        return False

class check_file_integrity(wifi_step):
    """ description:
        Copies a specified file from the DUT and checks its integrity
        it can verify bitwise integrity or md5sum integrity, compared to the original file accessed through the browser

        usage:
            wifi_steps.check_file_integrity(mode = "cmp", local_file = file_name, remote_file = device_file, serial = serial)()

        tags:
            wifi, browser, download, adb
    """
    rename_sufix = "_from_device_"
    def __init__(self, local_file, remote_file, mode = "md5", **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.local_file = local_file
        self.remote_file = remote_file
        self.mode = mode

    def do(self):
        adb_steps.pull_file(serial = self.serial, local = self.local_file + self.rename_sufix + self.serial, remote = self.remote_file, timeout = 120)()

    def check_condition(self):
        pass_message = "Files match!"
        fail_message = "Files do not match!"
        if self.mode == "md5":
            status = wifi_utils.check_file_md5(self.local_file, remote_file = self.local_file + self.rename_sufix + self.serial)
        elif self.mode == "cmp":
            status = wifi_utils.check_file_cmp(self.local_file, remote_file = self.local_file + self.rename_sufix + self.serial)
        else:
            fail_message2 =  "Invalid compare mode"
            self.set_errorm("", fail_message2)
            return False
        if status:
            self.step_data = True
            self.set_passm(pass_message)
            return True
        else:
            self.step_data = False
            self.set_errorm("", fail_message)
            return False

class download_file(wifi_step):
    """ description:
        Downloads a file to the DUT, from a given URL, using transfer over http or ftp.
        The check condition passes if the downloaded file size equals the expected one
    """
    def __init__(self, url, file_name, file_size, protocol,
                 check_file_downloaded=True, option=None ,negativetest=False, ap_name=None, interface5ghz="0" ,serial2=None, url_title="", **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.file_name = file_name
        self.file_size = file_size
        self.url = url
        self.url_title = url_title
        self.protocol = protocol
        self.serial2 = serial2
        self.check_file_downloaded = check_file_downloaded
        self.option = option
        self.negativetest = negativetest
        self.ap_name = ap_name
        self.interface5ghz = interface5ghz

    def do(self):
        adb_steps.command(serial = self.serial, command = "rm {}".format(self.device_info.download_path) + \
                                                          self.file_name + "*", ignore_error = True)()
        self.adb_connection.parse_cmd_output("pm clear com.android.chrome",
                    dont_split=True)
        browser_steps.open_chrome_first_time(serial = self.serial,
                                     intent = True,
                                     url_to_open = '127.0.0.1')()

        adb_steps.command(command="pm grant com.android.chrome android.permission.READ_EXTERNAL_STORAGE", serial=self.serial)()
        adb_steps.command(command="pm grant com.android.chrome android.permission.WRITE_EXTERNAL_STORAGE", serial=self.serial)()
        adb_steps.command(command="am start"
                    " -a android.intent.action.VIEW"
                    " -c android.intent.category.BROWSABLE"
                    " -n com.android.chrome/com.google.android.apps.chrome.Main"
                    " -d " + self.url, serial=self.serial)()

        if self.uidevice(textContains="Do you want to replace the existing").wait.exists(timeout=1000):
            ui_steps.click_button(serial = self.serial,
                                view_to_find = {"text": "Replace file"})()

        wifi_conf = self.adb_connection.parse_cmd_output("dumpsys wifi")
        self.wifi_info = wifi_utils.get_connection_info(wifi_conf)

    def check_condition(self):
        #if self.check_file_downloaded == True:
        time.sleep(10) #waiting for download to begin; sometimes the file is created late and file size check fails, breaking the test
        if self.protocol == "http":
            suffix = ""
        elif self.protocol == "ftp":
            suffix = "*"
        if self.check_file_downloaded:
            """Turn off ap and check for downloading status"""
            if (self.option == "diable_ap") | (self.option == "wifi_scan"):
              open_wifi_settings(serial=self.serial, use_adb=True)()
              if self.option == "disable_ap":
                  ap_steps.set_ap_wireless(state="off", interface5g=self.interface5ghz)()
                  time.sleep(5)
              status=wifi_utils.wifi_check_AP(ap_name=self.ap_name , serial=self.serial)
              if self.option == "wifi_scan":
                  if status:
                     message= "WiFi scan is success during ftp download"
                     self.set_passm(message)
                  else:
                     message = "WiFi scan is failed during ftp download"
              else:
                if status:
                   message = "ap_name is found even the ap is disabled "
                   self.set_errm("",message)
                   return False
            else:
               if self.option == "press_home":
                  status=ui_steps.press_home(serial=self.serial)()
               if not status:
                  message = "press home is failed during ftp transfer"
                  self.set_errm("",message)
                  return False

            (status, message) = adb_utils.check_download_completed(
                                serial = self.serial,
                                value = self.file_size,
                                name = self.file_name + suffix,
                                download_path = self.device_info.download_path,
                                polling_time = 10,
                                max_time = 300)
        else:
            (status, message) = adb_utils.check_download_progess(
                                serial=self.serial,
                                value=self.file_size,
                                name=self.file_name + suffix,
                                download_path=self.device_info.download_path,
                                polling_time=10)
        if status:
            self.set_passm(message)
            return True
        else:
            self.set_errorm("", message)
            netcfg_DUT = wifi_utils.get_netcfg_content(serial = self.serial)
            print "DUT netcfg content : \n", netcfg_DUT
            print "DUT WiFi connection info : \n", self.wifi_info
            if self.negative:
               self.set_passm(message)
               return True
            if self.serial2:
                netcfg_ref = wifi_utils.get_netcfg_content(serial = self.serial2)
                print "Reference device netcfg content : \n", netcfg_ref
                p2p_conn_info = wifi_utils.p2p_get_connection_info(go_serial = self.serial, slave_serial = self.serial2)
                print "P2P connection info : ", p2p_conn_info
            return False
        #else:
        #    return self.step_data

class get_dut_ip_address(wifi_step):

    def __init__(self, ip_address=None, timeout=0, regex=False, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.params = {'ip_address': ip_address}
        self.timeout = timeout
        self.regex = regex

    def do(self):
        # get the wifi dumpsys: adb shell "dumpsys wifi"
        wifi_conf_ip_addr = self.adb_connection.parse_cmd_output("dumpsys wifi")
        wifi_info_ip_addr = wifi_utils.get_connection_info(wifi_conf_ip_addr)
        self.dut_ip_address = wifi_info_ip_addr.get('ip_address')

    def check_condition(self):
        return self.dut_ip_address


class create_download_url_multiserver_ftp(wifi_step):
    """ description:
        Provides an URL to be delivered to the browser, in order to download a file
        In order to create the URL, it uses the following:
         the protocol (ftp or http),
         the Linux Host IP (in which http server runs in the current directory
         the tcp port used
         the filename
        usage:
            wifi_steps.create_url("generated.bin", 1024, local_path = ".", protocol = "http", port = "8186", serial = serial)()

        tags:
            wifi, browser, download
    """

    def __init__(self, file_name, file_size, local_path, protocol, port, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.file_name = file_name
        self.file_size = file_size
        self.local_path = local_path
        self.protocol = protocol
        self.port = port
        #self.serial = serial

    def do(self):
        wifi_utils.create_file(file_name=self.file_name, file_size=int(self.file_size), file_path=self.local_path)
        wifi_conf_ip_addr = self.adb_connection.parse_cmd_output("dumpsys wifi")
        wifi_info_ip_addr = wifi_utils.get_connection_info(wifi_conf_ip_addr)
        ftp_server_ip_address = wifi_info_ip_addr.get('ip_address')
        self.step_data = (self.protocol + "://" + str(ftp_server_ip_address) + ":" + self.port + "/" + self.file_name, ftp_server_ip_address)
        #generated_url = (self.protocol + "://" + str(ftp_server_ip_address) + ":" + self.port + "/" + self.file_name, ftp_server_ip_address)
        #print generated_url

    def check_condition(self):
        pass_message = "URL successfully generated!"
        fail_message = "URL generation failed due to invalid host/device IP"
        if "None" not in self.step_data[0]:
            self.set_passm(pass_message)
            return True
        else:
            self.set_errorm("", fail_message)
            return False

class create_download_url(wifi_step):
    """ description:
        Provides an URL to be delivered to the browser, in order to download a file
        In order to create the URL, it uses the following:
         the protocol (ftp or http),
         the Linux Host IP (in which http server runs in the current directory
         the tcp port used
         the filename
        usage:
            wifi_steps.create_url("generated.bin", 1024, local_path = ".", protocol = "http", port = "8186", serial = serial)()

        tags:
            wifi, browser, download
    """
    def __init__(self, file_name, file_size, local_path, protocol, port, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.file_name = file_name
        self.file_size = file_size
        self.local_path = local_path
        self.protocol = protocol
        self.port = port
        #self.serial = serial

    def do(self):
        wifi_utils.create_file(file_name = self.file_name, file_size = int(self.file_size), file_path = self.local_path)
        host_ip = wifi_utils.get_host_ip(self.serial)
        self.step_data = (self.protocol + "://" + str(host_ip)  + ":" + self.port+ "/" + self.file_name, host_ip)

    def check_condition(self):
        pass_message = "URL successfully generated!"
        fail_message = "URL generation failed due to invalid host/device IP"
        if "None" not in self.step_data[0]:
            self.set_passm(pass_message)
            return True
        else:
            self.set_errorm("", fail_message)
            return False

class install_WIFI_certificate(wifi_step):

    """ description:
            installs certificate from /sdcard/ with the password = <cert_pass>
            setting its name to <cert_name>

        usage:
            ui_steps.install_certificate(password = "1234")()

        tags:
            ui, android, click, button
    """

    def __init__(self, cert_pass="whatever", cert_name=None, dut_pin=None, wait_time=5, **kwargs):
        wifi_step.__init__(self, **kwargs)
        if cert_pass:
            self.cert_pass = cert_pass
        else:
            self.cert_pass = "whatever"
        if cert_name:
            self.cert_name = cert_name
        else:
            self.cert_name = "TLS_certificate"
        if dut_pin:
            self.dut_pin = dut_pin
        else:
            self.dut_pin = "1234"
        if wait_time:
            self.wait_time = int(wait_time)*1000
        else:
            self.wait_time = 5000

    def do(self):
        ui_steps.open_security_settings(serial = self.serial)()
        # In Android O, "Clear credentials" is available under "Encryption & credentials"

        #if self.device_info.dessert == "O":
        #    ui_steps.click_button(serial=self.serial, view_to_find={
        #        "textContains": "Encryption"})()
        ui_steps.click_button_if_exists(serial=self.serial,
            view_to_find={"textContains": "Encryption"})()

        if self.uidevice(className="android.widget.ListView", scrollable=True).wait.exists(timeout = self.wait_time):
            self.uidevice(scrollable = True).scroll.to(textContains="Install certificate")
        if self.uidevice(className="android.support.v7.widget.RecyclerView", scrollable=True).wait.exists(timeout = self.wait_time):
            self.uidevice(scrollable = True).scroll.to(textContains="Install certificate")
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains": "Install certificate"})()

        ui_steps.click_button_if_exists(serial = self.serial,
                                           view_to_find = {"descriptionContains":"Show roots"})()
        ui_steps.click_button_if_exists(serial = self.serial,
                                           view_to_find = {"textContains":"B free"})()
        if self.uidevice(className="android.widget.ListView", scrollable=True).wait.exists(timeout = self.wait_time):
            self.uidevice(scrollable = True).scroll.to(text="client.p12")
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text": "client.p12"},
                            view_to_check = {"textContains": "OK"})()

        if adb_utils.is_virtual_keyboard_on(serial = self.serial):
            ui_steps.press_back(serial = self.serial)()
        ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"resourceId":
                           "com.android.certinstaller:id/credential_password"},
                           value = self.cert_pass,
                           is_password = True)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains": "OK"})()
        ui_steps.edit_text( serial = self.serial,
                            view_to_find = {"resourceId":
                            "com.android.certinstaller:id/credential_name"},
                            value = self.cert_name)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains": "VPN and apps"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains": "Wi-Fi"},
                            view_to_check = {"textContains": "Wi-Fi"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains": "OK"})()
        if self.uidevice(resourceId="com.android.settings:id/password_entry").wait.exists(timeout = self.wait_time):
            ui_steps.edit_text(serial = self.serial,
                        view_to_find = {"resourceId":
                        "com.android.settings:id/password_entry"},
                        value = self.dut_pin,
                        is_password = True)()
            ui_steps.click_button(serial = self.serial,
                        view_to_find = {"textContains": "Continue"})()

    def check_condition(self):
        return ui_steps.wait_for_view_with_scroll(serial = self.serial,
                                  view_to_find = {"text": "Clear credentials",
                                  "enabled":"true"})()


class remove_certificates(wifi_step):

    """ description:
            removes certificates

        usage:
            ui_steps.remove_certificates(serial=self.serial)()

        tags:
            ui, android, click, button
    """

    def __init__(self, wait_time=5, pin=None, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.pin = pin
        if wait_time:
            self.wait_time = int(wait_time)*1000
        else:
            self.wait_time = 5000

    def do(self):
        clear_saved_networks(serial = self.serial)()
        ui_steps.open_security_settings(serial = self.serial)()
        # In Android O, "Clear credentials" is available under "Encryption & credentials"
        ui_steps.click_button_if_exists(serial=self.serial,
                               view_to_find={"textContains": "Encryption"})()
        ui_steps.wait_for_view_with_scroll(serial = self.serial,
                                                  view_to_find = {"text": "Clear credentials"})()
        if self.uidevice(text="Clear credentials", enabled=True).wait.exists(timeout = self.wait_time):
            self.uidevice(text="Clear credentials").click.wait()
            ui_steps.click_button(serial = self.serial,
                        view_to_find = {"textContains": "OK"})()
            if self.uidevice(text="Confirm your PIN").wait.exists(timeout = self.wait_time):
                ui_steps.edit_text(serial = self.serial,
                          view_to_find = {"resourceId": "com.android.settings:id/password_entry"},
                          value = self.pin,
                          is_password = True)()

                adb_utils.input_keyevent(serial = self.serial, key_number = 66)

    def check_condition(self):
        return ui_steps.wait_for_view_with_scroll(serial = self.serial,
                                                  view_to_find = {"text": "Clear credentials",
                                                                  "enabled": "False"})()



class open_wifi_settings(wifi_step):

    """ description:
            Opens the WiFi Settings page using an intent or UI.

        usage:
            wifi_steps.open_wifi_settings(serial=self.serial)()

        tags:
            ui, android, settings, wifi, intent
    """
    def __init__(self, use_adb=False, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.use_adb = use_adb

    def do(self):
        clean_command = "pm clear com.android.settings"
        self.process = self.adb_connection.run_cmd(command = clean_command,
                                    ignore_error = False,
                                    timeout = 10,
                                    mode = "sync")

        if self.use_adb == True:
            open_command = "am start -a android.intent.action.MAIN -n com.android.settings/.wifi.WifiSettings"
            self.process = self.adb_connection.run_cmd(command = open_command,
                                    ignore_error = False,
                                    timeout = 10,
                                    mode = "sync")
        else:
            ui_steps.open_settings(serial=self.serial)()
            time.sleep(3)
            ui_steps.click_button_common(serial=self.serial, view_to_find={"text":"Network & Internet"}, optional=True)()
            time.sleep(5)
            ui_steps.click_button_common(serial=self.serial, view_to_find={"textMatches":"Wi.Fi"}, view_to_check={"textMatches":"Wi.Fi"})()


    def check_condition(self):
        error_strings = [": not found", "Error"]
        output = self.process.stdout.read()
        for error in error_strings:
            if error in output:
                print "Error:"
                print output
                return False
        return True


class forget_wifi_network(wifi_step):

    """ description:
            Removes the provided WiFi SSID from known networks.

        usage:
            wifi_steps.forget_wifi_network(ap_name = "my_wifi_ssid")()

        tags:
            ui, android, settings, wifi
    """

    ap_name = None

    def __init__(self, ap_name, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name
        self.set_passm(self.ap_name)
        self.set_errorm("", self.ap_name)

    def do(self):
        if wifi_utils.is_known_AP(ap_name = self.ap_name,
                                  serial = self.serial,
                                  device = self.device_info):
            ui_steps.click_button(
                serial = self.serial,
                view_to_find = {"textContains": self.ap_name},
                view_to_check = self.device_info.wifi_saved_network_forget_btn_id)()
            ui_steps.click_button(
                serial = self.serial,
                view_to_find = self.device_info.wifi_saved_network_forget_btn_id,
                )()
        else:
            self.set_passm(self.ap_name + " is already unknown.")

    def check_condition(self):
        return not wifi_utils.is_known_AP(ap_name = self.ap_name,
                                          serial = self.serial,
                                          device = self.device_info)

class wifi_check_SSID_known(wifi_step):

    """ description:
            Checks if the provided AP name is known.
            The switch 'known' works as following:
            - if set to True (default value): the step is passed if the SSID is known
            - if set to False: the step is passed if the SSID is not known

        usage:
            wifi_steps.wifi_check_SSID_known(ap_name = "my_wifi_ssid")()

        tags:
            ui, android, settings, wifi
    """
    ap_name = None

    def __init__(self, ap_name, known = True, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name
        self.set_passm(self.ap_name)
        self.set_errorm("SSID check failed for: ", self.ap_name)
        self.known = known

    def do(self):
        set_wifi(state = "ON", serial = self.serial)()
        open_wifi_settings(serial = self.serial)()

    def check_condition(self):
        if self.known:
            return wifi_utils.is_known_AP(ap_name = self.ap_name,
                                          serial = self.serial,
                                          device = self.device_info)
        else:
            return not wifi_utils.is_known_AP(ap_name = self.ap_name,
                                              serial = self.serial,
                                              device = self.device_info)


class remove_network(wifi_step):

    """ description:
            Removes SSID(if known) from known AP list.

        usage:
            wifi_steps.remove_network(ap_name = "my_wifi_ssid")()

        tags:
            ui, android, settings, wifi
    """
    ap_name = None

    def __init__(self, ap_name, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name
        self.set_passm(self.ap_name)
        self.set_errorm("sss", self.ap_name)

    def do(self):
        set_wifi(state = "ON", serial = self.serial)()
        open_wifi_settings(serial = self.serial)()
        forget_wifi_network(ap_name = self.ap_name, serial = self.serial)()

    def check_condition(self):
        return wifi_check_SSID_known(ap_name = self.ap_name, known = False,
                                     serial = self.serial)()



class connect_wifi_from_UI(wifi_step):

    """ description:
            Connects to the given Wifi SSID with the given password. If the
            wrong_password switch is True, the step passes if the connection
            was unsuccessful.

        usage:
            wifi_steps.connect_wifi_from_UI(ap_name = ddwrt_ap_name,
                                 password = ddwrt_ap_pass,
                                 blocking = True,
                                 wrong_password = True)()

        tags:
            ui, android, settings, wifi
    """

    def __init__(self, ap_name, password=None, open_settings=True,security = True,wrong_password = False, show_password= False,scroll=False,**kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name
        self.password = password
        self.scroll = scroll
        self.security = security
        self.show_password = show_password
        self.open_settings = open_settings
        self.set_passm(ap_name + " with password: " + str(self.password))
        self.set_errorm("", ap_name + " with password: " + str(self.password))
        self.wrong_password = wrong_password
        if self.wrong_password:
            self.view_to_check = {"text": "Saved"}
        else:
            self.view_to_check = {"textContains": "Connected"}

    def do(self):
        if self.open_settings == "True":
            set_wifi(state="ON", serial=self.serial)()
            open_wifi_settings(serial=self.serial)()
        if self.scroll:
            ui_steps.wait_for_view_with_scroll(
                view_to_find={"text": self.ap_name},
                serial=self.serial)()
        if self.security:
            ui_steps.click_button(
                serial=self.serial,
                print_error="Error - AP was not selected",
                view_to_find={"text": self.ap_name},
                view_to_check={"text": "Password"})()
        else:
            ui_steps.click_button(
                serial=self.serial,
                print_error="Error - AP was not selected",
                view_to_find={"text": self.ap_name},
                view_to_check=self.view_to_check,
                wait_time=30000)()
        if self.show_password:
            ui_steps.click_button(
                serial=self.serial,
                print_error = "Error: could not check show password",
                view_to_find = {"text": "Show password"},
                view_to_check = {
                "checked": "true"})()

        if self.security:
            ui_steps.edit_text(
                serial = self.serial,
                print_error = "Error - Password was not set",
                view_to_find = {"resourceId": "com.android.settings:id/password"},
                value = self.password,
                is_password = True)()
            if self.show_password:
                ui_steps.wait_for_view(
                    serial = self.serial,
                    print_error = "Error: provided password is not seen",
                    view_to_find = {"text": self.password},
                    wait_time = 100)()
            ui_steps.click_button(
                serial = self.serial,
                print_error = "Error - Connection was not established",
                view_to_find = self.device_info.wifi_network_connect_id,
                view_to_check = self.view_to_check,optional=True,
                wait_time = 30000)()

    def check_condition(self):
        if self.wrong_password:
            outcome = self.uidevice(text = "WiFi Connection Failure").wait.exists(timeout=40000)
            if outcome == False:
                outcome = self.uidevice(text = "Authentication problem").wait.exists(timeout=40000)
            return outcome
        else:
            # Check is performed by the last step from do()
            return True
class connect_with_password(wifi_step):

    """ description:
            Connects to the given Wifi SSID with the given password. If the
            wrong_password switch is True, the step passes if the connection
            was unsuccessful.

        usage:
            wifi_steps.connect_with_password(ap_name = ddwrt_ap_name,
                                 password = ddwrt_ap_pass,
                                 blocking = True,
                                 wrong_password = True)()

        tags:
            ui, android, settings, wifi
    """

    def __init__(self, ap_name, password, wrong_password = False, scroll=False,**kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name
        self.password = password
        self.scroll = scroll
        self.set_passm(ap_name + " with password: " + str(self.password))
        self.set_errorm("", ap_name + " with password: " + str(self.password))
        self.wrong_password = wrong_password
        if self.wrong_password:
            self.view_to_check = {"text": "Saved"}
        else:
            self.view_to_check = {"textContains": "Connected"}

    def do(self):
        set_wifi(state="ON", serial=self.serial)()
        open_wifi_settings(serial=self.serial)()
        if self.scroll:
            ui_steps.wait_for_view_with_scroll(
                view_to_find={"text": self.ap_name},
                serial=self.serial)()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - AP was not selected",
            view_to_find = {"text": self.ap_name},
            view_to_check = {"text": "Password"})()
        ui_steps.edit_text(
            serial = self.serial,
            print_error = "Error - Password was not set",
            view_to_find = {"resourceId": "com.android.settings:id/password"},
            value = self.password,
            is_password = True)()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Connection was not established",
            view_to_find = self.device_info.wifi_network_connect_id,
            view_to_check = self.view_to_check,
            wait_time = 30000)()

    def check_condition(self):
        if self.wrong_password:
            outcome = self.uidevice(text = "WiFi Connection Failure").wait.exists(timeout=40000)
            if outcome == False:
                outcome = self.uidevice(text = "Authentication problem").wait.exists(timeout=40000)
            return outcome
        else:
            # Check is performed by the last step from do()
            return True


class connect_with_pass_change_orientation(wifi_step):

    """ description:
            Connects to the given Wifi SSID with the given password. An orientation
            change is done between selecting the AP and actual providing of the
            password.

        usage:
            wifi_steps.connect_with_pass_change_orientation(ap_name = ddwrt_ap_name,
                                                 password = ddwrt_ap_pass)()

        tags:
            ui, android, settings, wifi
    """

    def __init__(self, ap_name, password, device_type, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name
        self.password = password
        self.device_type = device_type

        self.set_passm(ap_name + " with password: " + str(self.password))
        self.set_errorm("", ap_name + " with password: " + str(self.password))

    def do(self):
        set_wifi(state="ON", serial=self.serial)()
        open_wifi_settings(serial=self.serial)()
        ui_steps.set_orientation(serial = self.serial,
                     orientation = "landscape",
                     target = self.device_type)()
        try:
            ui_steps.click_button(
                serial = self.serial,
                print_error = "Error - AP was not selected",
                view_to_find = {"text": self.ap_name},
                view_to_check = {"text": "Password"})()
        except BaseException as e:
            ui_steps.set_orientation(serial=self.serial,
                                     orientation="portrait",
                                     target=self.device_type)()
            raise NameError(e.message)
        ui_steps.set_orientation(serial = self.serial,
                     orientation = "portrait",
                     target = self.device_type)()
        if adb_utils.is_virtual_keyboard_on(serial = self.serial):
            ui_steps.press_back(serial = self.serial)()
        ui_steps.edit_text(
            serial = self.serial,
            print_error = "Error - Password was not set",
            view_to_find = {"resourceId": "com.android.settings:id/password"},
            value = self.password,
            is_password = True)()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Connection was not established",
            view_to_find = self.device_info.wifi_add_network_connect_btn_id,
            view_to_check = {"textContains": "Connected"},
            wait_time = 30000)()

    def check_condition(self):
            # if this point is reached then everything went ok
            return True


class connect_hidden_network_with_password(wifi_step):

    """ description:
            Connects to the given hidden Wifi SSID provided as parameter.

        usage:
            wifi_steps.connect_hidden_network_with_password(ap_name = ddwrt_ap_name,
                                 password = ddwrt_ap_pass)()

        tags:
            ui, android, settings, wifi
    """
    def __init__(self, ap_name, password, encryption = "WPA2", **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name
        self.password = password
        self.encryption = encryption
        self.set_passm(ap_name + " with password: " + str(self.password))
        self.set_errorm("", ap_name + " with password: " + str(self.password))

    def do(self):
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"descriptionContains": "More"},
            view_to_check = {"textContains": "Add network"})()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"textContains": "Add network"},
            view_to_check = {"textContains": "Network name"})()
        if adb_utils.is_virtual_keyboard_on(serial = self.serial):
            ui_steps.press_back(serial = self.serial)()
        ui_steps.edit_text(serial = self.serial,
            view_to_find = {"resourceId": "com.android.settings:id/ssid"},
            value = self.ap_name,
            is_password = False)()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"className": "android.widget.Spinner"})()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"textContains": self.encryption},
            wait_time = 10000)()
        ui_steps.edit_text(serial = self.serial,
            view_to_find = {"resourceId": "com.android.settings:id/password"},
            value = self.password,
            is_password = True)()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text": "Save"})()
        self.uidevice.wait.update()
        if self.uidevice(text = "Saved").wait.exists(timeout=1000):
            ui_steps.click_button(serial = self.serial,
                view_to_find = {"textContains": self.ap_name},
                view_to_check = {"textContains": "Connect"})()
            ui_steps.click_button(serial = self.serial,
                #view_to_find = {"textContains": "Connect"})()
                view_to_find = self.device_info.wifi_add_network_connect_btn_id)()

    def check_condition(self):
        self.uidevice(text = "Connected").wait.exists(timeout = 30000)
        return self.uidevice(text = "Connected").wait.exists(timeout=1000)


class toggle_airplane_mode_settings(wifi_step):

    def __init__(self, iterations = 1, wait_time = 10, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.iterations = int(iterations)
        self.wait_time = int(wait_time)

    def do(self):
        # open More Wireless Settings using intent
        adb_steps.command(serial = self.serial, command = "pm clear com.android.settings",
                                    mode = "sync", timeout = 10)()
        adb_steps.command(serial = self.serial, command = "am start -a android.settings.AIRPLANE_MODE_SETTINGS",
                                    mode = "sync", timeout = 10)()
        if self.device_info == "O":
            ui_steps.click_button(serial=self.serial,
                                  view_to_find={"descriptionContains": "Airplane mode"},
                                  view_to_check={"textContains": "Airplane mode"})()
        else:
            self.switch_initial = ui_utils.is_switch_on(serial = self.serial,
                                                        view_to_find = {"textContains": "Airplane mode"})

        if self.switch_initial:
            for i in range(self.iterations):
                self.set_errorm("test failed at iteration #", i)
                click_airplane_switch_from_settings(serial = self.serial, view_to_find = {"textContains": "Airplane mode"},
                                state = "OFF", wait_time = self.wait_time)()
                click_airplane_switch_from_settings(serial = self.serial, view_to_find = {"textContains": "Airplane mode"},
                                state = "ON", wait_time = self.wait_time)()
        else:
            for i in range(self.iterations):
                self.set_errorm("test failed at iteration #", i)
                click_airplane_switch_from_settings(serial = self.serial, view_to_find = {"textContains": "Airplane mode"},
                                state = "ON", wait_time = self.wait_time)()
                click_airplane_switch_from_settings(serial = self.serial, view_to_find = {"textContains": "Airplane mode"},
                                state = "OFF", wait_time = self.wait_time)()

    def check_condition(self):
        switch_final = ui_utils.is_switch_on(serial = self.serial,
                                          view_to_find = {"textContains": "Airplane mode"})
        return switch_final == self.switch_initial


class click_airplane_switch_from_settings(ui_steps.click_switch):
    """ description:
            Does the same as ui_steps.click_switch, but targeted for airplane mode
            on/off switch. It reuses the do method, but rewrites check_condition

        usage:
            click_airplane_switch_from_settings(
                view_to_find = {"className": "android.widget.Switch",
                                "instance": "1"},
                state = "ON",
                click_to_close_popup = {"text": "Agree"})()

        tags:
            ui, android, click, switch, enable, disable, wifi
    """
    # call the do method of click_switch step
    def __init__(self, view_to_find, state = "ON", click_to_close_popup = None,
                    wait_time = 10, **kwargs):
        ui_steps.click_switch.__init__(self, view_to_find, state,
                                      click_to_close_popup, **kwargs)
        self.state = state
        self.wait_time = int(wait_time)

    # def do(self):
    # the do method from click_switch step is called here by default

    # overwrite the check method
    def check_condition(self):
        if self.state == "ON":
            ui_steps.wait_for_view(view_to_find = {"className": "android.widget.Switch", "text" : "ON"}, timeout = self.wait_time, serial = self.serial)()
            return wifi_utils.check_airplane_mode_on(serial = self.serial)
        else:
            ui_steps.wait_for_view(view_to_find = {"className": "android.widget.Switch", "text" : "OFF"}, timeout = self.wait_time, serial = self.serial)()
            return not wifi_utils.check_airplane_mode_on(serial = self.serial)


class toggle_wifi_quick_settings_scan_window(wifi_step):

    """ description:
            Turns Wifi on or off from the quick settings menu.

        usage:
            wifi_steps.toggle_wifi_switch_quick_settings_scan_window(iterations = 7)()

        tags:
            ui, android, settings, wifi
    """
    def __init__(self, wait_time = 10, iterations = 1, **kwargs):
        self.iterations = int(iterations)
        self.wait_time = int(wait_time)
        wifi_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_notifications_menu(serial = self.serial)()

        for i in range(self.iterations):
            self.set_errorm("test failed at iteration #", i)
            expand_wifi_menu_quick_settings(serial = self.serial)()
            click_switch_wifi_menu_quick_settings(serial = self.serial)()

    def check_condition(self):
        return not wifi_utils.check_wifi_state_on(serial = self.serial)


class expand_wifi_menu_quick_settings(wifi_step):

    def __init__(self, wait_time = 10, **kwargs):
        self.wait_time = int(wait_time)
        wifi_step.__init__(self, **kwargs)
        self.initial_state = wifi_utils.check_wifi_state_on(serial = self.serial)

    def do(self):
        ui_steps.click_view(view = self.uidevice(text = "Wi-Fi"), serial = self.serial)()

    def check_condition(self):
        ui_steps.wait_for_view(view_to_find = {"className":"android.widget.Switch", "text":"ON"},
                        timeout = self.wait_time, serial = self.serial)
        for i in range(self.wait_time):
            if wifi_utils.check_wifi_state_on(serial = self.serial):
                return True
            time.sleep(1)
        return False


class click_switch_wifi_menu_quick_settings(wifi_step):

    def __init__(self, wait_time=5, **kwargs):
        self.wait_time = int(wait_time)
        wifi_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.click_view(serial = self.serial, view = self.uidevice(className = "android.widget.Switch"))()

    def check_condition(self):
        return ui_steps.wait_for_view(view_to_find = {"description":"Wifi off.."},
                    timeout = self.wait_time, serial = self.serial)\
                    and (not wifi_utils.check_wifi_state_on(serial = self.serial))


class connect_disconnect_UI_stress_2(wifi_step):

    """description:
            connects & disconnects the DUT to an AP for a specified
            number of times, using only the UI
            designed to be used exclusively with WPA PSK security
    """
    def __init__(self, ap_name, password, iterations = 1, **kwargs):
        self.ap_name = ap_name
        self.password = password
        self.iterations = int(iterations)
        wifi_step.__init__(self, **kwargs)

    def do(self):
        for i in range(self.iterations):
            self.set_errorm("test failed at iteration #", i)
            connect_disconnect_from_quick_settings(serial = self.serial, ap_name = self.ap_name, password = self.password)()

    def check_condition(self):
        return


class connect_disconnect_from_quick_settings(wifi_step):

    def __init__(self, ap_name, password, **kwargs):
        self.ap_name = ap_name
        self.password = password
        wifi_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_notifications_menu(serial = self.serial)()
        ui_steps.click_view(view = self.uidevice(text = "Wi-Fi"), serial = self.serial)()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - AP was not selected",
            view_to_find = {"text": self.ap_name},
            view_to_check = {"text": "Password"})()
        ui_steps.edit_text(
            serial = self.serial,
            print_error = "Error - Password was not set",
            view_to_find = {"resourceId": "com.android.settings:id/password"},
            value = self.password,
            is_password = True)()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Connection was not established",
            view_to_find = self.device_info.wifi_add_network_connect_btn_id)()
        wait_until_connected(serial = self.serial)()
        forget_wifi_network(ap_name = self.ap_name, serial = self.serial)()

    def check_condition(self):
        return wait_for_state(state="DISCONNECTED", serial = self.serial)()


class toggle_wifi_quick_settings(wifi_step):

    """ description:
            Turns Wifi on or off from the quick settings menu.

        usage:
            wifi_steps.toggle_wifi_switch_quick_settings(iterations = 7)()

        tags:
            ui, android, settings, wifi
    """
    def __init__(self, wait_time = 10, iterations = 1, **kwargs):
        self.iterations = int(iterations)
        self.wait_time = int(wait_time)
        wifi_step.__init__(self, **kwargs)
        self.initial_state = wifi_utils.check_wifi_state_on(serial = self.serial)

    def do(self):
        ui_steps.open_notifications_menu(serial = self.serial)()

        for i in range(self.iterations):
            self.set_errorm("test failed at iteration #", i)
            click_wifi_button_quick_settings(wait_time = self.wait_time, serial = self.serial)()
            click_wifi_button_quick_settings(wait_time = self.wait_time, serial = self.serial)()

    def check_condition(self):
        return wifi_utils.check_wifi_state_on(serial = self.serial) == self.initial_state


class click_wifi_button_quick_settings(wifi_step):

    def __init__(self, wait_time = 10, **kwargs):
        self.wait_time = int(wait_time)
        wifi_step.__init__(self, **kwargs)
        self.initial_state = wifi_utils.check_wifi_state_on(serial = self.serial)

    def do(self):
        ui_steps.click_view(view = self.uidevice(**self.device_info.quick_settings_wifi_id), serial = self.serial)()

    def check_condition(self):
        if self.initial_state == True:
            ui_steps.wait_for_view(view_to_find = self.device_info.quick_settings_wifi_disconnected_id,
                                   timeout = self.wait_time,
                                   serial = self.serial)
        else:
            ui_steps.wait_for_view(view_to_find = self.device_info.quick_settings_wifi_off_id,
                                   timeout = self.wait_time,
                                   serial = self.serial)
        if not (wifi_utils.check_wifi_state_on(serial = self.serial) == self.initial_state):
                return True
        return False


class toggle_airplane_mode_quick_settings(wifi_step):

    """ description:
            Turns Wifi on or off from the quick settings menu.

        usage:
            wifi_steps.toggle_wifi_switch_quick_settings(iterations = 7)()

        tags:
            ui, android, settings, wifi
    """
    def __init__(self, wait_time = 10, iterations = 1, **kwargs):
        self.iterations = int(iterations)
        self.wait_time = int(wait_time)
        wifi_step.__init__(self, **kwargs)
        self.initial_state = wifi_utils.check_airplane_mode_on(serial = self.serial)

    def do(self):
        ui_steps.open_notifications_menu(serial = self.serial)()

        for i in range(self.iterations):
            self.set_errorm("test failed at iteration #", i)
            click_am_button_quick_settings(wait_time = self.wait_time, serial = self.serial)()
            click_am_button_quick_settings(wait_time = self.wait_time, serial = self.serial)()

    def check_condition(self):
        return wifi_utils.check_airplane_mode_on(serial = self.serial) == self.initial_state


class click_am_button_quick_settings(wifi_step):

    def __init__(self, wait_time = 10, **kwargs):
        self.wait_time = int(wait_time)
        wifi_step.__init__(self, **kwargs)
        self.initial_state = wifi_utils.check_airplane_mode_on(serial = self.serial)

    def do(self):
        ui_steps.click_view(view = self.uidevice(descriptionStartsWith = "Airplane mode"), serial = self.serial)()

    def check_condition(self):
        for i in range(self.wait_time):
            if not (wifi_utils.check_airplane_mode_on(serial = self.serial) == self.initial_state):
                return True
            time.sleep(1)
        return False


class toggle_wifi_switch_from_settings(wifi_step):

    """ description:
            Turns Wifi on or off from the Wifi settings.

        usage:
            wifi_steps.toggle_wifi_switch_from_settings(iterations = 7, wait_time = 7)()

        tags:
            ui, android, settings, wifi
    """
    def __init__(self, wait_time = 15, iterations = 1, **kwargs):
        self.iterations = int(iterations)
        self.wait_time = int(wait_time)
        wifi_step.__init__(self, **kwargs)

    def do(self):
        open_wifi_settings(serial = self.serial)()

        self.switch_initial = ui_utils.is_switch_on(serial = self.serial,
                                          view_to_find = {"className": "android.widget.Switch"})

        if self.switch_initial:
            for i in range(self.iterations):
                self.set_errorm("test failed at iteration #", i)
                click_wifi_switch(serial = self.serial, view_to_find = {"className": "android.widget.Switch"},
                                state = "OFF", wait_time=self.wait_time)()
                click_wifi_switch(serial = self.serial, view_to_find = {"className": "android.widget.Switch"},
                                state = "ON", wait_time=self.wait_time)()
        else:
            for i in range(self.iterations):
                self.set_errorm("test failed at iteration #", i)
                click_wifi_switch(serial = self.serial, view_to_find = {"className": "android.widget.Switch"},
                                state = "ON", wait_time=self.wait_time)()
                click_wifi_switch(serial = self.serial, view_to_find = {"className": "android.widget.Switch"},
                                state = "OFF", wait_time=self.wait_time)()

    def check_condition(self):
        switch_final = ui_utils.is_switch_on(serial = self.serial,
                                          view_to_find = {"className": "android.widget.Switch"})
        return switch_final == self.switch_initial



class click_wifi_switch(ui_steps.click_switch):
    """ description:
            Does the same as ui_steps.click_switch, but targeted for Wi-Fi
            on/off switch. It reuses the do method, but rewrites check_condition

        usage:
            wifi_steps.click_wifi_switch(
                view_to_find = {"className": "android.widget.Switch",
                                "instance": "1"},
                state = "ON",
                click_to_close_popup = {"text": "Agree"})()

        tags:
            ui, android, click, switch, enable, disable, wifi
    """
    # call the do method of click_switch step
    def __init__(self, view_to_find, state = "ON", click_to_close_popup = None,
                    wait_time = 10000, **kwargs):
        ui_steps.click_switch.__init__(self,
                                       view_to_find = view_to_find,
                                       state = state,
                                       click_to_close_popup = click_to_close_popup,
                                       **kwargs)
        self.wait_time = wait_time


    # def do(self):
    # the do method from click_switch step is called here by default

    # overwrite the check method
    def check_condition(self):
        if self.step_data:
            if self.state == "OFF":
                text_to_find = "Disabled"
            elif self.state == "ON":
                text_to_find = "Enabled"
            adb_steps.command(serial = self.serial,
                              command = "am start -n com.android.settings/.wifi.WifiStatusTest")()
            if ui_steps.wait_for_view(view_to_find = {"text" : text_to_find,
                                                      "resourceId" : "com.android.settings:id/wifi_state"},
                                          timeout = self.wait_time,
                                          serial = self.serial)():
                adb_steps.input_back(serial = self.serial)()
                ui_steps.wait_for_view(view_to_find = {"resourceId" : "com.android.settings:id/switch_widget"},
                                       timeout = self.wait_time,
                                       serial=self.serial)()
                return self.switch.info['text'] == self.state
        return (self.switch.info['text'] == self.state)


class connect_disconnect_UI_stress(wifi_step):

    """description:
            connects & disconnects the DUT to an AP for a specified
            number of times, using only the UI
            designed to be used exclusively with WPA PSK security
    """
    def __init__(self, ap_name, password, iterations = 1, **kwargs):
        self.ap_name = ap_name
        self.password = password
        self.iterations = int(iterations)
        wifi_step.__init__(self, **kwargs)

    def do(self):
        for i in range(self.iterations):
            self.set_errorm("test failed at iteration #", i)
            connect_disconnect_UI(ap_name = self.ap_name, password = self.password, serial = self.serial)()

    def check_condition(self):
        return wait_for_state(state="DISCONNECTED", serial = self.serial)()


class connect_disconnect_UI(wifi_step):

    def __init__(self, ap_name, open_settings = True,password=None,security= True,ping= False,forget_ap= False, **kwargs):
        self.ap_name = ap_name
        self.password = password
        self.open_settings = open_settings
        self.ping = ping
        self.security = security
        wifi_step.__init__(self, **kwargs)

    def do(self):
        connect_wifi_from_UI(ap_name = self.ap_name, scroll=True, open_settings=self.open_settings,password = self.password, serial = self.serial)()
        wait_until_connected(serial = self.serial)()
        if self.ping:
            ping_gateway(serial=self.serial)()
        forget_wifi_network(ap_name = self.ap_name, serial = self.serial)()
        #clear_saved_networks(serial = self.serial)()

    def check_condition(self):
        return wait_for_state(state="DISCONNECTED", serial = self.serial)()


class set_from_wifi_settings(wifi_step):

    """ description:
            Turns Wifi on or off from the Wifi settings. Default value for the
            state is ON.

        usage:
            wifi_steps.set_from_wifi_settings(state = "OFF")()

        tags:
            ui, android, settings, wifi
    """
    def __init__(self, state = "ON", intent = False, wait_time = 10000, **kwargs):
        self.state = state
        self.intent = intent
        self.wait_time = int(wait_time)
        self.open_wifi_settings = open_wifi_settings
        wifi_step.__init__(self, **kwargs)

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.wifi.WifiSettings")()
        else:
            if self.open_wifi_settings:
                open_wifi_settings(serial = self.serial)()

        if self.state == "ON":
            reverse_state = "OFF"
        elif self.state == "OFF":
            reverse_state = "ON"

        switched = click_wifi_switch(serial = self.serial,
            view_to_find = {"className": "android.widget.Switch"},
            state = self.state, wait_time = self.wait_time)()
        if not switched:
            click_wifi_switch(serial = self.serial,
                view_to_find = {"className": "android.widget.Switch"},
                state = reverse_state, wait_time=self.wait_time)()
            click_wifi_switch(serial = self.serial,
                view_to_find = {"className": "android.widget.Switch"},
                state = self.state, wait_time=self.wait_time)()

    def check_condition(self):
        switch_on = ui_utils.is_switch_on(serial = self.serial,
                                          view_to_find = {"className": "android.widget.Switch"})
        if self.state == "ON":
            return switch_on
        elif self.state == "OFF":
            return not switch_on


class check_wifi_state_disconnected(wifi_step):
    """ description:
            Checks the connection state of the Wifi is disconnected.

        usage:
            wifi_steps.check_wifi_state_disconnected(ap_name = ddwrt_ap_name,
                                                     security=dut_security,
                                                     timeout=100,
                                                     serial = serial)()

        tags:
            android, wifi
    """
    def __init__(self, ap_name=None, security=None, encryption=None, wrong_password=False, timeout=200, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name
        self.security = security
        self.wrong_password = wrong_password
        self.encryption = encryption
        self.timeout = timeout

    def do(self):
        self.decision_logcat = True
        if self.wrong_password:
            self.decision_logcat = False

            if self.security.lower() != "wep":
                while self.timeout > 0:
                    self.decision_logcat = "WRONG_KEY" in \
                                           self.adb_connection.parse_logcat(grep_for="CTRL-EVENT-SSID-TEMP-DISABLED")
                    if not self.decision_logcat:
                        self.decision_logcat = "AUTH_FAILED" in \
                                    self.adb_connection.parse_logcat(grep_for="CTRL-EVENT-SSID-TEMP-DISABLED")
                    if not self.decision_logcat:
                        self.decision_logcat = "EAP_AUTH_FAILURE" in \
                                    self.adb_connection.parse_logcat(grep_for="EAP_AUTH_FAILURE")
                    if not self.decision_logcat:
                        self.decision_logcat = "CTRL-EVENT-EAP-FAILURE" in \
                                    self.adb_connection.parse_logcat(grep_for="CTRL-EVENT-EAP-FAILURE")
                    if self.decision_logcat:
                        break
                    time.sleep(2)
                    self.timeout += -2

            else:
                found = 0
                while self.timeout > 0:
                    self.decision_logcat = "CMD_AUTO_CONNECT did save config" in \
                                    self.adb_connection.parse_logcat(grep_for="CMD_AUTO_CONNECT did save config")
                    if self.decision_logcat:
                        found += 1
                        if found > 4:
                            break
                        adb_steps.command("logcat -c",
                                          serial = self.serial)()
                        time.sleep(5)
                if found > 1:
                    self.decision_logcat = True

        self.decision_dumpsys = wait_for_state(state="DISCONNECTED",serial = self.serial)()

    def check_condition(self):
        if self.decision_logcat and self.decision_dumpsys:
            return True
        else:
            return False



class check_connection_info(wifi_step):
    """ description:
            Check the WiFi connection information.

        usage:
            wifi_steps.check_connection_info(SSID = "ddwrt", Security='WPA-PSK')()

            Use <parama_name>="None" to expect the setting not to be present on DUT.

            Example of possible values for all supported parameters:
            {'DHCP_server': '192.168.1.1',
             'DNS_addresses': '8.8.8.8,8.8.4.4',
             'Frequency': '2437MHz',
             'Gateway': '192.168.1.1',
             'Link_speed': '65Mbps',
             'MAC': 'dc:85:de:b9:5c:db',
             'SSID': 'Android Core QA',
             'Security': 'WPA2-PSK',
             'group_cipher': 'TKIP',
             'ip_address': '192.168.1.122',
             'p2p_device_address': 'de:85:de:b9:5c:db',
             'pairwise_cipher': 'CCMP',
             'state': 'CONNECTED/CONNECTED'}

        tags:
            android, settings, wifi
    """


    def __init__(self, state=None, SSID=None, Link_speed=None, Frequency=None,
                 Security=None, group_cipher=None, pairwise_cipher=None,
                 ip_address=None, Gateway=None, MAC=None, p2p_device_address=None,
                 DHCP_server=None, DNS_addresses=None, timeout=0, regex=False, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.params = {'DHCP_server': DHCP_server,
                        'DNS_addresses': DNS_addresses,
                        'Frequency': Frequency,
                        'Gateway': Gateway,
                        'Link_speed': Link_speed,
                        'MAC': MAC,
                        'SSID': SSID,
                        'Security': Security,
                        'ip_address': ip_address,
                        'p2p_device_address': p2p_device_address,
                        'state': state,
                        'pairwise_cipher': pairwise_cipher,
                        'group_cipher': group_cipher}
        self.timeout = timeout
        self.regex = regex

    def do(self):
        # get the wifi dumpsys: adb shell "dumpsys wifi"
        wifi_conf = self.adb_connection.parse_cmd_output("dumpsys wifi")
        self.wifi_info = wifi_utils.get_connection_info(wifi_conf)

    def check_condition(self):
        while self.timeout > -1:
            outcome = True
            error_msg = ""
            for param_name in self.params.keys():
                if self.params[param_name] != None:
                    if self.wifi_info[param_name] == "UNKNOWN/IDLE":
                        if self.params[param_name] == "CONNECTED/CONNECTED":
                            outcome = False
                    elif self.params[param_name] == "None":
                        if self.wifi_info[param_name] != None:
                            outcome = False
                            error_msg = error_msg + "wrong '{0}' parameter. Expected '{1}',"\
                                .format(param_name, self.params[param_name]) +\
                                " but actual value is '{0}'\n".format(self.wifi_info[param_name])

                    elif not self.regex:
                        if self.params[param_name] != self.wifi_info[param_name]:
                            outcome = False
                            error_msg = error_msg + "wrong '{0}' parameter. Expected '{1}',"\
                                .format(param_name, self.params[param_name]) +\
                                " but actual value is '{0}'\n".format(self.wifi_info[param_name])
                    else:
                        m = re.search(self.params[param_name], self.wifi_info[param_name])

                        if m == None:
                            outcome = False
                            error_msg = error_msg + "wrong '{0}' parameter. Expected '{1}',"\
                            .format(param_name, self.params[param_name]) +\
                            " but actual value is '{0}'\n".format(self.wifi_info[param_name])

            if outcome:
                break
            # print connection info in case check fails
            print "wifi connection info : \n", self.wifi_info
            self.timeout -= 2
            time.sleep(2)
            if self.timeout > -1:
                wifi_conf = self.adb_connection.parse_cmd_output("dumpsys wifi")
                self.wifi_info = wifi_utils.get_connection_info(wifi_conf)
        self.set_errorm("", error_msg)
        return outcome

class set_wifi_state(wifi_step):
    """
    Description: Turns Wifi on or off from command lineDefault value for the
            state is ON.
        usage:
            wifi_steps.set_wifi_state(state = "OFF")()

        tags:
            adb, android, wifi
    """

    def __init__(self,state = "ON",use_adb=True,open_wifi_settings=True, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.state = state
        self.use_adb = use_adb
        self.open_wifi_settings = open_wifi_settings
        self.set_passm("Setting WiFi state to " + self.state)

    def do(self):
        if self.state == "ON":
            wifi_state = "enable"
        else:
            wifi_state = "disable"
        if self.open_wifi_settings:
           open_wifi_settings(serial=self.serial)()
        current_wifi_state=wifi_utils.check_wifi_state_on(serial = self.serial)
        if self.state == "ON":
            if current_wifi_state:
                 message = "wifi state is already ON ,hence no action is performed"
                 self.set_passm(message)
        else:
             if not current_wifi_state:
                 message = "wifi state is already OFF ,hence no action is performed"
                 self.set_passm(message)

        if self.use_adb :
            clean_command = "pm clear com.android.settings"
            self.process = self.adb_connection.run_cmd(command = clean_command,
                                    ignore_error = False,
                                    timeout = 10,
                                    mode = "sync")
            if self.state == "ON":
                if not current_wifi_state:
                    message = "wifi is turned ON"
                    self.set_passm(message)
                    self.adb_connection.parse_cmd_output("svc wifi {}".format(wifi_state),
                        dont_split=True)
            else:
                if current_wifi_state:
                    message = "wifi is turned OFF"
                    self.set_passm(message)
                    self.adb_connection.parse_cmd_output("svc wifi {}".format(wifi_state),
                        dont_split=True)

            #self.adb_connection.parse_cmd_output("am start -n"
            #                " com.android.settings/.wifi.WifiStatusTest",
            #                dont_split=True)
        else:

            if self.state == "ON":
                if not current_wifi_state:
                    message = "wifi is turned ON"
                    self.set_passm(message)
                    ui_steps.click_button(serial=self.serial,
                                  view_to_find={"resourceId": "com.android.settings:id/switch_text"},
                                  view_to_check={"textContains": self.state })()
            else:
                if current_wifi_state:
                    message = "wifi is turned OFF"
                    self.set_passm(message)
                    ui_steps.click_button(serial=self.serial,
                                  view_to_find={"resourceId": "com.android.settings:id/switch_text"},
                                  view_to_check={"textContains": self.state })()


            #self.adb_connection.parse_cmd_output("am start -n"
            #                " com.android.settings/.wifi.WifiStatusTest",
            #                dont_split=True)



    def check_condition(self):
        # if self.state == "ON":
        #     text_to_find = "Enabled"
        #     self.adb_connection.parse_cmd_output("echo 10 > /proc/net/rtl8723bs/log_level",
        #                 dont_split=True, ignore_error=True)
        # else:
        #     text_to_find = "Disabled"
        # ## TODO: investiagate alternate verification possibility: without UI
        # ret_value = False
        # for counter in range(5):
        #     ui_steps.wait_for_view(view_to_find = {"textContains":"Wi-Fi state:"},
        #                    timeout = 5000,
        #                    serial = self.serial)()
        #     if self.uidevice(text="Wi-Fi state:").right(text=text_to_find):
        #         ret_value = True
        #         break
        #     time.sleep(1)
        # if ret_value == False:
        #     print "'{}' not found in com.android.settings/.wifi.WifiStatusTest.".format(text_to_find)
        # return ret_value

        # below wifi utility returns True when wifi in "ON" else False
        for counter in range(5):
            time.sleep(1)
            status = wifi_utils.check_wifi_state_on(self.serial)
            if self.state == "ON":
                self.step_data = True if status else False
            else:
                if status:
                    self.step_data = False
                else:
                    self.step_data = True
            return self.step_data

# Create an alias for set_from_wifi_settings to set_wifi
#set_wifi = set_from_wifi_settings
set_wifi = set_wifi_state

class wait_until_connected(wifi_step):
    """ description: Waits until the device is connected to a wifi network.
    """

    def __init__(self, timeout=120, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.timeout = timeout
        self.connected = 'CONNECTED/CONNECTED'
        self.wifi_info = None

    def do(self):
        self.resolution = wait_for_state(serial=self.serial, state=self.connected, timeout=self.timeout)()

    def check_condition(self):
        return self.resolution

class wait_for_state(wifi_step):
    """ description: Waits until the wifi is in the desired state.
    """

    def __init__(self, state='CONNECTED/CONNECTED', timeout=150, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.timeout = timeout
        self.state = state
        self.wifi_info = None

    def do(self):
        while self.timeout > 0:
            wifi_conf = self.adb_connection.parse_cmd_output("dumpsys wifi")
            self.wifi_info = wifi_utils.get_connection_info(wifi_conf)
            # check the state when both parts are provided
            if "/" in self.state:
                if self.wifi_info['state'] == self.state:
                    break
            # check only the first part of the state (CONNECTED/DISCONNECTED)
            else:
                if self.wifi_info['state'].split("/")[0] == self.state:
                    break

            time.sleep(2)
            self.timeout -= 1

    def check_condition(self):
        if self.timeout > 0:
            self.step_data = True
            return True
        else:
            self.step_data = False
            return False


class p2p_wait_for_parameter_state(wifi_step):
    """ description: Waits until the desired parameter reach the given value.
                     Fails if the value does not happen inside the timeout period.
                     For possible values of 'param_name' see
                     wifi_utils.p2p_get_connection_parameter().
    """

    def __init__(self, param_name, param_state, go_serial = None, slave_serial = None, timeout=30, **kwargs):
        wifi_step.__init__(self, serial=go_serial, **kwargs)
        self.timeout = timeout
        self.param_name = param_name
        self.param_state = param_state
        self.go_serial = go_serial
        self.slave_serial = slave_serial

    def do(self):
        while self.timeout > 0:
            param_value = wifi_utils.p2p_get_connection_parameter(self.param_name,
                                                go_serial = self.go_serial,
                                                slave_serial = self.slave_serial)

            if self.param_state == param_value:
                break

            time.sleep(2)
            self.timeout += -2

    def check_condition(self):
        if self.timeout > 0:
            self.step_data = True
            return True
        else:
            self.step_data = False
            return False



class scan_and_check_ap(wifi_step):

    """ description:
            Check weather an AP SSID can be found by a Wi-Fi scan.

            NOTE: Carefull when looking for an AP SSID that is saved as it will
            show up even when the AP SSID is not visible or in range!

            <option> parameter selects the type of scann:
                - "ser_off" - by setting wifi off and then on
                - "refresh" - by

        usage:
            wifi_steps.scan_and_check_ap("MyAP", should_exist=True)()

        tags:
            ui, android, ssid, scan
    """

    def __init__(self, ap, option="set_off", trycount=10, open_wifi=None, should_exist=True, **kwargs):
        self.ap = ap
        self.option = option
        self.trycount = trycount
        self.should_exist = should_exist
        self.open_wifi = open_wifi

        wifi_step.__init__(self, **kwargs)
        self.set_passm(str(ap) + " scanned")
        if self.should_exist:
            self.set_errorm("", "Could not find AP SSID after scan: " + self.ap)
        else:
            self.set_errorm("", "AP SSID was found by scan when it should not: " + self.ap)

        self.exclude_ap_names = ["Wi?Fi", "On"]
        if self.ap in self.exclude_ap_names:
                print "WARNING: You are using bad AP names for testing: " + ",".join(self.exclude_ap_names)
        self.found_ap = None

    def do(self):
        # Making sure we are in Settings - Wi-Fi menu and turning on/off-on Wi-Fi (in order to force a scan)
        if self.option == "set_off":
            set_wifi(serial = self.serial, state="OFF")()
            time.sleep(1)
            set_wifi(serial = self.serial, state="ON")()
        elif self.option == "refresh":
            if not self.open_wifi:
                open_wifi_settings(serial=self.serial)()
            # Click "More"
            ui_steps.click_button(serial = self.serial,
                view_to_find = {"descriptionContains": "More"},
                view_to_check = {"textContains": "Refresh"})()

            # Click "Refresh"
            ui_steps.click_button(serial =self.serial,
                view_to_find = {"textContains": "Refresh"},
                view_to_check = {"textContains": "Refresh"},
                view_presence = False)()
        else:
            raise BlockingError("Invalid scanning option: {}\n".format(self.option) +\
                                "Should be on of: ['set_off', 'refresh']")
        if not self.open_wifi:
            open_wifi_settings(serial=self.serial)()
        time.sleep(1)
        # Checking if the AP SSID is in the current view. We do this every second for self.trycount seconds.
        self.found_ap = self.uidevice(text=self.ap).wait.exists(timeout = 1000)
        iteration = 0
        while not self.found_ap and iteration < self.trycount:
            if self.uidevice(scrollable = True).exists:
                self.found_ap = self.uidevice(scrollable = True).scroll.to(text=self.ap)

            iteration += 1
            time.sleep(1)

    def check_condition(self):
        return (self.found_ap == self.should_exist)


class clear_saved_networks(wifi_step):

    """ description:
            Clear all saved networks. Can remove a maximum of <max_entries> networks.

        usage:
            wifi_steps.clear_saved_networks(serial=self.serial)()
            OR
            wifi_steps.clear_saved_networks(max_entries=10)()

        tags:
            ui, android, ssid, clear, wifi
    """

    def __init__(self, max_entries=20, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.max_entries = max_entries
        self.steppassed = False

    def do(self):
        # Turn on Wi-Fi and go to Settings - Wi-Fi menu and Turn
        set_wifi(serial=self.serial, state="ON")()
        open_wifi_settings(serial=self.serial)()

        # Click the More button
        if not ui_steps.click_button_if_exists(serial=self.serial,
                   view_to_find = self.device_info.wifi_more_options_id)():
            if self.uidevice(scrollable = True):
                self.uidevice(scrollable = True).scroll.toEnd()

        # Look for the 'Saved Networks' button.
        # If it exists, click it. If not, it means there are no saved Wi-Fi networks.
        if self.uidevice(text="Saved networks").wait.exists(timeout=1000):
            ui_steps.click_button(serial=self.serial,
                                  view_to_find = {"text":"Saved networks"},
                                  view_to_check = {"text":"Saved networks"})()

            # Get the top saved network from the ListView
            while self.max_entries > 0:
                if not self.uidevice(**self.device_info.wifi_saved_networks_list_id)\
                    .child(**self.device_info.wifi_saved_networks_list_element_id)\
                    .wait.exists(timeout=1000):
                    break
                self.uidevice(**self.device_info.wifi_saved_networks_list_id)\
                    .child(**self.device_info.wifi_saved_networks_list_element_id)\
                    .click.wait()
                self.uidevice(**self.device_info.wifi_saved_network_forget_btn_id).wait.exists(timeout=1000)
                if not ui_steps.click_button_common(serial=self.serial,
                                      view_to_find = self.device_info.wifi_saved_network_forget_btn_id,optional=True,
                                      scroll=False, view_to_check = {"text":"Saved networks"})():
                   break

                '''ui_steps.click_button(serial=self.serial,
                                      view_to_find = self.device_info.wifi_saved_network_forget_btn_id,
                                      view_to_check = {"text":"Saved networks"})()

                self.max_entries -= 1
                '''
                if not ui_steps.click_button_common(serial=self.serial,
                                                view_to_find=self.device_info.wifi_saved_network_forget_btn_id,
                                                optional=True,
                                                scroll=False, view_to_check={"text": "Saved networks"})():
                    break
            # If we still have an entry in the list, the step is failed.
            if self.max_entries == 0:
                self.step_data = False
                self.set_errorm("", "Network still in 'Saved Networks' list.")
            else:
                self.step_data = True
        else:
            self.step_data = True

    def check_condition(self):
        return self.step_data


class set_wifi_security(wifi_step):

    def __init__(self, security, password=None, EAP_method=None, phase_2_auth=None, user_certificate=None, identity=None, anonymous_identity=None, ca_certificate=None, clear_text_boxes=True, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.security = security
        self.password = password
        self.EAP_method = EAP_method
        self.phase_2_auth = phase_2_auth
        self.user_certificate = user_certificate
        self.identity = identity
        self.anonymous_identity = anonymous_identity
        self.ca_certificate = ca_certificate
        self.clear_text = clear_text_boxes
        self.step_data = False

    def do(self):
        # Complete security fields
        if self.security:
            ui_steps.click_button_with_scroll(serial = self.serial,
                view_to_find = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/security"},
                view_to_check = {"textContains": self.security})()
            ui_steps.click_button_with_scroll(serial = self.serial,
                view_to_find = {"textContains": self.security},
                view_to_check = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/security"})()

            if self.password:
                ui_steps.edit_text(serial = self.serial, clear_text = self.clear_text,
                    view_to_find = {"resourceId": "com.android.settings:id/password"},
                    value = self.password,
                    scroll = True,
                    is_password = True)()

            if self.EAP_method:
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/method"})()
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"textContains": self.EAP_method},
                    view_to_check = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/method"})()

            if self.phase_2_auth:
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/phase2"})()
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"textContains": self.phase_2_auth},
                    view_to_check = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/phase2"})()

            if self.EAP_method and not self.user_certificate and \
               ui_utils.is_view_displayed(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/ca_cert"}):
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/ca_cert"})()
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"textContains": self.device_info.wifi_ca_certificate_none_id},
                    view_to_check = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/ca_cert"})()

            if self.user_certificate:
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/user_cert"})()

                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"textContains": self.user_certificate},
                    view_to_check = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/user_cert"})()

            if self.ca_certificate:
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/ca_cert"})()

                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"textContains": self.ca_certificate},
                    view_to_check = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/ca_cert"})()

            if self.identity:
                ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                    view_to_find = {"resourceId": "com.android.settings:id/identity"},
                    value = self.identity,
                    is_password = False)()

            if self.anonymous_identity:
                ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                    view_to_find = {"resourceId": "com.android.settings:id/anonymous"},
                    value = self.anonymous_identity,
                    is_password = False)()

            if self.device_info.dessert in ['N'] and self.EAP_method == "TLS" and \
            ui_utils.is_view_displayed(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/ca_cert"}):
                ui_steps.click_button_with_scroll(serial=self.serial,
                    view_to_find={"className": "android.widget.Spinner",
                                  "resourceId": "com.android.settings:id/ca_cert"})()
                ui_steps.click_button_with_scroll(serial=self.serial,
                    view_to_find={"textContains": self.device_info.wifi_ca_certificate_none_id},
                    view_to_check={"className": "android.widget.Spinner",
                                    "resourceId": "com.android.settings:id/ca_cert"})()

        self.step_data = True

    def check_condition(self):
        return self.step_data

class set_wifi_advanced_options(wifi_step):

    def __init__(self, proxy=None, proxy_pac_url=None, proxy_hostname=None, proxy_port=None, proxy_bypass=None,
        ip_settings=None, ip_address=None, gateway=None, network_prefix_length=None, dns1=None, dns2=None, clear_text_boxes=True, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.proxy = proxy
        self.proxy_pac_url = proxy_pac_url
        self.proxy_hostname = proxy_hostname
        self.proxy_port = proxy_port
        self.proxy_bypass = proxy_bypass
        self.ip_settings = ip_settings
        self.ip_address = ip_address
        self.gateway = gateway
        self.network_prefix_length = network_prefix_length
        self.dns1 = dns1
        self.dns2 = dns2
        self.clear_text = clear_text_boxes
        self.step_data = False

        self.advanced = (self.proxy or self.ip_settings) != None


    def do(self):
        # If we want to set 'proxy' or 'ip settings', we need to check the 'Advanced" checkbox.
        if self.advanced:
            ui_steps.click_checkbox_button(serial=self.serial,
                    view_to_find = {"resourceId": "com.android.settings:id/wifi_advanced_togglebox"},
                    state = "ON",
                    relationship = "sibling")()

            # Complete the proxy settings
            if self.proxy:
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/proxy_settings"})()
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"textContains": self.proxy})()

                if self.proxy_hostname:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/proxy_hostname"},
                        value = self.proxy_hostname,
                        is_password = False)()

                if self.proxy_port:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/proxy_port"},
                        value = self.proxy_port,
                        is_password = False)()

                if self.proxy_bypass:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/proxy_exclusionlist"},
                        value = self.proxy_bypass,
                        is_password = False)()

                if self.proxy_pac_url:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/proxy_pac"},
                        value = self.proxy_pac_url,
                        is_password = False)()

            # Complete the IP Settings
            if self.ip_settings:
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"className": "android.widget.Spinner", "resourceId": "com.android.settings:id/ip_settings"})()
                ui_steps.click_button_with_scroll(serial = self.serial,
                    view_to_find = {"textContains": self.ip_settings})()

                if self.gateway:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/gateway"},
                        value = self.gateway,
                        is_password = False)()

                if self.network_prefix_length:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/network_prefix_length"},
                        value = self.network_prefix_length,
                        is_password = False)()

                if self.dns1:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/dns1"},
                        value = self.dns1,
                        is_password = False)()

                if self.dns2:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/dns2"},
                        value = self.dns2,
                        is_password = False)()

                if self.ip_address:
                    ui_steps.edit_text(scroll = True, clear_text = self.clear_text, serial = self.serial,
                        view_to_find = {"resourceId": "com.android.settings:id/ipaddress"},
                        value = self.ip_address,
                        is_password = False)()
        self.step_data = True

    def check_condition(self):
        return self.step_data

class add_network(wifi_step):

    """ description:
            Add a Wi-Fi network.

        usage:
            wifi_steps.add_network(ssid='MyAP', security='None')()
            wifi_steps.add_network(ssid='MyAP', security='WEP', password='awsomepassword123')()

        valid_config - it will check weather the Save button is available to click

        tags:
            ui, android, ssid, wifi
    """

    def __init__(self, ssid, security, password=None, EAP_method=None, phase_2_auth=None, user_certificate=None, ca_certificate=None, identity=None, anonymous_identity=None,
        proxy=None, proxy_pac_url=None, proxy_hostname=None, proxy_port=None, proxy_bypass=None,
        ip_settings=None, ip_address=None, gateway=None, network_prefix_length=None, dns1=None, dns2=None,
        valid_config=True, apply_config=True, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ssid = ssid
        self.security = security
        self.password = password
        self.EAP_method = EAP_method
        self.phase_2_auth = phase_2_auth
        self.user_certificate = user_certificate
        self.ca_certificate = ca_certificate
        self.identity = identity
        self.anonymous_identity = anonymous_identity
        self.proxy = proxy
        self.proxy_pac_url = proxy_pac_url
        self.proxy_hostname = proxy_hostname
        self.proxy_port = proxy_port
        self.proxy_bypass = proxy_bypass
        self.ip_settings = ip_settings
        self.ip_address = ip_address
        self.gateway = gateway
        self.network_prefix_length = network_prefix_length
        self.dns1 = dns1
        self.dns2 = dns2
        self.valid_config = valid_config
        self.apply_config = apply_config

    def wrap_do(self):
        # Go to Settings - Wi-Fi menu and turn on Wi-Fi
        set_wifi(serial=self.serial, state="ON")()
        open_wifi_settings(serial=self.serial)()

        # TODO: Implement steps in ui_steps for using spinners and check boxes?

        # Click "More"
        if self.device_info.dessert in ["L", "M"]:
            ui_steps.click_button(serial=self.serial,
                                  view_to_find={"descriptionContains": "More"},
                                  view_to_check={"textContains": "Add network"})()
        if self.device_info.dessert in ["N"]:
            ui_steps.wait_for_view_with_scroll(serial=self.serial, timeout=10000,
                                               view_to_find={"resourceId": "android:id/icon_frame"},
                                               iterations=5)()

        # Click "Add network"
        ui_steps.click_button_with_scroll(serial=self.serial,
                                          view_to_find={"textContains": "Add network"},
                                          view_to_check={"textContains": "Network name"})()

        # Complete the SSID text field
        ui_steps.edit_text(serial=self.serial,
                           view_to_find={"resourceId": "com.android.settings:id/ssid"},
                           value=self.ssid,
                           is_password=False)()
    def do(self):
        self.wrap_do()
        try:
            set_wifi_security(serial=self.serial, security=self.security, password=self.password, EAP_method=self.EAP_method, phase_2_auth=self.phase_2_auth,
                user_certificate=self.user_certificate, identity=self.identity, anonymous_identity=self.anonymous_identity, ca_certificate=self.ca_certificate, clear_text_boxes=False)()
        except Exception:
            self.logger.debug("Install certificate again for TLS test retry")
            ui_steps.press_home(serial=self.serial)()
            install_WIFI_certificate(serial=self.serial, cert_pass="whatever", cert_name="TLS_certificate", dut_pin=1234)()
            self.wrap_do()

        set_wifi_advanced_options(serial=self.serial, proxy=self.proxy, proxy_pac_url=self.proxy_pac_url, proxy_hostname=self.proxy_hostname, proxy_port=self.proxy_port,
            proxy_bypass=self.proxy_bypass, ip_settings=self.ip_settings, ip_address=self.ip_address, gateway=self.gateway, network_prefix_length=self.network_prefix_length,
            dns1=self.dns1, dns2=self.dns2, clear_text_boxes=False)()

        # Click Save
        apply_config_btn = {"className": "android.widget.Button", "enabled": True}
        apply_config_btn.update(self.device_info.wifi_add_network_save_btn_id)
        if self.apply_config:
            if self.valid_config:
                if self.device_info.dessert != "O":
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = apply_config_btn,
                                          view_to_check = {"descriptionContains": "More"})()
                else:
                    ui_steps.click_button(serial=self.serial,
                                          view_to_find=apply_config_btn,
                                          view_to_check=None)()
            else:
                apply_config_btn.update({"enabled": False})
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = apply_config_btn)()
        else:
            ui_steps.click_button(serial = self.serial,
                view_to_find = apply_config_btn)()

        # Sometimes, the AP will not connected at once after the account has been saved, so force to connect to the AP
        if self.apply_config:
            if self.uidevice(resourceId = "android:id/title", text = self.ssid).wait.exists(timeout=1000):
                self.uidevice(resourceId = "android:id/title", text = self.ssid).click.wait()
                if self.uidevice(textMatches = "^(?i)connect$").exists:
                    self.uidevice(textMatches = "^(?i)connect$").click.wait()

    def check_condition(self):
        # If we made it here without errors, it means the step is passed.
        return True


class modify_network(wifi_step):

    """ description:
            Modify an existing Wi-Fi network.

        usage:
            wifi_steps.modify_network(ssid='MyAP', password='1234')()

        valid_config - it will check weather the Save button is available to click

        tags:
            ui, android, ssid, wifi
    """


    def __init__(self, ssid, password=None, proxy=None, proxy_pac_url=None, proxy_hostname=None, proxy_port=None, proxy_bypass=None,
        ip_settings=None, ip_address=None, gateway=None, network_prefix_length=None, dns1=None, dns2=None, valid_config=True, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ssid = ssid
        self.password = None
        self.proxy = proxy
        self.proxy_pac_url = proxy_pac_url
        self.proxy_hostname = proxy_hostname
        self.proxy_port = proxy_port
        self.proxy_bypass = proxy_bypass
        self.ip_settings = ip_settings
        self.ip_address = ip_address
        self.gateway = gateway
        self.network_prefix_length = network_prefix_length
        self.dns1 = dns1
        self.dns2 = dns2
        self.valid_config = valid_config

        self.clear_text = True
        self.advanced = (self.proxy or self.ip_settings) != None
        self.ssid_view = {"textContains": self.ssid, "resourceId":"android:id/title"}
        self.modify_network_view = {"textContains": "Modify network", "resourceId":"android:id/title"}

    def do(self):
        # Go to Settings - Wi-Fi menu and turn on Wi-Fi
        set_wifi(serial=self.serial, state="ON")()
        open_wifi_settings(serial=self.serial)()

        # Scroll to find the SSID
        if self.uidevice(className="android.widget.ScrollView", scrollable=True).wait.exists(timeout=1000):
            self.uidevice(scrollable = True).scroll.to(**self.ssid_view)

        # Long click the wifi SSID
        ui_steps.long_click(serial=self.serial,
            view_to_find = self.ssid_view,
            view_to_check = self.modify_network_view)()

        # Click Modify Network
        ui_steps.click_button(serial=self.serial,
            view_to_find = self.modify_network_view,
            view_to_check = {"textContains": self.ssid})()

        if adb_utils.is_virtual_keyboard_on(serial = self.serial):
            ui_steps.press_back(serial = self.serial)()

        # Modify the password, if applicable
        if self.password:
            ui_steps.edit_text(serial = self.serial, clear_text = True,
                view_to_find = {"resourceId": "com.android.settings:id/password"},
                value = self.password,
                scroll = False,
                is_password = True)()

        # Set the advanced options
        set_wifi_advanced_options(serial=self.serial, proxy=self.proxy, proxy_pac_url=self.proxy_pac_url, proxy_hostname=self.proxy_hostname, proxy_port=self.proxy_port,
            proxy_bypass=self.proxy_bypass, ip_settings=self.ip_settings, ip_address=self.ip_address, gateway=self.gateway, network_prefix_length=self.network_prefix_length,
            dns1=self.dns1, dns2=self.dns2, clear_text_boxes=self.clear_text)()

        # Click Save
        if self.valid_config:
            ui_steps.click_button(serial = self.serial,
                view_to_find = {"className": "android.widget.Button", "textMatches": "(?i)save", "enabled": True})()
        else:
            ui_steps.click_button(serial = self.serial,
                view_to_find = {"className": "android.widget.Button", "textMatches": "(?i)save", "enabled": False})()

    def check_condition(self):
        # If we made it here without errors, it means the step is passed.
        return True

class ping_gateway(wifi_step):
    """
    Description:
                Pings the DUT gateway <trycount> times.
                Will fail if the packet loss is less then or equal to <target_percent>
    """

    def __init__(self, trycount=2, timeout=15, target_percent=50,**kwargs):
        wifi_step.__init__(self, **kwargs)
        self.trycount = trycount
        self.timeout = timeout
        self.target_percent = target_percent

    def do(self):
        wifi_conf = self.adb_connection.parse_cmd_output("dumpsys wifi")
        self.wifi_info = wifi_utils.get_connection_info(wifi_conf)
        (self.status, loss, ping_output) = wifi_utils.ping(ip=self.wifi_info['Gateway'], trycount=self.trycount,
                                      target_percent=self.target_percent,timeout=self.timeout, serial=self.serial)

        ip=self.wifi_info['Gateway']

        if loss:
            step_message = "=== pinging address " + str(ip) + " lost " + str(loss) + "% out of " + str(self.trycount) + " packets ==="
        else:
            step_message = "Could not determine loss percent. Ping output was: \" {} \"".\
                            format(ping_output)

        self.set_passm(step_message)
        self.set_errorm("", step_message)

    def check_condition(self):
        return self.status

class ping_ip(wifi_step):
    """
    Description:
                Pings the IP for <trycount> times.
                It will fail if the packet loss is less then or equal to <target_percent>
    """

    def __init__(self, ip=None, trycount=2, timeout=15,
                 target_percent=50, negative = False, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ip = ip
        self.trycount = trycount
        self.timeout = timeout
        self.target_percent = target_percent
        self.negative = negative

    def do(self):
        if self.ip != None:
            # run ping only if the destination ip address is valid
            (self.status, loss, ping_output) = wifi_utils.ping(ip=self.ip, trycount=self.trycount,
                                      target_percent=self.target_percent,
                                      timeout=self.timeout, serial=self.serial)
            if loss:
                step_message = "=== pinging address " + str(self.ip) + " lost " + str(loss) + "% out of " + str(self.trycount) + " packets ==="
            else:
                step_message = "Could not determine loss percent. Ping output was: \" {} \"".\
                                format(ping_output)
            self.set_passm(step_message)
            self.set_errorm("", step_message)
        else:
            step_message = 'Ping failure due to invalid destination IP address!!! Check device connectivity!!! IP is: ' + str(self.ip)
            self.set_errorm("", step_message)
            self.status = False
            self.negative = False # in case of invalid destination IP,
                                  # ping_ip step should fail also for "negative" cases

    def check_condition(self):
        if self.negative:
            return not self.status
        return self.status


class ping_ipv6_ip(wifi_step):
    """
    Description:
                Pings the ipv6 IP for <trycount> times.
                It will fail if the packet loss is less then or equal to <target_percent>
    """

    def __init__(self, ip=None, trycount=2, timeout=15,
                 target_percent=50, negative = False, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ip = ip
        self.trycount = trycount
        self.timeout = timeout
        self.target_percent = target_percent
        self.negative = negative
        print ip

    def do(self):
        if self.ip != None:
            # run ping only if the destination ip address is valid
            try:
                (self.status, loss, ping_output) = wifi_utils.ping(ip=self.ip, trycount=self.trycount,
                                          target_percent=self.target_percent,
                                          timeout=self.timeout, serial=self.serial)
            except AssertionError:
                (self.status, loss, ping_output) = wifi_utils.ping_ipv6(ip=self.ip, trycount=self.trycount,
                                          target_percent=self.target_percent,
                                          timeout=self.timeout, serial=self.serial)
            if loss:
                step_message = "=== pinging address " + str(self.ip) + " lost " + str(loss) + "% out of " + str(self.trycount) + " packets ==="
            else:
                step_message = "Could not determine loss percent. Ping output was: \" {} \"".\
                                format(ping_output)
            self.set_passm(step_message)
            self.set_errorm("", step_message)
        else:
            step_message = 'Ping failure due to invalid destination IP address!!! Check device connectivity!!! IP is: ' + str(self.ip)
            self.set_errorm("", step_message)
            self.status = False
            self.negative = False # in case of invalid destination IP,
                                  # ping_ip step should fail also for "negative" cases

    def check_condition(self):
        if self.negative:
            return not self.status
        return self.status


class find_available_ip(wifi_step):
    """
        returns the first IP that is available from a range of IPs

    """

    def __init__(self, ip_range = None, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.step_data = None
        self.ip_range = ip_range

    def do(self):
        if self.ip_range == None:
            self.ip_range = wifi_utils.get_ip_range(self.serial)
        if self.ip_range:
            start_ip = self.ip_range.split("_")[0]
            end_ip = self.ip_range.split("_")[1]
            current_ip = start_ip

            found = False

            while not found and current_ip != end_ip:
                if current_ip.split('.')[-1]!= '255' and current_ip.split('.')[-1]!= '0' and current_ip.split('.')[-1]!= '1':
                    found = not wifi_utils.ping(current_ip,trycount=1,serial=self.serial)[0]
                if not found:
                    current_ip = wifi_utils.int_to_ip(wifi_utils.ip_to_int(current_ip) + 1)
            if found:
                self.step_data = current_ip
            return self.step_data
        else:
            step_message = 'Could not determine available IP addresses range! Check device connectivity!!!'
            self.set_errorm("", step_message)

    def check_condition(self):
        if self.step_data:
            return self.step_data
        else:
            return  False


class check_network_information(wifi_step):
    """
    Description: Check the network information on device.
    """

    def __init__(self, ap_name, status=None, link_speed=None,
                 frequency=None, security=None, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.ap_name = ap_name

        self.params_to_check = {}
        if status != None:
            self.params_to_check["Status"]=status
        if link_speed != None:
            self.params_to_check["Link speed"]=link_speed
        if frequency != None:
            self.params_to_check["Frequency"]=frequency
        if security != None:
            self.params_to_check["Security"]=security


    def do(self):
        error_msg = ""
        self.decision = True
        set_wifi(serial=self.serial, state="ON")()
        open_wifi_settings(serial=self.serial)()

        # Click "More"
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text": self.ap_name},
            view_to_check = {"textContains": "Signal strength"})()

        for param in self.params_to_check.keys():
            if not self.uidevice(text=param).down(textContains=self.params_to_check[param]):
                error_msg += "Parameter '{0}' has different value than expected '{1}'.\n".\
                             format(param, self.params_to_check[param])
                self.decision = False
        self.set_errorm("", error_msg)

    def check_condition(self):
        return self.decision



class check_mac(wifi_step):
    """
    Description: Checks the MAC in GUI.
    """

    def __init__(self, **kwargs):
        wifi_step.__init__(self, **kwargs)

    def do(self):
        open_wifi_settings(serial = self.serial)()
        # Click "More"
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"descriptionContains": "More"},
            view_to_check = {"textContains": "Advanced"})()

        # Click "Advanced"
        ui_steps.click_button(serial =self.serial,
            view_to_find = {"textContains": "Advanced"},
            view_to_check = {"textContains": u"Advanced Wi\u2011Fi"})()

        self.mac = self.uidevice(text="MAC address").down(className="android.widget.TextView").info['text']

    def check_condition(self):
        return wifi_utils.check_mac_format(self.mac)


class open_wifi_direct_settings(wifi_step):
    """
    Description: Opens the Wifi Direct settings.
    """
    def __init__(self, use_adb=False ,**kwargs):
        wifi_step.__init__(self, **kwargs)
        self.use_adb = use_adb

    def do(self):
        if self.use_adb:
            if self.uidevice(resourceId="android:id/alertTitle").wait.exists(timeout = 1000):
                adb_steps.command(serial = self.serial, timeout = 10,
                        command = "input keyevent 4")() # preventive clear any pop-up blocking the screen
            self.adb_connection.parse_cmd_output("am start -n"
                        " com.android.settings/.Settings\$WifiP2pSettingsActivity",
                                                 dont_split=True)
        else:
            open_wifi_settings(serial=self.serial)()
            time.sleep(3)
            ui_steps.click_button_common(serial=self.serial, view_to_find={"textMatches": "Wi.Fi preferences"},
                                          optional=True)()
            time.sleep(3)
            ui_steps.click_button_common(serial=self.serial, view_to_find={"descriptionContains": "More"},
                                          optional=True)()
            ui_steps.click_button_common(serial=self.serial, view_to_find={"text": "Advanced"})()
            ui_steps.click_button_common(serial=self.serial, view_to_find={"textMatches": "Wi.Fi Direct"})()
            time.sleep(5)

    def check_condition(self):
        """if not self.uidevice(text="Remembered groups").wait.exists(timeout=1000):
            ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                           view_to_find = {"text":"Remembered groups"},
                                           iterations = 1)()"""
        ##checking for RENAME DEVICE as this is avaialble in M,N,O
        return self.uidevice(textStartsWith="RENAME DEVICE").wait.exists(timeout=1000)


class p2p_respond_to_connect(wifi_step):
    """
    Description: Responds to wifi direct connection request.
    """

    def __init__(self, master_name, accept_connect = True, known_device=False, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.accept_connect = accept_connect
        self.master_name=master_name
        self.known_device = known_device

    def do(self):
        if not self.known_device:
            response = self.device_info.wifi_p2p_connect_response_accept_btn_id
            if not self.accept_connect:
                response = self.device_info.wifi_p2p_connect_response_decline_btn_id

            self.uidevice(**response).wait.exists(timeout=20000)
            self.uidevice(**response).click.wait()

    def check_condition(self):

        ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                           view_to_find = {"text":self.master_name},
                                           iterations = 1)()
        if self.accept_connect:
            text_to_find = "Connected"
        else:
            text_to_find = "Available"

        return wifi_utils.wait_p2p_state(self.uidevice, self.master_name, state=text_to_find)

class p2p_connect_devices(wifi_step):
    """
    Description: Connects 2 devices.
    """

    def __init__(self, slave_name, master_name, serial2, accept_connect=True,
                 known_device=False, no_response=False, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.slave_name = slave_name
        self.serial2 = serial2
        self.master_name = master_name
        self.accept_connect = accept_connect
        self.known_device = known_device
        self.no_response = no_response

    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.press_home(serial = self.serial2)()
        set_wifi_state(serial = self.serial, state = "ON")()
        set_wifi_state(serial = self.serial2, state = "ON")()

        #open_wifi_direct_settings(serial = self.serial)()
        #open_wifi_direct_settings(serial = self.serial2)()

        #####search_for_devices()()
        ## TODO: investiage: wait.gone(SEARCHING...) ????
        ## TODO: investigate the need to use "search_devices" steps if the
        ## devices list is empty
        # adding scan for cases where the intended peer is not (yet) discovered
        p2p_scan_devices(serial = self.serial)()
        p2p_scan_devices(serial = self.serial2)()

        ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                           view_to_find = {"text":self.slave_name})()
        wifi_utils.wait_p2p_state(self.uidevice, self.slave_name, state="Available")

        self.uidevice(text=self.slave_name).click.wait()
        ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                           view_to_find = {"text":self.slave_name})()

        if not self.known_device:
            wifi_utils.wait_p2p_state(self.uidevice, self.slave_name, state="Invited")
        else:
            wifi_utils.wait_p2p_state(self.uidevice, self.slave_name, state="Connected")

        ## on slave device click accept
        if not self.no_response:
            p2p_respond_to_connect(master_name = self.master_name, serial = self.serial2,
                               accept_connect=self.accept_connect,
                               known_device=self.known_device)()

    def check_condition(self):
        state = "Invited"
        if self.accept_connect:
            state = "Connected"
        if wifi_utils.wait_p2p_state(self.uidevice, self.slave_name, state=state):
            return True
        else:
            self.set_errorm("", "Connection state '{0}' not found for connection '{1}'".format(state, self.slave_name))
            return False


class p2p_connect_devices_prompt(wifi_step):
    """
    Description: Connects 2 devices.
    """

    def __init__(self, slave_name, master_name, serial2, platform_cli=None, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.slave_name = slave_name
        self.serial2 = serial2
        self.master_name = master_name
        self.device_info = platform_cli

    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.press_home(serial = self.serial2)()
        set_wifi_state(serial = self.serial, state = "ON")()
        set_wifi_state(serial = self.serial2, state = "ON")()

        #open_wifi_direct_settings(serial = self.serial)()
        #open_wifi_direct_settings(serial = self.serial2)()

        #####search_for_devices()()
        ## TODO: investiage: wait.gone(SEARCHING...) ????
        ## TODO: investigate the need to use "search_devices" steps if the
        ## devices list is empty
        # adding scan for cases where the intended peer is not (yet) discovered
        for iteration in range(0, 3):
            #p2p_scan_devices(serial = self.serial)()
            #p2p_scan_devices(serial = self.serial2)()
            open_wifi_direct_settings(serial = self.serial)()
            open_wifi_direct_settings(serial = self.serial2)()
            time.sleep(3)
            ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                           view_to_find = {"text":self.slave_name})()
            wifi_utils.wait_p2p_state(self.uidevice, self.slave_name, state="Available")

            self.uidevice(text=self.slave_name).click.wait()
            ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                           view_to_find = {"text":self.slave_name})()

            response = self.device_info.wifi_p2p_connect_response_accept_btn_id

            self.uidevice_2 = ui_device(serial=self.serial2)

            if self.uidevice_2(text="Invitation to connect").wait.exists(timeout=1000):
                self.uidevice_2(**response).click.wait()
            if wifi_utils.wait_p2p_state(self.uidevice_2, self.master_name, state='Connected'):
                break

            if wifi_utils.wait_p2p_state(self.uidevice, self.slave_name, state='Invited'):
                self.uidevice(text=self.slave_name).click.wait()
                if self.uidevice(textContains="Cancel invitation").wait.exists(timeout=1000):
                    self.uidevice(text="OK").click.wait()
                ui_steps.press_home(serial=self.serial)()

    def check_condition(self):

        state = "Connected"
        if wifi_utils.wait_p2p_state(self.uidevice, self.slave_name, state=state):
            return True
        else:
            self.set_errorm("", "Connection state '{0}' not found for connection '{1}'".format(state, self.slave_name))
            return False

class p2p_connect_devices_cli(wifi_step):
    """
    Description: Connects 2 devices through wpa_cli connection request guaranteeing group ownership
    The client uses UI to respond to the invitation
    """

    def __init__(self, slave_name, master_name, serial2, accept_connect=True,
                 known_device=False, platform_go=None, platform_cli=None, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.slave_name = slave_name
        self.serial2 = serial2
        self.master_name = master_name
        self.accept_connect = accept_connect
        self.known_device = known_device
        if platform_go:
            self.platform_go = platform_go
        else:
            self.platform_go = statics.Device(serial=self.serial)

        if platform_cli:
            self.platform_cli = platform_cli
        else:
            self.platform_cli = statics.Device(serial=self.serial2)

    def do(self):
        for i in range(0,5): # max 3 connect attempts
            slave_mac = wifi_utils.p2p_get_interface_mac(serial = self.serial2, get_if_tool=self.platform_cli.get_interfaces_tool)
            adb_steps.command(serial = self.serial, command = self.platform_go.p2p_wpa_cli_connect(mac_address=slave_mac))()

            time.sleep(2)
            timer = 0
            while timer < 5:
                self.uidevice_2 = ui_device(serial = self.serial2)
                if self.uidevice_2(text="Invitation to connect").wait.exists(timeout=1000):
                    state_after_respond = wifi_utils.p2p_respond_to_connect_cli(
                                master_name=self.master_name,
                                serial=self.serial2,
                                accept_connect=self.accept_connect,
                                known_device=self.known_device,
                                device_info=self.platform_cli)
                    break
                time.sleep(1)
                timer += 1
            if self.uidevice(textContains = "The tablet will temporarily disconnect from Wi-Fi while it's connected to").wait.exists(timeout=10000):
                raise Exception('Connection error! - WiFi Frequency Channel Conflict! - Please rerun the test')
            state = "Invited"
            if self.accept_connect:
                state = "Connected"
            if wifi_utils.wait_p2p_state(self.uidevice_2, self.master_name, state=state):
                break
            else:
                if self.uidevice_2(text="Invitation to connect").wait.exists(timeout=1000):
                    adb_steps.command(serial = self.serial2, timeout = 10,
                        command = "input keyevent 4")() # to clear the Invitation pop up on the client and free the screen for next Scan
                p2p_scan_devices(serial = self.serial )()
                p2p_scan_devices(serial = self.serial2)()
                time.sleep(5) #waiting for scan to publish discovered devices

    def check_condition(self):
        state = "Invited"
        if self.accept_connect:
            state = "Connected"
        if wifi_utils.wait_p2p_state(self.uidevice_2, self.master_name, state=state):
            return True
        else:
            self.set_errorm("", "Connection state '{0}' not found for connection '{1}'".format(state, self.slave_name))
            return False

class p2p_forget_group(wifi_step):
    """
    Description: Removes a wifi direct group.
    """

    def __init__(self, group_name, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.group_name = group_name

    def do(self):
        open_wifi_direct_settings(serial = self.serial)()
        if self.group_name != "Available" and self.uidevice(text=self.group_name).wait.exists(timeout=1000):
            forget_retries = 0
            while forget_retries < 3:
                try:
                    ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                           view_to_find = {"text":self.group_name},
                                           iterations = 1)()
                    ui_steps.click_view(serial = self.serial,
                                        view = self.uidevice(text=self.group_name),
                                        view_to_check = {"textContains": "Forget this group"})()
                    break
                except:
                    time.sleep(1)
                forget_retries += 1

            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text": "OK"},
                            view_presence = False,
                            view_to_check = {"text": self.group_name})()

    def check_condition(self):
        # if we got here, the step is passed
        return True


class p2p_forget_all_groups(wifi_step):
    """
    Description: Removes all groups.
    """

    def __init__(self, **kwargs):
        self.groups_exist = True
        wifi_step.__init__(self, **kwargs)
        self.msg = None

    def __collect_group_to_forget(self):
        """ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                               view_to_find = {"text":"Remembered groups"},
                               iterations = 1)()"""
        ui_steps.wait_for_view_common(serial=self.serial, timeout=30000, optional=True,
                            view_to_find = {"text": "Remembered groups"},
                            iterations = 1)()
        if not self.uidevice(text="Remembered groups"):
            self.msg = "No action as 'Rememnbered group' is not found"
            return
        try:
            self.group = self.uidevice(text="Remembered groups").down(className="android.widget.TextView")
        except IndexError as e:
            self.logger.debug("Treated group index Error: {0}".format(str(e.message)))
            self.group = self.uidevice(text="Remembered groups").down(className="android.widget.TextView")
        except uiautomator.NotFoundHandler as e:
            self.logger.debug("Remembered groups not found exception:{0}".format(str(e.message)))
            self.group = self.uidevice(text="Remembered groups").down(className="android.widget.TextView")
        return self.group

    def do(self):
        open_wifi_direct_settings(serial = self.serial)()
        self.uidevice.wait.idle(timeout = 10000)
        time.sleep(3)
        if self.__collect_group_to_forget():
            if self.groups_exist:
                while self.group:
                    p2p_forget_group(serial = self.serial, group_name=self.group.info['text'])()
                    self.__collect_group_to_forget()

    def check_condition(self):
        if self.msg is not None:
            self.set_passm(self.msg)
        # if we got here, the step is passed
        return True

class p2p_disconect(wifi_step):
    """
    Description: Disconnects a device.
    """
    # TODO: investigate if needs reworking???: control both devices in the same time
    #

    def __init__(self, peer_name, text="Connected", text_to_check="Disconnect", **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.peer_name = peer_name
        self.text = text
        self.text_to_check = text_to_check

    def do(self):
        open_wifi_direct_settings(serial = self.serial)()
        self.uidevice.wait.idle(timeout = 10000)
        try:
            # this could throw error if device disconnects in the mean time and
            # no longer appears in the list of Peer Devices
            ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                               view_to_find = {"text":self.peer_name},
                                               iterations = 1)()
            if self.uidevice(text=self.peer_name).sibling(text=self.text):
                disconnect_retries = 0
                while disconnect_retries < 3:
                    try:
                        ui_steps.click_view(serial = self.serial,
                                        view = self.uidevice(text=self.peer_name),
                                        view_to_check = {"textContains": self.text_to_check})()
                        break
                    except:
                        time.sleep(1)
                    disconnect_retries += 1

                ui_steps.click_button(serial = self.serial,
                                view_to_find = {"text": "OK"})()
        except Exception, e:
            print "P2P disconnect peer exception : \n", e
            print "P2P disconnect peer exception type : \n", type(e)
            if "Did not find view" in str(e.message):
                # selected peer no longer connected. nothing to disconnect
                self.logger.debug("Treated device no longer connected Error: {}".format(e))
            else:
                raise


    def check_condition(self):
        # check disconnected
        ## TODO: use search before check??? on both devices???
        return self.uidevice(text=self.peer_name).sibling(text="Available") != None

class p2p_disconect_all(wifi_step):
    """
    Description: Disconnects all conenctions.
    """

    def __init__(self, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.msg=None

    def __disconnect_devices_of_type(self, peer_type, text_to_check):
        """ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                           view_to_find = {"text":"Peer devices"},
                                           iterations = 1)()"""
        if not ui_steps.wait_for_view_common(serial=self.serial, timeout=30000,
                                           view_to_find={"text": "Peer devices"}, optional=True)():
            self.msg= "No peer devices are listed"
            return

        if self.uidevice(text="Peer devices").down(text=peer_type) != None:
            # for situations where due to device state change (going from connected to available due to timeout)
            # we put the disconnect items list creation and disconnect action in a try/catch sequence
            try:
                device_connected = True
                #~ connected_peer = self.uidevice(text="Peer devices").down(resourceId="android:id/title").down(text=peer_type)
                connected_peer = self.uidevice(text="Peer devices").down(text=peer_type)
            except Exception, e:
                print "Connected peer selection exception : \n", e
                print "Connected peer selection exception type : \n", type(e)
                # if the selection of the UI element throws such an error, it is no longer on the screen due to transition
                # hence we simply log the error and mark it as not connected any more
                if "com.android.uiautomator.core.UiObjectNotFoundException: UiSelector" in str(e.message) or "IndexError" in str(e.message):
                    device_connected = False
                    self.logger.debug("Treated Selector Error: {}".format(e))
                else:
                # for different errors, we raise them
                    raise
            if device_connected:
                while connected_peer:
                    p2p_disconect(serial = self.serial, text = peer_type, text_to_check = text_to_check, peer_name=connected_peer.info['text'])()
                    ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 30000,
                                               view_to_find = {"text":"Peer devices"},
                                               iterations = 1)()
                    try:
                        if self.uidevice(text="Peer devices").down(text=peer_type) != None:
                            #~ connected_peer = self.uidevice(text="Peer devices").down(resourceId="android:id/title").down(text=peer_type)
                            connected_peer = self.uidevice(text="Peer devices").down(text=peer_type)
                        else:
                            connected_peer = None
                    except Exception, e:
                        print "Connected peer selection exception : \n", e
                        print "Connected peer selection exception type : \n", type(e)
                        # if the selection of the UI element throws such an error, it is no longer on the screen due to transition
                        # hence we simply log the error and mark it as not connected any more
                        if "com.android.uiautomator.core.UiObjectNotFoundException: UiSelector" in str(e.message) or "IndexError" in str(e.message):
                            connected_peer = None
                            self.logger.debug("Treated Selector Error 2: {}".format(e))
                        else:
                            # for different errors, we raise them
                            raise


    def do(self):
        open_wifi_direct_settings(serial = self.serial)()
        # for connected devices
        self.__disconnect_devices_of_type(peer_type = "Connected", text_to_check="Disconnect")
        # for invited devices
        self.__disconnect_devices_of_type(peer_type = "Invited", text_to_check="Cancel invitation")

    def check_condition(self):
        # if we got here, the step is passed
        if self.msg is not None:
            self.set_passm(self.msg)
        return True


class p2p_rename_device(wifi_step):
    """
    Description: Renames a p2p device.
    """

    def __init__(self, new_name, commit_change = True, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.new_name = new_name
        self.commit_change = commit_change
        self.msg=None


    def do(self):
        open_wifi_direct_settings(serial = self.serial)()
        """ui_steps.wait_for_view_with_scroll(serial = self.serial, timeout = 10000,
                               view_to_find = {"text":"Peer devices"},
                               iterations = 1)()"""
        if not ui_steps.wait_for_view_common(serial=self.serial, timeout=10000,
                                           view_to_find={"text": "Peer devices"}, optional=True)():
            self.msg = "No action as Peer devices Text is not found"

        try:
            current_name = self.uidevice(text="Peer devices").up(className="android.widget.TextView").info['text']
        except Exception as e:
            self.logger.debug("Treated peer devices index Exception: {0}".format(str(e.message)))
            current_name = self.uidevice(text="Peer devices").up(className="android.widget.TextView").info['text']
        except uiautomator.NotFoundHandler as e:
            self.logger.debug("Treated peer device not found exception: {0}".format(str(e.message)))
            current_name = self.uidevice(text="Peer devices").up(className="android.widget.TextView").info['text']
        # only rename if new_name and actual_name are different
        if current_name != self.new_name:
            if self.uidevice(text="Search for devices").wait.exists(timeout=1000):
                ui_steps.click_view(serial=self.serial,
                                    view=self.uidevice(text="Search for devices")
                                    )()
            ui_steps.click_view(serial = self.serial,
                                view = self.uidevice(**self.device_info.wifi_p2p_rename_device_id),
                                view_to_check = {"textMatches": "(?i)rename device", "resourceId": "android:id/alertTitle"})()

            ui_steps.edit_text(serial = self.serial, clear_text = True,
                view_to_find = {"className": "android.widget.EditText"},
                value = self.new_name,
                scroll = False)()
            if self.commit_change:
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"text": "OK"})()
            else:
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"text": "Cancel"})()

    def check_condition(self):
        if self.msg is not None:
           self.set_passm(self.msg)
        result = self.uidevice(text=self.new_name, index=0)
        if self.commit_change:
            return self.uidevice(text=self.new_name, index=0) != None
        else:
            return not self.uidevice(text=self.new_name, index=0) == None


class p2p_scan_devices(wifi_step):
    """
    Description: Scans for p2p devices.
    """

    def __init__(self, **kwargs):
        wifi_step.__init__(self, **kwargs)

    def do(self):
        open_wifi_direct_settings(serial = self.serial)()
        # clear invited peers, if any
        p2p_disconect_all(serial=self.serial)()
        # this is a workaroud, on sopia 3GR, clicking the 'search for devices' widget cannot take effect
        # So go to home first and then come back to refresh the UI by force
        if self.device_info._target==None:
            self.device_info._initialize_target()
        if isinstance(self.device_info._target, statics.s3gr10m6s_trusty):
            ui_steps.press_home(serial = self.serial)()
            open_wifi_direct_settings(serial = self.serial)()
        if self.uidevice(**self.device_info.wifi_p2p_search_for_devices_id):
            ui_steps.click_view(serial = self.serial,
                        view = self.uidevice(**self.device_info.wifi_p2p_search_for_devices_id),
                        view_to_check = self.device_info.wifi_p2p_device_searching_id)()

    def check_condition(self):
        # if we got here, the step is passed
        return True


class p2p_check_connection_info(check_connection_info):
    """ description:
            Check the WiFi Direct connection information.

        usage:
            wifi_steps.p2p_check_connection_info(state="CONNECTED/CONNECTED", go_name='master2')()

            Use <parama_name>="None" to expect the setting not to be present on DUT.

            Example of possible values for all supported parameters:
            {'go_interface_name': 'p2p0',
             'go_ip': '192.168.49.1',
             'go_mac': '6e:71:d9:fb:a6:33',
             'go_name': 'slave2',
             'group_name': 'DIRECT-mj-slave2',
             'group_state': 'GroupCreatedState',
             'slave_interface_name': 'p2p0',
             'slave_ip': '192.168.49.8',
             'slave_mac': '42:e2:30:5b:ea:4b',
             'slave_name': 'master2',
             'state': 'CONNECTED/CONNECTED'}

        tags:
            android, settings, wifi, p2p
    """

    def __init__(self, go_serial, slave_serial, go_ip=None, slave_ip=None,
                 group_name=None, go_mac=None, slave_mac=None, go_name=None, slave_name=None,
                 state=None, group_state=None, slave_interface_name=None, go_interface_name=None,
                 timeout=0, regex=False, **kwargs):

        check_connection_info.__init__(self, serial=go_serial, **kwargs)
        self.params = {'go_ip': go_ip,
                       'slave_ip': slave_ip,
                       'group_name': group_name,
                       'go_mac': go_mac,
                       'slave_mac': slave_mac,
                       'go_name': go_name,
                       'slave_name': slave_name,
                       'state': state,
                       'group_state': group_state,
                       'slave_interface_name': slave_interface_name,
                       'go_interface_name': go_interface_name}
        self.go_serial = go_serial
        self.slave_serial = slave_serial
        self.timeout = timeout
        self.regex = regex
        check_connection_info.params = self.params
        check_connection_info.regex = regex
        check_connection_info.timeout = timeout

    def do(self):
        # get the wifip2p dumpsys: adb shell "dumpsys wifip2p"
        self.wifi_info = wifi_utils.p2p_get_connection_info(go_serial=self.go_serial,
                                                slave_serial=self.slave_serial)
        check_connection_info.wifi_info = self.wifi_info

        elapsed_time = 0;
        while self.wifi_info['state'] == 'CONNECTED/CONNECTED' and self.wifi_info['slave_name'] == None:
            self.wifi_info = wifi_utils.p2p_get_connection_info(go_serial=self.go_serial,
                                                    slave_serial=self.slave_serial)
            check_connection_info.wifi_info = self.wifi_info
            time.sleep(3)
            elapsed_time += 3
            if elapsed_time == 30:
                raise Exception('Connection error! - slave and group owner are reversed!')

    def check_condition(self):
        # if we got here, the step is passed
        return check_connection_info.check_condition(self)


class p2p_create_download_url(base_step):
    """ description:
        Provides an URL to be delivered to the browser, in order to download a file from another device through WiFi Direct
        In order to create the URL, it uses the following:
         the protocol (ftp or http),
         the device p2p interface IP (on which ftp server runs in the specified directory)
         the tcp port used
         the filename
        usage:
            adb_steps.p2p_create_download_url(file_name = "generated.bin", file_size = 1024, device_path = "/data/ftpdfiles", protocol = "ftp", port = "20211", serial = serial)()

        tags:
            wifi, browser, download
    """
    def __init__(self, file_name, file_size, device_path, protocol, port_number, serial2, server_on_go = True, **kwargs):
        base_step.__init__(self, **kwargs)
        if kwargs.has_key("serial"):
            self.serial = kwargs["serial"]
        self.file_name = file_name
        self.file_size = file_size
        self.device_path = device_path
        self.protocol = protocol
        self.port_number = port_number
        self.serial2 = serial2
        self.server_on_go = server_on_go


    def do(self):
        wifi_utils.create_file(file_name = self.file_name, file_size = int(self.file_size), file_path = ".")

        if self.server_on_go:
            ip = 'go_ip'
        else:
            ip = 'slave_ip'

        ftp_srv_serial = self.serial

        adb_steps.command(serial = ftp_srv_serial,
                command = "rm -rf " + self.device_path)()
        adb_steps.command(serial = ftp_srv_serial,
                command = "mkdir " + self.device_path)()
        adb_steps.push_file(serial = ftp_srv_serial,
                  local = "./" + self.file_name,
                  remote = self.device_path)()

        conn_info = wifi_utils.p2p_get_connection_info(go_serial = self.serial, slave_serial = self.serial2)
        device_ip = conn_info[ip]
        self.step_data = (self.protocol + "://" + str(device_ip)  + ":" + str(self.port_number) + "/" + self.file_name, device_ip)

    def check_condition(self):
        pass_message = "URL successfully generated!"
        fail_message = "URL generation failed"
        if "None" not in self.step_data[0]:
            self.set_passm(pass_message)
            return True
        else:
            self.set_errorm("", fail_message)
            return False


class p2p_kill_ftpd(wifi_step):
    """ description:
            checks if an instance of ftpd / tcpsvd is already running on the specified device
            if it is, it kills it
        usage:
            adb_steps.kill_ftpd(process_name = "tcpsvd", serial = serial)()
        tags:
            adb, process, pid
    """
    def __init__(self, process_name = "tcpsvd", **kwargs):
        self.process_name = process_name
        wifi_step.__init__(self, **kwargs)
        self.set_errorm("", "Could not kill PID of " + self.process_name)
        self.set_passm("Killed PID of " + self.process_name)

    def do(self):
        pids = self.adb_connection.pgrep(grep_for = self.process_name)
        self.adb_connection.kill_all(pids)

    def check_condition(self):
        server_pids = self.adb_connection.pgrep(grep_for = self.process_name)
        return len(server_pids) == 0


class p2p_start_ftpd(wifi_step):
    """ description:
            copies busybox binary on device /data/busybox directory, installs it and starts ftp server
        usage:
            adb_steps.start_ftpd(port_number = 20211, root_dir = "/data/ftpdfiles/", copy_from, serial = serial)()
        tags:
            android, adb, busybox, wifi_direct
    """
    nexus_devices = ['bullhead','angler', "hammerhead",'shamu','fugu','volantisg','volantis','razor','razorg','mantaray','nakasi','nakasig']
    def __init__(self, port_number = "20211", root_dir = "/data/ftpdfiles/", copy_from = str(os.environ['PYTHONPATH'].rstrip(':')), **kwargs):
        self.port_number = port_number
        self.root_dir = root_dir
        self.copy_from = copy_from.split(":")[0] + '/testlib/external/busybox'
        wifi_step.__init__(self, **kwargs)

    def do(self):
        device_model = self.adb_connection.get_prop(prop = "ro.product.device").strip()
        # is ftpd is started on Nexus, use busybox for ARM
        # otherwize, use x86 32 bit version
        if str(device_model) in self.nexus_devices:
            suffix = "-armv7l"
        else:
            suffix = "-i686"
        adb_steps.command(serial = self.serial,
                command = "rm -rf /data/busybox")()
        adb_steps.command(serial = self.serial,
                command = "mkdir /data/busybox/")()
        adb_steps.push_file(serial = self.serial,
                  local = self.copy_from + suffix,
                  remote = "/data/busybox")()
        adb_steps.command(serial = self.serial,
                command = "chmod 777 /data/busybox/busybox" + suffix)()
        adb_steps.command(serial = self.serial,
                command = "/data/busybox/busybox" + suffix + " --install /data/busybox/")()
        adb_steps.command(serial = self.serial,
                command = "chmod -R 777 /data/busybox/")()
        p = Process(target = wifi_utils.p2p_start_ftpdd, args=(self.port_number, self.root_dir, self.serial))
        p.start()

    def check_condition(self):
        output = self.adb_connection.parse_cmd_output("netstat", grep_for = self.port_number)
        output_list = output.split('\n')
        if len(output_list) == 0:
            return False
        else:
            return True


class kill_iperf_server(wifi_step):
    """ description:
            stops iperf server
        usage:
            adb_steps.start_iperf(serial = go_serial)()
        tags:
            android, adb, iperf, wifi, p2p, wifi direct
    """
    def __init__(self, **kwargs):
        wifi_step.__init__(self, **kwargs)

    def do(self):
        iperf_pids = self.adb_connection.get_pid("iperf")
        if iperf_pids:
                while iperf_pids != None:
                    adb_steps.kill_process(serial = self.serial, process_name = "iperf")()
                    iperf_pids = self.adb_connection.get_pid("iperf")

    def check_condition(self):
        output = self.adb_connection.parse_cmd_output("ps", grep_for = "iperf")
        output_list = output.split('\n')
        if len(output_list) == 0 or (len(output_list) == 1 and output_list == ['']):
            return True
        else:
            return False


class start_iperf_server(wifi_step):
    """ description:
            copies iperf binary on device /data/iperf directory and
            starts it as server
        usage:
            adb_steps.start_iperf(serial = go_serial)()
        tags:
            android, adb, iperf, wifi, p2p, wifi direct
    """

    def __init__(self, kill_server=False, protocol = "udp", copy_from = str(os.environ['PYTHONPATH'].rstrip(':') + '/testlib/external/iperf_android'), **kwargs):
        self.protocol = protocol
        self.copy_from = copy_from
        self.kill_server = kill_server
        wifi_step.__init__(self, **kwargs)

    def start_server(self):
        adb_steps.command(serial = self.serial,
                command = "rm -rf /data/iperf")()
        adb_steps.command(serial = self.serial,
                command = "mkdir /data/iperf/")()
        adb_steps.push_file(serial = self.serial,
                  local = self.copy_from,
                  remote = "/data/iperf")()
        adb_steps.command(serial = self.serial,
                command = "mv /data/iperf/iperf_android /data/iperf/iperf_server;chmod 777 /data/iperf/iperf_server")()
        if self.protocol == "udp":
            protocol_string = "-u"
        elif self.protocol == "tcp":
            protocol_string = "-t"
        iperfserver_command = "/data/iperf/iperf_server -s " + protocol_string
        self.process = self.adb_connection.run_cmd(command = iperfserver_command,
                                    ignore_error = False,
                                    timeout = 10,
                                    mode = "async")

    def do(self):
        iperf_pids = self.adb_connection.get_pid("iperf")
        if iperf_pids:
            if self.kill_server:
                while iperf_pids != None:
                    adb_steps.kill_process(serial = self.serial, process_name = "iperf")()
                    iperf_pids = self.adb_connection.get_pid("iperf")
                self.start_server()
        else:
            self.start_server()


    def check_condition(self):
        output = self.adb_connection.parse_cmd_output("ps", grep_for = "iperf")
        output_list = output.split('\n')
        if len(output_list) == 0 or (len(output_list) == 1 and output_list == ['']):
            return False
        else:
            return True


class iperf_transfer(wifi_step):
    """ description:
            copies iperf binary on device /data/iperf directory
            starts iperf as client and begins transfer to server's Ip
        usage:
            adb_steps.start_iperf(serial = go_serial, duration = "20", expect_interrupt = False, protocol = "udp")()
        tags:
            android, adb, iperf, wifi, p2p, wifi direct
    """

    def __init__(self, serial2, transfer_type="p2p", negative=False, download_type = "server", protocol = "udp", duration = 60, copy_from = str(os.environ['PYTHONPATH'].rstrip(':') + '/testlib/external/iperf_android'),
                            expect_interrupt = False, **kwargs):
        self.protocol = protocol
        self.duration = duration
        self.copy_from = copy_from
        self.expect_interrupt = expect_interrupt
        self.serial2 = serial2
        self.negative = negative
        self.transfer_type = transfer_type
        self.download_type = download_type
        wifi_step.__init__(self, **kwargs)

    def do(self):
        if self.protocol == "udp":
            protocol_string = "-u"
        elif self.protocol == "tcp":
            protocol_string = "-t"

        if self.transfer_type == "p2p":
            conn_info = wifi_utils.p2p_get_connection_info(go_serial = self.serial, slave_serial = self.serial2)
            if self.download_type == "server":
                device_ip = conn_info["go_ip"]
                self.adb_connection = connection_adb(serial=self.serial2)
                self.serial_iperf = self.serial2
            else:
                device_ip = conn_info["slave_ip"]
                self.adb_connection = connection_adb(serial=self.serial)
                self.serial_iperf = self.serial

            adb_steps.command(serial = self.serial_iperf,
                    command = "rm -rf /data/iperf")()
            adb_steps.command(serial = self.serial_iperf,
                    command = "mkdir /data/iperf/")()
            adb_steps.push_file(serial = self.serial_iperf,
                      local = self.copy_from,
                      remote = "/data/iperf")()
            adb_steps.command(serial = self.serial_iperf,
                    command = "chmod 777 /data/iperf/iperf_android")()

            iperfclient_command = "/data/iperf/iperf_android -c " + device_ip + " -t " +\
                       str(self.duration) + " " + protocol_string +\
                       " -b 100K 2>&1"

            self.output = self.adb_connection.parse_cmd_output(cmd = iperfclient_command,
                                    timeout = self.duration + 10)

        elif self.transfer_type == "wifi":
            self.serial_iperf = self.serial
            if self.download_type == "client":
                self.adb_connection = connection_adb(serial=self.serial2)
                self.serial_iperf = self.serial2

            local_connection = connection_local()

            device_ip = wifi_utils.get_connection_parameter("ip_address", serial=self.serial_iperf)
            iperfclient_command = "iperf -c " + device_ip + " -i 2 -t " +\
                       str(self.duration) + " " + protocol_string +\
                       " -b 20M 2>&1"

            output, error = local_connection.run_cmd(command = iperfclient_command)
            self.output = output + error

    def check_condition(self):
        return_val = False
        output_list = self.output.strip().split('\n')

        if "WARNING: did not receive ack" in self.output:
            line = "Iperf: Did not receive ACK."
            line = self.output
        else:
            report_found = False
            for line in output_list:
                if report_found:
                    break
                if "Server Report:" in line:
                    report_found = True
            # parse the report line
            m = re.search("(\d+(\.\d+)?)%", line)
            if m:
                packet_loss = float(m.group(1))
                if packet_loss < 10.0:
                    return_val = True
            else:
                line = "Iperf report not found"

        self.set_passm("Negative: " + str(self.negative) + " " + line)
        self.set_errorm("iperf, Negative: " + str(self.negative), line)

        if self.negative:
            return not return_val
        return return_val


class check_lease_time(wifi_step):
    """ description:
            Verify in log for a specific message
            dhcp_lease_time(seconds) on AP is set on minutes but in log we
            check for seconds

        usage:
            wifi_steps.check_lease_time(serial=self.serial)()

        tags:
            wifi
    """

    def __init__(self, dhcp_lease_time, **kwargs):
        wifi_step.__init__(self, **kwargs)
        self.dhcp_lease_time = dhcp_lease_time

    def do(self):
        #conversion from minutes to seconds
        self.dhcp_lease_time = int(self.dhcp_lease_time) * 60

    def check_condition(self):
        logcat_steps.grep_for(serial=self.serial,
                              grep_for_text="lease time {0}".format(self.dhcp_lease_time),
                              text_presence=True)()
        #the validation is included in the grep for step
        return True

class ping_ipv6_gateway(wifi_step):
    """
    Description:
                Pings the DUT gateway <trycount> times.
                Will fail if the packet loss is less then or equal to <target_percent>
    """

    def __init__(self, trycount=2, timeout=15, target_percent=50,**kwargs):
        wifi_step.__init__(self, **kwargs)
        self.trycount = trycount
        self.timeout = timeout
        self.target_percent = target_percent

    def do(self):
        wifi_conf = self.adb_connection.parse_cmd_output('dumpsys wifi | grep ipv6')
        wifi_conf_split = wifi_conf.split()
        ap_ip_prefix = 'fe80'
        ap_ip_index = [wifi_conf_split.index(i) for i in wifi_conf_split if ap_ip_prefix in i]
        ip = wifi_conf_split[ap_ip_index[0]]
        #self.wifi_info = wifi_utils.get_connection_info(wifi_conf)\
        (self.status, loss, ping_output) = wifi_utils.ping(ip=ip, trycount=self.trycount,
                                      target_percent=self.target_percent,timeout=self.timeout, serial=self.serial)

        if loss:
            step_message = "=== pinging address " + str(ip) + " lost " + str(loss) + "% out of " + str(self.trycount) + " packets ==="
        else:
            step_message = "Could not determine loss percent. Ping output was: \" {} \"".\
                            format(ping_output)

        self.set_passm(step_message)
        self.set_errorm("", step_message)

    def check_condition(self):
        return self.status



class check_ipv6_address(wifi_step):
    """ description:
            Checks if the ipv6 address is set correctly

        usage:
            wifi_steps.check_ipv6_address(ipv6_prefix="2001:1234:5678:9abc",
                                         ap_ipv6_address="2001:1234:5678:9abc::1",
                                         serial=serial)()

        tags:
            wifi, ipv6, check, address
    """

    def __init__(self, ipv6_prefix, ap_ipv6_address, dut_static_ip=False, negative=False, **kwargs):
        self.ipv6_prefix = ipv6_prefix
        self.ap_ipv6_address = ap_ipv6_address
        self.dut_static_ip = dut_static_ip
        self.negative = negative
        wifi_step.__init__(self, **kwargs)
        self.set_errorm("", "Could not set the ip with prefix {0} on DUT".format(self.ipv6_prefix))
        self.set_passm("Ip with prefix {0} set successfully on DUT".format(self.ipv6_prefix))
        self.result = True

    def do(self):
        dut_ipv6 = wifi_utils.get_dut_ipv6_address(serial=self.serial, static_ip=self.dut_static_ip)
        # Check if the address is a valid ipv6 address
        if not wifi_utils.is_valid_ipv6_address(dut_ipv6):
            self.result = False
        # Check if the ap ipv6 prefix is in the DUT ipv6 address
        elif not self.negative and self.ipv6_prefix not in dut_ipv6:
            self.result = False
        else:
            ping_ipv6_gateway(trycount=10, serial=self.serial, negative=self.negative)()

    def check_condition(self):
        return self.result


class set_dut_ipv6(wifi_step):
    """ description:
            Sets the ipv6 address on the DUT

        usage:
            steps.radvd_enable(ipv6_ip="2001:1234:5678:9abc::1")()


        tags:
            dut, config, ipv6
    """
    def __init__(self, ipv6_ip, **kwargs):
        self.ipv6_ip = ipv6_ip
        wifi_step.__init__(self, **kwargs)

    def do(self):
        # Add the ipv6 address to wlan0 interface
        command = "ip addr add {0} dev wlan0".format(self.ipv6_ip)
        self.adb_connection.run_cmd(command=command)

    def check_condition(self):
        output = self.adb_connection.parse_cmd_output(cmd="ip -6 addr show wlan0")
        return self.ipv6_ip in output


class configure_hotSpot(wifi_step):
    """ description:
            Sets the HotSpot  on the DUT

        usage:
            wifi_steps.set_hotSpot(hotSpot_name="TEST_HotSpot", hotSpot_security="WPA2 PSK", hotSpot_pass="12345678,hotSpot_band="2.4")()

        tags:
            dut, hotspot
    """
    def __init__(self, hotSpot_name,hotSpot_security, hotSpot_pass
                 , hotSpot_band,**kwargs):
        wifi_step.__init__(self, **kwargs)
        self.hotSpot_name = hotSpot_name
        self.hotSpot_security = hotSpot_security
        self.hotSpot_pass=hotSpot_pass
        self.hotSpot_band=hotSpot_band

    def do(self):
        # open the settings > More
        """ui_steps.open_wifi_settings(serial=self.serial, intent=True)()"""
        """ this is for version O
        open_wifi_settings(serial=self.serial)()"""
        ui_steps.open_settings(serial=self.serial)()

        # open the More
        """ui_steps.open_app_from_settings(serial=self.serial, view_to_find =
                            {"text": "More"}, view_to_check = {"text": "NFC"},
                                        wait_time = 100)()"""
        ui_steps.click_button_common(serial=self.serial,
                                     view_to_find={"textContains": "More"},
                                     view_to_check={"text": "Tethering & portable hotspot"},
                                     optional=True,wait_time=100)()
        ui_steps.click_button_common(serial=self.serial,
                                     view_to_find={"text": "Network & Internet"},
                                     view_to_check={"textContains": "hotspot"},
                                     optional=True, wait_time=100)()

        # click on Tethering & portable hotspot
        """ui_steps.click_button(serial=self.serial, view_to_find = {"text":
                            "Tethering & portable hotspot"},
                            view_to_check = {"text": "USB tethering"})()"""
        ui_steps.click_button_common(serial=self.serial, view_to_find={"textContains":
                             "hotspot"},
                              view_to_check={"textContains": "hotspot"},
                              optional=True,wait_time=100)()

        # click on Set up Wi-Fi hotspot
        ui_steps.click_button(serial=self.serial, view_to_find = {"textContains":
                            "Set up Wi"},
                            view_to_check = {"text": "Network name"})()

        # enter the hotspot name
        ui_steps.edit_text(serial=self.serial, view_to_find = {"resourceId":
                            "com.android.settings:id/ssid"},
                            value = self.hotSpot_name)()

        # open the security drop down menu
        ui_steps.click_button(serial=self.serial, view_to_find=
                                    {"resourceId": "android:id/text1"},
                              view_to_check={"textContains": self.hotSpot_security})()

        # select the security type
        ui_steps.click_button(serial=self.serial, view_to_find={"textContains":self.hotSpot_security},
                              view_to_check={"textContains": self.hotSpot_security})()

        # if the security is WPA2 we need to enter the password
        if self.hotSpot_security != "None":
            ui_steps.click_button(serial=self.serial,
                                            view_to_find={"text": "Show password"},
                                                     view_to_check={"checked": "true"})()
            ui_steps.edit_text(serial=self.serial, view_to_find = {"resourceId":
                            "com.android.settings:id/password"},
                               value = self.hotSpot_pass, is_password = True, text="Show password",
                               view_to_check={"checkable": "True", "checked": "false"})()

        # open the Ap band drop down menu
        """ui_steps.click_button(serial=self.serial, view_to_find = {"resourceId":
                            "com.android.settings:id/choose_channel"},
                            view_to_check = {"textContains": "2.4"})()"""
        ui_steps.click_button(serial=self.serial, view_to_find={"textContains": "AP Band"},
                             view_to_check={"textContains": "2.4"})()

        # select the security type
        """ui_steps.click_button(serial=self.serial, view_to_find = {"textContains":
                            self.hotSpot_band},
                            view_to_check = {"text": "Security"})()"""

        # click on save button
        """ui_steps.click_button(serial=self.serial, view_to_find = {"textContains":
                            "save"},
                            view_to_check = {"textContains": "hotspot"})()"""

        ui_steps.click_button(serial=self.serial,
                              view_to_find=self.device_info.wifi_add_network_save_btn_id,
                              view_to_check={"textContains": "hotspot"})()



    def check_condition(self):
        # click on Set up Wi-Fi hotspot
        ui_steps.click_button(serial=self.serial,
                              view_to_find = {"textContains": "Set up Wi"},
                              view_to_check = {"text": self.hotSpot_name})()

        # close the set up wi-Fi hotspot window
        ui_steps.click_button(serial=self.serial,
                              view_to_find = self.device_info.wifi_saved_network_cancel_btn_id,
                              view_to_check = {"text": "USB tethering"})()
        return True


class set_hotSpot(wifi_step):
    """ description:
            Turn ON the HotSpot on the DUT

        usage:
            wifi_steps.turn_on_hotSpot(hotSpot_state=ON)()

        tags:
            dut, hotspot
    """
    def __init__(self, hotSpot_state,**kwargs):
        wifi_step.__init__(self, **kwargs)
        self.hotSpot_state = hotSpot_state

    def do(self):
        ui_steps.open_settings(serial=self.serial, intent=True)()

        # open the More
        """ui_steps.open_app_from_settings(serial=self.serial, view_to_find =
                            {"text": "More"}, view_to_check = {"text": "NFC"},
                                        wait_time = 100)()"""
        """ui_steps.open_app_from_settings(serial=self.serial,
                                        view_to_find={"text": "More"},
                                        view_to_check={"text": "Tethering & portable hotspot"},
                                        wait_time=100)()"""
        ui_steps.click_button_common(serial=self.serial,
                                     view_to_find={"text": "More"},
                                     view_to_check={"text": "Tethering & portable hotspot"},
                                     optional=True,wait_time=100)()
        ui_steps.click_button_common(serial=self.serial,
                                     view_to_find={"text": "Network & Internet"},
                                     view_to_check={"textContains": "hotspot"},
                                     optional=True, wait_time=100)()

        # click on Tethering & portable hotspot
        """ui_steps.click_button(serial=self.serial, view_to_find = {"text":
                            "Tethering & portable hotspot"},
                            view_to_check = {"text": "USB tethering"})()"""
        ui_steps.click_button_common(serial=self.serial,
                                     view_to_find={"textContains": "hotspot"},
                                     view_to_check={"textContains": "hotspot"},
                                     optional=True, wait_time=100)()

        # check if portable wifi hotspot  switch  is on or off
        """ui_steps.click_switch(serial=self.serial,
                              view_to_find={"className":"android.widget.TextView","textContains":"Portable Wi"}
                              , right_of = True, state = self.hotSpot_state)()"""
        ui_steps.click_button(serial=self.serial,
                              view_to_find={"textContains":'Fi hotspot'}
                              , right_of=True, state=self.hotSpot_state)()

    def check_condition(self):
        self.dut_state = False
        if self.hotSpot_state == "ON":
            #ping_gateway_hotSpot(serial=self.serial)()
            self.dut_state = True
        elif self.hotSpot_state == "OFF":
            wait_for_state(serial=self.serial, state="DISCONNECTED")()
            self.dut_state = True
        return self.dut_state



class ping_gateway_hotSpot(wifi_step):
    """
    Description:
                Pings the DUT gateway <trycount> times.
                Will fail if the packet loss is less then or equal to <target_percent>
    """

    def __init__(self, **kwargs):
        wifi_step.__init__(self, **kwargs)

    def do(self):
        wifi_conf = self.adb_connection.parse_cmd_output("dumpsys wifi")
        self.wifi_info = wifi_utils.get_connection_info(wifi_conf)
        if self.wifi_info['ip_address'] == wifi_defaults.wifi["hotSpot_ip"]:
            ping_ip(self.wifi_info['ip_address'], serial=self.serial)
            self.status = True
        else:
            step_message = "Device " + self.wifi_info['ip_address'] + " address is not " \
                           "the expected Hot Spot host IP: " + wifi_defaults.wifi["hotSpot_ip"]
            self.set_passm(step_message)
            self.set_errorm("",step_message)
            self.status = False

    def check_condition(self):
        return self.status
