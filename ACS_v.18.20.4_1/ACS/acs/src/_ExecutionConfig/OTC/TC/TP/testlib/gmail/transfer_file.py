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
@summary: module for file transfer such as download file
receive file on bluetooth
@since: 10/10/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

import os
import time
import threading
from testlib.util.common import g_common_obj


class BrowserTranf:


    class Locator(object):

        def __init__(self, device):
            self.d = device

        @property
        def msg_download_complete(self):
            """ UI notification: 'Download complete' in notification bar """
            return self.d(text="Download complete.")

        @property
        def notification_panel(self):
            """UI notification panel"""
            return self.d(\
                resourceId="com.android.systemui:id/notification_panel")

        @property
        def btn_clear_all_notifications(self):
            """ UI image button
            'Clear all notifications.' in notification bar """
            return self.d(description="Clear all notifications.")

    def __init__(self, cfg):
        self.d =  g_common_obj.get_device()
        self._locator = BrowserTranf.Locator(self.d)
        self.cfg = cfg

    @staticmethod
    def download(url):
        """
        @summary: Download file from browser
        @param url: download url
        """
        print "[INFO] Download: %s" % url
        cmd = "am start -S -n com.android.chrome/.Main -d %s" % url
        g_common_obj.adb_cmd_capture_msg(cmd)

    def check_all_downloads_complete(self, amount):
        """
        @summary: Check all download from browser complete
        @param amount: amount of downloading
        """
        # wait device receive file complete
        timeout = 30*60 # 30 minutes
        complete_amount = 0
        self.d.open.notification()
        while timeout > 0:
            if self._locator.msg_download_complete.exists is False:
                print "[INFO] Downloading ..."
                time.sleep(6)
                timeout -= 6
                continue
            else:
                complete_amount = self._locator.msg_download_complete.count
                if complete_amount < amount:
                    print "[INFO] Current %d/%d downloads complete" % \
                    (complete_amount, amount)
                else:
                    print "[INFO] All downloads complete"
                    break
        # hidden notification panel
        self._locator.notification_panel.swipe.up()
        assert complete_amount == amount, \
        "[ERROR] All downloads not complete with 30 mins"

    def clear_all_notifications(self):
        """ Clear all notifications """
        self.d.open.notification()
        self._locator.btn_clear_all_notifications().click()

class BluetoothTranf:
    """
    Bluetooth Transfer
    """

    ReceiveFolder = "/sdcard/bluetooth"

    class Locator(object):
        """ Helper class for locate object on UI """

        def __init__(self, device):
            self.d = device

        @property
        def btn_more_options(self):
            """ UI image button 'More options' """
            return self.d(description="More options")

        @property
        def btn_onoff_switch(self):
            """ UI switch button to turn on/off bluetooth """
            return self.d(text="Bluetooth").\
            right(className="android.widget.Switch")

        @property
        def btn_accept(self):
            """ UI button 'Accept' of Bluetooth File Transfer """
            return self.d(text="Accept", className="android.widget.Button")

        @property
        def btn_pair(self):
            """ UI button 'Pair' on prompt 'Bluetooth pairing request' """
            return self.d(text="Pair")

        @property
        def msg_bluetooth_turning_on(self):
            """ UI message display 'Turning Bluetooth on' """
            return self.d(text="Turning Bluetooth on")

        @property
        def msg_bluetooth_pairing_request(self):
            """ UI display prompt 'Bluetooth pairing request' """
            return self.d(text="Bluetooth pairing request")

        @property
        def device_bluetooth_name(self):
            """ UI display device itself Bluetooth name """
            return self.d(text="Available devices").\
            up(resourceId="android:id/title")

        @property
        def device_bluetooth_visibility(self):
            """ UI display device itself Bluetooth visiable to other devices """
            return self.d(text="Available devices").\
            up(resourceId="android:id/summary")

        @property
        def notification_panel(self):
            """UI notification panel"""
            return self.d(\
                resourceId="com.android.systemui:id/notification_panel")

        @property
        def notice_bluetooth_file_transfer(self):
            """ UI notification display
            'Bluetooth share: Incoming file' in Notification Panel """
            return self.d(text="Bluetooth share: Incoming file")

        @property
        def notice_bluetooth_file_receiving(self):
            """ UI notification display
            'Bluetooth share: Receiving ...' in Notification Panel """
            return self.d(textStartsWith="Bluetooth share: Receiving")

        @property
        def btn_clear_all_notifications(self):
            """ UI image button\
            'Clear all notifications.' in notification bar """
            return self.d(description="Clear all notifications.")

    def __init__(self, cfg):
        self.d = g_common_obj.get_device()
        self._locator = BluetoothTranf.Locator(self.d)
        self.cfg = cfg

    def send_file_from_host(self, fpath):
        """
        @summary: Send file from host to device
        @param fpath: absolute path of host file to be sent to device
        1. Open Bluetooth Settings on device
        2. Turn on Bluetooth
        3. Set device visible to other devices
        4. Send file from host by bluetooth
        5. Device receive file
        """
        assert fpath, "[ERROR] Please set send file in config"
        print "[INFO] Send file from host to device using bluetooth"
        print "[INFO] File: %s" % fpath
        mac_addr = self.__get_device_mac_addr()
        fname = os.path.basename(fpath)
        suffix = os.path.splitext(fpath)[1]
        assert suffix, "[ERROR] Not support send file without file extension"
        # set device visible to other devices
        self.__launch_from_am()
        self.__turn_on_bluetooth()
        self.__set_device_visibility(True)
        # send file from host
        self.__check_host_find_bluetooth_device()
        self.__del_file(fname)
        self.__send_file(fpath, mac_addr)
        #threading.Thread(\
            #target=self.__send_file, args=(fpath, mac_addr,)).start()
        time.sleep(5)
        self.__device_receive_file(fname)

    def __send_file(self, fpath, mac_addr):
        """
        send file
        """
        def _sender():
            """
            sender
            """
            send_cmd = 'bluetooth-sendto --device=%s "%s"' % \
            (mac_addr, fpath)
            print "[INFO] Bluetooth send file cmd: %s" % send_cmd
            os.popen(send_cmd)
        threading.Thread(target=_sender).start()

    @staticmethod
    def __del_file(fname):
        """ Delete file on device sent by bluetooth """
        cmd = 'rm -f %s/%s' % (BluetoothTranf.ReceiveFolder, fname)
        g_common_obj.adb_cmd_capture_msg(cmd)

    def check_device_receive_sucess(self, fpath):
        """ Check device receive file success """
        print "[INFO] Check device receive file success"
        fname = os.path.basename(fpath)
        # wait device receive file complete
        timeout = 30*60 # 30 minutes
        self.d.open.notification()
        self._locator.\
        notice_bluetooth_file_receiving.wait.exists(timeout=6*1000)
        while timeout > 0:
            if self._locator.notice_bluetooth_file_receiving.exists:
                print "[INFO] Device receiving file ..."
                time.sleep(6)
                timeout -= 6
                continue
            else:
                print "[INFO] Device receive file complete"
                break
        # hidden notification panel
        self._locator.notification_panel.swipe.up()
        # check file on device
        cmd = 'ls %s/%s' % (BluetoothTranf.ReceiveFolder, fname)
        pip = g_common_obj.adb_cmd_capture_msg(cmd)
        assert "No such file or directory" not in pip, \
        "[ERROR] Device receive file: %s fail" % fname
        print "[INFO] Device receive file success"

    def clear_all_notifications(self):
        """ Clear all notifications """
        print "[INFO] Clear all notifications"
        if self._locator.notification_panel.exists is False:
            self.d.open.notification()
        self._locator.btn_clear_all_notifications.wait.exists(timeout=3*1000)
        if self._locator.btn_clear_all_notifications.exists:
            self._locator.btn_clear_all_notifications.click()
        else:
            # hidden notification panel
            self._locator.notification_panel.swipe.up()

    def __launch_from_am(self):
        """ Launch Bluetooth Settting from am """
        print "[INFO] Launch Bluetooth settings from am"
        cmd = 'am start -a android.settings.BLUETOOTH_SETTINGS; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        msg_fail = "[ERROR] Launch Bluetooth settings fail"
        ret = int(res_list[-1].strip())
        assert ret == 0, msg_fail
        self._locator.btn_more_options.wait.exists(timeout=3*1000)
        assert self._locator.btn_more_options.exists, msg_fail

    def __turn_on_bluetooth(self):
        """ Turn on Bluetooth """
        if self._locator.btn_onoff_switch.info['text'] == "ON":
            print "[INFO] Bluetooth already turn on"
            return
        self._locator.btn_onoff_switch.click()
        assert self._locator.btn_onoff_switch.info['text'] == "ON", \
        "[ERROR] Turn on bluetooth fail within 10s"

    def turn_off_bluetooth(self):
        """ Turn off Bluetooth """
        self.__launch_from_am()
        if self._locator.btn_onoff_switch.info['text'] == "OFF":
            print "[INFO] Bluetooth already turn off"
            return
        self._locator.btn_onoff_switch.click()
        assert self._locator.btn_onoff_switch.info['text'] == "OFF", \
        "[ERROR] Turn off bluetooth fail within 10s"

    def __set_device_visibility(self, visable):
        """
        @summary: Set device bluetooth visable or not visable to other devices
        @param visable: True = set to visable, False = set to not visable
        1. turn on bluetooth on device
        2. click on bluetooth name
        """
        print "[INFO] Set device visible: %s" % visable
        if visable != self.__is_bluetooth_visable():
            self._locator.device_bluetooth_name.click()
        assert visable == self.__is_bluetooth_visable()

    def __is_bluetooth_visable(self):
        """
        @summary: Check bluetooth is visable to other devices or not
        @return: True = visable, False = not visable
        """
        if "Not visible" in self._locator.device_bluetooth_visibility.info['text']:
            return False
        return True

    def __check_host_find_bluetooth_device(self):
        """
        @summary: Check host can find device as bluetooth device
        @return: True = detect success, False = detect fail
        """
        mac_addr = self.__get_device_mac_addr()
        print "[INFO] Check host can detect device: %s" % mac_addr
        # check host can find device
        list_device_cmd = 'hcitool scan'
        pip = os.popen(list_device_cmd).read()
        assert mac_addr in pip, "[ERROR] Host can't find bluetooth device: %s" % mac_addr

    def __device_receive_file(self, fname):
        """
        @summary: Device receive file which sent by bluetooth
        1. Open notification panel
        2. Click 'Bluetooth share: Incoming file' notification
        3. Click 'Accept' button on 'File transfer' prompt
        """
        print "[INFO] Start receive file"
        # if 'Bluetooth pairing request' prompt appear
        if self._locator.msg_bluetooth_pairing_request.exists:
            self._locator.btn_pair.click()
        self.d.open.notification()
        self._locator.notice_bluetooth_file_transfer.click()
        self._locator.btn_accept.click()

    def __get_device_mac_addr(self):
        """
        @summary: Get device MAC address
        """
        mac_addr = self.cfg.get("mac_addr").strip()
        assert mac_addr, "[ERROR] Please set device MAC address in config file"
        return mac_addr
