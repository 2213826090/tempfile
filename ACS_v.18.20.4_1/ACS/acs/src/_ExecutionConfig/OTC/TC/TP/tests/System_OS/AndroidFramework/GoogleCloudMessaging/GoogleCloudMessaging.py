# -*- coding:utf-8 -*-

'''
@summary: Goolge cloud messaging test.
@since: 07/04/2016
@author: Lijin Xiong
'''

import time
from threading import Thread
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import ShellUtils,AdbUtils,Settings,Environment,CommonUtils,Chrome,UiAutomatorUtils
from testlib.util.log import Logger
from string import Template
from testlib.androidframework.fetch_resources import resource
from testlib.gps.common import GPS_Common

LOG = Logger.getlogger(__name__)


class Messaging(UIATestBase):

    TAG = "SystemOsTests"
    gcm_receiver_package = "gcm.play.android.samples.com.gcmquickstart"
    gcm_receiver_tag = "MyGcmListenerService"
    gcm_sender_jar = "gcm_message_sender.jar"

    def setUp(self):
        super(Messaging, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "GCM", "gcm_listener")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)
        self._jar_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "GCM", "gcm_sender")
        self._proxy_file = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "Proxy", "http_proxy")
        with open(self._proxy_file) as pf:
            self._proxy = pf.read().strip('\n')

        """
        Get Android version:
        adb shell getprop | grep ro.build.version.sdk
        """
        cmd = 'getprop | grep ro.build.version.sdk'
        sdk_string = g_common_obj.adb_cmd_capture_msg(cmd)
        if '24' in sdk_string:
            self.android_version = "N"
        elif '23' in sdk_string:
            self.android_version = "M"
        elif '22' in sdk_string:
            self.android_version = "L"

    def gen_unique_google_cloud_message(self):
        return "google cloud message " + CommonUtils.get_current_time_string()

    def send_google_cloud_message_from_host(self, message_text):
#         send_message_jar = os.path.join(Environment.tmp_dir_path, Messaging.gcm_sender_jar)
        send_message_jar = self._jar_path
        try:
            #send_message_cmd = Template("java -jar " + send_message_jar + ' "$message"').substitute(message=message_text)
            #result = ShellUtils.run_shell_cmd(send_message_cmd)
            #time.sleep(10)
            #self.assertTrue("message_id" in result)
            send_message_cmd_pxy = Template("export " + "$http_pxy" + ";" +
                                            "java -jar " + send_message_jar + ' "$message"').substitute(http_pxy=self._proxy, message=message_text)
            result_with_proxy = ShellUtils.run_shell_cmd(send_message_cmd_pxy)
            time.sleep(10)
            self.assertTrue("message_id" in result_with_proxy, "could not send cloud message from host")
        except Exception, e:
            print e

    def wait_for_gcm_message(self, message_text):
        # wait for message to arrive on DUT
        for i in range(15):
            time.sleep(10)
            messages_received = AdbUtils._run_adb_cmd("logcat -d | grep " + Messaging.gcm_receiver_tag,
                                                     adb_shell=False, add_ticks=False)
            if message_text in messages_received:
                LOG.info("google cloud message was received on DUT")
                return
        self.assertTrue(False, "did not receive any google cloud message")

    def test_google_cloud_messaging(self):
        try:
            message_text = self.gen_unique_google_cloud_message()
#             AdbUtils.start_activity_with_package_name(Messaging.gcm_receiver_package, "--pct-syskeys 0 -v 500")
            #g_common_obj.launch_app_am('gcm.play.android.samples.com.gcmquickstart', 'gcm.play.android.samples.com.gcmquickstart.MainActivity')
            g_common_obj.launch_app_am('gcm.play.android.samples.com.gcmquickstart', '.MainActivity')
            assert self.d(packageName=Messaging.gcm_receiver_package).wait.exists(timeout=20000),\
                            "google cloud messaging receiver app failed to start on DUT"
            time.sleep(2)
            self.send_google_cloud_message_from_host(message_text)
            self.wait_for_gcm_message(message_text)
        finally:
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    def test_google_cloud_messaging_disable_wifi(self):
        try:
#             Settings.disable_wifi()
#             time.sleep(4)  # wait for wifi to be switched off
            message_text = self.gen_unique_google_cloud_message()
#             AdbUtils.start_activity_with_package_name(Messaging.gcm_receiver_package, "--pct-syskeys 0 -v 500")
            g_common_obj.launch_app_am('gcm.play.android.samples.com.gcmquickstart', 'gcm.play.android.samples.com.gcmquickstart.MainActivity')
            time.sleep(5)
            self.assertTrue(self.d(packageName=Messaging.gcm_receiver_package).wait.exists(timeout=20000),
                            "google cloud messaging receiver app failed to start on DUT")
            Settings.disable_wifi()
            time.sleep(5)
            self.send_google_cloud_message_from_host(message_text)
            time.sleep(5)  # wait a little before turning on wifi
            Settings.enable_wifi()
            time.sleep(30)  # wait more time for WiFi reconnection in a congested wireless environment
            GPS_Common().check_if_wifi_connected()
            self.wait_for_gcm_message(message_text)
        finally:
            Settings.enable_wifi()
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    def test_google_cloud_messaging_web_browsing(self):
        try:
            message_text = self.gen_unique_google_cloud_message()
#             AdbUtils.start_activity_with_package_name(Messaging.gcm_receiver_package, "--pct-syskeys 0 -v 500")
            g_common_obj.launch_app_am('gcm.play.android.samples.com.gcmquickstart', 'gcm.play.android.samples.com.gcmquickstart.MainActivity')
            time.sleep(5)
            self.assertTrue(self.d(packageName=Messaging.gcm_receiver_package).wait.exists(timeout=20000),
                            "google cloud messaging receiver app failed to start on DUT")

            time.sleep(5)
            def browse_pages():
                url_list = ["https://www.google.de", "http://www.w3schools.com/",
                            "https://www.wikipedia.org/", "https://500px.com/"]
                Chrome.launch()
                for url in url_list:
                    Chrome.go_to_url(url)
                    time.sleep(3)

            def send_gcm_message():
                self.send_google_cloud_message_from_host(message_text)

            browse_web_thread = Thread(target=browse_pages)
            send_gcm_message_thread = Thread(target=send_gcm_message)
            browse_web_thread.start()
            time.sleep(5)
            send_gcm_message_thread.start()
            browse_web_thread.join()
            send_gcm_message_thread.join()

            self.wait_for_gcm_message(message_text)
        finally:
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)