#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
#from testlib.em.constants_def import *
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import WifiSetting
from testlib.em.apps import GarageModeTest
from testlib.em.settings import DateSetting
from testlib.em.tools import get_server_ip, get_config_value
#from testlib.em.relay08 import get_relay_obj

class Garage(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        #self.relay = get_relay_obj()
        self.s0ix = get_s0ix_obj()
        self.serverip = get_server_ip()
        assert self.serverip != ""
        self.garage = GarageModeTest()
        self.garage.adb_root()
        self.garage.install()
        self.garage.clean_download_folder()
        self.garage.grant_permissions()
        self.wifi = WifiSetting()
        self.wifi.connect_wifi_by_conf("wifi_adb")
        assert self.wifi.check_wifi_connect()
        self.ds = DateSetting()
        self.ds.set_property()
        super(Garage, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        #super(Garage, self).tearDown()

    def check_download_task_in_garage_mode(self, url, sleep_time, time_to_set = "23:59"):
        self.ds.set_time(time_to_set)
        download_list = self.garage.schedule_job(20, url)
        self.s0ix.sleep(sleep_time)
        #assert enter_s3
        wake_time = self.garage.get_time()
        start_time = self.garage.get_job_event_time("start")
        if not start_time:
            assert False, "Job not started"
        print "Job started at %s:%s:%s" % start_time
        h, m, s = start_time
        assert h == 0 and m == 0, "Job start time error"

        finished_time = self.garage.get_job_event_time("finished")
        if not finished_time:
            assert False, "Job not finished"
        print "Job finished at %s:%s:%s" % finished_time
        h, m, s = finished_time
        wake_h, wake_m, wake_s = map(int, wake_time.split(":"))
        print "Wake up at %s:%s:%s" % (wake_h, wake_m, wake_s)
        finished_time_seconds = 3600 * h + 60 * m + s
        wake_up_time_seconds = 3600 * wake_h + 60 * wake_m + wake_s
        assert finished_time_seconds < wake_up_time_seconds, "Job finished time error"

    def test_enter_garage_mode_parked_for_more_than_3_minutes(self):
        url_single = get_config_value("garage", "url_single")
        url_single = "http://" + self.serverip + url_single
        self.check_download_task_in_garage_mode(url_single, 120)

    def test_enter_garage_mode_parked_for_one_night(self):
        url_single = get_config_value("garage", "url_single")
        url_single = "http://" + self.serverip + url_single
        sleep_time = 12 * 3600
        seconds = 24 * 3600 + 300 - sleep_time
        h = seconds / 3600
        m = (seconds % 3600) / 60
        time_to_set = "%02d:%02d" % (h, m)
        self.check_download_task_in_garage_mode(url_single, sleep_time, time_to_set)

    def test_enter_garage_mode_downloaded_batch_files(self):
        url_batch = get_config_value("garage", "url_batch")
        url_batch = "http://" + self.serverip + url_batch
        self.check_download_task_in_garage_mode(url_batch, 200)

    def test_system_in_deep_sleep_mode_two_times(self):
        self.ds.set_time("23:59")
        stat_inc = self.s0ix.suspend_resume(120)
        assert stat_inc == 2

    def test_offline_data_sync_corrected(self):
        url_single = get_config_value("garage", "url_single")
        download_file_path = os.path.join(self.garage.download_folder, os.path.basename(url_single))
        md5sum_conf = get_config_value("garage", "md5sum")
        url_single = "http://" + self.serverip + url_single
        self.check_download_task_in_garage_mode(url_single, 120)
        md5sum = self.garage.get_md5sum(download_file_path)
        assert md5sum_conf == md5sum, "Download file error"

