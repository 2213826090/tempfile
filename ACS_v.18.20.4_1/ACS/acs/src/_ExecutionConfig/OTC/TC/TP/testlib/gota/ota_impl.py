# -*- coding: utf-8 -*-
import os
import time
import urllib2
import re
import sys
import subprocess
import requests
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.util.config import TestConfig
from testlib.multimedia.multimedia_log import MultimediaLogger
logger = MultimediaLogger.instance()

old_percent = 0
def progress(width, percent):
    global old_percent
    if percent != old_percent:
        print "\r%s %d%%" % (('%%-%ds' % width) % (width * percent / 100 * '='), percent),
        old_percent = percent
    if percent >= 1:
        sys.stdout.flush()

class otaImpl:
    """
       Implements System android framework actions.

    """
    config = TestConfig()

    def __init__ (self, cfg_file=''):
        self._test_name = __name__
        self.d = g_common_obj.get_device()
        self.serial = g_common_obj2.getSerialNumber()
        self.product_name = self.get_product_name()
        logger.info("DUT's product name is %s" % self.product_name)
        self.cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        assert self.get_cfg(), "Cannot execute the platform ota test"
        self.local_path_base = self.cfg.get("local_path_base")
        self.origin_url = self.cfg.get("origin_url")
        self.flash_script = self.cfg.get("flash_script")
        self.serial_port = self.get_flash_serial_port()
        self.full_ota_url = self.cfg.get("full_ota_url")
        self.incremental_ota_url = self.cfg.get("incremental_ota_url")

    def get_flash_serial_port(self):
        """
        get device ID and debug Card serial port for BXTP flash script
        :return: dictionary with device Id and serial no
        """
        if 'cti-' in self.execute_cmd("hostname"):
            logger.info("Current host is CTI, use cit serial info")
            serial_port = {self.serial: self.get_CTI_debugCard_serial()}
        else:
            logger.info("Current host is ordinary host")
            serial_port_no = self.cfg.get("serial_port_no")
            serial_info = self.execute_cmd("ls " + serial_port_no)
            logger.debug("Get the debugCard serial info: %s" % serial_info)
            assert 'No such file' not in serial_info, "Cannot find the debugCard serial port"
            serial_port = {self.serial: serial_port_no}
        return serial_port

    def get_cfg(self):
        if self.product_name in ["gordon_peak"]:
            self.cfg = self.config.read(self.cfg_file, 'ota_gordon_peak')
        elif self.product_name in ["bxtp_abl"]:
            self.cfg = self.config.read(self.cfg_file, 'ota_bxtp_abl')
        else:
            logger.warning("This platform ota package is not definition")
            return False
        return self.cfg

    def get_CTI_debugCard_serial(self):
        serial_port_no_cti = self.cfg.get("serial_port_no_cti")
        cti_serial = serial_port_no_cti + '_debugCard_' + self.serial
        serial_info = self.execute_cmd("ls " + cti_serial)
        logger.debug("Get the debugCard serial info: %s" % serial_info)
        assert 'No such file' not in serial_info, "Cannot find the debugCard serial port"
        return cti_serial

    def get_product_name(self):
        return g_common_obj.adb_cmd_capture_msg("getprop ro.product.name")

    def execute_cmd(self, cmd):
        tmp_process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out, err = tmp_process.communicate()

        logger.debug("Output info:%s" % out.strip())
        return out.strip()

    def load_url(self, url):
        req = urllib2.Request(url)
        response = urllib2.urlopen(req)
        return response.read()

    def get_build_zip_from_url(self, msg):
        '''
        :param msg: read build url base info.
        :return:build zip name
        '''
        build_pattern = r'zip">(.*)</a>'
        get_no = re.compile(build_pattern)
        assert re.findall(get_no, msg) != [], "The zip does not exist"
        return re.findall(get_no, msg)[-1]

    def get_build_url(self, url_base):
        """
        To get the build link
        :param url_base:
        :return: build all link
        """
        build_zip = self.get_build_zip_from_url(self.load_url(url_base))
        return os.path.join(url_base, build_zip)

    def get_url_local_path(self, url):
        """
        To get the file link download path
        :param url: file link
        :return: local file path
        """
        return os.path.join(self.local_path_base, os.path.basename(os.path.abspath(url)))

    def download_file(self, url):
        '''
        Dowload file to local path
        :param url:file link
        :return:True, download success.
        '''
        chunk_size = 1024
        # local_path = os.path.join(local_path,os.path.basename(os.path.abspath(url)))
        local_path = self.get_url_local_path(url)
        logger.debug("Download %s to %s" % (url, local_path))
        # req_url = requests.get(url, auth=self.auth, verify=False, stream=True)
        req_url = requests.get(url, verify=False, stream=True)
        content_size = int(req_url.headers['content-length'])
        logger.debug("Download build size :%s K" % (content_size/chunk_size))
        if not req_url:
            logger.debug("not such file")
            return False
        if os.path.exists(local_path):
            logger.debug("try to remove old %s" % local_path)
            os.remove(local_path)
        time.sleep(3)
        if not os.path.exists(os.path.dirname(os.path.abspath(local_path))):
            os.makedirs(os.path.dirname(os.path.abspath(local_path)))
        with open(local_path, 'wb+') as code:
            for chunk in req_url.iter_content(chunk_size):
                code.write(chunk)
                current_size = os.path.getsize(local_path)
                current_percent = int(current_size * 100.0 / content_size)
                progress(100, current_percent)
        progress(100, 100)
        print '\n'
        if os.path.exists(local_path) and os.path.getsize(local_path) / chunk_size == content_size:
            logger.info("\n Build download %s success" % url)
            return True
        else:
            return False

    def download_build(self, url):
        '''
        Dowload build to local path
        :param url: the upper level of build link
        :param local_build_path_base: the path which build download to
        :return:
        '''
        self.download_file(self.get_build_url(url))

    def get_device_build(self):
        incre_cmd = "adb shell getprop ro.build.version.incremental"
        return self.execute_cmd(incre_cmd)

    def flash_build_bxtp(self, timeout=60):
        '''
        Flash DUT to specified old build
        :param timeout:
        :return: True, Flash success
        '''
        flash_cmd = 'python %s --serial_port="%s" --image_zip=%s' % (self.get_url_local_path(self.flash_script)\
                                                            , self.serial_port,\
                                                            self.get_url_local_path(self.get_build_url(self.origin_url)))

        logger.info("Flash BXT P Image via command:\n %s" % flash_cmd)
        self.execute_cmd(flash_cmd)
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.check_flash_buid(self.origin_url):
                logger.debug("flash to origin build success")
                return True
            time.sleep(5)
        return False

    def check_flash_buid(self, url):
        build_zip = self.get_build_zip_from_url(self.load_url(url))
        logger.debug("check the falshed buidl:%s"%build_zip)
        ota1_pattern = r"-([a-zA-Z0-9]*)"
        get_ota_pattern = re.compile(ota1_pattern)
        if self.get_device_build() in re.findall(get_ota_pattern, build_zip)[-1]:
            logger.debug("Device falsh to target build success")
            return True
        else:
            return False

    def check_device_sideload(self):
        devices_info = subprocess.check_output(["adb", "devices"], stderr=subprocess.STDOUT)
        logger.debug("'adb devices' call output: " + str(devices_info.strip()))
        devices = [i.split("\t") for i in devices_info.split(os.linesep)]
        sideload_info = [i[1] == "sideload" for i in devices if i[0] == self.serial]
        return all(sideload_info) and sideload_info

    def update_ota_build(self, ota_build_url, timeout=60):
        ota_build = self.get_url_local_path(self.get_build_url(ota_build_url))
        # Enter sideload mode
        self.execute_cmd("adb root")
        time.sleep(1)
        enter_sideload_cmd = 'adb reboot sideload-auto-reboot'
        self.execute_cmd(enter_sideload_cmd)
        # Check sideload mode
        for _ in range(10):
            time.sleep(3)
            if self.check_device_sideload():
                logger.debug('Enter into sideload mode ok, will do updating')
                break
        if not self.check_device_sideload():
            logger.debug('Failed enter sideload mode')
            return False
        # Try to update
        update_ota_cmd = 'adb sideload %s' % ota_build
        self.execute_cmd(update_ota_cmd)
        start_time = time.time()
        self.execute_cmd("adb reboot")
        while time.time() - start_time < timeout:
            if self.check_flash_buid(ota_build_url):
                logger.debug("ota updated success")
                return True
            time.sleep(5)
        return False

    def ota_update_adbsideload(self, ota_url=''):
        """
        Mainly function to test ota via adb sideload:
        1. Download flash build script, specified build, ota package
        2. Flash DUT to specified build
        3. OTA updated via adb sidleload
        :param ota_url:
        :return:
        """
        self.case_name = sys._getframe().f_back.f_code.co_name
        logger.info("Test case :%s" % self.case_name)
        self.download_file(self.flash_script)
        self.download_build(self.origin_url)
        self.download_build(ota_url)
        assert self.flash_build_bxtp(), 'Flash origin build failed'
        assert self.update_ota_build(ota_url), 'OTA updated failed'
        logger.debug("Case %s is pass!" % self.case_name)