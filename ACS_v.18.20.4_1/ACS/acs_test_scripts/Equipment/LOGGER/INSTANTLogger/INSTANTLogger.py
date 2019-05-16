
import os
from os import path
import shutil
import traceback
import zipfile
import logging
import threading
import time
import shlex
import Queue

import sys
import nose
import atexit
import subprocess

from urlparse import urljoin
from urlparse import urlparse

from Core.CampaignMetrics import CampaignMetrics
from Core.PathManager import Folders
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Caching import SysErrors, ArtifactoryCacheManager, compute_file_hash
import UtilitiesFWK.Utilities as Util
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.LOGGER.Interface.IINSTANTLogger import IINSTANTLogger
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.HttpDownloaderUtil import HttpDownloaderUtil

import os
import fileinput
from ConfigParser import ConfigParser
from os.path import join, exists, basename, dirname, expanduser
from client import ReportClient

from tools import AdbCommand, logger, reportTime, mkdir, uniqueID, Command
from tools import getDevices, getDeviceProps, get_context
from tools import write_line, format_output, zip_folder
import filecmp

import json

from Device.DeviceManager import DeviceManager

LOCATION_NOT_FOUND_EXCEPTION = '%s not found.'

LOGCAT_FILE_NAME = 'logcat.txt'

DMESGLOG_FILE_NAME = 'dmesg.txt'

FAILURE_SNAPSHOT_NAME = 'failure.png'

PASS_SNAPSHOT_NAME = 'pass.png'

ANDROID_LOG_SHELL = '%s %s %s logcat -v threadtime'

ANDROID_KMSGLOG_SHELL = '%s %s %s shell cat /proc/kmsg'

DMESGLOG_FILE_EX = 'dmesg.%s.txt'

PROCESS_FILE_NAME = 'ps.txt'

class TestCaseContext(object):
    """
    Test case context which is injected to the context of test case
    instance by plugin when prepareTestCase.
    """
    def __init__(self, report_root=None, serial=None):
        self.__user_log_dir = None
        self.__report_root = report_root
        self.serial = None
        self.case_id = 0
        self.case_dir_name = None
        self.case_start_time = None
        self.case_end_time = None
        self.error_case_dir = None
        self.fail_case_dir = None
        self.pass_case_dir = None
        self.serial = serial
        self.crash_num = None
    def setup(self, tid, val):
        """set test case log dir"""
        self.case_id = tid
        timestr = time.strftime('%Y-%m-%d %H:%M:%S', \
                                time.localtime(int(time.time())))
        self.case_start_time = timestr
        self.case_dir_name = val
        _tname = '%s%s%s' % (val, '@',
                 str(self.case_start_time).replace(' ', '_'))
        _dir = mkdir(join(self.__report_root, 'fail'))
        self.fail_case_dir = join(_dir, _tname)
        _dir = mkdir(join(self.__report_root, 'error'))
        self.error_case_dir = join(_dir, _tname)
        _dir = mkdir(join(self.__report_root, 'pass'))
        self.pass_case_dir = join(_dir, _tname)
        if self.serial:
            _dir = join(self.__report_root, 'tmp', self.serial, _tname)
        else:
            _dir = join(self.__report_root, 'tmp', _tname)
        self.__user_log_dir = join(_dir, 'logs')

        os.environ['PYUNIT_USER_LOG_DIR'] = self.__user_log_dir

        mkdir(self.__user_log_dir)
        
    @property
    def user_log_dir(self):
        """get test log dir"""
        return self.__user_log_dir

    @property
    def fail_screenshot_at_failure(self):
        """get failed test case screenshot dir"""
        return join(self.fail_case_dir, 'logs', FAILURE_SNAPSHOT_NAME)

    @property
    def screenshot_at_pass(self):
        """get pass test case screenshot dir"""
        return join(self.pass_case_dir, 'logs', PASS_SNAPSHOT_NAME)

    @property
    def error_screenshot_at_failure(self):
        """get error test case screenshot dir"""
        return join(self.error_case_dir, 'logs', FAILURE_SNAPSHOT_NAME)

    @property
    def pass_log_dir(self):
        """get pass TC log dir"""
        return join(self.pass_case_dir, 'logs')

    @property
    def fail_log_dir(self):
        """get fail TC log dir"""
        return join(self.fail_case_dir, 'logs')

    @property
    def error_log_dir(self):
        """get error TC log dir"""
        return join(self.error_case_dir, 'logs')

    @property
    def pass_log(self):
        """get pass TC log"""
        return join(self.pass_case_dir, 'log.zip')

    @property
    def fail_log(self):
        """get fail TC log"""
        return join(self.fail_case_dir, 'log.zip')

    @property
    def error_log(self):
        """get error TC log"""
        return join(self.error_case_dir, 'log.zip')

def _get_process_logs(tc_path, serial=None):
    """record the dmesg logs before test"""
    try:
        _serial = '-s ' + serial if serial else ''
        AdbCommand('adb %s shell ps > /sdcard/%s 2>&1' % (\
                        _serial, PROCESS_FILE_NAME)).run()
        AdbCommand('adb %s pull /sdcard/%s %s' % (\
                        _serial, PROCESS_FILE_NAME, tc_path)).run()
    except Exception, err:
        logger.debug('get initial device logs error:\n' + str(err))


def _get_screenshot_logs(tc_path, fname, serial=None):
    """get screenshot"""
    try:
        _serial = '-s ' + serial if serial else ''
        AdbCommand('adb %s shell screencap /data/local/tmp/%s' %\
                    (_serial, fname)).run()
        AdbCommand('adb %s pull /data/local/tmp/%s %s' %\
                    (_serial, fname, tc_path)).run()
        AdbCommand('adb %s shell rm /data/local/tmp/%s' %\
                    (_serial, fname)).run()
    except Exception, err:
        logger.debug('erro: capture screenshot failed\n%s' % str(err))

def _get_kernel_logs(tc_path, ts_path=None, serial=None):
    """record the dmesg logs before test"""
    try:
        _serial = '-s ' + serial if serial else ''
        AdbCommand('adb %s shell dmesg > /sdcard/%s 2>&1' % (\
                        _serial, DMESGLOG_FILE_EX % (1))).run()
        AdbCommand('adb %s pull /sdcard/%s %s' % (\
                        _serial, DMESGLOG_FILE_EX % (1), ts_path)).run()
        dmesg0 = join(ts_path, DMESGLOG_FILE_EX % (0))
        dmesg1 = join(ts_path, DMESGLOG_FILE_EX % (1))
        dmesg = join(tc_path, DMESGLOG_FILE_NAME)
        if exists(dmesg0):
            os.system("diff -s %s %s |grep '^< '|cut -d' ' -f2- > %s" %\
                             (dmesg1, dmesg0, dmesg))
        else:
            shutil.copyfile(dmesg1, dmesg)
        shutil.copyfile(dmesg1, dmesg0)
    except Exception, err:
        logger.debug('get initial device logs error:\n' + str(err))

def _get_dumpsys_logs(tc_path, serial=None):
    """get dumpsys logs"""
    try:
        _serial = '-s ' + serial if serial else ''
        AdbCommand('adb %s shell dumpsys window > /data/local/tmp/dumpsys_window.log 2>&1' % (_serial)).run()
        AdbCommand('adb %s pull /data/local/tmp/dumpsys_window.log %s' % (_serial, tc_path)).run()
        AdbCommand('adb %s shell dumpsys meminfo > /data/local/tmp/dumpsys_meminfo.log 2>&1' % (_serial)).run()
        AdbCommand('adb %s pull /data/local/tmp/dumpsys_meminfo.log %s' % (_serial, tc_path)).run()
        AdbCommand('adb %s shell dumpsys cpuinfo > /data/local/tmp/dumpsys_cpuinfo.log 2>&1' % (_serial)).run()
        AdbCommand('adb %s pull /data/local/tmp/dumpsys_cpuinfo.log %s' % (_serial, tc_path)).run()
        AdbCommand('adb %s shell dumpsys wifi > /data/local/tmp/dumpsys_wifi.log 2>&1' % (_serial)).run()
        AdbCommand('adb %s pull /data/local/tmp/dumpsys_wifi.log %s' % (_serial, tc_path)).run()
        AdbCommand('adb %s shell dumpsys activity > /data/local/tmp/dumpsys_activity.log 2>&1' % (_serial)).run()
        AdbCommand('adb %s pull /data/local/tmp/dumpsys_activity.log %s' % (_serial, tc_path)).run()
        AdbCommand('adb %s shell dumpsys diskstats > /data/local/tmp/dumpsys_diskstats.log 2>&1' % (_serial)).run()
        AdbCommand('adb %s pull /data/local/tmp/dumpsys_diskstats.log %s' % (_serial, tc_path)).run()
    except Exception, err:
        logger.debug('erro: dumpsys failed\n%s' % str(err))

def _get_crash_num(crash_tool_path, serial, timeout=1200):
    if not os.path.exists(crash_tool_path):
        raise Exception("Crash Tool Not Found.")

    _serial = "-snn " + serial if serial else ""

    cmd = Command("java -jar {0} {1} -getEventCount".format(crash_tool_path, _serial))
    (status, output, error) = cmd.start(timeout=timeout)

    if not status == 0:
        raise Exception("Crash Tool return failure. exit code = {0}".format(status))
    
    crash_num = output.split(",")[2]
    return int(crash_num)


def _get_crash_log_file(crash_tool_path, tc_path, crash_num, serial, timeout=1200):
    EVENTS_FILE = "events.json"
    _serial = "-s " + serial if serial else ""
    cut_serial = "-snn " + serial if serial else ""

    if not os.path.exists(crash_tool_path):
        raise Exception("Crash Tool Not Found.")

    cmd = Command("java -jar {0} {3} -getEvents {2}/{1}".format(crash_tool_path, EVENTS_FILE, tc_path, cut_serial))
    (status, output, error) = cmd.start(timeout=timeout)

    if not status == 0:
        raise Exception("Crash Tool return failure. exit code = {0}".format(status))

    with file("{0}/{1}".format(tc_path, EVENTS_FILE), "r") as fv:
        jsonstr = fv.read()
        events = json.loads(jsonstr)
        crashes = [event for event in events if event["eventName"] == "CRASH"]
        
        with file("{0}/crash_events.json".format(tc_path), "w") as crash_json:
            crash_json.write(json.dumps(crashes, indent=2))

        #for crash in crashes[-crash_num:]:
        #    crashdir = "{0}/{1}".format(tc_path, crash["crashdir"].split("/")[-1])
        #    if not os.path.exists(crashdir):
        #        os.mkdir(crashdir)
        #    cmd = Command("adb {2} pull {0} {1}".format(crash["crashdir"], crashdir, _serial))
        #    (status, output, error) = cmd.start(timeout=timeout)


def _calculate_log_issue(log_dir, rules):
    '''
    return a issue calculated result based on issue rules config
    '''
    def _file_search(filepath, keywords):
        '''search issue by keyword'''
        b_search = 0
        for line in fileinput.FileInput(filepath):
            for _keyw in keywords:
                if line.find(_keyw) > 0:
                    b_search += 1
        return b_search
    ret = []
    if not rules:
        return ret
    for key, data in rules.iteritems():
        filenames = [_file.strip(' "') for _file in data['logs'].split(',')\
                        if _file]
        keywords = [_keyw.strip(' "') for _keyw in data['keywords'].split(',')\
                        if _keyw]
        severity = data.get('severity', 'others')
        _count = 0
        for _filename in filenames:
            _path = join(log_dir, _filename)
            if exists(_path):
                _count += _file_search(_path, keywords)
        if _count:
            ret.append({'issue_name':key, 'severity':severity, 'count':_count})
    return ret

ANDROID_TOMBSTONE_PATH = '/data/tombstones'

ANDROID_ANRLOGS_PATH = '/data/anr/traces.txt'

KMEMLEAK_FILE = 'kmemleak.txt'

def _remove_duplicated_files(src, dest):
    """remove same file"""
    src_files = []
    dest_files = []
    for root, _, filenames in os.walk(src):
        for _file in filenames:
            src_files.append(os.path.join(root, _file))

    for root, _, filenames in os.walk(dest):
        for _file in filenames:
            dest_files.append(os.path.join(root, _file))

    for _file in src_files:
        for _filed in dest_files:
            if filecmp.cmp(_file, _filed):
                try:
                    os.remove(_file)
                except Exception:
                    pass
                break

def _make_log(path, serial=None):
    """pull log/snapshot from device to local report folder"""
    _serial = '-s ' + serial if serial else ''
    try:
        # tombstone
        AdbCommand('adb %s pull %s %s' % (_serial,
                         ANDROID_TOMBSTONE_PATH, path)).run()
        tb_tmp_path = mkdir(join(expanduser('~'), 'tombstones', serial or ''))
        _remove_duplicated_files(path, tb_tmp_path)
        os.system('rm -rf %s/*' % tb_tmp_path)
        AdbCommand('adb %s pull %s %s' % (_serial,
                         ANDROID_TOMBSTONE_PATH, tb_tmp_path)).run()
        # anr
        AdbCommand('adb %s pull %s %s/%s' % (_serial,
                         ANDROID_ANRLOGS_PATH, path, 'anr.txt')).run()
        AdbCommand('adb %s shell rm %s' % (_serial,
                         ANDROID_ANRLOGS_PATH)).run()
    except Exception, err:
        logger.debug('get error in pulling device logs:\n' + str(err))

def _get_testcase_context(test):
    '''get test case context'''
    if hasattr(test, 'contexts'):
        return getattr(test, 'contexts')
    return None


def _get_issue_def(config):
    '''Get issue definition'''
    ret = {}
    cfp = ConfigParser()
    cfp.read(config)
    for _section in cfp.sections():
        ret[_section] = dict(cfp.items(_section))
    return ret


class LogcatHandler(object):
    """Android logcat logger"""
    def __init__(self, bridge='adb', serial=None):
        self.__bridge = bridge
        self.__serial = serial
        self.cache_thread = None

    def start(self):
        """start logcat collecting process"""
        cmd = None
        exe = self.__bridge
        if self.__serial:
            cmd = ANDROID_LOG_SHELL % (exe, '-s', self.__serial)
        else:
            cmd = ANDROID_LOG_SHELL % (exe, '', '')
        cmds = shlex.split(cmd)
        self.cache_thread = LogCacheWrapper(cmds)
        self.cache_thread.setDaemon(True)
        self.cache_thread.start()

    def stop(self):
        """Stop logcat collecting process"""
        if self.cache_thread:
            self.cache_thread.stop()

class LogCacheWrapper(threading.Thread):
    """commn logger process wrapper"""
    def __init__(self, cmd):
        threading.Thread.__init__(self)
        self.__fd = None
        #self.daemon = True
        self.__cmd = cmd
        self.__queue = Queue.Queue()
        atexit.register(self.stop)

    def stop(self):
        """Stop logger process"""
        if self.__fd and self.__fd.poll() == None:
            self.__fd.terminate()
            logger.info("Stop logger: %s" % self.__cmd)

    def save(self, path, prefix_str=None):
        """save log to file"""
        try:
            if not self.__queue.qsize():
                return False
            with open(path, 'w+') as p_fd:
                if prefix_str:
                    p_fd.write(prefix_str)
                for _ in range(self.__queue.qsize()):
                    line = self.__queue.get(block=True)
                    p_fd.write(line)
            return True
        except Exception, err:
            logger.debug('error: save log\n%s' % str(err))
            return False

    def run(self):
        """Start logger process"""
        while True:
            if self.__fd and self.__fd.poll() == None:
                for line in iter(self.__fd.stdout.readline, ''):
                    self.__queue.put(line)
            else:
                self.__fd = self.restart()
                logger.info("Start logger: %s" % " ".join(self.__cmd))
            time.sleep(2)

    def restart(self):
        """Restart logger when it lost somehow"""
        return subprocess.Popen(self.__cmd,
                                stderr=subprocess.STDOUT,
                                stdout=subprocess.PIPE,
                                close_fds=True)

SESSION_URL_TEMPLATE = '/smartserver/index.html#/smartserver/session/'

class INSTANTLogger(EquipmentBase, IINSTANTLogger):
    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor that always returns the same instance of ArtifactManager

        """
        EquipmentBase.__init__(self, name, model, eqt_params)

        self._device = DeviceManager().get_device('PHONE1')

        sw_release = self._device.device_properties.sw_release

        sw_release = sw_release.split(' ')[3]
        self._logger.info('sw_release: ' + sw_release)

        device_id = self._device.device_properties.device_id
        self._logger.info('device_id: ' + device_id)

        deviceprops = getDeviceProps (device_id)
        self.__bench_params = bench_params

        self.__campaignmetrics = CampaignMetrics.instance()

        self._total_tc_count = self.__campaignmetrics.total_tc_count
        self._total_tc_executed = 0

        rpt_url_elements = urlparse(self.__bench_params.get_param_value('auth'))
        self._session_url_base = None
        if (rpt_url_elements.scheme):
            self._session_url_base = rpt_url_elements.scheme + "://"
        self._session_url_base = self._session_url_base + rpt_url_elements.hostname
        if (rpt_url_elements.port):
            self._session_url_base = self._session_url_base + ':' + str(rpt_url_elements.port)
        self._session_url_base = self._session_url_base + SESSION_URL_TEMPLATE

        _client_conf = {}
        _client_conf.update({'username': self.__bench_params.get_param_value('username'), \
                             'password':  self.__bench_params.get_param_value('password'), \
                             'auth':  self.__bench_params.get_param_value('auth'), \
                             'session_create':  self.__bench_params.get_param_value('session_create'), \
                             'session_update':  self.__bench_params.get_param_value('session_update'), \
                             'case_update':  self.__bench_params.get_param_value('case_update'), \
                             'file_upload':  self.__bench_params.get_param_value('file_upload'), \
                             'product': self.__bench_params.get_param_value('product'),
                             'revision': sw_release,
                             'deviceid': device_id,
                             'uuid': deviceprops['uuid'],
                             'crash_tool_url': deviceprops['crash_tool_url'],
                             'screen_width': self.__bench_params.get_param_value('screen_width'),
                             'screen_height': self.__bench_params.get_param_value('screen_height')})



        _client_conf.update({'planname': 'acs_test'})


        self.__tags = None

        self.issue_tag_file = os.path.dirname(os.path.abspath(__file__)) + '/issues_def.config'

        self._logger.info('issue_tag_file: ' + self.issue_tag_file)

        if exists(self.issue_tag_file):
            self.__tags = _get_issue_def(self.issue_tag_file)

        self.result = {'payload': None, 'extras': None}
        self.start_time = reportTime()
        self.cycle_id = 1
        self.session_id = uniqueID()
        self._log_dir = os.path.join(expanduser("~"), \
                                     self.__bench_params.get_param_value('logdir'))

        logger.set_root_dir(self._log_dir)
        logger.startLogging()

        self._serial = device_id


        self.__logcat_handler = None
        self.__dmesg_handler = None
        self.__dev = {}
        self._cycle_dmesg_log = None
        self._pass_report_dir = None
        self._fail_report_dir = None
        self._error_report_dir = None
        self._timeout_report_dir = None
        self._report_dir = None
        self._case_report_dir = None
        self.stream = None
        self.method_name = None
        self.start_tid = 1
        self.adb_failsave = False

        self._report_dir = mkdir(join(self._log_dir, 'report',
                                 self.start_time.replace(' ', '_')))

        self.__client = ReportClient(**_client_conf)
        token = self.__client.regist()
        if not token:
            self.logger.error("couldn't get token from report server.")
            self.logger.error("disable report uploading")
            self.__client = None

    def init(self):
        """
        initialize attributes

        """
        self.logger.info("Initialization")

    def initContext(self, suite, casename):
        self.ctx = TestCaseContext(self._report_dir, self._serial)
        self.ctx.setup(self._total_tc_executed, casename)
        try:
            crashtooluploader = self.__bench_params.get_param_value('crashtooluploader')
            self.logger.debug("Crashtooluploader @ " + crashtooluploader)
            try:
                self._crash_num = _get_crash_num(crashtooluploader, self._serial)
                self.logger.debug("{0} Crash(es) on board".format(self._crash_num))
            except Exception, ex:
                self.logger.debug("Get Crash Log Exception")
                self.logger.debug(ex)
        except:
            self.logger.debug("Crashtooluploader not found, Skip")

        setattr(suite, 'contexts', self.ctx)

    def setOutputStream(self, stream):
        """Get handle on output stream so the plugin can print id #n"""
        self.stream = stream

    def __ping_adb(self):
        """adb ping to ensure device availability"""
        output = AdbCommand('%s -s %s shell echo ping' % ('adb',
                     self._serial)).run()
        return output

    def __check_bridge(self):
        """Check if device is connected"""
        output = ''
        try:
            output = self.__ping_adb()
        except Exception, err:
            logger.error('connect debug bridge exception:' + str(err))
        if not output or output.strip() != 'ping':
            logger.error('connect debug bridge fail')
            return False
        return True

    def __check_bridge_retry(self, times, action=False):
        """Repeatedly check if device is connected for times"""
        for _ in range(0, times):
            if self.__check_bridge():
                return True
            if action:
                os.system('adb kill-server')
                logger.info("kill adb server, then re-connect")
            time.sleep(8)
        return self.__check_bridge()


    @property
    def log_dir(self):
        return self._log_dir

    @log_dir.setter
    def log_dir(self, value):
        self._log_dir = value

    @property
    def report_dir(self):
        return self._report_dir

    @report_dir.setter
    def report_dir(self, value):
        self._report_dir = value

    @property
    def serial(self):
        return self._serial

    @serial.setter
    def serial(self, value):
        self._serial = value

    @property
    def total_tc_count(self):
        return self._total_tc_count

    @total_tc_count.setter
    def total_tc_count(self, value):
        self._total_tc_count = value

    @property
    def total_tc_executed(self):
        return self._total_tc_executed

    @total_tc_executed.setter
    def total_tc_executed(self, value):
        self._total_tc_executed = value

    def begin(self):
        """begin test session"""
        session_properties = {'sid': self.session_id,
                              'starttime': self.start_time,
                              'progress': self.cycle_id}
        if self.__client:
            if self.cycle_id == 1:
                self.__client.createSession(**session_properties)
            else:
                self.__client.updateSession(**session_properties)

        self._logger.info('Live report session url: '+ self._session_url_base + self.session_id)

        if not self.__check_bridge_retry(1):
            logger.error('runner exit due to %s\n' % 'adb echo ping failed')
        self._pass_report_dir = mkdir(join(self._report_dir, 'pass'))
        self._fail_report_dir = mkdir(join(self._report_dir, 'fail'))
        self._error_report_dir = mkdir(join(self._report_dir, 'error'))
        self._timeout_report_dir = mkdir(join(self._report_dir, 'timeout'))
        self.__logcat_handler = LogcatHandler(serial=self.serial)
        self.__logcat_handler.start()

    def startTest(self, test):

        ctx = get_context(test)
        try:
            self.stream.write('#%d %s ' % (ctx.case_id, ctx.case_start_time))
        except Exception, err:
            logger.debug('error: output stream write\n%s' % str(err))
        self.__check_bridge_retry(3, self.adb_failsave)

    def stopTest(self, test):
        """after test case stop"""
        if (self.__tags):
            self.result['payload'].update({'tags': \
                 _calculate_log_issue(self.log_dir, self.__tags)})

        self.start_tid += 1
        ctx = get_context(test)
        if not ctx:
            return

        _make_log(ctx.user_log_dir, self._serial)

        logdir = ctx.user_log_dir
        zip_folder(logdir, join(dirname(logdir), 'log.zip'))
        try:
            shutil.move(dirname(logdir), self._case_report_dir)
        except Exception, err:
            logger.debug('move log error: \n' + str(err))

        if self.__client:
            self.__client.updateTestCase(**self.result)

    def __save_common_logs(self, test, errmsg=None):
        """save logs"""
        ctx = get_context(test)
        if not ctx:
            return
        # trace log
        if errmsg:
            write_line(join(ctx.user_log_dir, 'tc_trace.txt'),
                               format_output(errmsg))
        # logcat
        __msg = '******test case #%d %s start******\r\n' %\
                          (ctx.case_id, self.method_name)
        logcat_file = join(ctx.user_log_dir, LOGCAT_FILE_NAME)
        try:
            self.__logcat_handler.cache_thread.save(logcat_file, __msg)
        except Exception, err:
            logger.debug('error: create logcat file failed\n%s' % str(err))

        # dmesg
        _get_kernel_logs(tc_path=ctx.user_log_dir, ts_path=self._report_dir,
                                 serial=self._serial)
        # ps
        _get_process_logs(tc_path=ctx.user_log_dir, serial=self._serial)

        # snap screen
        snapshot_name = 'failure.png' if errmsg else 'pass.png'
        _get_screenshot_logs(tc_path=ctx.user_log_dir, fname=snapshot_name,
                                 serial=self._serial)
        # dumpsys
        _get_dumpsys_logs(tc_path=ctx.user_log_dir, serial=self._serial)

        # crashlog
        try:
            crashtooluploader = self.__bench_params.get_param_value('crashtooluploader')
            self.logger.debug("crashtooluploader @ " + crashtooluploader)
            self.logger.debug("Original carshes on board = " + str(self._crash_num))
            try:
                crash_num = _get_crash_num(crashtooluploader, self._serial)
                new_crash_num = crash_num - self._crash_num
                
                self.logger.debug("New {0} Crash(es) on board".format(new_crash_num))
                if new_crash_num:
                    self.logger.debug("log dir @ " + ctx.user_log_dir)
                    _get_crash_log_file(crashtooluploader, ctx.user_log_dir, crash_num, serial=self._serial)
                        
            except Exception, ex:
                self.logger.debug("Get Crash Log Exception")
                self.logger.debug(ex)
        except:
            self.logger.debug("Crashtooluploader not found, Skip")


    def handleFailure(self, test, err):
        '''
        Called on addFailure. To handle the failure yourself and
        prevent normal failure processing, return a true value.
        '''

        self.__save_common_logs(test, errmsg=err)

        self.result.clear()
        ctx = _get_testcase_context(test)
        if not ctx:
            return
        self.result.update(
            {'extras':
                 {'screenshot_at_last': ctx.fail_screenshot_at_failure,
                  'log': ctx.fail_log,
                 }
            })

    def handleError(self, test, err):
        '''
        Called on addError. To handle the failure yourself and
        prevent normal error processing, return a true value.
        '''
        self.__save_common_logs(test, errmsg=err)

        self.result.clear()
        ctx = _get_testcase_context(test)
        if not ctx:
            return
        self.result.update(
            {'extras': {'screenshot_at_last': ctx.error_screenshot_at_failure,
                        'log': ctx.error_log,
                        }
            })

    def addFailure(self, test, err, capt=None, tbinfo=None):
        '''when TC failure, update live monitor '''


        ctx = _get_testcase_context(test)
        if not ctx:
            return

        self.handleFailure(test, err)

        ctx.case_end_time = reportTime()
        self._case_report_dir = self._fail_report_dir

        self.result.update(
            {'payload': {'tid': ctx.case_id,
                         'casename': ctx.case_dir_name,
                         'starttime': ctx.case_start_time,
                         'endtime': ctx.case_end_time,
                         'result': 'fail'
                         }
            })
        self._log_dir = ctx.fail_log_dir

    def addError(self, test, err, capt=None):
        '''when TC error, update live monitor '''
        ctx = _get_testcase_context(test)
        if not ctx:
            return

        self.handleError(test, err)

        ctx.case_end_time = reportTime()
        self._case_report_dir = self._error_report_dir

        self.result.update(
            {'payload': {'tid': ctx.case_id,
                         'casename': ctx.case_dir_name,
                         'starttime': ctx.case_start_time,
                         'endtime': ctx.case_end_time,
                         'result': 'error'
                         }
            })
        self._log_dir = ctx.error_log_dir

    def addSuccess(self, test, capt=None):
        '''when TC success, update live monitor '''
        ctx = _get_testcase_context(test)
        if not ctx:
            return

        ctx.case_end_time = reportTime()
        self.__save_common_logs(test)
        self._case_report_dir = self._pass_report_dir

        self.result.clear()
        self.result.update(
            {'payload': {'tid': ctx.case_id,
                         'casename': ctx.case_dir_name,
                         'starttime': ctx.case_start_time,
                         'endtime': ctx.case_end_time,
                         'result': 'pass'
                         },
             'extras': {'screenshot_at_last': ctx.screenshot_at_pass,
                        'log': ctx.pass_log,
                       }
            })
        self._log_dir = ctx.pass_log_dir

    def finalize(self, result):
        '''Test end, clean up'''
        self.__logcat_handler.stop()

        session_properties = {'sid': self.session_id, 'endtime': reportTime()}
        if self.__client:
            self.__client.updateSession(**session_properties)
        self._logger.info('Live report session url: '+ self._session_url_base + self.session_id)
