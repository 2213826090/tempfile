#!/usr/bin/python
# -*- coding:utf-8 -*-
'''
module used to call android/tizen platform debug bridge tool
'''
import os
import time
import stat
import copy
import uuid
import datetime
import shlex
import zipfile
import signal
import logging
import threading
import traceback
import subprocess
import logging.handlers
import ConfigParser
from os.path import join, exists, dirname

FILE_LOG_LEVEL = "DEBUG"

CONSOLE_LOG_LEVEL = "INFO"

LOCAL_TIME_STAMP_FORMAT = '%Y-%m-%d_%H:%M:%S'

REPORT_TIME_STAMP_FORMAT = '%Y-%m-%d %H:%M:%S'

LEVELS = {"CRITICAL" : 50,
        "ERROR" : 40,
        "WARNING" : 30,
        "INFO" : 20,
        "DEBUG" : 10,
        "NOTSET" : 0,
       }

CONFIG_BASIC = {'product':None, 'revision':None, 'deviceid':None,\
                  'screen_width':None, 'screen_height':None}
CRASH_REPO_URL =\
'http://report1.amr.corp.intel.com/telemetry/machine.php?guilty='

PREV_TIME = None

def write_line(filepath, content):
    '''append a line to file'''
    try:
        open(filepath, 'w+').write(str(content))
    except:
        pass

def format_output(err):
    '''
    change the output format of exception
    '''
    return ''.join(err).replace('\n', '\r\n')

def zip_folder(folder_name, file_name, includeEmptyDIr=False):
    '''
    create a zip file for folder
    '''
    empty_dirs = []
    try:
        ziper = zipfile.ZipFile(file_name, 'w', zipfile.ZIP_DEFLATED)
        for root, dirs, files in os.walk(folder_name):
            empty_dirs.extend([d for d in dirs if\
                           os.listdir(os.path.join(root, d)) == []])
            for name in files:
                ziper.write(os.path.join(root, name), name)
            if includeEmptyDIr:
                for folder in empty_dirs:
                    zif = zipfile.ZipInfo(os.path.join(root, folder) + os.sep)
                    ziper.writestr(zif, '')
            empty_dirs = []
    except Exception, err:
        logger.debug('error: zip folder \n%s' % str(err))
        return False
    finally:
        if ziper != None:
            ziper.close()
    return True

def write_kmsg_record(msg, bridge='adb', serial=None):
    """transmit kmsg record before each TC"""
    exe = bridge
    try:
        _serial = ' -s ' + serial if serial else ''
        AdbCommand('%s%s shell echo \"%s\" > /proc/kmsg' %\
                       (exe, _serial, msg)).run()
    except Exception, err:
        logger.debug('write kmsg record failed:\n' + str(err))

def readTestPlan(fname, section):
    '''
    Get test case list from test case plan file.
    @type name: string
    @param name: the path of test case plan file
    @rtype: list
    @return: a list of test case
    '''
    parser = ConfigParser.RawConfigParser()
    parser.optionxform = str
    parser.read(fname)
    items = parser.items(section)
    return [(key, val) for key, val in items]

def uniqueID():
    '''return a unique id of test session.'''
    return str(uuid.uuid1())

def reportDir(seed):
    """get report directory"""
    return mkdir(join(os.getcwd(), 'report', seed.replace(' ', '_')))

def reportTime():
    '''return time stamp format with REPORT_TIME_STAMP_FORMAT'''
    global PREV_TIME
    cur_time = int(time.time())
    if cur_time <= PREV_TIME:
        cur_time = PREV_TIME + 1
    PREV_TIME = cur_time
    return time.strftime(REPORT_TIME_STAMP_FORMAT, time.localtime(cur_time))

def revertTime(str_time):
    """convert time from time stamp"""
    return datetime.datetime.strptime(str_time, REPORT_TIME_STAMP_FORMAT)

def getRealPath(in_path):
    '''generate abs path'''
    if not in_path:
        return None
    out_path = os.path.expanduser(in_path)
    if not os.path.isabs(out_path):
        out_path = os.path.join(os.getcwd(), in_path)
    return out_path

def get_context(test):
    """get test case context"""
    if hasattr(test, 'contexts'):
        return getattr(test, 'contexts')
    return None

def mkdir(path):
    '''
    create a folder
    @type path: string
    @param path: the path of folder
    @rtype: string
    @return: the path of the folder, return None if fail to create folder
    '''
    if not os.path.exists(path):
        os.makedirs(path)
    return path

def forcerm(fn, path, excinfo):
    '''
    force delete a folder
    @type path: string
    @param path: the path of folder
    @type excinfo: string
    @param excinfo: the output info when exception
    '''
    if fn is os.rmdir:
        os.chmod(path, stat.S_IWRITE)
        os.rmdir(path)
    elif fn is os.remove:
        os.chmod(path, stat.S_IWRITE)
        os.remove(path)

def getDevices():
    """get DUT list"""
    devices = {}
    ret = AdbCommand('adb devices', retry=2, timeout=5).run()
    if ret.find('List of devices attached') == -1:
        raise Exception('adb error: %s' % ret)
    _ = [i.strip() for i in ret.strip().split('\n')]
    _.remove('List of devices attached')
    if not _:
        return devices
    try:
        devices = dict([i.strip().split('\t') for i in _])
    except:
        devices = {}
    return devices

def getDeviceProps(serial):
    """get device property"""
    cfg = copy.copy(CONFIG_BASIC)
    cfg['deviceid'] = serial
    product = AdbCommand('adb -s %s shell getprop ro.product.device' %\
                  serial, retry=2, timeout=5).run()
    cfg['product'] = product.strip() if product else 'unknown'
    revision = AdbCommand(\
        'adb -s %s shell getprop ro.build.version.incremental' %\
                  serial, retry=2, timeout=5).run()
    cfg['revision'] = revision.strip() if revision else 'unknown'
    try:
        tm_uuid = AdbCommand('adb -s %s shell cat /data/telemetry/machine_id' %\
                    serial, retry=2, timeout=5).run()
        cfg['uuid'] = tm_uuid.strip() if not 'No such file or directory' in\
                    tm_uuid else 'unknown'
    except:  # adb version > 1.0.32
        cfg['uuid'] = 'unknown'
    cfg['crash_tool_url'] = CRASH_REPO_URL
    os.environ['ANDROID_SERIAL'] = serial
    return cfg

class Command(object):
    """
    subprocess commands in a different thread with TIMEOUT option.
    """
    command = None
    process = None
    status = None
    output, error = '', ''

    def __init__(self, command):
        if isinstance(command, basestring):
            command = shlex.split(command)
        self.command = command

    def start(self, timeout=None, **kwargs):
        """ Run a command then return: (status, output, error). """
        def target(**kwargs):
            """run target"""
            try:
                self.process = subprocess.Popen(self.command, **kwargs)
                self.output, self.error = self.process.communicate()
                self.status = self.process.returncode
            except:
                self.error = traceback.format_exc()
                self.status = -1
        # default stdout and stderr
        if 'stdout' not in kwargs:
            kwargs['stdout'] = subprocess.PIPE
        if 'stderr' not in kwargs:
            kwargs['stderr'] = subprocess.PIPE
        # thread
        thread = threading.Thread(target=target, kwargs=kwargs)
        thread.start()
        thread.join(timeout)
        if thread.is_alive():
            self.kill_proc(self.process.pid)
            thread.join()
        return self.status, self.output, self.error

    @staticmethod
    def kill_proc(pid):
        """kill process"""
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass

class AdbCommand(Command):
    """run adb command in shell"""
    def __init__(self, cmd, retry=3, timeout=10):
        Command.__init__(self, cmd)
        self.retry = retry
        self.timeout = timeout

    def run(self):
        """execute command"""
        output = None
        error = None
        status = -1
        while self.retry:
            status, output, error = Command.start(self, timeout=self.timeout,\
                         stderr=subprocess.STDOUT)
            if not status:
                return output
            self.retry -= 1
            time.sleep(1)
        if output:
            raise Exception(str(output))
        elif error:
            raise Exception(str(error))
        return ""

class Logger(object):
    """class used to print log"""
    _instance = None
    _mutex = threading.Lock()

    def __init__(self, level="DEBUG"):
        '''
        constructor of Logger
        '''
        self._logger = logging.getLogger("NoseRunner")
        self._logger.setLevel(LEVELS[level])
        requests_log = logging.getLogger("requests")
        requests_log.setLevel(logging.WARNING)
        self._formatter = logging.Formatter(\
            "[%(asctime)s - %(levelname)s ] %(message)s", '%Y-%m-%d %H:%M:%S')
        self._formatterc = logging.Formatter(\
            "[%(asctime)s - %(levelname)s] %(message)s", '%Y-%m-%d %H:%M:%S')

    def set_root_dir(self, root_dir):
        self.root_dir = root_dir

    def startLogging(self):
        self.add_file_logger()
        self.add_console_logger()

    def add_file_logger(self, log_file="test.log", file_level="DEBUG"):
        '''
        generate file writer
        @type log_file: string
        @param log_file: the path of log file
        @type file_level: string
        @param file_level: the log output level.Defined in global LEVELS
        '''
        logFolder = 'log'
        if exists(self.root_dir):
            logFolder = join(self.root_dir, logFolder)
            if not os.path.exists(logFolder):
                mkdir(logFolder)
        else:
            mkdir(logFolder)
        log_file = join(logFolder, log_file)
        if not os.path.exists(log_file):
            open(log_file, 'w')

        fh = logging.handlers.RotatingFileHandler(log_file, mode='a',\
                        maxBytes=1024*1024*1, backupCount=100, encoding="utf-8")
        fh.setLevel(LEVELS[file_level])
        fh.setFormatter(self._formatter)
        self._logger.addHandler(fh)

    def add_console_logger(self, console_level="INFO"):
        '''
        generate console writer
        @type console_level: string
        @param console_level: the level of console
        '''
        ch = logging.StreamHandler()
        ch.setLevel(LEVELS[console_level])
        ch.setFormatter(self._formatterc)
        self._logger.addHandler(ch)

    @staticmethod
    def getLogger(level="DEBUG"):
        '''
        return the logger instance
        @type level: string
        @param level: the level of logger
        @rtype: Logger
        @return: the instance of Logger
        '''
        if Logger._instance == None:
            Logger._mutex.acquire()
            Logger._instance = Logger(level)
            Logger._mutex.release()
        return Logger._instance

    def debug(self, msg):
        '''
        print message for debug level
        @type msg: string
        @param msg: the content of msg
        '''
        if msg is not None:
            self._logger.debug(msg)

    def info(self, msg):
        '''
        print message for info level
        @type msg: string
        @param msg: the content of msg
        '''
        if msg is not None:
            self._logger.info(msg)

    def warning(self, msg):
        '''
        print message for warning level
        @type msg: string
        @param msg: the content of msg
        '''
        if msg is not None:
            self._logger.warning(msg)

    def error(self, msg):
        '''
        print message for error level
        @type msg: string
        @param msg: the content of msg
        '''
        if msg is not None:
            self._logger.error(msg)

    def critical(self, msg):
        '''
        print message for critical level
        @type msg: string
        @param msg: the content of msg
        '''
        if msg is not None:
            self._logger.critical(msg)

logger = Logger.getLogger()
