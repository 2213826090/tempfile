#/usr/bin/env python


import os, time
import sys
import logging
from testlib.base import base_utils


class testlib_log():

    __metaclass__ = base_utils.SingletonType

    def __init__(self, log_path = None, log_name = None, log_stdout = True, header = None, debug_level = True):
        log_formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
        self.logger = logging.getLogger()
        if "LOG_PATH" in os.environ:
            log_path = os.environ["LOG_PATH"]
        else:
            if log_path is None:
                dirfmt = "logs_%4d-%02d-%02d_%02d-%02d-%02d"
                log_path = dirfmt % time.localtime()[0:6]
            else:
                dirfmt = "logs_%4d-%02d-%02d_%02d-%02d-%02d"
                log_path = os.path.join(log_path, dirfmt % time.localtime()[0:6])

        try:
            if not os.path.exists(log_path):
                os.makedirs(log_path)
        except OSError:
            pass

        if log_stdout:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(log_formatter)
            self.logger.addHandler(console_handler)
        if log_name:
            file_handler = logging.FileHandler("{0}/{1}.log".format(log_path, log_name))
            file_handler.setFormatter(log_formatter)
            self.logger.addHandler(file_handler)
        if debug_level:
            self.logger.setLevel(logging.DEBUG)

    def debug(self, message):
        self.logger.debug(message)

    def info(self, message):
        self.logger.info(message)

    def error(self, message):
        self.logger.error(message)


class log_stdout_stderr(object):
    '''
    Helper class to log STDOUT and STDERR
    to both STDOUT and to a file
    '''
    def __init__(self, file_name):
        self.file = open(file_name, 'w')
        self.stdout = sys.stdout
        self.stderr = sys.stderr
        sys.stdout = self
        sys.stderr = self

    def close(self):
        if self.stdout and sys:
            sys.stdout = self.stdout
            self.stdout = None
        if self.stderr and sys:
            sys.stderr = self.stderr
            self.stderr = None
        if self.file:
            self.file.close()
            self.file = None

    def write(self, data):
        self.file.write(data.decode('ascii', 'ignore'))
        self.stdout.write(data)

    def flush(self):
        self.file.flush()
        try:
            self.stdout.flush()
        except:
            pass

    def __del__(self):
        if self:
            self.close()
