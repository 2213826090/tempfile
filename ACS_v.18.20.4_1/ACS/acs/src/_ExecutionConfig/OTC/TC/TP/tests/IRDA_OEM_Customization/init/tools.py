#!/usr/bin/env python
#-*- encoding: utf-8 -*-

import os
import ConfigParser


class ConfigHandle():
    def __init__(self):
        self.__CUR_FILE__ = os.path.realpath(__file__)
        self.__BASE_PATH__ = os.path.split(self.__CUR_FILE__)[0]
        self.__CONF_FILE__ = self.__BASE_PATH__ + os.sep + "configuration.conf"

    def read_conf(self, title, key):
        cf = ConfigParser.ConfigParser()
        cf.read(self.__CONF_FILE__)
        result = cf.get(title, key)
        return result

    def read_ftp(self):
        cf = ConfigParser.ConfigParser()
        cf.read(self.__CONF_FILE__)
        value = cf.items("andftp")
        return dict(value)

    def read_wifi(self):
        cf = ConfigParser.ConfigParser()
        cf.read(self.__CONF_FILE__)
        value = cf.items("wifi")
        return dict(value)

    def read_content_url(self):
        url = self.read_conf("content_url", "url")
        return url
    def read_host_file_path(self):
        host_file_path=self.read_conf("content_url","host_file_path")
        return host_file_path

class DeviceHandle:
    def __init__(self):
        pass

    def getDevice(self, message):
        return message.find("\tdevice\n") != -1

    def getAll(self, message):
        return message.find("\tdevice\n") != -1 or message.find("\tunauthorized\n") != -1

    def getDevices(self):
        msgs = os.popen("adb devices").readlines()
        devices = filter(self.getDevice, msgs)
        result = map(lambda x: x.split()[0], devices)
        return result

    def getAllDevices(self):
        msgs = os.popen("adb devices").readlines()
        devices = filter(self.getAll, msgs)
        result = map(lambda x: x.split()[0], devices)
        return result

if __name__ == "__main__":
    print DeviceHandle().getAllDevices()
