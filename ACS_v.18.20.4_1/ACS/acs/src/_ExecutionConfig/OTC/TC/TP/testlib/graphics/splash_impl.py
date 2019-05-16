# -*- coding: utf-8 -*-
from testlib.graphics.common import FileSystem, Logcat, AdbExtension


StaticSplashFolder_M = "/splash/"
StaticSplashFolder = "/vendor/splash/"
AnimationFolder = "/vendor/splash/"
StaticSplashFileName = "splash.png"


class SplashImpl(object):

    def __init__(self):
        self.fs = FileSystem()
        self.log = Logcat()
        self.adbExt = AdbExtension()

    def check_static_splashfile(self):
        filelist = self.fs.get_file_list(folder=StaticSplashFolder)
        if StaticSplashFileName not in str(filelist):
            filelist = self.fs.get_file_list(folder=StaticSplashFolder_M)
        assert len(filelist) > 0, "Did not find static splash file."
        assert StaticSplashFileName in str(filelist), "Got wrong splash file name."

    def check_animation_splashfile(self):
        filelist = self.fs.get_file_list(folder=AnimationFolder)
        assert len(filelist) > 0, "Did not find animation splash file."

    def check_earlyEvs_init(self):
        self.adbExt._adb_reboot()
        assert self.log.check_dmesg_info(keyword="\'earlyEvs|rvc\'", assertword='boot completed')\
               or self.log.check_dmesg_info(keyword="\'earlyEvs|rvc\'", assertword='exited with status 0'),\
            "earlyEvs(rvc) initiation failed."

    def get_bootanimation_time(self):
        timestamp = self.log.get_timestamp_dmesg(keyword="\'earlyEvs|rvc\'")
        boottime = int(float(timestamp[-1]) - float(timestamp[0]))
        print "Boot animation time is: < %d > sec" % boottime
        return boottime

    def earlyaudio_check(self):
        self.adbExt._adb_reboot()
        timestamp = self.log.get_timestamp_dmesg(keyword="earlyaudioalsa")
        if timestamp == []:
            return 0
        if float(timestamp[-1]) > 7:
            return self.log.check_dmesg_info(keyword="earlyaudioalsa", assertword="exited with status 0")
        else:
            return 0
