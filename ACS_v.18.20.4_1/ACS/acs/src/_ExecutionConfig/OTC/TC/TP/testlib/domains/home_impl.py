#!/usr/bin/python
"""
@summary: The class for home Implements
"""
#-*- encoding: utf-8 -*-

import time
from nose.tools import assert_equals
from testlib.util.common import g_common_obj

class HomeImpl:
    '''
    Implements Home UI actions.
    '''
    def __init__ (self, cfg):
        self.d = g_common_obj.get_device()
        self.cfg = cfg

    def launch_app(self, appname, packagename):
        """
        @summary: launch app
        """
        g_common_obj.launch_app_from_home_sc(appname)
        time.sleep(2)
        assert_equals(self.d(packageName=packagename).exists, True)

    def launch_app_from_recent(self):
        """
        @summary: launch app from recent
        """
        self.d.press.recent()
        self.d(\
            resourceId = "com.android.systemui:id/task_view_thumbnail").click()
        assert_equals(self.d(\
            resourceId = "com.android.systemui:id/task_view_thumbnail").\
            exists, False)
        assert_equals(self.d(\
            packageName = "com.google.android.googlequicksearchbox").\
        exists, False)

    def back_home(self):
        '''
        Exit recent app.
        '''
        print "[Info] ---Exit recent app."
        self.d.press.home()
        assert_equals(self.d(\
            packageName = "com.google.android.googlequicksearchbox").\
        exists, True)

    def launch_wallpapers(self):
        '''
        lauch wallpapers from home page
        '''
        self.d.press.home()
        self.d(\
            resourceId="com.google.android.googlequicksearchbox:id/active")\
        .long_click()
        self.d(text = "Wallpapers").click()

    def set_wallpaper_randomly(self):
        '''
        Set wallpaper randomly.
        '''
        from random import randrange
        i = randrange(1, 5, 1)
        print "[Info] ---Set wallpaper index %d." % i
        self.d(resourceId = "com.google.android.googlequicksearchbox:id/wallpaper_image", \
        instance = i).click.wait()
        self.d(text = "Set wallpaper").click.wait()
        assert_equals(self.d(text = "Set wallpaper").exists, False)
        self.d.press.back()
        self.d.press.back()