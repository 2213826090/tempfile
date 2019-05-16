# coding: UTF-8
import os
import time
from nose.tools import assert_equals
from testlib.util.common import g_common_obj

class SoundSearchImpl:
    """
    Implements Sound Search UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def add_to_home_screen(self):
        '''
        Add Sound Search to home screen.
        '''
        print "[Info] ---Add Sound Search to home screen."
        self.d.press.home()
        if self.d(resourceId="com.android.vending:id/widget_content").exists:
            self.d(resourceId="com.android.vending:id/widget_content").drag.to(400, 100)
        if self.d(text="What's this song?").exists:
            return
        self.d(resourceId="com.google.android.googlequicksearchbox:id/active").long_click()
        self.d(text="Widgets").click.wait()
        for i in range(10):
            if self.d(text="Sound Search").exists:
                break
            self.d().swipe.left()
        assert self.d(text="Sound Search").exists
        self.d(text="Sound Search").long_click()
        time.sleep(1)
        assert self.d(text="What's this song?").exists

    def start_search(self):
        '''
        Start Sound Search.
        '''
        print "[Info] ---Start Sound Search."
        self.d(text="What's this song?").click.wait()
        assert self.d(description="Listening, click to stop").exists

    def stop_search(self):
        '''
        Stop Sound Search.
        '''
        print "[Info] ---Stop Sound Search."
        if self.d(description="Listening, click to stop").exists:
            if self.d(text="Oops, network failed. Retry?").exists:
                self.d(description="Close").click.wait()
            else:
                self.d(description="Listening, click to stop").click.wait()
        assert self.d(text="What's this song?").exists

    def remove_from_home_screen(self):
        '''
        Remove Sound Search from home screen.
        '''
        print "[Info] ---Remove Sound Search from home screen."
        if self.d(text="What's this song?").exists:
            self.d(text="What's this song?").drag.to(400, 100)
        assert not self.d(text="What's this song?").exists
