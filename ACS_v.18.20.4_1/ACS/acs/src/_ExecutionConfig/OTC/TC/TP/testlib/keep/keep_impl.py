"""
@summary: module for keep application
@since: 10/2/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""

from testlib.util.common import g_common_obj
import time
import os

class KeepImpl:
    """
    Keep functions.
    """
    keep_package = "com.google.android.keep"
    keep_activity = ".activities.BrowseActivity"

    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_new_note(self):
            """ UI button new note """
            return self.d(resourceId = "com.google.android.keep:id/new_note_button")

        @property
        def btn_title_text(self):
            """ UI button title text """
            return self.d(resourceId = "com.google.android.keep:id/add_item_text_view")

        @property
        def btn_add_picture(self):
            """ UI button add picture """
            return self.d(resourceId = "com.google.android.keep:id/menu_add_picture")

        @property
        def btn_take_photo(self):
            """ UI button take photo """
            return self.d(text = "Take photo")

        @property
        def btn_shutter(self):
            """ UI button shutter """
            return self.d(resourceId = "com.android.camera2:id/shutter_button")

        @property
        def btn_done(self):
            """ UI button Done """
            return self.d(resourceId = "com.android.camera2:id/done_button")

        @property
        def btn_menu_archive(self):
            """ UI button Done """
            return self.d(resourceId = "com.google.android.keep:id/menu_archive")

        @property
        def btn_menu_delete(self):
            """ UI button menu delete """
            return self.d(resourceId = "com.google.android.keep:id/menu_delete")

        @property
        def btn_home(self):
            """ UI button home """
            return self.d(resourceId = "android:id/home")

        @property
        def btn_archive(self):
            """ UI button Archive """
            return self.d(text = "Archive")

        @property
        def btn_text_description(self):
            """ UI button text description """
            return self.d(resourceId = "com.google.android.keep:id/description")

        @property
        def btn_ok_gotit(self):
            """ UI button text resource id """
            return self.d(resourceId = "com.android.camera2:id/ok_button")

        @property
        def btn_gotit(self):
            """ UI button text resource id"""
            return self.d(resourceId="com.google.android.keep:id/got_it_button")

        @property
        def btn_menu(self):
            """ UI button meun """
            return self.d(className="android.widget.ImageButton")

        @property
        def btn_option(self):
            """ UI button option """
            return self.d(description="More options")

        @property
        def btn_delete(self):
            """ UI button delete """
            return self.d(text="Delete note")

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = KeepImpl.Locator(self.d)

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    def launch_from_am(self):
        """
            Launch Keep from am
        """
        print "[INFO] Launch Keep from am"
        g_common_obj.launch_app_am(KeepImpl.keep_package, \
            KeepImpl.keep_activity)
        time.sleep(2)

    def stop_from_am(self):
        """
            Stop Keep from am
        """
        print "[INFO] Stop Keep from am"
        g_common_obj.stop_app_am(KeepImpl.keep_package)

    def quit_app(self):
        for i in range(3):
            self.d.press.back()

    def create_a_note(self):
        """
        This test used to test create a note function.
        The test case spec is following:
        1. Launch the "keep" and add a images.
        2. save file then delete archive notes.
        """

        self.launch_from_am()
        if self._locator.btn_gotit.exists:
            self._locator.btn_gotit.click.wait()
        self._locator.btn_new_note.click.wait()
        self._locator.btn_title_text.set_text("test")
        time.sleep(2)
        self._locator.btn_add_picture.click.wait()
        self._locator.btn_take_photo.click.wait()
        time.sleep(4)
        print "[INFO]: Take a photo"
        if self._locator.btn_ok_gotit.exists:
            self._locator.btn_ok_gotit.click.wait()
            time.sleep(2)
        print "[INFO]: Finished"
        self._locator.btn_shutter.click.wait()
        time.sleep(4)
        self._locator.btn_done.click.wait()
        time.sleep(4)
        self._locator.btn_menu_archive.click.wait()
        self._locator.btn_menu.click.wait()
        self._locator.btn_archive.click.wait()
        time.sleep(2)
        self._locator.btn_text_description.click.wait()
        self._locator.btn_option.click.wait()
        self._locator.btn_delete.click.wait()
        time.sleep(2)
        self.quit_app()