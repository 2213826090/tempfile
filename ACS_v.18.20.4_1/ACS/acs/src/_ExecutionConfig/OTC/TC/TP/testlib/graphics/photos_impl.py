# Copyright (C) 2015  Zhang,RongX Z <rongx.z.zhang@intel.com>
# Intel Corporation All Rights Reserved.
# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.
# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for PhotosImpl operation
@since: 08/17/2015
@author: Zhang,RongX Z
'''
import os
import time
from random import randint
from testlib.util.log import Logger
from testlib.util.common import g_common_obj
import testlib.graphics.decorator as decorator
from testlib.graphics.common import pkgmgr, busybox_obj, get_current_focus_window, windows_info, adb32
from testlib.graphics.common import get_resource_from_atifactory, file_sys, multi_display, dbsetting
from abc import ABCMeta
from testlib.util.repo import Artifactory
from testlib.util.config import TestConfig
from testlib.graphics.compare_pic_impl import ComparePicImpl


PHOTOS_APK_CLASSNAME = 'com.google.android.apps.photos'
PHOTOS_ACTIVITY_OLD = '.localmedia.ui.LocalFoldersActivity'
CONFIG_FILE = "tests.common.photos.conf"
PARENT_EFFECTS = ['Light', 'Color', 'Pop']
LIGHT_CHILD_EFFECTS = ['Exposure', 'Contrast', 'Whites', 'Highlights', 'Shadows', 'Blacks', 'Vignette']
COLOR_CHILD_EFFECTS = ['Saturation', 'Warmth', 'Tint', 'Skin tone', 'Deep blue']

LOG = Logger.getlogger(__name__)


class PhotoAPKInterface(object):
    __metaclass__ = ABCMeta


class PhotosImplDefault(PhotoAPKInterface):

    """
    Default Photos Test Implement Class
    """

    def __init__(self, cfg={}):
        self.d = g_common_obj.get_device()
        self.device = self.d
        busybox_obj.setup()

    def deploy_photo_content(self, tag='', name='', directory='photos_dir', local_file=''):
        '''
        deploy photo contents before run tests.
        :param tag: input config file tag name. such as: Pictures / Videos / PicturesFolder ..
        :param name: input head name for each content. such as: picture_001 ~ 005 / jpg_10resolution ..
        :param directory: input push to directory, normally using head name is fine: photos_dir / video_dir
        :param local_file: Set the full path if the previous param is True, else blank it.
        :return: None
        '''
        seperate_dir = ['photos_dir', 'video_dir']
        # ==================set push to directory==================
        if directory not in seperate_dir:
            push_path = directory
        else:
            push_path = TestConfig().read(CONFIG_FILE, 'Photos').get(directory)
        # ==================start push==================
        if local_file == '':
            arti_ = TestConfig().read(section='artifactory').get('location')
            arti = Artifactory(arti_)
            if not 'Folder' in tag:
                photo = TestConfig().read(CONFIG_FILE, tag).get(name)
                cmd = "adb push %s %s" % (arti.get(photo), push_path)
                os.system(cmd)
            else:
                getPhotos = TestConfig().read(CONFIG_FILE, tag).get(name)
                isolatePath = getPhotos.split(';')[0]
                isolateConts = getPhotos.split(';')[1]
                repoPath = isolatePath.split('\"')[1].replace('Path:','').strip(' ')
                contents = isolateConts.split('\"')[1].replace('Content:', '').strip(' ').split(',')
                for _ in contents:
                    cont = _.strip(' ')
                    contentPath = repoPath + '/' + cont
                    localPath = arti.get(contentPath)
                    cmd = "adb push %s %s" % (localPath, push_path)
                    os.system(cmd)
        else:
            cmd = "adb push %s %s" % (local_file, push_path)
            os.system(cmd)
        picture = file_sys.get_file_list(push_path)[0]
        file_sys.rescan_file(picture)

    def launch_photos_am(self):
        status = self.open_local_folder()
        time.sleep(3)
        if not status:
            g_common_obj.adb_cmd("am start -n com.google.android.apps.photos/.home.HomeActivity")
            for _ in range(3):
                if self.device(textContains='Google Photos uses face').exists and \
                        self.device(textMatches='[O|o][N|n]').exists:
                    LOG.debug("back up & sync display, try to init photos")
                    x = self.device.info['displayHeight'] / 2
                    y = self.device.info['displayWidth'] / 2
                    self.device().click(x, y)
                if self.device(textContains="Keep backup off").exists:
                    self.device(textMatches="Keep off|KEEP OFF").click()
                if self.device(textStartsWith="Never show").exists:
                    self.device(textStartsWith="Never show").click()
                    if self.device(textMatches="SKIP|Skip").exists:
                        self.device(textMatches="SKIP|Skip").click()
                    if self.device(textMatches="CANCEL|Cancel").exists:
                        self.device(textMatches="CANCEL|Cancel").click()
                if self.device(text="NOT NOW").exists:
                    self.device(text="NOT NOW").click.wait()
                if self.device(description="Photos, selected, tab, 2 of 3").exists:
                    LOG.debug("init completed")
                    break
            assert not self.d(textContains='Google Photos uses face').exists, \
                LOG.debug("init updated photos failed")
            if self.device(text="Device folders"):
                self.device(text="Device folders").click.wait()
            time.sleep(2)
            # It will be display Photos if not log in google account on bxt-p m
            if self.device(text="Photos"):
                self.device(text="Photos").click.wait()

    def stop_photos_am(self):
        """ Stop photos via adb am command
        """
        print "Stop photos by adb am"
        g_common_obj.stop_app_am("com.google.android.apps.photos")

    def uninstall_photos_package(self):
        print "Uninstall photos package."
        g_common_obj.adb_cmd("pm clear com.google.android.apps.photos")
        g_common_obj.adb_cmd("pm uninstall com.google.android.apps.photos")

    def open_a_picture(self, folder_name="Pictures"):
        assert self.device(text=folder_name).exists, "Launch photos app failed."
        if self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").exists:
            self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").click.wait()
        else:
            time.sleep(1)
            assert self.device(scrollable=True).scroll.vert.to(text=folder_name), \
                        "Folder Name: %s not found." % folder_name
            time.sleep(1)
            if self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").exists:
                self.device(text=folder_name,
                            resourceId="com.google.android.apps.photos:id/collection_title").click.wait()
        # Add AIA uiobjViewGroup
        if self.device(resourceId="com.google.android.apps.photos:id/title").exists:
            self.device(resourceId="com.google.android.apps.photos:id/title").click.wait()
        elif self.device(resourceId="com.google.android.apps.photos:id/list_photo_title_view").exists:
            self.device(resourceId="com.google.android.apps.photos:id/list_photo_title_view").click.wait()
        if self.device(descriptionStartsWith="Photo ").exists:
            self.device(descriptionStartsWith="Photo ").click()
        else:
            self.device(textContains="day").click.wait()
        time.sleep(2) # Sleep before uiautomator restart.

    def view_pictures_in_thumbnailsMode(self, folder_name="Pictures"):
        if self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").exists:
            self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").click.wait()
        else:
            self.device(scrollable=True).scroll.vert.to(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title")
            self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").click.wait()

    def open_photo_menu(self):
        """
        click photos menu
        """
        if self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow"):
            self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow").click.wait()
        if self.device(resourceId="com.google.android.apps.photos:id/photos_overflow_icon"):
            self.device(resourceId="com.google.android.apps.photos:id/photos_overflow_icon").click.wait()

    def pictures_slideshow(self):
        "after clicking photos menu,then click Slideshow"
        self.open_photo_menu()
        self.device(text="Slideshow").click.wait()

    def rm_delete_photos(self):
        cmd = "rm -rf /sdcard/DCIM/Camera/*;\
               rm -rf /sdcard/PicsArt/*;\
               rm -rf /sdcard/Skitch/*;\
               rm -rf /sdcard/Download/*;\
               rm -rf /sdcard/Pictures/*;\
               rm -rf /sdcard/Movies/*;\
               rm -rf /sdcard/*.mp4;\
               rm -rf /sdcard/*.3gp;\
               rm -rf /sdcard/*.png;\
               rm -rf /sdcard/Photo\ Grid/*;"
        clean_folders = [i.split('-rf')[-1].strip() for i in cmd.split(";") if i != '']
        clean_files = []
        for i in clean_folders:
            _list = file_sys.get_file_list(i)
            if _list != None: clean_files += _list
        cmd = "\"" + cmd + "\""
        g_common_obj.adb_cmd_capture_msg(cmd)
        if not clean_files is None:
            if len(clean_files) > 50:
                adb32._adb_reboot()
            else:
                for cf in clean_files:
                    file_sys.rescan_file(cf.replace(' ', '\ '))

    def refresh_sdcard(self):
        # find exclude Android directory
        getList = file_sys.get_file_list("/sdcard/ -path /sdcard/Android -prune -o -print")
        rescan_file_extensions = ['jpg', 'gif', 'bmp', 'webp', 'png', 'jpeg', 'wbmp', 'mp4', '3gp', 'mkv', 'mpeg']
        try:
            filter_files = [f for f in getList if f.split(".")[-1].lower() in rescan_file_extensions]
            if len(filter_files) > 50:
                adb32._adb_reboot()
            else:
                for f in filter_files:
                    file_sys.rescan_file(f)
        except:
            LOG.info("sdcard is clean")
        # cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file:///mnt/sdcard/'
        # g_common_obj.adb_cmd(cmd)

    def view_a_picture_fullscreen(self):
        "after opening a picture,then click the center to make it full screen"
        Width = self.device.info.get("displayWidth")
        Height = self.device.info.get("displayHeight")
        self.device.click(Width / 2, Height / 2)
        time.sleep(3)
        # Skip hints if first time get into full screen mode.
        self.device.click(Width / 2, Height)
        time.sleep(.5)
        self.device.swipe(Width, 0, Width, Height)
        if self.device(className="android.widget.RelativeLayout",
                           resourceId="com.android.systemui:id/header").exists:
            self.device.press.back()
        assert not self.device(description="More options").exists, 'check full screen fail'

    def delete_a_picture(self):
        if self.device(resourceId="com.google.android.apps.photos:id/delete_device_copy").exists:
            self.device(resourceId="com.google.android.apps.photos:id/delete_device_copy").click.wait()
        else:
            self.open_photo_menu()
            self.device(textContains="Delete").click.wait()
            time.sleep(2)
            if self.device(resourceId="com.google.android.apps.photos:id/delete_confirmation_button").exists:
                self.device(resourceId="com.google.android.apps.photos:id/delete_confirmation_button").click.wait()
        if self.device(resourceId="android:id/button1"):
            self.device(resourceId="android:id/button1").click.wait()
        self.refresh_sdcard()

    def view_picture_details(self):
        """
        After opening a picture,check its details
        """
        self.device(resourceId="com.google.android.apps.photos:id/details").click.wait()

    def edit_a_picture(self):
        "after opening a picture,then edit a picture"
        self.device(resourceId="com.google.android.apps.photos:id/edit").click.wait()

    def click_crop_tools(self):
        """
        after opening a picture, then click edit button and click crop button
        """
        if self.device(resourceId="com.google.android.apps.photos:id/edit").exists:
            self.edit_a_picture()
        self.device(resourceId="com.google.android.apps.photos:id/cpe_crop_tool").click.wait()

    def rotate_90_degrees(self):
        """
        after opening a picture ,click edit button,click crop button,then click rotate 90 degrees button.
        """
        self.device(resourceId="com.google.android.apps.photos:id/cpe_rotate_90").click.topleft()

    def rotate_right_45_degrees(self):
        """
        after opening a picture ,click edit button,click crop button,then rotate picture right 45 degrees by flip bar.
        """
        bounds = self.device(resourceId="com.google.android.apps.photos:id/cpe_straighten_slider").bounds
        left = bounds["left"]
        top = bounds["top"]
        right = bounds["right"]
        bottom = bounds["bottom"]
        bar_y = (top + bottom) / 2
        total_x = right - left
        print "cpe_straighten_slider is total length: total_x=%s" % (total_x)
        start_x = left
        print "start location: start_x=%s" % (start_x)
        target_x = start_x
        print "target location: target_x=%s" % (target_x)
        self.device(resourceId="com.google.android.apps.photos:id/cpe_straighten_slider")\
            .drag.to(target_x, bar_y)
        time.sleep(2)

    def crop_to_center(self, start_corner=""):
        ex = self.device.info.get("displayWidth") / 2
        ey = self.device.info.get("displayHeight") / 2
        if start_corner.lower() == "left_top":
            self.device(descriptionStartsWith="Left top corner").drag.to(ex, ey, steps=100)
        elif start_corner.lower() == "left_bottom":
            self.device(descriptionStartsWith="Left bottom corner").drag.to(ex, ey, steps=100)
        elif start_corner.lower() == "right_top":
            self.device(descriptionStartsWith="Right top corner").drag.to(ex, ey, steps=100)
        elif start_corner.lower() == "right_bottom":
            self.device(descriptionStartsWith="Right bottom corner").drag.to(ex, ey, steps=100)
        else:
            raise Exception("Wrong start corner value !")

    def save_picture_after_rotation(self):
        """
        after rotating a picture,save it.
        """
        if self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept"):
            self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").click.wait()
        time.sleep(1)
        if self.device(resourceId="com.google.android.apps.photos:id/cpe_save_button"):
            self.device(resourceId="com.google.android.apps.photos:id/cpe_save_button").click.wait()
        time.sleep(2)
        if self.device(text="SAVE").wait.exists(timeout=60000):
            self.device(text="SAVE").click.wait(timeout=5000)
        time.sleep(8)

    def add_adjustments_to_photos(self, adjustments_name, percent_size):
        """
        after clicking edit button,then change adjustments
        parameter adjustments_name is description of uiautomator wiget,just like
        "Apply enhancements automatically", "Apply vignette enhancements",and so on
        parameter percent_size means that how much you want to change to the picture
        The cpe strength bar from left to right iscorresponding 0-100
        for example: add vignette effect to 75%,you should make adjustments_name to
         Apply vignette enhancements and make percent_size to 75
        """
        self.edit_a_picture()
        time.sleep(3)
        if not self.device(text="Auto").exists:
            self.device(resourceId="com.google.android.apps.photos:id/cpe_adjustments_tool").click.wait()
        self.device(description=adjustments_name).click.wait()
        time.sleep(1)
        if self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").exists:
            bounds = self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_seek_bar").bounds
            left = bounds["left"]
            top = bounds["top"]
            right = bounds["right"]
            bottom = bounds["bottom"]
            bar_y = (top + bottom) / 2
            total_x = right - left
            print "cpe_strength_seek_bar is total length: total_x=%s" % (total_x)
            start_x = left
            print "start location: start_x=%s" % (start_x)
            target_x = total_x * percent_size / 100 + start_x
            print "target location: target_x=%s" % (target_x)
            self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_seek_bar")\
                .drag.to(target_x, bar_y)
            time.sleep(2)
            self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").click.wait()
            time.sleep(1)
        self.device(resourceId="com.google.android.apps.photos:id/cpe_save_button").click.wait()
        time.sleep(3)

    def apply_filter_to_photos(self, filter_name):
        """
        after clicking edit button,then click filter button
        """
        items = []
        self.edit_a_picture()
        time.sleep(1)
        self.device(resourceId="com.google.android.apps.photos:id/cpe_looks_tool").click.wait()

        if self.device(className="android.widget.ImageView", descriptionStartsWith="filter").wait.exists(timeout=3000):
            items = [item.contentDescription for item in self.device(descriptionStartsWith="filter")]
        assert len(items) > 0, "UI: Not found available filter items"

        print "[DEBUG] will choose photo filter: %s" % (filter_name)
        choosed = ""
        if filter_name in items:
            self.device(scrollable=True).scroll.horiz.to(descriptionContains=filter_name)
            self.device(descriptionContains=filter_name).click()
            choosed = filter_name
        else:
            rand_idx = randint(1, len(items))
            random_choose = items[rand_idx]
            self.device(scrollable=True).scroll.horiz.to(description=random_choose)
            print "[DEBUG] There is no %s, random choose %s" % (filter_name, random_choose)
            self.device(description=random_choose).click()
            choosed = random_choose
        print "[DEBUG] choosed photo filter: %s" % (choosed)

        time.sleep(1)
        if self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").exists:
            self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").click.wait()
        time.sleep(1)
        self.device(text="SAVE").click.wait()
        time.sleep(3)

    def set_picture_as_wallpaper(self):
        """
        after opening a picture and open picture menu,then set picture as Wallpaper
        """
        time.sleep(3)
        self.open_photo_menu()
        if self.device(text="Set as").exists:
            self.device(text="Set as").click.wait()
        elif self.device(text="Use as"):
            self.device(text="Use as").click.wait()
        self.device(text="Wallpaper").click.wait()
        time.sleep(3)
        self.device(textMatches="Set\ wallpaper|SET\ WALLPAPER").click.wait()
        time.sleep(2)
        if self.device(textContains="Home screen"):
            self.device(textContains="Home screen").click.wait()

    def WallpaperSetter(self, image_path):
        """
        Open an image with Photos app and then set it as wallpaper
        """
        g_common_obj.adb_cmd("am start -a android.intent.action.VIEW -d file://%s -t image/* \
        -n com.google.android.apps.photos/com.google.android.apps.photos.setwallpaper.SetWallpaperActivity" % (image_path))
        self.d(resourceId="com.google.android.apps.photos:id/cpe_save_button").click.wait()

    def ImageEditor(self, image_path):
        g_common_obj.adb_cmd("am start -a android.intent.action.VIEW -d file://%s -t image/* \
        -n com.google.android.apps.photos/com.google.android.apps.consumerphotoeditor.fragments.ConsumerPhotoEditorActivity" % (image_path))

    def set_picture_as_contact_photo(self, contact_name):
        """
        after opening a picture and open picture menu,then set picture as contact photo
        """
        self.open_photo_menu()
        if self.device(text="Set as").exists:
            self.device(text="Set as").click.wait()
        elif self.device(text="Use as").exists:
            self.device(text="Use as").click.wait()
        self.device(text="Contact photo").click.wait()
        time.sleep(2)
        if not self.device(text=contact_name).exists:
            self.device(scrollable=True).scroll.to(text=contact_name)
            time.sleep(2)
        self.device(text=contact_name).click.wait()
        time.sleep(2)
        if self.device(text="Photos").exists:
            self.device(text="Photos").click.wait()
        if self.device(text="Always").exists:
            self.device(text="Always").click.wait()
        time.sleep(2)
        if self.device(resourceId="com.google.android.apps.photos:id/cpe_save_button"):
            self.device(resourceId="com.google.android.apps.photos:id/cpe_save_button").click.wait()
        # The above comand can't work on bxt-p m, so add the below code
        time.sleep(2)
        if self.device(text="SAVE"):
            self.device(text="SAVE").click.wait()
        time.sleep(5)

    def open_local_folder(self):
        cmdstr = 'am start -S -n %s/%s' % (PHOTOS_APK_CLASSNAME, PHOTOS_ACTIVITY_OLD)
        status = g_common_obj.adb_cmd_capture_msg(cmdstr)
        return False if 'does not exist' in status or 'Stopping:' in status else True

    def relaunchPhotos(self):
        g_common_obj.adb_cmd_capture_msg("am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file://storage/sdcard1")
        time.sleep(5)
        g_common_obj.launch_app_am(
            "com.google.android.apps.photos", "com.google.android.apps.photos.localmedia.ui.LocalFoldersActivity")
        time.sleep(5)

    def delete_photos_in_a_folder(self, folder_name, num):
        """
        args:fold_name:the fold name to be deleted
            num:the pictures number to be deleted
        """
        if not self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").exists:
            try:
                self.device(scrollable=True).scroll.vert.to(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title")
            except:
                print "Scroll not found!"
        self.device(text=folder_name).click.wait()
        if self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow"):
            self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow").click.wait()
        if self.device(resourceId="com.google.android.apps.photos:id/photos_overflow_icon"):
            self.device(resourceId="com.google.android.apps.photos:id/photos_overflow_icon").click.wait()
        self.device(textContains="Select").click.wait()
        for i in range(1, num + 1):
            # for N
            if self.device(text="Today"):
                if self.device(text="Today").down(className="android.view.ViewGroup", index=i):
                    self.device(text="Today").down(className="android.view.ViewGroup", index=i).click.wait()
                if self.device(text="Today").down(className="android.view.View", index=i):
                    self.device(text="Today").down(className="android.view.View", index=i).click.wait()
            # for M
            else:
                self.device(className="android.view.View", index=i - 1).click.wait()
        time.sleep(3)
        #assert self.device(resourceId="com.google.android.apps.photos:id/action_bar_title", text=num), "choose picture failed"
        if self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow"):
            self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow").click.wait()
        if self.device(resourceId="com.google.android.apps.photos:id/action_mode_close_button"):
            self.device(resourceId="com.google.android.apps.photos:id/action_mode_close_button").click.wait()
        if self.device(textContains="Delete device copy").exists:
            self.device(textContains="Delete device copy").click.wait()
        elif self.device(textContains="Delete").exists:
            self.device(textContains="Delete").click.wait()
        if self.device(text="Delete"):
            self.device(text="Delete").click.wait()
        elif self.device(text="DELETE").exists:
            self.device(text="DELETE").click.wait()

    def check_if_folder_deleted(self, fold_name):
        assert not self.device(text=fold_name, resourceId="com.google.android.apps.photos:id/collection_title").exists, "delete folder failed"

    def play_video(self, folder_name="Movies"):
        if not self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").exists:
            self.device().scroll.vert.to(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title")
        if self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title"):
            self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").click.wait()
        if self.device(className="android.view.View"):
            self.device(className="android.view.View").click.wait()
        # add below code for bxt-p m
        time.sleep(2)
        if self.device(resourceId="com.google.android.apps.photos:id/list_photo_tile_view"):
            self.device(resourceId="com.google.android.apps.photos:id/list_photo_tile_view").click.wait()
        if self.device(className="android.view.ViewGroup").exists:
            self.device(className="android.view.ViewGroup").click.wait()

    def play_video_command(self, video_path, duration=.5):
        """
        Play video via command line
        """
        cmd = 'getprop | grep ro.build.version.sdk'
        sdk_string = g_common_obj.adb_cmd_capture_msg(cmd)
        if '23' in sdk_string:
            g_common_obj.adb_cmd("am start -a android.intent.action.VIEW -d file://%s -t video/*" % (video_path))
            if self.device(text="Open with").exists:
                self.device(text="Photos").click.wait()
            if self.device(resourceId="android:id/button_once").exists:
                self.device(resourceId="android:id/button_once").click.wait()
            if self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").exists:
               self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").click()
        else:
            g_common_obj.adb_cmd("am start -a android.intent.action.VIEW -d file://%s -t video/* \
        -n com.google.android.apps.photos/com.google.android.apps.photos.pager.HostPhotoPagerActivity" % (video_path))
            if self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").exists:
               self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").click()
        windows_info.disable_fullscreen_hint()
        assert PHOTOS_APK_CLASSNAME in get_current_focus_window(), "Video not started."
        time.sleep(duration)

    def open_image_command(self, image_path):
        """
        Open an image file via command line
        """
        cmd = 'getprop | grep ro.build.version.sdk'
        sdk_string = g_common_obj.adb_cmd_capture_msg(cmd)
        if '23' in sdk_string:
            g_common_obj.adb_cmd("am start -a android.intent.action.VIEW -d file://%s -t image/*" % (image_path))
            if self.device(text="Open with").exists:
                self.device(text="Photos").click()
            if self.device(resourceId="android:id/button_always"):
                self.device(resourceId="android:id/button_always").click.wait()
            if self.device(resourceId="android:id/button_once").exists:
                self.device(resourceId="android:id/button_once").click.wait()
        else:
            g_common_obj.adb_cmd("am start -a android.intent.action.VIEW -d file://%s -t image/* \
        -n com.google.android.apps.photos/com.google.android.apps.photos.pager.HostPhotoPagerActivity" % (image_path))

    def pause_play_video(self):
        for _ in range(0, 3):
            x = self.device.info["displayWidth"]
            y = self.device.info["displayHeight"]
            self.device.click(x / 2, y / 2)
            time.sleep(2)
            self.device.click(x / 2, y / 2)
            if self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").exists:
                break

    def resume_play_video(self):
        """
        after pausing play video,then resume video playing.
        """
        for _ in range(0, 3):
            if self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").exists:
                self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").click.wait()
            time.sleep(1)
            if not self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").exists:
                break
        assert not self.device(resourceId="com.google.android.apps.photos:id/photos_videoplayer_play_button").exists, "resume playing video failed"

    def slide_video_processbar_forward(self):
        self.pause_play_video()
        time.sleep(1)
        self.device(resourceId="com.google.android.apps.photos:id/video_player_progress").\
            drag.to(resourceId="com.google.android.apps.photos:id/video_total_time")
        time.sleep(2)
        self.resume_play_video()

    def slide_video_processbar_backward(self):
        self.pause_play_video()
        time.sleep(1)
        self.device(resourceId="com.google.android.apps.photos:id/video_player_progress").\
            drag.to(resourceId="com.google.android.apps.photos:id/video_current_time")
        time.sleep(2)
        self.resume_play_video()

    def change_display_effect(self, effType='Light', effVol=0):
        '''
        Change enhance effect
        :param effType: Choose type which you want to change.
        :param effVol: Input change volume, should be 0 ~ 100.
        :return: None
        '''
        if self.device(resourceId="com.google.android.apps.photos:id/edit").exists:
            self.device(resourceId="com.google.android.apps.photos:id/edit").click.wait()
            time.sleep(2)
        if self.device(descriptionContains="Enhance ").exists:
            self.device(descriptionContains="Enhance ").click.wait()
            time.sleep(.5)
        Light_expand_button = self.device(resourceIdMatches=".*expand_button", descriptionContains="Light")
        Color_expand_button = self.device(resourceIdMatches=".*expand_button", descriptionContains="Color")
        # -------------init screen-------------
        if not self.device(text='Pop').exists:
            if effType in LIGHT_CHILD_EFFECTS:
                Light_expand_button.click()
            else:
                Color_expand_button.click()
        # -------------done-------------
        if effType in LIGHT_CHILD_EFFECTS:
            Light_expand_button.click()
            self.device(resourceId="com.google.android.apps.photos:id/cpe_adjustments_subslider_scrollview")\
                .scroll.to(text=effType)
        elif effType in COLOR_CHILD_EFFECTS:
            Color_expand_button.click()
            self.device(resourceId="com.google.android.apps.photos:id/cpe_adjustments_subslider_scrollview")\
                .scroll.to(text=effType)
        time.sleep(1)
        effect_seekbar_locator = self.device(descriptionContains=effType, className="android.widget.SeekBar")
        bottom_ = effect_seekbar_locator.info["bounds"]['bottom']
        top_ = effect_seekbar_locator.info["bounds"]['top']
        left_ = effect_seekbar_locator.info["bounds"]['left']
        right_ = effect_seekbar_locator.info["bounds"]['right']
        y = (bottom_ - top_) / 2 + top_
        if effVol == 0:
            x = left_
        elif effVol == 100:
            x = right_
        else:
            x = effVol * (left_ + right_) / 100
        print "Set effect %s to " % effType, effVol,"%"
        effect_seekbar_locator.drag.to(x, y)
        time.sleep(7)
        # -------------init again-------------
        if not self.device(text='Pop').exists:
            if effType in LIGHT_CHILD_EFFECTS:
                Light_expand_button.click()
            else:
                Color_expand_button.click()

    def save_changes(self):
        self.device(text="SAVE").click.wait(timeout=5000)
        time.sleep(8)
        assert self.device(resourceId="com.google.android.apps.photos:id/edit"), "Fail to save pic!"

    def undo_edit(self):
        if self.device(resourceIdMatches=".*actionbar_overflow").exists:
            self.device(resourceIdMatches=".*actionbar_overflow").click.wait()
        else:
            self.device(descriptionContains="More ").click.wait()
        self.device(textContains='Undo').click.wait()
        assert not self.device(text="SAVE"), "Undo operation failed!"

    def check_saved_pic_result(self, picPath="/sdcard/Pictures"):
        '''
        pull pictures to local path and then use rms value to compare those pic.
        :param picPath: Set pic folder where pics are stored.
        :return: rms value; (> 0 means taking effect.)
        '''
        # Format picture path, Fix duplicate folder issue on adb 36.
        if picPath.endswith('/'): picPath = picPath[:-1]
        _save_pic_to = g_common_obj.globalcontext.user_log_dir + picPath
        if not os.path.exists('/'.join(_save_pic_to.split('/')[:-1])):
            os.makedirs('/'.join(_save_pic_to.split('/')[:-1]))
        else:
            os.system("rm -rf %s/*" % ('/'.join(_save_pic_to.split('/')[:-1])))
        # ------pull pic to local cache and get names------
        g_common_obj.pull_file(_save_pic_to, picPath)
        picList = [g_common_obj.globalcontext.user_log_dir + i for i in file_sys.get_file_list(picPath)]
        # start compare
        LOG.info("----------Local Pic1: %s" % picList[0])
        LOG.info("----------Local Pic2: %s" % picList[1])
        rmsValue = ComparePicImpl().compare_pic(picList[0], picList[1])
        # clean local pics
        print "Clean folder: %s" % '/'.join(_save_pic_to.split('/')[:-1])
        os.system("rm -rf %s" % ('/'.join(_save_pic_to.split('/')[:-1])))
        return rmsValue

    def check_saved_pic_result_by_screenshot(self, picPath):
        '''
        Image tool does not support images such as: wbmp, webp, bmp,
        So we have to check the effect by screenshots.
        '''
        try:
            pictures = file_sys.get_file_list(picPath)
            pic1, pic2 = pictures[0], pictures[1]
        except:
            raise Exception("Fail to get pictures in folder: %s" % picPath)

        self.stop_photos_am()
        self.open_image_command(pic1)
        screenshot1 = multi_display.get_screenshot_forMultiDisplay(0)
        self.stop_photos_am()
        self.open_image_command(pic2)
        screenshot2 = multi_display.get_screenshot_forMultiDisplay(0)
        rmsValue = ComparePicImpl().compare_pic(screenshot1, screenshot2)
        print ">>>>>Result is: %f" % rmsValue
        return rmsValue

    def pic_zoom_in(self, value=20, repeat=1):
        # zoomVolume > 20 is recommended.
        center_x = self.device.info.get("displayWidth") / 2
        center_y = self.device.info.get("displayHeight") / 2
        print "-" * 10, "Start zoom in", "-" * 10
        for r in range(repeat):
            time.sleep(1)
            self.device().gesture((center_x, center_y+10), (center_x, center_y-10)).to\
                                ((center_x, (center_y - value)),(center_x, (center_y + value)),
                                steps=20)

    def pic_zoom_out(self, value=20, repeat=1):
        # zoomVolume > 20 is recommended.
        center_x = self.device.info.get("displayWidth") / 2
        center_y = self.device.info.get("displayHeight") / 2
        print "-" * 10, "Start zoom out", "-" * 10
        for r in range(repeat):
            time.sleep(1)
            self.device().gesture((center_x, (center_y - value)),(center_x, (center_y + value))).to\
                                ((center_x, center_y-10), (center_x, center_y+10),
                                steps=20)

    # new method for check number of pics
    def check_pic_number(self, path="/sdcard/Pictures"):
        cmd = "ls %s | grep -cE '.*'" % path
        num = int(g_common_obj.adb_cmd_capture_msg(cmd))
        print '>' * 6, 'Current pic num: %d' % num
        return num

    def only_pull_saved_pic(self, path="/sdcard/Pictures"):
        '''
        Use this method after edit -> saved pic.
        :param path: Set pic folder where pic is stored.
        :return: local path for that pic.
        '''
        if not path.endswith('/'): path = path + '/'
        _save_pic_to = g_common_obj.globalcontext.user_log_dir + path
        if not os.path.exists(_save_pic_to):
            os.makedirs(_save_pic_to)
        else:
            os.system("rm -rf %s*" % _save_pic_to)
        pics = file_sys.get_file_list(path)
        assert len(pics) > 1, "Failt to save pics."
        saved_pic_name = [i for i in pics if int(i.split('.')[0][-1]) == len(pics)][0].split('/')[-1]
        remotePath = path + saved_pic_name
        localPath = _save_pic_to + saved_pic_name
        print "Pull %s to %s" % (remotePath, localPath)
        g_common_obj.pull_file(localPath, remotePath)
        return localPath


PhotosImpl = PhotosImplDefault


class PhotosImpl_1_21(PhotosImplDefault):

    @decorator.restart_uiautomator
    def apply_filter_to_photos(self, filter_name):
        """
        after clicking edit button,then click filter button
        """
        items = []
        self.edit_a_picture()
        time.sleep(1)
        self.device(resourceId="com.google.android.apps.photos:id/cpe_looks_tool").click.wait()

        if self.device(className="android.view.View", descriptionStartsWith="filter").wait.exists(timeout=3000):
            items = [item.contentDescription for item in self.device(descriptionStartsWith="filter")]
        assert len(items) > 0, "UI: Not found available filter items"

        print "[DEBUG] will choose photo filter: %s" % (filter_name)
        choosed = ""
        if filter_name in items:
            if self.device(scrollable=True):
                self.device(scrollable=True).scroll.horiz.to(descriptionContains=filter_name)
                self.device(descriptionContains=filter_name).click.wait()
            else:
                self.device(descriptionContains=filter_name).click.wait()
            choosed = filter_name
        else:
            rand_idx = randint(1, len(items))
            random_choose = items[rand_idx]
            self.device(scrollable=True).scroll.horiz.to(description=random_choose)
            print "[DEBUG] There is no %s, random choose %s" % (filter_name, random_choose)
            self.device(description=random_choose).click()
            choosed = random_choose
        print "[DEBUG] choosed photo filter: %s" % (choosed)

        time.sleep(1)
        if self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").exists:
            self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").click.wait()
        time.sleep(1)
        self.device(text="SAVE").click.wait()
        time.sleep(3)


class PhotosImpl_2_5(PhotosImplDefault):

    @decorator.restart_uiautomator
    def apply_filter_to_photos(self, filter_name):
        """
        after clicking edit button,then click filter button
        """
        items = []
        self.edit_a_picture()
        time.sleep(1)
        self.device(resourceId="com.google.android.apps.photos:id/cpe_looks_tool").click.wait()

        if self.device(clickable=True, descriptionStartsWith="filter").wait.exists(timeout=3000):
            items = [item.contentDescription for item in self.device(descriptionStartsWith="filter")]
        assert len(items) > 0, "UI: Not found available filter items"

        print "[DEBUG] will choose photo filter: %s" % (filter_name)
        choosed = ""
        if filter_name in items:
            self.device(scrollable=True).scroll.horiz.to(descriptionContains=filter_name)
            self.device(descriptionContains=filter_name).click()
            choosed = filter_name
        else:
            rand_idx = randint(1, len(items) - 1)
            random_choose = items[rand_idx]
            self.device().scroll.horiz.to(description=random_choose)
            print "[DEBUG] There is no %s, random choose %s" % (filter_name, random_choose)
            self.device(description=random_choose).click()
            choosed = random_choose
        print "[DEBUG] choosed photo filter: %s" % (choosed)

        time.sleep(1)
        if self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").exists:
            self.device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").click.wait()
        
        time.sleep(3)
        if self.device(text="SAVE").exists:
            self.device(text="SAVE").click.wait()
        time.sleep(3)

    def add_adjustments_to_photos(self, adjustments_name, percent_size):
        self.apply_filter_to_photos(adjustments_name)
        self.edit_a_picture()
        time.sleep(3)
        self.device(descriptionStartsWith="Crop ").click.wait()

        bounds = self.device(resourceId="com.google.android.apps.photos:id/cpe_straighten_slider").bounds
        left = bounds["left"]
        top = bounds["top"]
        right = bounds["right"]
        bottom = bounds["bottom"]
        bar_y = (top + bottom) / 2
        total_x = right - left
        print "cpe_strength_seek_bar is total length: total_x=%s" % (total_x)
        start_x = left
        print "start location: start_x=%s" % (start_x)
        target_x = total_x * percent_size / 100 + start_x
        print "target location: target_x=%s" % (target_x)
        self.device(resourceId="com.google.android.apps.photos:id/cpe_straighten_slider").drag.to(target_x, bar_y)
        time.sleep(2)
        self.device(textMatches='DONE|Done|done').click.wait(timeout=3000)
        time.sleep(1)
        self.device(text="SAVE").wait.exists(timeout=5000)
        self.device(text="SAVE").click.wait(timeout=5000)
        time.sleep(3)


class PhotosImpl_2_13(PhotosImpl_2_5):

    def open_a_picture(self, folder_name="Pictures"):
        self.device(resourceId="com.google.android.apps.photos:id/collection_title").wait.exists(timeout=10000)
        if self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").exists:
            self.device(text=folder_name, resourceId="com.google.android.apps.photos:id/collection_title").click.wait()

        self.device(resourceId="com.google.android.apps.photos:id/collection_title").wait.exists(timeout=10000)
        self.device(clickable=True, descriptionStartsWith="Photo taken on").wait.exists(timeout=3000)
        self.device(clickable=True, descriptionStartsWith="Photo taken on", instance=0).click.wait()

    def delete_photos_in_a_folder(self, folder_name, num):

        self.device().scroll.vert.to(text=folder_name)
        self.device(text=folder_name).click.wait()
        self.device(resourceId="com.google.android.apps.photos:id/photos_overflow_icon").click.wait()
        self.device(textContains="Select").click.wait()

        for i in range(1, num + 1):
            self.device(clickable=True, descriptionStartsWith="Photo taken on", instance=i - 1).click.wait()
        time.sleep(3)

        self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow").click.wait()
        for _ in range(3):
            if self.device(textContains="Delete").exists:
                self.device(textContains="Delete").click.wait()
            if self.device(textMatches="DELETE|Delete").exists:
                self.device(textMatches="DELETE|Delete").click.wait()
            if self.device(textContains="Delete",
                           resourceId="com.google.android.apps.photos:id/delete_confirmation_button").exists:
                self.device(textContains="Delete",
                            resourceId="com.google.android.apps.photos:id/delete_confirmation_button").click.wait()

    def pause_play_video(self):
        x = self.device.info["displayWidth"]
        y = self.device.info["displayHeight"]
        start_time = time.time()
        while time.time() - start_time < 120:
            time.sleep(1)
            self.device.click(x / 2, y / 2)
            time.sleep(1)
            if self.device(descriptionContains="Pause").exists:
                self.device(descriptionContains="Pause").click()
                return

implement = None


def get_photo_implement():
    global implement

    if implement:
        return implement

    apk_path = get_resource_from_atifactory(CONFIG_FILE, 'Photos', 'apk')
    pkgmgr.apk_install(apk_path)
    pkgmgr.hide_package("com.android.car.hvac")
    if dbsetting.get_skip_first_use_hints() in ['null', 0]:
        dbsetting.set_skip_first_use_hints(True)

    implement = PhotosImpl_2_13()

    LOG.debug('Load %s' % implement.__class__)
    return implement
