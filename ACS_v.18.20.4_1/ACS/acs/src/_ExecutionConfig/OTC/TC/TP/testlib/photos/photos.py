# coding: UTF-8
import os
import sys
import time
import datetime
from random import randrange
import ConfigParser
import re
from math import pow
from testlib.common.common import g_common_obj2
from testlib.common.base import UiWindows
BASE_PATH = os.path.dirname(__file__)
from testlib.common.impl import ImplCommon

class Photos(ImplCommon):
    """
    Multi-media functions.
    """

    package = "com.google.android.apps.photos"
    activity = ".home.HomeActivity"

    @staticmethod
    def startApp():
        g_common_obj2.launch_app_from_home_sc("Photos")
        time.sleep(10)
        d = g_common_obj2.get_device()
        time.sleep(3)
        if d(text="ALL").exists and not d(text="No thanks").exists:
            g_common_obj2.stop_app_am(Photos.package)
            return
        for _ in range(10):
            if d(text="Later").exists:
                d(text="Later").click()
            if d(text="ALL").exists and not d(text="No thanks").exists:
            #if not self.d(resourceId="android:id/progress").exists:
                break
            if d(text="Cancel").exists:
                d(text="Cancel").click.wait()
            if d(text="No thanks").exists:
                d(text="No thanks").click.wait()
            if d(text="All").exists:
                d(text="ALL").click.wait()
            time.sleep(2)
        d.press.back()
        d(description="Photos").click.wait()
        time.sleep(2)
        if d(text="Later").exists:
            d(text="Later").click()
        if d(text="No thanks").exists:
            d(text="No thanks").click.wait()
        time.sleep(4)
        g_common_obj2.back_home()

    def launchPhotos(self):
        g_common_obj2.launch_app_am("com.google.android.apps.plus", \
                                   "com.google.android.apps.photos.phone.PhotosLauncherActivity")
        time.sleep(2)
        if self.d(text="Later").exists:
            self.d(text="Later").click()
        if self.d(resourceId="android:id/button1").exists:
            self.d(resourceId="android:id/button1").click()
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click()

    def homeMenu(self):
        return self.d(className="android.widget.ImageButton", description="Open navigation drawer")

    def openPhotosFolders(self):
        self.homeMenu().click()
        self.d(text="On device").click()

    def getFolder(self, folderName):
        if self.d(text = folderName).exists:
            return self.d(text = folderName)
        else :
            self.d(scrollable=True).scroll.to(text = folderName)
            return self.d(text = folderName)

    def openFolder(self, folderName):
        self.getFolder(folderName).click()
        time.sleep(3)

    def getLastChild(self, obj):
        i=0
        while True:
            if obj.child(index=i).exists:
                i = i+1
        return obj.child(index=i-1)

    def openCameraFolder(self):
        self.launchPhotos()
        self.openPhotosFolders()
        self.getFolder("Camera").click.wait()

    def _videoPlayObj(self):
        return self.d(resourceId="com.google.android.apps.plus:id/media_player_fragment_container")


    def isVideo(self):
        self._videoPlayObj().click()
        self.d(resourceId="com.google.android.apps.plus:id/videoplayer").click()
        ret = self.d(resourceId="android:id/pause").exists
        self.d.press.back()
        return ret

    def isPlaying(self):
        if not self.d(resourceId="android:id/time").exists:
            self.d(resourceId="com.google.android.apps.plus:id/videoplayer").click()
        return self.d(resourceId="android:id/pause").exists

    #reTimePattern=re.compile("\d+", flags)
    def __getVideoTime(self, **kwargs):
        if not self.d(resourceId="android:id/time").exists:
            self.d(resourceId="com.google.android.apps.plus:id/videoplayer").click()
        t = self.d(**kwargs).text
        tl = t.split(":")
        retTime = 0
        index=0
        for each in tl[::-1]:
            retTime = retTime + int(each)*pow(60,index)
            index += 1
        return retTime

    def getCurrentTime(self):
        return self.__getVideoTime(resourceId="android:id/time_current")

    def getVideoLenTime(self):
        return self.__getVideoTime(resourceId="android:id/time")

    def videoPlayback(self, x=1):
        # hardcoding the coordinates for the first item in the grid
        # proved to be easier to maintain
        if "gmin" in g_common_obj2.getprop("ro.board.platform") and\
                "ecs_e7" in g_common_obj2.getprop("ro.product.board"):
            return self.d.click(50, 300)
        return self.elementOpen(self.isVideo, x)

    def videoDelete(self):
        self.openCameraFolder()
        self.d(description="More options").click.wait()
        self.d(textContains="Delete all").click.wait()
        self.d(text="Delete").click.wait()
        time.sleep(4)
        self.homeMenu().wait.exists(timeout=10000)
        self.homeMenu().click()
        self.d(text="Trash").click()
        self.d(description="More options").click()
        self.d(text="Empty trash").click()
        self.d(text="Empty trash", resourceId="android:id/button1").wait.exists(timeout=10000)
        self.d(text="Empty trash", resourceId="android:id/button1").click()

    def isPic(self):
        return self.d(resourceId="com.google.android.apps.plus:id/edit").exists

    def elementOpen(self, testElementFunc, x=1):
        self.openCameraFolder()
        time.sleep(1)
        indexStart=0
        for i in range(indexStart,x+indexStart):
            if not self.d(resourceId="com.google.android.apps.plus:id/grid").child(index=i).exists:
                break
            self.d(resourceId="com.google.android.apps.plus:id/grid").child(index=i).click()
            assert testElementFunc(), "failed"
            self.d.press.back()
        else:
            return
        uiChildlast = UiWindows(self.d(resourceId="com.google.android.apps.plus:id/grid").child(index=i-1).info)
        canRecogizeEnd = False
        if uiChildlast.getBottom()- self.d.info[u'displayHeight'] > uiChildlast.getHeight()/2:
            canRecogizeEnd = True
        print "can recogize end:", canRecogizeEnd
        uiGrid = UiWindows(self.d(resourceId="com.google.android.apps.plus:id/grid").info[u"bounds"])
        uiChild1 = UiWindows(self.d(resourceId="com.google.android.apps.plus:id/grid").child(index=indexStart).info[u"bounds"])
        uiChild2 = UiWindows(self.d(resourceId="com.google.android.apps.plus:id/grid").child(index=indexStart).down().info[u"bounds"])
        print uiChild1
        print uiChild1.getMidPoint()
        print uiChild2
        print uiChild2.getMidPoint()
        sx,sy=uiGrid.getMidPoint()
        dy = uiChild1.getMidPoint()[1]
        dy = uiChild2.getMidPoint()[1]-dy
        print sx,sy, sx, sy-dy, dy, uiChild1.getHeight()
        lineNum = 1
        element = self.d(resourceId="com.google.android.apps.plus:id/grid").child(index=indexStart)
        for k in range(indexStart+1,x+indexStart):
            element = element.right()
            if element is not None:
                uiChildi = UiWindows(element.info)
            else:
                break
            if uiChildi.getMidPoint()[0]>uiChild1.getMidPoint()[0]:
                lineNum = lineNum+1
            else:
                break
        print "linenNum:",lineNum
        nums = i
        print "numbs:",nums
        self.d.swipe(sx,sy, sx, sy-dy*1.3,50)
        j=0
        while i<x:
            if j == lineNum:
                self.d.swipe(sx,sy, sx, sy-dy,50)
                if canRecogizeEnd:
                    print UiWindows(self.d(resourceId="com.google.android.apps.plus:id/grid")\
                        .child(index=5).info)
                    print UiWindows(self.d(resourceId="com.google.android.apps.plus:id/grid").\
                        child(index=5).info).getBottom()
                    print self.d.info[u'displayHeight']
                    assert UiWindows(self.d(resourceId="com.google.android.apps.plus:id/grid")\
                        .child(index=nums).info).getBottom()>self.d.info[u'displayHeight'], "Failed to find"
                j=0
            print "open:", i+1
            self.d(resourceId="com.google.android.apps.plus:id/grid").child(index=nums+j).click()
            assert testElementFunc(), "failed"
            self.d.press.back()
            self.d(resourceId="com.google.android.apps.plus:id/grid").wait.exists(timeout=10000)
            i=i+1
            j=j+1

    def picOpen(self, x=1):
        # hardcoding the coordinates for the first item in the grid
        # proved to be easier to maintain
        if "gmin" in g_common_obj2.getprop("ro.board.platform") and\
                "ecs_e7" in g_common_obj2.getprop("ro.product.board"):
            return self.d.click(50, 300)
        return self.elementOpen(self.isPic, x)

    def picDelete(self):
        self.videoDelete()

    def openFist(self):
        self.d(resourceId="com.google.android.apps.plus:id/grid").child(index=0).click()

    def play(self):
        self._videoPlayObj().click()

    def isVideoPlayEnd(self):
        return self._videoPlayObj().exists

    def playNextPic(self):
        self.d(scrollable=True).scroll.horiz()

    def checkAlbum(self):
        self.openFist()
        time.sleep(0.5)
        assert self.d(resourceId="com.google.android.apps.plus:id/edit").exists or \
            self.d(resourceId="com.google.android.apps.plus:id/media_player_fragment_container").exists, "failed"
        self.d.press.back()

    def checkDifferentAlbums(self):
        notFolder=0
        for i in range(50):
            if not self.d(resourceId="com.google.android.apps.plus:id/tiles").child(index=i).child(resourceId="com.google.android.apps.plus:id/collection_title").exists:
                notFolder = notFolder+1
                if notFolder>4:
                    break
                continue
            notFolder=0
            self.d(resourceId="com.google.android.apps.plus:id/tiles").child(index=i).child(resourceId="com.google.android.apps.plus:id/collection_title").click()
            self.checkAlbum()
            self.d.press.back()

    def editPicture(self):
        self.d(resourceId="com.google.android.apps.plus:id/edit").click()
        i=0
        while self.d(resourceId="com.google.android.apps.plus:id/filter_list").child(index=i).exists:
            self.d(resourceId="com.google.android.apps.plus:id/filter_list").child(index=i).click()
            #child(index=0).click()
            j=0
            while self.d(resourceId="com.google.android.apps.plus:id/filter_parameter_panel_container").child(index=j).exists:
                self.d(resourceId="com.google.android.apps.plus:id/filter_parameter_panel_container").child(index=j).click()
                time.sleep(3)
                j = j+1
            j=0
            while self.d(resourceId="com.google.android.apps.plus:id/preset_list").child(index=j).exists:
                self.d(resourceId="com.google.android.apps.plus:id/preset_list").child(index=j).click()
                time.sleep(3)
                j = j+1
            self.d(resourceId="com.google.android.apps.plus:id/cancel_button").click()
            i = i+1
        self.d(text="Done").click()

    def more_options(self):
        self.d(className = "android.view.View",index = 1).click.wait()
        self.d(description = "More options",className = "android.widget.ImageButton").click.wait()

    def slideshow(self, duration):
        """click more options and slideshow"""
        difftime=0
        starttime=datetime.datetime.now()
        print "duration: %d" % duration
        while difftime < duration:
        #for i in range(slide_show_times):
            #self.d(className = "android.view.View",index = 1).click.wait()
            self.openFist()
            self.d(description = "More options").click.wait()
            #assert_equals(self.d(text = "Slideshow").exists, True)
            self.d(text = "Slideshow").click.wait()
            time.sleep(180)
            self.d.press.back()
            time.sleep(2)
            endtime=datetime.datetime.now()
            difftime=(endtime-starttime).seconds
            print difftime

    def display_image(self, display_number):
        """display image"""
        self.d(className = "android.view.View",index = 1).click.wait()
        for i in range(display_number):
            self.playNextPic()

    def playFirstVideo(self):
        self.openFist()
        time.sleep(1)
        self._videoPlayObj().click()
        #time.sleep(0.5)

    def repeatPlay(self):
        if  not self._videoPlayObj().exists:
            self.d.press.back()
        time.sleep(1)
        assert self._videoPlayObj().exists
        self._videoPlayObj().click.wait()
