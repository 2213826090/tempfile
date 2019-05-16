'''
Created on Oct 26, 2015

@author: shankang
'''
from abc import ABCMeta, abstractmethod
from _pyio import __metaclass__

class CameraAppInterface(object):
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def cleanMediaFiles(self):
        """
        Used to clean the media files in some folders; eg. /mnt/sdcard/DCIM/Camera
        """
        pass

    @abstractmethod
    def startCameraApp(self):
        """
        Used to start the camera application
        """
        pass

    @abstractmethod
    def stopCameraApp(self):
        """
        Used to stop the camera application
        """
        pass

    @abstractmethod
    def selectMode(self, mode="Camera"):
        """
        Used to select a mode such as photo, video, panorama, hdr, zsl, continous and so on...
        """
        pass

    @abstractmethod
    def switchRearOrFront(self, lens="Back"):
        """
        Used to switch rear or front camera
        """
        pass

    @abstractmethod
    def setFlash(self, flash="off"):
        """
        Used to control the flash; on, off, auto
        """
        pass

    @abstractmethod
    def setGrid(self,grid="off"):
        """
        Used to control the grid; on, off
        """
        pass

    @abstractmethod
    def setTimer(self, timer="off"):
        """
        Used to control the timer
        """
        pass

    @abstractmethod
    def getAllVideoResolutions(self):
        """
        Return all of the video resolutions
        """
        pass

    @abstractmethod
    def setVideoResolution(self, resolution):
        """
        Used to control the video resolution, used with the getAllVideoResolutions
        """
        pass

    @abstractmethod
    def getAllCameraMode(self):
        """
        Return all of the camera mode
        """
        pass

    @abstractmethod
    def getAllPhotoResolutions(self):
        """
        Return all of the photo resolutions
        """
        pass

    @abstractmethod
    def setPhotoResolution(self, resolution):
        """
        Used to control the photo resolution, used with the getAllPhotoResolutions
        """
        pass

    @abstractmethod
    def capturePhoto(self, num=1):
        """
        Used to capture num photos
        """
        pass

    @abstractmethod
    def reviewPhoto(self, num=1):
        """
        Used to review num photos
        """
        pass

    @abstractmethod
    def recordVideo(self, num=1, duration=1):
        """
        Used to capture num duration videos
        """
        pass

    @abstractmethod
    def reviewVideo(self, num=1, duration=1):
        """
        Used to review num duration videos
        """
        pass

    @abstractmethod
    def snapShotDuringVideo(self, num=1, duration=1):
        """
        Used to snapshot num pictures during a duration video
        """
        pass