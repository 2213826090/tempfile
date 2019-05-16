#!/usr/bin/env python

'''
Created on Mar 10, 2016

@author: shankang
'''
#from PIL import Image, ImageStat
#from PIL.ExifTags import TAGS
import numpy as np
import os
import cv2
#import sys
import subprocess
from testlib.util.log import Logger
logger = Logger.getlogger()
#from subprocess import Popen, PIPE

# currentDir = os.path.abspath(os.path.dirname(__file__))
#tool = currentDir + os.sep + "IQCheck"

class CheckIQ:
    def __init__(self, toolPath="", tempDir=""):
        if toolPath != "":
            self.tool = toolPath #The folder where "IQCheck" exists
        else:
            from testlib.multimedia.multimedia_setting import MultiMediaSetting
            from testlib.camera.CameraCommon import CameraCommon
            multimedia_setting = MultiMediaSetting(CameraCommon().DEFAULT_CONFIG_FILE)
            localPath = CameraCommon().getTmpDir()
            binName = "IQCheck"
            remoteToolBinary = multimedia_setting.download_file_to_host("Multimedia_Camera/apk/"+binName)
            self.tool = localPath+"/"+binName
            print self.tool
            if os.path.exists(self.tool)==False:
                os.system("cp "+remoteToolBinary+" "+self.tool)
            os.system("chmod 777 "+self.tool)
        self.tempDir = tempDir

    class Region:
        """
        when max is True, then it is the max region
        when max is False:
            if w==0 || h==0, then it is the full region
            else w!=0 && h!=0, then use the crop region(x,y,w,h)
        The full region by default
        """
        def __init__(self, x=0, y=0, w=0, h=0, max=False):
            self.x = x
            self.y = y
            self.w = w
            self.h = h
            self.max = max
    
    """
    check whether there is any invalid lines in the picture
    return errMsg
    if errMsg=="", then there is no corruption detected
    """
    def checkCorrupt(self, file):
        errMsg = ""
        cmd = self.tool + " -d " + self.tempDir + " -c checkCorrupt -f " + file
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        for s in output[0][:-1].split("\n"):
            if s.find("Detected corruption") != -1:
                errMsg += s
                break
        if errMsg != "":
            errMsg += " File:" + file
        return errMsg
    
    """
    check whether this a border in the picture
    return yes, no, small
    yes: a suitable border was found
    no: no border was found
    small: a border was found, but is was too small
    """
    def findBorder(self, file, region=Region()):
        msg = ""
        rOp = ""
        if region.max == True:
            rOp += " -r max "
        elif region.max == False and (region.w > 0 and region.h > 0):
            rOp += " -r " + str(region.x) + "," + str(region.y) + "," + str(region.w) + "," + str(region.h)
        cmd = self.tool + " -d " + self.tempDir + " -c findBorder -f " + file + rOp
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        for s in output[0][:-1].split("\n"):
            if s.find("Picture border was not found") != -1:
                msg = "no"
                break
            elif s.find("Picture border found, but was too small") != -1:
                msg = "small"
                break
            elif s.find("Picture border found") != -1:
                msg = "yes"
                break
        return msg
    
    """
    find how many squares are there in the picture
    return the number of the square
    Color board related picture
    """
    def detectSquare(self, file, region=Region()):
        rOp = ""
        if region.max == True:
            rOp += " -r max "
        elif region.max == False and (region.w > 0 and region.h > 0):
            rOp += " -r " + str(region.x) + "," + str(region.y) + "," + str(region.w) + "," + str(region.h)
        cmd = self.tool + " -d " + self.tempDir + " -c detectSquare -f " + file + rOp
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        for s in output[0][:-1].split("\n"):
            if s.find("Found") != -1:
                number = int(s.split(" ")[1])
                break
        return number
    
    """
    find how many rectangles are there in the picture
    return the number of the rectangles
    """
    def detectRect(self, file, region=Region()):
        msg = ""
        rOp = ""
        if region.max == True:
            rOp += " -r max "
        elif region.max == False and (region.w > 0 and region.h > 0):
            rOp += " -r " + str(region.x) + "," + str(region.y) + "," + str(region.w) + "," + str(region.h)
        cmd = self.tool + " -d " + self.tempDir + " -c detectRect -f " + file + rOp
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        for s in output[0][:-1].split("\n"):
            if s.find("Found") != -1:
                number = int(s.split(" ")[1])
                break
        return number
    
    """
    find whether there are any invalid lines in the picture
    return errMsg
    if greenOnly is set true, only green lines will be detected, otherwise all the lines will be detected
    """
    def detectInvalidLine(self, file, greenOnly=True):
        errMsg = ""
        if greenOnly == True:
            cmd = self.tool + " -d " + self.tempDir + " -c detectLine -f " + file + " -t green"
        else:
            cmd = self.tool + " -d " + self.tempDir + " -c detectLine -f " + file + " -t all"
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
#        print output
        for s in output[0][:-1].split("\n"):
            if s.find("Found") != -1:
                errMsg += "; " + s
                if greenOnly == True:
                    break
            elif s.find("Detected corruption") != -1:
                errMsg += "; " + s
        return errMsg
    
    """
    give the compare HSV result between the two pictures
    return the compared HSV result
    """
    def cmpHist(self, file, file1, region=Region()):
        msg = ""
        rOp = ""
        if region.max == True:
            rOp += " -r max "
        elif region.max == False and (region.w > 0 and region.h > 0):
            rOp += " -r " + str(region.x) + "," + str(region.y) + "," + str(region.w) + "," + str(region.h)
        cmd = self.tool + " -d " + self.tempDir + " -c cmpHist -f " + file + " -f1 " + file1 + rOp
        logger.debug("cmpHist cmd=%s" % cmd)
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        logger.debug("cmpHist output=%s" % str(output))
        for s in output[0][:-1].split("\n"):
            if s.find("CompareResult") != -1:
                number = float(s.split(" ")[1])
                break
        return number

    """
    give color temperatue of the picture
    return temperatue value
    """
    def getTemp(self, file):
        msg = ""
        cmd = self.tool + " -d " + self.tempDir + " -c getTemp -f " + file
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        for s in output[0][:-1].split("\n"):
            if s.find("ColorTemperature") != -1:
                number = float(s.split(" ")[1])
                break
        return number

    """
    return the clarity number of the picture
    The larger number, the more clear of the picture
    """
    def getClarity(self, file):
        msg = ""
        cmd = self.tool + " -d " + self.tempDir + " -c getClarity -f "+file
        output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
        for s in output[0][:-1].split("\n"):
            if s.find("Clarity")!=-1:
                #print s
                number = float(s.split(" ")[1])
                break
        return number

    """
    check whether a picture is blurred or sharp
    return True if the picture is blurred
    return False if the picture is sharp 
    """
    def detectBlur(self, file, threshold=0.08, min_zero=0.026, region=Region()):#0.05, 0.025
        msg = ""
        rOp=""
        if region.max==True:
            rOp+=" -r max "
        elif region.max==False and (region.w>0 and region.h>0):
            rOp+=" -r "+str(region.x)+","+str(region.y)+","+str(region.w)+","+str(region.h)
        cmd = self.tool + " -d " + self.tempDir + " -c detectBlur -f "+file+" -t "+str(threshold)+" -z "+str(min_zero)+rOp
        output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
        print "threshold=%f, min_zero=%f" %(threshold, min_zero)
        print output[0][:-1]
        for s in output[0][:-1].split("\n"):
            if s.find("blurred")!=-1:
                blurred = True
                break
            elif s.find("sharp")!=-1:
                blurred = False
                break
        return blurred

    """
    give blurry number of the picture
    return blurry value
    """
    def getBlurry(self, file, region=Region()):
        msg = ""
        rOp = ""
        if region.max == True:
            rOp += " -r max "
        elif region.max == False and (region.w > 0 and region.h > 0):
            rOp += " -r " + str(region.x) + "," + str(region.y) + "," + str(region.w) + "," + str(region.h)
        cmd = self.tool + " -d " + self.tempDir + " -c getBlurry -f " + file + rOp
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        for s in output[0][:-1].split("\n"):
            if s.find("BlurryNumber")!=-1:
                number = float(s.split(" ")[1])
#                l = s.split(" ")[1].split(",")
#                print l
#                number = float(l[len(l)-1])
                break
        return number
    
    """
    give the compare SURF similarity result between the two pictures
    return the similarity value
    file is the referenced picture
    """
    def getSimilarity(self, file, file1, region=Region()):
        msg = ""
        rOp = ""
        if region.max == True:
            rOp += " -r max "
        elif region.max == False and (region.w > 0 and region.h > 0):
            rOp += " -r " + str(region.x) + "," + str(region.y) + "," + str(region.w) + "," + str(region.h)
        cmd = self.tool + " -d " + self.tempDir + " -c getSimilarity -f " + file + " -f1 " + file1 + rOp
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        for s in output[0][:-1].split("\n"):
            if s.find("Matching Percentage") != -1:
                s = s.split(":")[1]
                s = "".join(s.split())
                number = float(s[:-1])
                break
        return number
    
    """
    get the bar code numbers from a given picture
    jarFile is the full path of zxing-analyzer.jar which can also helps to get the result
    if analyzeMode is set True, it will check whether where are any invalid lines in the picture
    return [bar code number string, errMsg]
    """
    def getBarcode(self, file, jarpath, analyzeMode=False):
        msg = ""
        rOp = ""
        if analyzeMode == True:
            rOp += " -a true "
        cmd = self.tool + " -d " + self.tempDir + " -c getBarCode -f " + file +" -j "+jarpath+rOp
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT).communicate()
        numberString = ""
        errString = ""
        for s in output[0][:-1].split("\n"):
            if s.find("getBarcode") != -1:
                numberString = s.split(":")[1]
            elif s.find("Error found:") != -1:
                errString = s.split(":")[1]
        return [numberString, errString]

class CheckZoom:
    
    def getZoomRatio(self, file):
    #    print file
        import pyexiv2
        metadata = pyexiv2.ImageMetadata(file)
        metadata.read()
        ratio = metadata['Exif.Photo.DigitalZoomRatio'].value
        return str(ratio)
        
    def getAngle(self,pt2,w,h):
        import math
        pt0 = np.array((w/2, h/2))
        pt1 = np.array((pt2[0], h/2))
        dis = self.dist(pt0, pt2)
        print "distance is %d"%(dis)
        if dis<5:
            return "center"
        dy = pt1[1] - pt2[1]
        dx = pt1[0] - pt0[0]
        value = dx/math.sqrt((dx*dx + dy*dy) + 1e-10)
    #    dx1 = pt1[0] - pt0[0]
    #    dy1 = pt1[1] - pt0[1]
    #    dx2 = pt2[0] - pt0[0]
    #    dy2 = pt2[1] - pt0[1]
    #    value = (dx1*dx2 + dy1*dy2)/math.sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10)
    #    print value
        angle = int(math.acos(value)*180/math.pi)
        if pt2[1] > pt0[1]:
            angle = 360 - angle
        if angle in range(0,90):
            return "topRight"
        elif angle in range(90,180):
            return "topLeft"
        elif angle in range(180,270):
            return "bottomLeft"
        elif angle in range(270,360):
            return "bottomRight"
       
    def dist(self, x,y):   
        return np.sqrt(np.sum((x-y)**2))
    
    """
    check the zoom effect, file1 is the no zoomed picture, file2 is the zoomed picture; 
    and the zoom ration needs to be a integer, otherwise the function will return False directly
    return True if the zoom effect is correct
    """    
    def checkZoomEffect(self, file1, file2):
        from testlib.multimedia.multimedia_setting import MultiMediaSetting
        from testlib.camera.CameraCommon import CameraCommon
        localPath = CameraCommon().getTmpDir()
        multimedia_setting = MultiMediaSetting(CameraCommon().DEFAULT_CONFIG_FILE)
        remotejarFile = multimedia_setting.download_file_to_host("Multimedia_Camera/apk/sikuli-api-ex2.jar")
        print remotejarFile
        jarFile = localPath+"/sikuli-api-ex2.jar"
        print jarFile
        if os.path.exists(jarFile)==False:
            os.system("cp "+remotejarFile+" "+jarFile)
            os.system("chmod 777 "+jarFile)
        os.environ['CLASSPATH'] = jarFile
        from jnius import autoclass
        Sikuli = autoclass('com.intel.sikuli.image.analysis.Sikuli')
        self.sikuli = Sikuli()
        
        img = cv2.imread(file1)
    #    print img.shape
        samll_mage = cv2.imread(file2)
    #    print samll_mage.shape
        if (samll_mage.shape[1] > img.shape[1]) or (samll_mage.shape[0] > img.shape[0]):
            print "size of pic1 should >= size of pic2"
            exit(-1)
        ratio1 = self.getZoomRatio(file1)
        ratio = self.getZoomRatio(file2)
        print ratio1,ratio
        def changeFormat(t_str):
            from fractions import Fraction
            if "/" in t_str:
                t_str = Fraction(t_str)
            return float(t_str)
        ratio1 = changeFormat(ratio1)
        ratio = changeFormat(ratio)
        print ratio1,ratio
        if ratio1 > ratio:
            print "zoom ratio of pic2 should >= zoom ratio of pic2"
            exit(-1)
        rat = ratio
        if rat>1:
            l = file1.split("/")
            pic_name = l[len(l)-1].split(".")[0]
            picPath1 = os.path.dirname(os.path.abspath(file1))+os.sep+pic_name+"_ori.jpg"
            cv2.imwrite(picPath1, img)
            file1 = picPath1
            l = file2.split("/")
            pic_name = l[len(l)-1].split(".")[0]
            picPath2 = os.path.dirname(os.path.abspath(file2))+os.sep+pic_name+"_zoom.jpg"
            samll_mage = cv2.imread(file2)
            newx,newy = int(samll_mage.shape[1]/rat),int(samll_mage.shape[0]/rat) #new size (w,h)
            newimage = cv2.resize(samll_mage,(newx,newy))
            cv2.imwrite(picPath2, newimage)
            file2 = picPath2
        ret = self.sikuli.sikuliFind(file1, file2, "image", "0.8")
        print ret
        ret_l = ret.split(" ")
        if ret_l[0]=="true":
            print "Zoom effect is correct"
            x=int(ret_l[1].split("=")[1])
            y=int(ret_l[2].split("=")[1])
            w=int(ret_l[3].split("=")[1])
            h=int(ret_l[4].split("=")[1])
    #        a = np.array((img.shape[1]/2,img.shape[0]/2))
    #        b = np.array((x+w/2,y+h/2))
    #        dist_a_b = dist(a,b)
    #       shape[0]: height/ shape[1]: width
            print self.getAngle(np.array((x+w/2,y+h/2)), img.shape[1], img.shape[0])
    #        print "---w=%d, h=%d, distance=%d"%(img.shape[1], img.shape[0], dist_a_b)
    #        print "%d, %d, %d, %d"%(x,y,w,h)
            if rat>1:
                cv2.rectangle(img, (x, y), (x+w, y+h), (127, 255, 0), 2)
                cv2.imwrite(os.path.dirname(os.path.abspath(file2))+os.sep+pic_name+"_frame.jpg", img)
            return True
        else:
            print "Zoom effect is not correct"
            return False

#def main():
#     checkIQ = CheckIQ("/data/iqcheck/IQCheck","/home/test/tmp")
#     print checkIQ.getBarcode("/data/opencv_test/barDetect/zxing/tmp/005.jpg", "/work/eclipse/workspace/bar_detect/bin/zxing-analyzer.jar")
#     print checkIQ.getBarcode("/data/opencv_test/barDetect/zxing/tmp/006.jpg", "/work/eclipse/workspace/bar_detect/bin/zxing-analyzer.jar", analyzeMode=True)
#    adjustFOV()
#    print checkIQ.getSimilarity(sys.argv[1], sys.argv[2])
#    print checkIQ.getBlurry(sys.argv[1])
#    print checkIQ.getTemp(sys.argv[1])
#    print checkIQ.cmpHist(sys.argv[1], sys.argv[2])
#    print checkIQ.detectInvalidLine(sys.argv[1], False)
#     print checkIQ.detectRect("/tmp/logs/sc_disable_facktrack2.png", CheckIQ.Region(0, 212, 720, 908, False))
#     print checkIQ.detectRect("/tmp/logs/sc_enable_facktrack2.png", CheckIQ.Region(0, 212, 720, 908, False))
#    print checkIQ.detectSquare(sys.argv[1])
#    print checkIQ.findBorder(sys.argv[1], CheckIQ.Region(0,144,720,998,False))
#    print checkIQ.findBorder(sys.argv[1], CheckIQ.Region())
#    print checkIQ.findBorder(sys.argv[1], CheckIQ.Region(max=True))
#    print checkIQ.findBorder(sys.argv[1], CheckIQ.Region(0,50,720,800,False))
#    print checkIQ.checkCorrupt(sys.argv[1])
#if __name__ == "__main__":
#      main()
