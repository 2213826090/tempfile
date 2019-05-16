#! /usr/bin/python
import time
import qrcode
try:
    from PIL import Image, ImageDraw
except ImportError:
    import Image, ImageDraw
import sys
import os
import string
import re
from multiprocessing.dummy import Pool as ThreadPool
from StringIO import StringIO
import subprocess 

videoSrcPath=None
def get_image_paths(folder):
    return (os.path.join(folder,f)
        for f in os.listdir(folder)
        if 'png' in f)

def add_barcode(picPath, targetDir, addInfo, scale=500, index=0):
    picName = os.path.split(picPath)[-1]
    info = "%s:%02d%03d"%(addInfo,index,int(fetch_digit(picName))/scale)
    bg = Image.open(picPath)
    qr = qrcode.QRCode(version=2,error_correction=qrcode.constants.ERROR_CORRECT_L, box_size=10, border=4)
    qr.add_data(info)
    qr.make()
    im = qr.make_image()
    bg.paste(im,(0,0))
    global videoModPicDir
    bpath = os.path.join(targetDir, picName)
    bg.save(bpath)

def fetch_digit(string):
    p=re.compile('([0-9]){1,}')
    q=p.search(string)
    r=q.group(0)
    return r

def get_media_params(path):
    cmdList = ["mediainfo", path]
    p = subprocess.Popen(cmdList,stdout=subprocess.PIPE)
    p.wait()
    out = p.stdout.readlines()
    #print out
    for line in out:#open(mediainfo_log):
        sys.stdout.write(line)
        if "Width" in line:
            txt=line.replace(" ","")
            w=fetch_digit(txt)
        if "Height" in line:
            txt=line.replace(" ","")
            h=fetch_digit(txt)
        if "Frame rate" in line:
            if "fps" in line:
                txt=line.replace(" ","")
                videoSrcFPS=fetch_digit(txt)
    videoRes=w+"x"+h
    print videoRes,videoSrcFPS
    return videoRes,videoSrcFPS

# def getFfmpegParams(path):
#     cmdList = ["ffmpeg -i", path]
#     p = subprocess.Popen(cmdList,stdout=subprocess.PIPE)
#     p.wait()
#     out = p.stdout.readlines()
#     #print out
#     for line in out:#open(mediainfo_log):
#         sys.stdout.write(line)
#         if "Width" in line:
#             txt=line.replace(" ","")
#             w=fetch_digit(txt)
#         if "Height" in line:
#             txt=line.replace(" ","")
#             h=fetch_digit(txt)
#         if "Frame rate" in line:
#             if "fps" in line:
#                 txt=line.replace(" ","")
#                 videoSrcFPS=fetch_digit(txt)
#     videoRes=w+"x"+h
#     print videoRes,videoSrcFPS
#     return videoRes,videoSrcFPS

def convert(videoPath, addInfo, expectVideoFormat, expectFPS, index=0):
    if 'mp4' == expectVideoFormat:
        suffix = 'mp4'
    if 'vp8' == expectVideoFormat:
        suffix = 'webm'
    videoName = os.path.split(videoPath)[-1]
    if index != 0:
        videoNameOutput="target-%02d-"%index+videoName.split(".")[0]+"."+suffix
    else:
        videoNameOutput="target-" + videoName.split(".")[0]+"."+suffix
    videoUploadPath=os.path.dirname(videoPath)
    videoNameDir=os.path.join(videoUploadPath, videoName+"-out")
    videoSrcPicDir = os.path.join(videoNameDir, "video_src_pics")
    videoModPicDir = os.path.join(videoNameDir, "video_mod_pics")
    videoSrcPath=os.path.join(videoNameDir, videoName)
    videoTargetPath=os.path.join(videoNameDir, videoNameOutput)
    audioOut = os.path.join(videoNameDir, videoName.split(".")[0]+".mp3")
    print "++++Prepare video++++"
    if not os.path.exists(videoNameDir):
        os.mkdir(videoNameDir)
    if not os.path.exists(videoSrcPicDir):
        os.mkdir(videoSrcPicDir)
    if not os.path.exists(videoModPicDir):
        os.mkdir(videoModPicDir)

    os.system('cp ' + videoPath + ' ' + videoSrcPath)

    print "++++Get media infomation++++"
    videoRes,videoSrcFPS = get_media_params(videoSrcPath)

    print "++++Tranfer video to pictures++++"
    os.system('ffmpeg -i '+videoSrcPath+' -r '+str(videoSrcFPS)+' -f image2 '+videoSrcPicDir+'/image-%6d.png')
    os.system('ffmpeg -i '+videoSrcPath+' -vn ' + audioOut )

    print "++++Add barcode++++"
    folder = os.path.abspath(videoSrcPicDir)

    scale = int(videoSrcFPS)*5

    for each in get_image_paths(folder):
        print each
        add_barcode(each, videoModPicDir, addInfo, scale, index)

    print "++++Recombine video++++"
    if expectFPS is None:
        expectFPS = videoSrcFPS
    print 'ffmpeg -threads 10 -framerate '+str(videoSrcFPS)+' -i '+videoModPicDir+'/image-%06d.png -i "'+audioOut+'" -s:v '+videoRes+' -vcodec libx264 -profile:v high -pix_fmt yuv420p -r '+str(expectFPS)+' '+videoTargetPath
    if 'mp4' == expectVideoFormat:
        os.system('ffmpeg -threads 10 -framerate '+str(videoSrcFPS)+' -i '+videoModPicDir+'/image-%06d.png -i "'+audioOut+'" -s:v '+videoRes+' -vcodec libx264 -profile:v high -pix_fmt yuv420p -r '+str(expectFPS)+' '+videoTargetPath)
    if 'vp8' == expectVideoFormat:
        os.system('ffmpeg -threads 10 -framerate '+str(videoSrcFPS)+' -i '+videoModPicDir+'/image-%06d.png -i "'+audioOut+'" -s:v '+videoRes+' -vcodec libvpx -b:v 1900K -crf 10 -pix_fmt yuv420p -r '+str(expectFPS)+' '+videoTargetPath)

    print "videoNameOutput:",videoNameOutput

def main():
    if len(sys.argv)<=1:
        print "addbarcode.py \"video path\" [bar code prefix(default:media test)] [target format:mp4 or vp8] [except FPS]"
    src = sys.argv[1]
    addInfo = "media test" if len(sys.argv) <=2 else sys.argv[2]
    f = 'mp4' if len(sys.argv) <=3 else sys.argv[2]
    fps = None if len(sys.argv) <= 4 else sys.argv[3]
    convert(src, addInfo, f, fps)
#     for i in range(1, 20):
#         convert(src, addInfo, f, fps, i)

if __name__ == "__main__":
    exit(main())

