#!/usr/bin/python

# Intel copyright reserved.
# Author: Wan Shuang (shuang.wan@intel.com)
# Date: 2012/11/23
# coding: UTF-8
import os
import sys
import time
import select

import threading
import Queue
import StringIO

from beanstalk import serverconn
from beanstalk import job
import beanstalk
import ConfigParser

def getConfValue(confFile, sectionName, optionName):
    try:
        cf = ConfigParser.ConfigParser()
        cf.read(confFile)
        return cf.get(sectionName, optionName)
    except:
        return None

__dir__=os.path.dirname(__file__)
os.system("%s"%os.path.join(__dir__, "unset_proxy.sh"))
confDir=os.path.join(os.path.abspath(__dir__), "..", "..", "testplan", "cps")
print confDir
confDir = os.path.normpath(confDir)
print confDir
if os.environ.get("TEST_DATA_ROOT"):
    confDir = os.environ.get("TEST_DATA_ROOT")
confFile = os.path.join(confDir, "tests.cps.conf")
print confFile
NewImgTestQueue = Queue.Queue()# .LifoQueue()
jobList = []
jobLock = threading.Lock()

Path = sys.path[0]

if not os.path.exists(os.path.join(__dir__,"debug")):
    cpsCmd = 'curl -X PUT "%s/delay.spi?RC_page=1\&from_page=90\&rc_addr1=30\&rc_addr2=30\&rc_port=%s\&rc_ctrl=%d"'
else:
    cpsCmd = "/opt/igas-engine/usb.sh %s %s %s"
print cpsCmd
cpsIp = getConfValue(confFile, "TestCPS", "main_ip")
cpsMac = getConfValue(confFile, "TestCPS", "mac_addr")
print cpsIp, cpsMac
arpCmd = "arp -s %s %s"%(cpsIp, cpsMac)
print arpCmd
os.system(arpCmd)
CpsGap=0.5
class PostExec(threading.Thread):
    def __init__(self, postList, listLock):
        threading.Thread.__init__(self)
        self.l = postList
        self.ll = listLock
        #print self.l
        #print self.ll
    def run(self):
        print "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        while True:
            i=0
            while i < len(self.l):
                each = self.l[i]
                if each[0] == 0:
                    os.system(cpsCmd%(cpsIp,each[2],35))
                    print time.time(), cpsCmd%(cpsIp,each[2],35)
                    with self.ll:
                        #self.ll.acquire()
                        each[0] = 1
                        each[1] = time.time()+ each[1]
                        #self.ll.release()
                    time.sleep(CpsGap)
                    #print "modify", each
                    #print "modify",self.l[i]
                elif each[0] == 1:
                    if time.time()>= each[1]:
                        os.system(cpsCmd%(cpsIp,each[2],36))
                        print time.time(), cpsCmd%(cpsIp,each[2],36)
                        self.ll.acquire()
                        del self.l[i]
                        self.ll.release()
                        i = i-1
                        time.sleep(CpsGap)
                        #print "del", each
                else:
                    self.ll.acquire()
                    del self.l[i]
                    self.ll.release()
                    i = i-1
                time.sleep(0.1)
                i = i+1

class ImgTestRunner(threading.Thread):
    def __init__(self, queue, usage, postList, listLock):
        threading.Thread.__init__(self)
        self.ImgTestQueue = queue
        self.usage = usage
        self.postList = postList
        self.postListLock = listLock

    def run(self):
        while True:
            imgurl = self.ImgTestQueue.get()
            self.execCpsCmd(imgurl)
            self.ImgTestQueue.task_done()
            time.sleep(0.1)

    def execCpsCmd(self, cmd):
        try:
            cmdList = cmd.split()
            print cmd
            try:
                if cmdList[0] == "cps" and len(cmdList)>=2:
                    #port = int(cmdList[1])
                    try:
                        sleeptime = float(cmdList[2])
                    except:
                        sleeptime = 1
                else:
                    return
            except:
                return
            tmp = [0, sleeptime,  cmdList[1]]
            #print "add", tmp
            with self.postListLock:
                #self.postListLock.acquire()
                self.postList.append(tmp)
                #self.postListLock.release()
            #print "add", tmp,"end"
        except:
            raise
            return

def ImgTestDaemonLoop(connection,usagemodel, queue):
    while True:
        ImgTestReq = connection.reserve()
        ImgFile = ImgTestReq.data
        #print "Detected an image test request: %s" % ImgFile
        queue.put(ImgFile)
        ImgTestReq.Finish()
        time.sleep(0.1)

def main():
    try:
        SevAddr = sys.argv[1]
        try:
            SevPort = int(sys.argv[2])
        except:
            SevPort = 11300

        UsageModel = sys.argv[3]

        postThread = PostExec(jobList, jobLock)
        postThread.setDaemon(True)

        RunnerThread = ImgTestRunner(NewImgTestQueue, UsageModel, jobList, jobLock)
        RunnerThread.setDaemon(True)

        postThread.start()
        RunnerThread.start()

        connection = serverconn.ServerConn(SevAddr, SevPort)

        connection.watch(UsageModel)

        connection.job = job.Job

        ImgTestDaemonLoop(connection, UsageModel, NewImgTestQueue)

        NewImgTestQueue.join()
    except KeyboardInterrupt, e:
        print e
        raise
    except Exception, e:
        print "Usage: MsgQueueMonitor.sh <server name> <server port> <usage model>"
        raise
        sys.exit(1)
if __name__ == '__main__':
    main()















