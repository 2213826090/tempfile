#!/usr/bin/python
# Intel copyright reserved.
# Author: Wan Shuang (shuang.wan@intel.com)
# Date: 2012/11/21

import sys
import time
import select

from beanstalk import serverconn
from beanstalk import job
import beanstalk

def PutImgTestRequest (connection, ImgLocation, MsgTube):
    connection.use(MsgTube)
    TestReq = job.Job(data=ImgLocation, conn=connection)
    TestReq.Queue()

def main():
    try:
        SevAddr = sys.argv[1]
        try:
            SevPort = int(sys.argv[2])
        except:
            SevPort = 11300

        ImgLocation = sys.argv[3]

        MsgTube = sys.argv[4]

        connection = serverconn.ServerConn(SevAddr, SevPort)
        connection.job = job.Job
        PutImgTestRequest (connection, ImgLocation, MsgTube)
    except Exception, e:
        print "Usage: MsgQueueProxy.py Server Port ImgLocation MsgTube"
        raise
        sys.exit(1)
if __name__ == '__main__':
    main()
