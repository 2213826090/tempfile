from testlib.common.common import g_logger
import os
from testlib.common.map import DeviceMap

class USBCutter(object):
    def init(self, serial=None):
        self.mapObj = DeviceMap()
        self.sn = self.mapObj.getValue("usbcutter")
        if self.sn is None:
            self.sn = os.popen(r"/opt/igas-engine/clewarecontrol -l 2>/dev/null | sed -n 's/.*serial number: \(.*\).*/\1/gp'").read().strip()

    def usbCutter(self, on=True):
        self.init()
        tmp = 0 if on else 1
        if self.sn:
            cmd = r"/opt/igas-engine/clewarecontrol -d %s -c 1 -as 0 %d"%(self.sn, tmp)
            g_logger.debug(cmd)
            g_logger.info(os.popen(cmd).read())
        else:
            g_logger.warning("couldn't find USB cutter")

usbCutterObj = USBCutter()

def unplugUsb():
    usbCutterObj.usbCutter(False)

def plugUsb():
    usbCutterObj.usbCutter(True)




