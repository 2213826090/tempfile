
from testlib.util.config import TestConfig
from testlib.common.common import g_common_obj2
import ConfigParser


MAP_FILE_NAME="map.conf"

class DeviceMap(object):
    """
    This class use to get some specific conf value
    use DeviceMap().XXX, XXX is the section in the map.conf, if this section exists, 
    return the specific deivce value, if XXX is start with "common", will return common option value.
    and if the device serial ID is not exist, return None.
    if sections not exists, will raise an AttributeError
    """
    def __init__(self):
        #self.sn = g_common_obj2.getSerialNumber()
        self.cfgObj = TestConfig()

    def __getattr__(self, name):
        sectionName = name
        self.sn = g_common_obj2.getSerialNumber()
        sn = self.sn
        if name.startswith("common_"):
            sectionName = name.replace("common_", "")
            sn = "common"
        old_optionxform = ConfigParser.ConfigParser.optionxform
        ConfigParser.ConfigParser.optionxform = lambda self, x:x
        tempAttrDict = self.cfgObj.read(MAP_FILE_NAME, sectionName)
        ConfigParser.ConfigParser.optionxform = old_optionxform
        if tempAttrDict:
            #print tempAttrDict
            #print sn
            if sn in tempAttrDict:
                setattr(self, name, tempAttrDict[sn])
                return getattr(self, name)
            return None
        raise AttributeError(name)

    def getValue(self, name, default=None):
        try:
            ret = getattr(self, name)
            if ret is None:
                return default
            return ret
        except AttributeError:
            if default is not None:
                return default
            else:
                raise


#g_devce_map_obj = DeviceMap()


