import os
from testlib.util.uiatestbase import UIATestBase
from testlib.common.base import Base
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
import types
from testlib.util.repo import Artifactory


def downFile(url, raiseException=True):
    url=url.strip()
    u, f = os.path.split(url)
    ret = Artifactory(u+"/").get(f)
    if not ret and raiseException:
        raise Exception("download file failed: %s" %url)
    return ret

class UIATestBase2(UIATestBase, Base):
    """
    Android UI Automataed Test Case Implementation

    """

#     def __init__(self, *args, **kwargs):
#         super(UIATestBase2, self).__init__(*args, **kwargs)
#         print "=================================================="
#         self.getTestPlan()

    def setUp(self):
        super(UIATestBase2, self).setUp()
        if hasattr(self, "impl") and isinstance(self.impl, (types.ClassType, type)):
            self.d = g_common_obj.get_device()
            self.impl=self.impl(self.d)
        self.comObj = g_common_obj2
        self.sn = self.comObj.getSerialNumber()
        if hasattr(self, "apklist"):
            self.apklist = self.apklist.split(",")
            for eachApk in self.apklist:
                f = downFile(eachApk)
                self.comObj.adb_cmd_common("install %s"%f,3600)
        if not hasattr(self, "artifactory_root"):
            self.artifactory_root = ""
#         if not hasattr(self, "push_target"):
#             self.push_target = os.path.join("/sdcard/", self.__class__.__name__)
        self.download_cache_list = []
        if hasattr(self, "download_list"):
            self.download_list = self.download_list.split(",")
            for each in self.download_list:
                each = each.strip()
                if not each.startswith("http"):
                    each = os.path.join(self.artifactory_root, each)
                self.download_cache_list.append(downFile(each))
        self.cache_file_list = []
        if hasattr(self, "unzip_list"):
            self.unzip_list = [x.strip() for x in self.unzip_list.split(",")]
            #cache_file_list = [os.path.basename(x.strip()) for x in self.download_cache_list]
            for each in self.download_cache_list:
                dirPath, fileName = os.path.split(each)
                if fileName in self.unzip_list:
                    retMsg = os.popen("unzip -o \"%s\" -d \"%s\" | grep inflating | awk -F: '{print $2}' "%(each, dirPath)).read()
                    self.cache_file_list.extend([x.strip() for x in retMsg.splitlines()])
                else:
                    self.cache_file_list.append(each)
        else:
            self.cache_file_list= self.download_cache_list
        if hasattr(self, "push_target"):
            if self.cache_file_list:
                self.comObj.adb_cmd_common(r'shell mkdir "%s"'%self.push_target)
            for each in self.cache_file_list:
                self.comObj.adb_cmd_common(r'push "%s" "%s/"'%(each, self.push_target),3600)
        if hasattr(self, "adb_pre_cmd"):
            self.comObj.adb_cmd_common(self.adb_pre_cmd)

    def tearDown(self):
        super(UIATestBase2, self).tearDown()
        if hasattr(self, "uninstall_package_list"):
            self.uninstall_package_list = self.uninstall_package_list.split(",")
            for eachPackage in self.uninstall_package_list:
                eachPackage = eachPackage.strip()
                if eachPackage:
                    self.comObj.adb_cmd_common("uninstall %s"%eachPackage)
        if hasattr(self, "push_target"):
            if self.cache_file_list:
                for each in self.cache_file_list:
                    filename = os.path.basename(each)
                    self.comObj.adb_cmd(r'rm "%s"'%os.path.join(self.push_target, filename))


    def getTestPlan(self):
        module = self.__class__.__module__
        classname = self.__class__.__name__
        for each in dir(self.__class__):
            if each.lower().startswith("test"):
                print "%s.%s.%s"%(module, classname, each)

class UIATestType(type):
    def __getattr__(self, name):
        tmp = name.split("__")
        if len(tmp)>=2 and tmp[0] and (tmp[0].lower().startswith("test") or tmp[0].lower().startswith("parmeter")):
            tf = getattr(self, tmp[0])
            if type(tf) is types.UnboundMethodType:
                args = []
                kwargs = {}
                for eachArg in tmp[1:]:
                    eachArgSplit = eachArg.split("_")
                    if len(eachArgSplit)==1:
                        args.append(eachArg)
                    elif len(eachArgSplit)==2:
                        kwargs[eachArgSplit[0]] = eachArgSplit[1]
                    else:
                        raise AttributeError(name)
                def func(self):
#                     print args
#                     print kwargs
                    return getattr(self, tmp[0])(*args, **kwargs)
                func.__name__ = name
                setattr(self, name, func)
                return getattr(self, name)
        raise AttributeError(name)

class UIATestBase3(UIATestBase2):
    """
    UIATestBase3: inherit this class to pass parmeters from plan file:
    "__" split each paremeters
    "_" split keyword and value in paremeter
    module.class.testxxx__1__2 -> self.testxxx(1,2)
    module.class.testxxx__1__2__a_2 -> self.testxxx(1,2,a=2)
    parmeter is another prefix of testfunction
    """
    __metaclass__ = UIATestType


