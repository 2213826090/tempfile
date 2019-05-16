import sys
import os
import unittest
from time import sleep
import ConfigParser
import time
from ubuntu_sso.utils.ui import TOS_LABEL
from functools import wraps
import nose
from nose.tools import *
import datetime

base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)

from testlib.common.common import g_common_obj2 as g_common_obj

def mkdirs(path):
    if not os.path.exists(path):
        os.makedirs(path)

__TEMP_DIR = None
def getTmpDir():
    pathList = ["/var/log", "/tmp", "~/tmp"]
    for each in pathList:
        if os.access(each, os.R_OK|os.W_OK):
            path=each
            break
    else:
        path="~/tmp"
    global __TEMP_DIR
    if __TEMP_DIR is None:
        __TEMP_DIR = os.path.join(path, "oat")
    path = os.path.expanduser(path)
    path = os.path.join(path, "oat", g_common_obj.getSerialNumber(), datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f"))
    path = os.path.normpath(path)
    mkdirs(path)
    return path

def clearTmpDir():
    if __TEMP_DIR:
        os.system("rm -rf %s"%__TEMP_DIR)

def getConfValue(confFile, sectionName, optionName):
    try:
        cf = ConfigParser.ConfigParser()
        cf.read(confFile)
        return cf.get(sectionName, optionName)
    except:
        return None

def toStandardPath(path):
    return os.path.normcase(os.path.normpath(os.path.abspath(path)))

def isPathEque(path1, path2):
    return toStandardPath(path1) == toStandardPath(path2)

def iterLists(*lists):
    for eachList in lists:
        if isinstance(eachList, list) or isinstance(eachList, tuple):
            for each in eachList:
                yield each
        else:
            yield eachList


class Base(object):
    _trap_attr = ("section",)
    def __getattr__(self, name):
        if name not in self._trap_attr:
            conf = self.getSelfConf(name)
            if conf is not None:
                return conf
        raise AttributeError(name)
        #return super(Base,self).__getattr__(name)

    def getConfDir(self):
        if "TEST_DATA_ROOT" in os.environ:
            return os.environ.get("TEST_DATA_ROOT", ".")
        else:
            return os.environ.get("TEST_REPO_ROOT", ".")

    def getConfValue(self, confFile, sectionName, optionName):
        return getConfValue(confFile, sectionName, optionName)

    def searchConf(self, sectionName, optionName):
        confList = str(self.__class__.__module__).split(".")
        genPath = lambda l:os.path.join(self.getConfDir(), ".".join(iterLists(l,"conf"))) 
        for i in range(len(confList), 0, -1):
            confFile = genPath(confList[:i])
            if os.path.isfile(confFile):
                ret = self.getConfValue(confFile, sectionName, optionName)
                if ret is not None:
                    setattr(self, optionName, ret)
                    return ret
        return None

    def getSelfConf(self, optionName):
        if not hasattr(self, "section"):
            self.section = self.__class__.__name__
            self.section = os.path.splitext(self.section)[0]
        ret = self.searchConf(self.section, optionName) 
        if ret is None:
            raise AttributeError(optionName)
        return ret

    def getTmpDir(self):
        return getTmpDir()

class BaseCase(Base, unittest.TestCase):
    """
    test to change wallpaper 
    """
    commonObj = g_common_obj
    @classmethod
    def setUpClass(cls):
        super(BaseCase, cls).setUpClass()
        cls.d = None

    @classmethod
    def tearDownClass(cls):
        super(BaseCase, cls).tearDownClass()

    def setUp(self):
        super(BaseCase, self).setUp()
        if hasattr(self, 'contexts'):
            #print "contexts", self.contexts
            g_common_obj.set_context(self.contexts)
        if not self.d:
            self.d = g_common_obj.get_device()
        # Start the execption handling.
        g_common_obj.start_exp_handle()
        self.current_orien = self.d.orientation
        self.d.orientation = "n"
        self.d.press.home()
        self.commonObj = g_common_obj

    def tearDown(self):
        g_common_obj.assert_exp_happens()
        super(BaseCase, self).tearDown()
        g_common_obj.stop_exp_handle()
        self.d.press.home()
        g_common_obj.close_background_apps()
        self.d.press.home()
        self.d.orientation = self.current_orien


    def launch_app_from_home(self, app_name):
        """launch photos app and classification by folder"""
        self.d.press.home()
        g_common_obj.launch_app_from_home_sc(app_name, "Apps")
        print "open app success"

    def getSC(self):
        if not hasattr(self, "index"):
            self.index = 0
        if not hasattr(self, "targetDir"):
            self.targetDir = self.getTmpDir()
        self.index += 1
        scpath = os.path.join(self.targetDir, "sc%04d.png"%self.index)
        self.d.screenshot(scpath)
        return scpath

    def scrollToFind(self,**kwargs):
        if self.d(**kwargs).exists:
            return self.d(**kwargs)
        return self.d(scrollable = True).scroll.to(**kwargs)


    def getRelPath(self, path):
        if os.path.isabs(path):
            return path
        return os.path.join(os.path.basename(os.path.dirname(sys.modules[self.__class__.__module__].__file__)), path)

    def checkBootToAndroid(self, timeout):
        start = time.time()
        while time.time()-start<timeout:
            time.sleep(1)
            if os.system("adb devices | grep %s | grep device > /dev/null"%self.d.server.adb.default_serial) == 0:
                try:
                    print self.d.info
                except:
                #except BaseException, e:
                    #print e
                    continue
                break
        else:
            assert False, "adb reboot timeout"

    def launchAppByName(self, appName):
        return g_common_obj.launchAppByName(appName)

    def unlock(self, raiseError=False):
        self.d.wakeup()
        if self.d(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
            self.d(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.right()
        if self.d(resourceId="com.android.systemui:id/lock_icon").exists:
            w=self.d.info[u'displayWidth']
            h=self.d.info[u'displayHeight']
            self.d.swipe(w/2,h,w/2,0)

def add_parameter(func):
    @wraps
    #@istest
    class testWrapper(object):
        def __init__(self, func):
            self.func = func
            self.kwargs, self.args = {}, []
            self.instance = None
            self.__name__ = func.__name__
            self.func_name = func.func_name
            self.func_code = func.func_code
            self.func_code.co_varnames
#             print dir(func)
#             print "-----------------"
#             for each in dir(func):
#                 print each, ":", getattr(func, each)
#                 print ""
#             print "-----------------"

        def __getattribute__(self, *args, **kwargs):
            print args
#             if args[0] == "__class__":
#                 return self.instance.__class__
            return object.__getattribute__(self, *args, **kwargs)

        def __get__(self, instance, owner):
            self.instance = instance
#             if self.instance:
#                 self.func = getattr(self.instance, name)
            return self

        def __getattr__(self, attr):
            print attr
#             if hasattr(self.func, attr):
#                 return getattr(self.func, attr)
            self.kwargs, self.args = {}, []
            argList = attr.split("__")
            if argList[0] != "parameter":
                raise AttributeError(attr)
            for each in argList[1:]:
                if not each:
                    continue
                tmp = each.split("_")
                if len(tmp)==1:
                    self.args.append(tmp[0])
                elif len(tmp)==2:
                    self.kwargs[tmp[0]]=tmp[1]
                else:
                    raise Exception("Invalid parameter: %s"%attr)
            return self
 
        def __call__(self, *args, **kwargs):
            self.args = [[self.instance] if self.instance else []] + list(args) + self.args
            self.kwargs.update(kwargs)
            print self.args
            print self.kwargs
            return self.func(*self.args, **self.kwargs)

    return testWrapper(func)

class UiWindows(object):
    def __init__(self, bound):
        if u"top" not in bound:
            bound = bound[u"bounds"]
        self.t=bound[u"top"]
        self.l=bound[u"left"]
        self.r=bound[u"right"]
        self.b=bound[u"bottom"]

    def __str__(self):
        return "lefttop: %d, %d    rightbottom: %d, %d"%(self.l, self.t, self.r, self.b)
    def getWidth(self):
        return self.r - self.l
    def getHeight(self):
        return self.b - self.t
    def getMidPoint(self):
        return (self.l+self.r)/2 , (self.t+self.b)/2
    def getTop(self):
        return self.t
    def getLeft(self):
        return self.l
    def getRight(self):
        return self.r
    def getBottom(self):
        return self.b


