# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

"""
@summary: ExceptionImpl class
@since: 07/23/2015
@author: Zhao, Xiangyi
"""
import os
import uiautomator
from testlib.util.log import Logger
from testlib.util.device import TestDevice
from testlib.util.globalcontext import GlobalContext
from testlib.graphics.common import wifi_ctrl
from uiautomator import Selector
from uiautomator import JsonRPCError
from uiautomator import JsonRPCMethod
from uiautomator import JsonRPCClient
from uiautomator import Adb
from uiautomator import next_local_port
from uiautomator import NotFoundHandler
import socket
import urllib2
import urllib3
from httplib import HTTPException

LOG = Logger.getlogger(__name__)

class Exception_analyze(object):
    """exception class for can not found uiobject"""

    @classmethod
    def __init__ (self, cfg={}):
#         self.device = g_common_obj.get_device()
        self.device = click_common.get_device()

    @classmethod
    def analysis_crash(self):
        """check whether key string in uiobject"""

        self.device = click_common.get_device()
        print "call analysis method success!"
        crash_str = "crash,has stopped,isn't responding"
        self.crash_list = crash_str.split(",")
        print self.crash_list
        self.crash_list = list(self.crash_list)
        crash_mark = False

        try:
            for i in range(0, len(self.crash_list)):
                print i
                print self.crash_list[i]
                if self.device(textContains=self.crash_list[i]).exists:
                    print "Please check uiobject: %s" % (self.crash_list)
                    return crash_mark == True
                i = i + 1
            if crash_mark == False:
                self.analysis_wifi()
            else:
                print "Crash happened."
        except Exception as e:
            print e

    @classmethod
    def analysis_wifi(self):
        state, _ = wifi_ctrl.get_status()
        print "wifi state is %s" % (state)
        if state == "COMPLETED":
            print "other check !"
        else:
            assert False, "please check wifi connection."

class ClickImpl(uiautomator.AutomatorDevice):
  
    """ClickImpl"""

    def __init__(self, serial=None, local_port=None):
        self.server = AutomatorServer(serial=serial, local_port=local_port)

    def __call__(self, **kwargs):
        return ClickObjectImpl(self, Selector(**kwargs))

    def click(self, x, y):
        """catch click exception"""
        try:
            return self.server.jsonrpc.click(x, y)
        except Exception as e:
            print e

Device = ClickImpl

def param_to_property(*props, **kwprops):
    if props and kwprops:
        raise SyntaxError("Can not set both props and kwprops at the same time.")
  
    class Wrapper(object):
  
        def __init__(self, func):
            self.func = func
            self.kwargs, self.args = {}, []
  
        def __getattr__(self, attr):
            if kwprops:
                for prop_name, prop_values in kwprops.items():
                    if attr in prop_values and prop_name not in self.kwargs:
                        self.kwargs[prop_name] = attr
                        return self
            elif attr in props:
                self.args.append(attr)
                return self
            raise AttributeError("%s parameter is duplicated or not allowed!" % attr)
  
        def __call__(self, *args, **kwargs):
            if kwprops:
                kwargs.update(self.kwargs)
                self.kwargs = {}
                print "Now is param_to_property call"
                return self.func(*args, **kwargs)
            else:
                new_args, self.args = self.args + list(args), []
                return self.func(*new_args, **kwargs)
    return Wrapper

class AutomatorServer(uiautomator.AutomatorServer, object):

    """start and quit rpc server on device.
    """
    __jar_files = {
        "bundle.jar": "libs/bundle.jar",
        "uiautomator-stub.jar": "libs/uiautomator-stub.jar"
    }
    handlers = NotFoundHandler()  # handler UI Not Found exception

    def __init__(self, serial=None, local_port=None):
        self.uiautomator_process = None
        self.adb = Adb(serial=serial)
        self.device_port = 9008
        if local_port:
            self.local_port = local_port
        else:
            try:  # first we will try to use the local port already adb forwarded
                for s, lp, rp in self.adb.forward_list():
                    if s == self.adb.device_serial() and rp == 'tcp:%d' % self.device_port:
                        self.local_port = int(lp[4:])
                        break
                else:
                    self.local_port = next_local_port()
            except:
                self.local_port = next_local_port()

    @property
    def jsonrpc(self):
        server = self
        ERROR_CODE_BASE = -32000

        def _JsonRPCMethod(url, method, timeout, restart=True):
            _method_obj = JsonRPCMethod(url, method, timeout)

            def wrapper(*args, **kwargs):
                URLError = urllib3.exceptions.HTTPError if os.name == "nt" else urllib2.URLError
                try:
                    return _method_obj(*args, **kwargs)
                except (URLError, socket.error, HTTPException) as e:
                    if restart:
                        server.stop()
                        server.start()
                        return _JsonRPCMethod(url, method, timeout, False)(*args, **kwargs)
                    else:
                        raise
                except JsonRPCError as e:
                    if e.code >= ERROR_CODE_BASE - 2:
                        server.stop()
                        server.start()
                        print "i am here & do something in this place will work!"
                        Exception_analyze.analysis_crash()
                        return _method_obj(*args, **kwargs)
                    raise
            return wrapper

        return JsonRPCClient(self.rpc_uri,
                             timeout=int(os.environ.get("JSONRPC_TIMEOUT", 90)),
                             method_class=_JsonRPCMethod)

class ClickObjectImpl(uiautomator.AutomatorDeviceObject):

    """ClickImpl"""

    def __init__(self, device, selector):
#         super(ClickObjectImpl, self).__init__(device, selector)
        self.device = device
        self.jsonrpc = device.server.jsonrpc
        self.selector = selector

    @property
    def click(self):
        '''
        click on the ui object.
        Usage:
        d(text="Clock").click()  # click on the center of the ui object
        d(text="OK").click.wait(timeout=3000) # click and wait for the new window update
        d(text="John").click.topleft() # click on the topleft of the ui object
        d(text="John").click.bottomright() # click on the bottomright of the ui object
        '''
        @param_to_property(action=["tl", "topleft", "br", "bottomright", "wait"])
        def _click(action=None, timeout=3000):
            print "Now is _click"
            if action is None:
                try:
                    return self.jsonrpc.click(self.selector)
                except Exception as e:
                    print e
            elif action in ["tl", "topleft", "br", "bottomright"]:
                try:
                    return self.jsonrpc.click(self.selector, action)
                except Exception as e:
                    print e
            else:
                try:
                    return self.jsonrpc.clickAndWaitForNewWindow(self.selector, timeout)
                except Exception as e:
                    print e
        return _click

device = ClickObjectImpl

class ClickTestDevice(TestDevice):
    """
    TestDevice wrapper adb command calls and UiAutomator device object.
    The purpose of this class is to hide device serial to be used and
    ensure individual test case doesn't need to take care what device
    serial to be used if multiple device connected to host. So that t

    """
    def __init__(self, serial=None):
        self.serial = serial
        self.uia_device = None

    def get_device(self):
        """
        Lightweight wrapper for uiautomator device object.

        Use this wrapper to avoid handle device serial in individual
            test case.
        """

        if self.uia_device:
            return self.uia_device

        if self.serial:
            self.uia_device = Device(self.serial)
        else:
            self.uia_device = Device()

        return self.uia_device

class ClickCommon(object):
    """
    Common class is a lightweight wrapper to make using test case
    common utilities eaiser.

    """

    def __init__(self, context=None):
#         super(Common, self).__init__()
#         Common.__init__()
        self.globalcontext = GlobalContext()
        # TODO: override default context setting here if context object is not null.
        self.default_device = None
        self.default_test_device = None
        d = self.get_device()

    def set_context(self, context):
        """
        This method is used to setting global context object upon the
        conetext object injected by noseruner.

        The purpose of not using context object injected by noserunner directly
        is to de-couple with name convensions defined in noserunner. Otherwise
        this will introduce batch name revision if noserunner changed some
        properties' name.
        """

        if (context.user_log_dir is not None):
            self.globalcontext.user_log_dir = context.user_log_dir
            LOG.debug('User log directory was set to: %s according to context object' % self.globalcontext.user_log_dir)

        if (context.device_config['deviceid'] is not None):
            self.globalcontext.device_serial = context.device_config['deviceid']
            LOG.debug('Device serial was set to : %s according to context object ' % self.globalcontext.device_serial)

    def get_device(self, serial=None):
        if serial:
            return ClickTestDevice(serial).get_device()
        elif 'preferred_device' in os.environ:
            return ClickTestDevice(os.environ['preferred_device']).get_device()

        if not self.default_device:
            self.default_device = self.get_test_device().get_device()
        return self.default_device

    def get_test_device(self, serial=None):
        if serial:
            return ClickTestDevice(serial)

        if not self.default_test_device:
            self.default_test_device = ClickTestDevice(self.globalcontext.device_serial)
        return self.default_test_device

    def output(self):
        print "out put click_common.output"

click_common = ClickCommon()
