#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Python wrapper for audiostub """

from testlib.util.common import g_common_obj
from testlib.audio import resource
from testlib.util.instrumentedtestbase import InstrumentedBaseImpl

import sys
import os
import subprocess
import time
import json
import hashlib
import socket
import re
import collections

if 'localhost' not in os.environ.get('no_proxy', ''):
    os.environ['no_proxy'] = "localhost,%s" % os.environ.get('no_proxy', '')

try:
    import urllib2
except ImportError:
    import urllib.request as urllib2
try:
    from httplib import HTTPException
except:
    from http.client import HTTPException
try:
    if os.name == 'nt':
        import urllib3
except:  # to fix python setup error on Windows.
    pass


def U(x):
    if sys.version_info.major == 2:
        return x.decode('utf-8') if type(x) is str else x
    elif sys.version_info.major == 3:
        return x


def param_to_property(*props, **kwprops):
    if props and kwprops:
        raise SyntaxError(
            "Can not set both props and kwprops at the same time.")

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
            raise AttributeError("%s parameter is duplicated or not allowed!"
                                 % attr)

        def __call__(self, *args, **kwargs):
            if kwprops:
                kwargs.update(self.kwargs)
                self.kwargs = {}
                return self.func(*args, **kwargs)
            else:
                new_args, self.args = self.args + list(args), []
                return self.func(*new_args, **kwargs)
    return Wrapper

class JsonRPCError(Exception):

    def __init__(self, code, message):
        self.code = int(code)
        self.message = message

    def __str__(self):
        return "JsonRPC Error code: %d, Message: %s" % (self.code, self.message)

class JsonRPCMethod(object):

    if os.name == 'nt':
        pool = urllib3.PoolManager()

    def __init__(self, url, method, timeout=30):
        self.url, self.method, self.timeout = url, method, timeout

    def __call__(self, *args, **kwargs):
        if args and kwargs:
            raise SyntaxError("Could not accept both *args and **kwargs" +
                              " as JSONRPC parameters.")
        data = {"jsonrpc": "2.0", "method": self.method, "id": self.id()}
        if args:
            data["params"] = args
        elif kwargs:
            data["params"] = kwargs
        if os.name == "nt":
            res = self.pool.urlopen("POST",
                                    self.url,
                                    headers={"Content-Type":
                                             "application/json"},
                                    body=json.dumps(data).encode("utf-8"),
                                    timeout=self.timeout)
            jsonresult = json.loads(res.data.decode("utf-8"))
        else:
            result = None
            try:
                req = urllib2.Request(self.url,
                                      json.dumps(data).encode("utf-8"),
                                      {"Content-type": "application/json"})
                result = urllib2.urlopen(req, timeout=self.timeout)
                jsonresult = json.loads(result.read().decode("utf-8"))
            finally:
                if result is not None:
                    result.close()
        if "error" in jsonresult and jsonresult["error"]:
            code = jsonresult["error"]["code"]
            msg = jsonresult["error"]["message"]
            raise JsonRPCError(code, msg)
        return jsonresult["result"]

    def id(self):
        m = hashlib.md5()
        m.update(("%s at %f" % (self.method, time.time())).encode("utf-8"))
        return m.hexdigest()

class JsonRPCClient(object):

    def __init__(self, url, timeout=30, method_class=JsonRPCMethod):
        self.url = url
        self.timeout = timeout
        self.method_class = method_class

    def __getattr__(self, method):
        return self.method_class(self.url, method, timeout=self.timeout)

class Adb(object):

    def __init__(self, serial=None, adb_server_host=None, adb_server_port=None):
        self.__adb_cmd = None
        self.default_serial = serial if serial else os.environ.get("ANDROID_SERIAL", None)
        self.adb_server_host = str(adb_server_host if adb_server_host else 'localhost')
        self.adb_server_port = str(adb_server_port if adb_server_port else '5037')
        self.adbHostPortOptions = []
        if self.adb_server_host not in ['localhost', '127.0.0.1']:
            self.adbHostPortOptions += ["-H", self.adb_server_host]
        if self.adb_server_port != '5037':
            self.adbHostPortOptions += ["-P", self.adb_server_port]

    def adb(self):
        if self.__adb_cmd is None:
            if "ANDROID_HOME" in os.environ:
                filename = "adb.exe" if os.name == 'nt' else "adb"
                adb_cmd = os.path.join(os.environ["ANDROID_HOME"],
                                       "platform-tools", filename)
                if not os.path.exists(adb_cmd):
                    raise EnvironmentError(
                        "Adb not found in $ANDROID_HOME path: %s."
                        % os.environ["ANDROID_HOME"])
            else:
                import distutils
                if "spawn" not in dir(distutils):
                    import distutils.spawn
                adb_cmd = distutils.spawn.find_executable("adb")
                if adb_cmd:
                    adb_cmd = os.path.realpath(adb_cmd)
                else:
                    raise EnvironmentError("$ANDROID_HOME environment not set.")
            self.__adb_cmd = adb_cmd
        return self.__adb_cmd

    def cmd(self, *args, **kwargs):
        '''adb command, add -s serial by default. return the subprocess.Popen object.'''
        serial = self.device_serial()
        if serial:
            if " " in serial:  # TODO how to include special chars on command line
                serial = "'%s'" % serial
            return self.raw_cmd(*["-s", serial] + list(args))
        else:
            return self.raw_cmd(*args)

    def raw_cmd(self, *args):
        '''adb command. return the subprocess.Popen object.'''
        cmd_line = [self.adb()] + self.adbHostPortOptions + list(args)
        if os.name != "nt":
            cmd_line = [" ".join(cmd_line)]
        return subprocess.Popen(cmd_line, shell=True, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)

    def device_serial(self):
        devices = self.devices()
        if not devices:
            raise EnvironmentError("Device not attached.")

        if self.default_serial:
            if self.default_serial not in devices:
                raise EnvironmentError("Device %s not connected!"
                                       % self.default_serial)
        elif len(devices) == 1:
            self.default_serial = list(devices.keys())[0]
        else:
            raise EnvironmentError("Multiple devices attached" +
                                   " but default android serial not set.")
        return self.default_serial

    def devices(self):
        '''
        get a dict of attached devices.
        key is the device serial, value is device name.
        '''
        out = self.raw_cmd("devices").communicate()[0].decode("utf-8")
        match = "List of devices attached"
        index = out.find(match)
        if index < 0:
            raise EnvironmentError("adb is not working.")
        return dict([s.split("\t")
                     for s in out[index + len(match):].strip().splitlines()
                     if s.strip()])

    def forward(self, local_port, device_port):
        '''adb port forward. return 0 if success, else non-zero.'''
        return self.cmd("forward",
                        "tcp:%d" % local_port,
                        "tcp:%d" % device_port).wait()

    def forward_list(self):
        '''adb forward --list'''
        version = self.version()
        if int(version[1]) <= 1 \
           and int(version[2]) <= 0 \
           and int(version[3]) < 31:
            raise EnvironmentError("Low adb version.")
        lines = self.raw_cmd("forward", "--list").communicate()[0]\
            .decode("utf-8").strip().splitlines()
        return [line.strip().split() for line in lines]

    def forward_remove(self, local_port):
        ''' adb forward remove <local> '''
        return self.cmd("forward", "--remove", str(local_port))

    def version(self):
        '''adb version'''
        m = re.search(r"(\d+)\.(\d+)\.(\d+)",
                      self.raw_cmd("version").communicate()[0].decode("utf-8"))
        return [m.group(i) for i in range(4)]

_init_local_port = 9080

def next_local_port(adbHost=None):
    def is_port_listening(port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = s.connect_ex((str(adbHost) if adbHost else '127.0.0.1', port))
        s.close()
        return result == 0

    global _init_local_port
    _init_local_port = (_init_local_port + 1
                        if _init_local_port < 32764
                        else 9080)
    while is_port_listening(_init_local_port):
        _init_local_port += 1
    return _init_local_port

class NotFoundHandler(object):

    '''
    Handler for UI Object Not Found exception.
    It's a replacement of audiostub watcher on device side.
    '''

    def __init__(self):
        self.__handlers = collections.defaultdict(
            lambda: {'on': True, 'handlers': []})

    def __get__(self, instance, type):
        return self.__handlers[instance.adb.device_serial()]

class AutomatorServer(object):

    """start and quit rpc server on device.
    """
    handlers = NotFoundHandler()  # handler UI Not Found exception
    device_port = 9081

    def __init__(self, serial=None, local_port=None, app=None,
                 adb_server_host=None, adb_server_port=None):
        self.adb = Adb(serial=serial, adb_server_host=adb_server_host,
                       adb_server_port=adb_server_port)
        if local_port:
            self.local_port = local_port
        else:
            try:  # first we will try to use the local port already adb forward
                for s, lp, rp in self.adb.forward_list():
                    if s == self.adb.device_serial() and \
                       rp == 'tcp:%d' % self.device_port:
                        self.local_port = int(lp[4:])
                        break
                else:
                    self.local_port = next_local_port(adb_server_host)
            except:
                self.local_port = next_local_port(adb_server_host)
        self.app = app if app else AudioStubApp()

    def download(self, filename, url):
        with open(filename, 'wb') as file:
            res = None
            try:
                res = urllib2.urlopen(url)
                file.write(res.read())
            finally:
                if res is not None:
                    res.close()

    @property
    def jsonrpc(self):
        return self.jsonrpc_wrap(
            timeout=int(os.environ.get("jsonrpc_timeout", 90)))

    def jsonrpc_wrap(self, timeout):
        server = self
        ERROR_CODE_BASE = -32000

        def _JsonRPCMethod(url, method, timeout, restart=True):
            _method_obj = JsonRPCMethod(url, method, timeout)

            def wrapper(*args, **kwargs):
                URLError = urllib3.exceptions.HTTPError if os.name == "nt" \
                    else urllib2.URLError
                try:
                    return _method_obj(*args, **kwargs)
                except (URLError, socket.error, HTTPException) as e:
                    if restart:
                        server.stop()
                        server.start(timeout=30)
                        return _JsonRPCMethod(
                            url, method, timeout, False)(*args, **kwargs)
                    else:
                        raise
                except JsonRPCError as e:
                    if e.code >= ERROR_CODE_BASE - 1:
                        server.stop()
                        server.start()
                        return _method_obj(*args, **kwargs)
                    elif e.code == ERROR_CODE_BASE - 2 and self.handlers['on']:
                        # Not Found
                        try:
                            self.handlers['on'] = False
                            # any handler returns True
                            # will break the left handlers
                            any(handler(self.handlers.get('device', None))
                                for handler in self.handlers['handlers'])
                        finally:
                            self.handlers['on'] = True
                        return _method_obj(*args, **kwargs)
                    raise
            return wrapper

        return JsonRPCClient(self.rpc_uri,
                             timeout=timeout,
                             method_class=_JsonRPCMethod)

    def __jsonrpc(self):
        return JsonRPCClient(self.rpc_uri,
                             timeout=int(os.environ.get("JSONRPC_TIMEOUT", 90)))

    def start(self, timeout=5):
        self.app.startservice()
        self.adb.forward(self.local_port, self.device_port)

        while not self.alive and timeout > 0:
            time.sleep(0.1)
            timeout -= 0.1
        if not self.alive:
            raise IOError("RPC server not started!")

    def ping(self):
        try:
            return self.__jsonrpc().ping()
        except:
            return None

    @property
    def alive(self):
        '''Check if the rpc server is alive.'''
        return self.ping() == "pong"

    def stop(self):
        '''Stop the rpc server.'''
        res = None
        try:
            res = urllib2.urlopen(self.stop_uri)
        except:
            pass
        finally:
            if res is not None:
                res.close()
        try:
            self.app.startservice("stop")
            self.app.forcestop()
        except:
            pass

    @property
    def stop_uri(self):
        return "http://%s:%d/stop" % (self.adb.adb_server_host, self.local_port)

    @property
    def rpc_uri(self):
        return "http://%s:%d/jsonrpc/0" % (self.adb.adb_server_host,
                                           self.local_port)


class AudioStubApp(object):
    package_name = "com.intel.tests.audiostub"
    service_name = "multimedia_audio.MusicService"
    remote_apk_path = "audiostub.apk"

    def __init__(self, device=None):
        # FIXME: InstrumentatedBaseImpl not support multi-device
        self.instr = InstrumentedBaseImpl()
        self.instr.intialize()
        self.instr.test_pkg = self.package_name
        if not device:
            self.adb = g_common_obj.get_test_device()
        else:
            self.adb = device

    def install(self):
        apk_path = resource.get_app(self.remote_apk_path)
        self.adb.adb_cmd_common("install -r %s" % apk_path)

    def uninstall(self):
        self.adb.adb_cmd("pm uninstall %s" % self.package_name)

    def startservice(self, action=""):
        cmd = "am startservice -n %s/%s" % (self.package_name,
                                            self.service_name)
        if action:
            cmd += " -a %s" % action
        self.adb.adb_cmd(cmd)

    def forcestop(self):
        self.adb.adb_cmd("am force-stop %s" % self.package_name)

    def instr_run(self, testcase):
        self.instr.instr_run(testcase)


class AudioStubRPC(object):
    def __init__(self, device=None):
        if not device:
            device = g_common_obj.get_test_device()
        self.server = AutomatorServer(serial=device.serial,
                                      adb_server_host=device.adb_server_host,
                                      adb_server_port=device.adb_server_port,
                                      app=AudioStubApp(device))

    def start(self):
        ''' start RPC server '''
        self.server.start()

    def stop(self):
        ''' stop RPC server '''
        self.server.stop()

    def __getattr__(self, attr):
        ''' wrapper for jsonrpc methods '''
        return self.__dict__.get(attr) or getattr(self.server.jsonrpc, attr)


class MusicWidget(object):
    pkgname = "com.google.android.music"

    def __init__(self):
        self.d = g_common_obj.get_device()

    def add(self):  # noqa
        ''' add widget '''
        name = "Google Play Music"

        self.d.press.home()
        self.d.press.menu()
        time.sleep(3)
        if self.d(text="Widgets").exists:
            self.d(text="Widgets").click.wait()
        # Text change for N dessert.
        elif self.d(text="WIDGETS").exists:
            self.d(text="WIDGETS").click.wait()
        else:
            self.d.press.home()
            # enter Widgets list
            self.d(description="Apps").click.wait()
            self.d(text="Widgets").click.wait()
        # find the widget
        if self.d.exists(descriptionContains="Widgets page",
                         className="android.widget.LinearLayout"):
            mode = 'fling'
        else:
            mode = 'scroll'

        def move_next(pos=None):
            if mode == 'fling':
                self.d(scrollable=True).fling.horiz.forward()
            elif mode == 'scroll':
                self.d(scrollable=True).scroll()

        def find_widget_size():
            sizes = ["3 × 1", "2 × 1", "4 × 1"]
            if mode == 'scroll':
                self.d(scrollable=True).scroll.toBeginning()
            else:
                self.d(scrollable=True).fling.horiz.toBeginning()
            for _ in xrange(15):
                if self.d(text=name).exists:
                    for s in sizes:
                        if self.d(text=name).sibling(text=s).exists:
                            return s
                move_next()

        size = find_widget_size()
        # drag the widget, and add to the screen
        w = self.d(text=name) \
                .sibling(text=size) \
                .up(className="android.widget.ImageView")
        if (not w) or (not w.exists):  # image is down side
            w = self.d(text=name) \
                    .sibling(text=size)

        screen_width = int(self.d.info.get('displayWidth'))
        screen_height = int(self.d.info.get('displayHeight'))
        w.drag.to(screen_width/10, screen_height/2)  # drag to left-most screen

    def remove(self):
        ''' remove widget '''
        try:
            self._scroll2widget()
            # remove should in the top middle of screen
            screen_width = int(self.d.info.get('displayWidth'))
            w = self._get_widget()
            android_version = os.popen("adb shell getprop ro.build.version.release").readline()[0]
            if int(android_version) > 6:  # Remove button location is different between M and N
                w.drag.to(screen_width/4, 77, steps=50)
            else:
                w.drag.to(screen_width/2, 77, steps=50)
        except:
            pass

    def play(self):
        self._click_play_pause_btn()

    def pause(self):
        self._click_play_pause_btn()

    def _click_play_pause_btn(self):
        # it's wired that uiautomator can't get the right desc
        # even if the UI has changed, so just click play/pause btn
        self._scroll2widget()
        if self.d(description="Pause").exists:
            self.d(description="Pause").click.wait()
        elif self.d(description="Play").exists:
            self.d(description="Play").click.wait()

    def prev(self, loop = 1):
        self._scroll2widget()
        prev_btn = self.d(description="Previous")
        for i in range(loop):
            prev_btn.click()

    def next(self, loop = 1):
        self._scroll2widget()
        next_btn = self.d(description="Next")
        for i in range(loop):
            next_btn.click()

    def get_track_name(self):
        ''' get current track name from the widget '''
        w = self.get_widget()
        textview = w.child(resourceId="com.google.android.music:id/trackname")
        return textview.text

    def get_widget(self):
        ''' get the uiautomator widget object '''
        self._scroll2widget()
        return self._get_widget()

    def _get_widget(self):
        return self.d(packageName="com.android.launcher3",
                      description="Google Play Music")

    def _scroll2widget(self):
        '''
        scroll to music widget, if success, return True, raise Exception
        '''
        if not self.d(description="Apps").exists:  # if not in home screen
            self.d.press.home()
        for _ in range(20):
            if self._get_widget().exists:
                return True
            if self.d(scrollable=True).exists:
                self.d(scrollable=True).scroll.horiz()
        else:
            raise Exception("No widget found, add widget fails?")
