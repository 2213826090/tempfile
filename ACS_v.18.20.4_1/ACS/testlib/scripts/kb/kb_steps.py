#!/usr/bin/env python

#######################################################################
#
# @filename:    kb_steps.py
# @description: Local test steps
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.base.base_step import step as base_step
from testlib.utils.relay import Relayed_device
from testlib.scripts.connections.local import local_steps
from testlib.base.base_step import FailedError

import time

class perform_startup_wizard(base_step):

    def __init__(self, device, platform, version = None, **kwargs):
        self.relay = Relayed_device(kb_relay_port = device['kb']['relay_tty'],
                                    kb_port = device['kb']['kb_tty']
                                    )
        self.kb_device_port = device['kb']['port']
        self.platform = platform
        self.version = version
        base_step.__init__(self, **kwargs)

    def do(self):
        print "Trying to activate kb, platform - ", str(self.platform)
        time.sleep(10)
        self.relay.try_activate_kb(self.kb_device_port)
        time.sleep(5)
        print "Should be activated"

        # welcome screen
        if self.platform == "ECS28A":
            keys = ['<enter>', '<tab>', '<enter>']
        else:
            keys = ['<enter>', '<tab>', '<enter>']
        # wifi
        keys += ['<tab>', '<tab>', '<enter>', '<tab>', '<enter>']
        # date&time
        keys += ['<tab>', '<tab>', '<enter>']
        # name
        keys += ['<tab>', '<tab>', '<tab>', '<tab>', '<enter>']
        # protect your phone
        keys += ['<tab>', '<tab>', '<tab>', '<tab>', '<enter>', '<tab>', '<enter>']
        # google services
        if self.platform == "ECS28A":
            keys += ['<tab>', '<tab>', '<tab>', '<tab>', '<tab>', '<enter>', '<tab>', '<tab>', '<enter>']
        else:
            keys += ['<tab>', '<tab>', '<tab>', '<tab>', '<tab>', '<tab>', '<enter>', '<tab>', '<tab>', '<enter>']
        # allow
        keys += ['<tab>', '<enter>']
        for key in keys:
            print key
            self.relay.send_to_kb(key)
            time.sleep(2)
        time.sleep(2)
        # got it
        self.relay.send_to_kb('<enter>')

        self.relay.try_deactivate_kb()
        self.relay.close()

    def check_condition(self):
        return True


class first_boot_no_wizard(base_step):

    def __init__(self, device, **kwargs):
        self.relay = Relayed_device(kb_relay_port = device['kb']['relay_tty'],
                                    kb_port = device['kb']['kb_tty']
                                    )
        self.kb_device_port = device['kb']['port']
        base_step.__init__(self, **kwargs)

    def do(self):
        self.relay.try_activate_kb(self.kb_device_port)

        # allow
        keys += ['<tab>', '<enter>']
        # got it
        keys += ['<enter>', '<enter>']
        for key in keys:
            self.relay.send_to_kb(key)
            time.sleep(0.5)
        self.relay.try_deactivate_kb()
        self.relay.close()

    def check_condition(self):
        return True


class enable_usb_debugging(base_step):

    def __init__(self, device, platform = "ECS27B", **kwargs):
        self.relay = Relayed_device(kb_relay_port = device['kb']['relay_tty'],
                                    kb_port = device['kb']['kb_tty']
                                    )
        self.touch_relay = Relayed_device(relay_port = device["relay"]["tty"],
                                power_port = device["relay"]["power_port"],
                                v_up_port = device["relay"]["v_up_port"],
                                v_down_port = device["relay"]["v_down_port"],
                                always_allow_port = device["relay"]["always_allow_port"],
                                allow_ok_port = device["relay"]["allow_ok_port"],
                                )
        self.kb_device_port = device['kb']['port']
        self.device = device
        self.allowed = False
        self.platform = platform
        base_step.__init__(self, **kwargs)

    def do(self):
        print self.platform
        time.sleep(5)
        self.relay.try_activate_kb(self.kb_device_port)
        time.sleep(5)
        if self.platform == "ECS28A":
            # about app button
            keys = ['<tab>', '<tab>', '<tab>', '<tab>', '<tab>', '<tab>', '<tab>', '<tab>', '<enter>']
            # setting
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<rarrow>', '<rarrow>']
            keys += ['<rarrow>',  '<rarrow>', '<rarrow>', '<rarrow>', '<rarrow>', '<enter>']
            # about phone
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>']
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<enter>']
            # enable dev_options
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>']
            keys += ['<enter>', '<enter>', '<enter>', '<enter>', '<enter>', '<enter>', '<enter>', '<esc>']
            # dev options
            keys += ['<uparrow>', '<larrow>', '<enter>']
            # enable_usb_debugging
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>']
            keys += ['<downarrow>', '<downarrow>', 'downarrow']
            keys += ['<enter>', '<tab>', '<enter>']
            # home
            keys += ['<esc>', '<esc>', '<esc>']
        else:
            # all app button
            keys = ['<uparrow>', '<uparrow>', '<downarrow>', '<downarrow>', '<enter>']
            # setting
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<enter>']
            # about phone
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>']
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<enter>']
            # enable dev_options
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>']
            keys += ['<enter>', '<enter>', '<enter>', '<enter>', '<enter>', '<enter>', '<enter>', '<esc>']
            # dev options
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>']
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<enter>']
            # enable_usb_debugging
            keys += ['<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>', '<downarrow>']
            keys += ['<downarrow>', '<downarrow>']
            keys += ['<enter>', '<tab>', '<enter>']
            # home
            keys += ['<esc>', '<esc>', '<esc>']

        for key in keys:
            print key
            self.relay.send_to_kb(key)
            time.sleep(1)
        self.relay.try_deactivate_kb()
        time.sleep(5)

        # Allow USB debugging -- checkbox + OK
        # Trying 5 times and then exit with error
        for t in range(5):
            try:
                print "Try to check always allow and touch OK - ", str(t+1)
                self.touch_relay.allow_usb_debugging()
                time.sleep(5)
                self.relay.try_activate_kb(self.kb_device_port)
                time.sleep(5)
                self.relay.try_deactivate_kb()
                time.sleep(5)
                local_steps.wait_for_adb(serial = self.device["serial"],
                                     timeout = 10)()
                print "Success"
                self.allowed = True
                break
            except FailedError:
                print "Failed"
                pass

        self.relay.close()
        self.touch_relay.close()

    def check_condition(self):
        return self.allowed
