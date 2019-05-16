#!/usr/bin/env python

########################################################################
#
# @filename:    relay_step.py
# @description: relay test step
# @author:      aurel.constantin@intel.com
#
########################################################################

from testlib.base.base_step import step as base_step
from testlib.utils.relay import Relayed_device


class relay_step(base_step):
    '''helper class for all relay test steps'''

    def __init__(self, **kwargs):
        base_step.__init__(self, **kwargs)
        self.relay = Relayed_device(# "RLY08B" or "URMC32"
                                    relay_type = kwargs["relay_type"],
                                    # COM port used for communication. Ex: "/dev/ttyACM0"
                                    relay_port = kwargs["relay_port"] if "relay_port" in kwargs else None,
                                    # number of the relay connected to POWER button
                                    power_port = kwargs["power_port"] if "power_port" in kwargs else None,
                                    # number of the relay connected to VOL UP button
                                    v_up_port = kwargs["v_up_port"] if "v_up_port" in kwargs else None,
                                    # number of the relay connected to VOL DOWN button
                                    v_down_port = kwargs["v_down_port"] if "v_down_port" in kwargs else None,
                                    # number of the relay used to cut VC+ of the USB cable connected to the device
                                    USB_VC_cut_port = kwargs["USB_VC_cut_port"] if "USB_VC_cut_port" in kwargs else None)
