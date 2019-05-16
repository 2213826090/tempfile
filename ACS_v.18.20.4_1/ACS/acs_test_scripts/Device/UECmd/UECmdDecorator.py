"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This module contains utilities for Ue Commands classes
:author: kturban
"""

from ErrorHandling.DeviceException import DeviceException


def need(capability=None, critical=True, warning_return=None):
    """
    python decorator to ensure that an ue cmd can be run on the device

    This decorator can be used on:
        ue command method
        ue command whole class

    this decorator must handle both cases.
    each case has a different way to retrieve the device instance:

    class ueCmdClass(Base):
        # decorator will be applied to the whole class
        # the class can't instantiate if the device does not have the capability
        # to get the device, we need to get the 2nd argument 'device' to do the check
        @need('something')
        def __init__(self, device)
            self._device = device
            # do something else

        # decorator will be only applied on this method
        # to get the device, we need to get the 1st argument 'self' because the class has an attribute _device
        @need('something_else')
        def method(self):
            # do something

    :param capability: a capability necessary to run ue cmd
    :type capability: string

    :param critical: if set to True, an exception will be raised. Otherwise, just a warning logging message will be displayed
    :type critical: bool

    :param warning_return: value to return in case of warning

    :return a method call if possible
    """
    def needed_decorator(method):
        def check_capability(*args, **kwargs):
            # if used on ue cmd instance (or any class which have a _device attribute
            # we need to get the 1st argument on method call
            # see global comment for more details
            error_msg = "Device does not have {0} capability".format(capability)

            ue_cmd_obj = args[0]
            try:
                # TO DO:
                # add a device property on ue cmd classes to get the device
                # in order to make this decorator more generic (any class with device property attribute will be able to use this decorator)
                device = ue_cmd_obj._device
            except AttributeError:
                # if used on ue cmd class we need to get the 2nd argument on __init__ call
                # see global comment for more details
                device = args[1]
            ue_cmd_can_be_run = device.is_capable(capability)

            if ue_cmd_can_be_run:
                # call the method if capability is present
                return method(*args, **kwargs)
            # if missing capability but decorator option is set to not critical
            # method will not be executed but a warning message will be displayed
            elif not critical:
                device.logger.warning(error_msg)
                return warning_return
            # missing capability and decorator option is set as critical
            # method will not be executed and a device exception is raised
            else:
                raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, error_msg)
        # in order to keep calling method data:
        # - method documentation
        # - method name
        # - method dict
        check_capability.__name__ = method.__name__
        check_capability.__doc__ = method.__doc__
        check_capability.__dict__.update(method.__dict__)
        return check_capability
    # call the method below the decorator if checks done by needed_decorator method are ok
    return needed_decorator


def atproxy(method):
    """
    On most Android Intel DUT we need to activate the AT proxy to send AT command.
    But on Sofia devices, no need to activate the AT proxy, just retrieve from device configuration the AT cmd port
    This decorator check if the device need to activate the AT proxy:
    If yes => the method to open the AT proxy is executed and the AT cmd port opened is returned
    else => the AT cmd port is retrieved form device config and returned

    This decorator can be used on:
        ue command method

    :rtype: str
    :return: the AT cmd port
    """
    def atproxy_decorator(*args, **kwargs):
        # first argument of decorated method is the calling object
        ue_cmd_obj = args[0]
        device = ue_cmd_obj._device

        at_proxy_supported = device.get_config("isATProxySupported", "True", "str_to_bool")
        if at_proxy_supported:
            # call the method if capability is present
            return method(*args, **kwargs)
        else:
            return device.get_config("ATCmdPort", "COM1")

    return atproxy_decorator
