#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG AMPS
@summary: This module defines crash module interface
@since: 7/1/14
@author: cbonnard
"""
import abc


class ICrashModule(object):

    """
    Crash module interface
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def init(self, device_serial_number, cache_folder):
        """
        Initialize crash module

        :type device_serial_number: str
        :param device_serial_number:  ssn of the device
        :type cache_folder: str
        :param cache_folder:  cache folder path

        :rtype: bool
        :return: initialization status
        """

    @abc.abstractmethod
    def list(self):
        """
        List crash events available on device

        :rtype: list
        :return: list of crash events
        """

    @abc.abstractmethod
    def fetch(self, event_id=None, verdict=None):
        """
        Fetch event logs from the device, if any and retrieve them on local host

        :param verdict: Verdict of the test case

        :type event_id: str
        :param event_id: optional identifier of event

        :rtype: bool
        :return: fetch status
        """

    @abc.abstractmethod
    def upload(self, event_id):
        """
        Upload crashes from host to server

        :type event_id: str
        :param event_id: optional identifier of event

        :rtype: bool
        :return: upload status
        """

    @abc.abstractmethod
    def enable_clota(self):
        """
        Enable Crash Log Over the Air on device

        :rtype: bool
        :return: enabling status
        """

    @abc.abstractmethod
    def disable_clota(self):
        """
        Disable Crash Log Over the Air on device

        :rtype: bool
        :return: disabling status
        """

    @abc.abstractmethod
    def get_device_info(self):
        """
        Return device info dictionary

        :rtype: tuple status & dict
        :return: device information with its status
        """
