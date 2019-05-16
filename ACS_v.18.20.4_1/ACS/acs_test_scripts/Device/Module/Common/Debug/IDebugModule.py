#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This module defines debug module interface
@since: 9/5/14
@author: ahkhowaj
"""
import abc

class IDebugModule(object):
    """
    Debug module interface
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def init(self, device_serial_number):
        """
        :type device_serial_number: str
        :param device_serial_number:  ssn of the device
        init module values
        """

    @abc.abstractmethod
    def dump(self, verdict, dump_location):
        """
        Dump all soc debug information
        :type verdict: str
        :param verdict: test status
        :type dump_location: str
        :param dump_location: directory where data will be stored
        """

    @abc.abstractmethod
    def cleanup(self):
        """
        Stop logs collection, remove not requested debug logs
        """