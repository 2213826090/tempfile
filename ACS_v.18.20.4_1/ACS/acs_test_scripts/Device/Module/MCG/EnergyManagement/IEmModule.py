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

@organization: INTEL MCG PSI
@summary: This module defines em module interface
@since: 11/8/2014
@author: kturban
"""
import abc


class IEmModule(object):

    """
    Energy Management interface
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractproperty
    def em_properties(self):
        """
        Return em properties
        """

    @abc.abstractmethod
    def init(self):
        """
        Initialize em module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: init status
        """

    def parse_msic_response_from_shell(self, output, log_output=True):
        """
        Parses the response gotten from the get shell msic register
        in order to extract the following parameters:
        The function return at most 2 dictionaries (for battery and charger info)
        Each object in the returned dictionary is a dictionary build from the "tag=value" parsed line format

        :type output: str
        :param output: output result from msic registers embd uecmd

        :type  log_output: boolean
        :param log_output: allow output logging , used to avoid spaming log on autolog

        :rtype: dict
        :return: a dictionary that contains the msic battery and charger info
        """
