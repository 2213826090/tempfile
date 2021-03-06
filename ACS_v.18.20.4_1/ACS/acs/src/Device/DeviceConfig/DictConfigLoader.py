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
@summary: This module implements Sphinx Auto-generator of Documentation
@since: 4/3/14
@author: sfusilie
"""
from UtilitiesFWK.AttributeDict import AttributeDict


class DictConfigLoader(object):

    """
    Create one config loader that will take param from current
    global config dict. We'll have to create a new config loader
    based on xml files when ready
    """

    @staticmethod
    def load(config):
        """
        Load global device conf to the device conf objects

        :type config: dict
        :param config: device global conf dictionary
        """
        device_conf = AttributeDict()
        for key, value in config.items():
            device_conf[key] = value

        return device_conf
