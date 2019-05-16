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
@summary: This module defines camera module interface
@since: 16/01/2015
@author: vgomberx
"""
import abc


class ICameraModule(object):

    """
    Camera interface
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractproperty
    def camera_properties(self):
        """
        Return camera properties
        """

    @abc.abstractmethod
    def init(self):
        """
        Initialize camera module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: init status
        """

    def get_camera(self, camera, raise_error=True):
        """
        check and return the camera name if this one exist on configuration file

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :rtype: str
        :return: None if no camera is found , the camera apps name otherwise
        """
        pass

    def edit_camera_setting(self, camera, settings_path, back_quality=None, wizard_skip=None):
        """
        edit the camera setting

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :type settings_path: str
        :param settings_path: the setting file path (on the host side) to edit before pushing it back to the DUT

        :type back_quality: str
        :param back_quality: the back camera quality to be choose between : 1080P, 720P, 480P, LOW (lowest supported) , MAX (max supported)

        :type skip_wizard: str
        :param skip_wizard: some camera got a wizard the first time you open ,this allow to skip it, can be ON or OFF

        :rtype: boolean
        :return: True if a modification was done, False otherwise
        """
        pass
