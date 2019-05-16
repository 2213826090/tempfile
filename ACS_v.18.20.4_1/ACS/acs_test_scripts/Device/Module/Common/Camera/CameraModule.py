#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
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
@summary: Camera module to expose camera properties
@since: 16/01/2015
@author: vgomberx
"""
from Device.Module.DeviceModuleBase import DeviceModuleBase
from acs_test_scripts.Device.Module.Common.Camera.ICameraModule import ICameraModule
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.AttributeDict import AttributeDict
from UtilitiesFWK.Utilities import Global
from lxml import etree


class CameraModule(ICameraModule, DeviceModuleBase):
    def __init__(self):
        super(CameraModule, self).__init__()
        self._camera_properties = AttributeDict()

    @property
    def camera_properties(self):
        return self._camera_properties

    def init(self):
        """
        Initialize em module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: init status
        """
        verdict = Global.SUCCESS
        # for now : em_properties = module config
        # but we can imagine a generic em properties interface between windows/android/linux
        # module would be os specific but keys inside em_properties would be generic
        self._camera_properties = self.configuration
        self.orig_suffix = ".orig"
        return verdict

    def get_camera(self, camera, raise_error=True):
        """
        check and return the camera name if this one exist on configuration file

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :rtype: str
        :return: None if no camera is found , the camera apps name otherwise
        """
        result = None
        if camera == "default":
            self.logger.debug("searching for default camera")
            camera = self.camera_properties.default_camera_package

        if self.camera_properties.camera_package.get(camera) is None:
            if raise_error :
                error = "unknown camera {0} package from camera module configuration file".format(camera)
                self.logger.error(error)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)
        else:
            result = camera

        return result

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
        quality = str(back_quality).upper()
        error = ""
        camera = self.get_camera(camera)
        cam_settings = None
        global_pref_settings = self.camera_properties.get("prefs_setting_option_names")
        if global_pref_settings is None:
            error = "'prefs_setting_option_names' value does not exist on your camera module configuration file for this DUT"
        else:
            cam_settings = global_pref_settings.get(camera)
            if cam_settings is None:
                error = "'prefs_setting_option_names' does not have a declared entry for {0}".format(camera)

        if error != "":
            self.logger.error(error)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)

        # if we reach here it means that we have settings declared for this camera
        # TODO: so far I dont handle specific camera setting edition, so we will move both back_quality

        modification_done = False
        # write it
        camParser = etree.XMLParser(remove_blank_text=True)
        xml_pref = etree.parse(settings_path, parser=camParser)
        root_node = xml_pref.getroot()
        # edit back_quality
        if back_quality is not None:
            if self._edit_quality(quality, camera, cam_settings, root_node, "QUALITY_BACK_CAMERA") == True:
                modification_done = True

        # edit skip_wizard
        if wizard_skip is not None:
            if self._edit_skip_wizard(wizard_skip, camera, cam_settings, root_node) == True:
                modification_done = True

        if modification_done:
            docinfo = xml_pref.docinfo
            self.__indent(root_node)
            with open(settings_path, "w") as xml_file:
                xml_pref.write(xml_file, pretty_print=False,
                            encoding=docinfo.encoding, standalone=docinfo.standalone, xml_declaration=True)

        return modification_done

    def __indent(self, elem, level=0):
        """
        indent the xml node and add a carriage return
        to keep the same format as the one from camera pref setting
        """
        i = "\n" + level * "    "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "    "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                self.__indent(elem, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    def _edit_settings(self, etree_root, setting_dict, new_value):
        """
        edit general settings

        :rtype: boolean
        :return: True if a modification was done, False otherwise
        """
        modification_done = False
        element_found = False
        for element in etree_root.findall(setting_dict["option_type"]):
            # search option by name
            if element.get("name") == setting_dict["option_name"]:
                # if the option is already set than ignore the change
                do_we_edit = False
                if element.text is not None:
                    if str(element.text).upper() != str(new_value).upper():
                        do_we_edit = True
                else:
                    do_we_edit = True
                # else do the job
                if do_we_edit:
                    element.text = str(new_value)
                    modification_done = True
                element_found = True
                break

        # if no element found then we add it
        if not element_found:
            new_ele = etree.Element(setting_dict["option_type"], name=setting_dict["option_name"])
            new_ele.text = new_value
            etree_root.append(new_ele)
            modification_done = True

        return modification_done

    def _edit_wizard_custom_settings(self, etree_root, wizard_extra_dict):
        """
        edit custom settings
        the setting dict should be a dictionary with a key: [ type, value_to_set] structure
        
        :rtype: boolean
        :return: True if a modification was done, False otherwise
        """
        modification_done = False
        for key in wizard_extra_dict.keys():
            node_type, value_to_set = wizard_extra_dict.get(key)
            value_to_set = str(value_to_set)
            element_found = False
            do_we_edit = False
            for element in etree_root.findall(node_type):
                # search option by name
                if element.get("name") == key:
                    # if the option is already set then ignore the change
                    do_we_edit = False
                    if element.text is not None:
                        if str(element.text).upper() != str(value_to_set).upper():
                            do_we_edit = True
                    else:
                        do_we_edit = True
                    # else do the job
                    if do_we_edit:
                        element.text = str(value_to_set)
                        modification_done = True
                    element_found = True
                    break

            # if no element found then we add it
            if not element_found:
                new_ele = etree.Element(node_type, name=key)
                new_ele.text = value_to_set
                etree_root.append(new_ele)
                modification_done = True

        return modification_done

    def _edit_quality(self, quality, camera, cam_settings, root_node, camera_side):
        """
        edit quality

        :rtype: boolean
        :return: True if a modification was done, False otherwise
        """
        # parse all quality option for this board
        modification_done = False
        quality_list = None

        if quality == "MAX":
            quality_list = ["1080P", "720P", "480P"]
        elif quality == "LOW":
            quality_list = ["480P", "720P", "1080P"]

        # I dont handle dictionary key missing error , let's python crash instead
        cam_element = cam_settings.get(camera_side)
        if cam_element is not None:
            new_value = None
            if quality_list is not None:
                for possible_quality in quality_list:
                    if cam_element.get("option_value").get(possible_quality) is not None:
                        new_value = cam_element.get("option_value").get(possible_quality)
                        break
            else:
                # get the quality from the config file
                new_value = cam_element["option_value"].get(quality)

            if new_value is None:
                error = " camera %s %s setting option does not contains value %s if camera module file" % (camera, cam_element.get("option_name"), quality)
                self.logger.error(error)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)

            modification_done = self._edit_settings(root_node, cam_element, new_value)
        else:
            self.logger.warning("fail to get configuration info from prefs_setting_option_names.%s module for %s" % (camera, camera_side))

        return modification_done

    def _edit_skip_wizard(self, wizard_value, camera, cam_settings, root_node):
        """
        edit skip wizard when camera is launch

        :rtype: boolean
        :return: True if a modification was done, False otherwise
        """
        # parse all quality option for this board
        modification_done = False
        # I dont handle dictionary key missing error, let's python crash instead
        wizard_setting_name = "SKIP_WIZARD"
        wizard_extra = "SKIP_WIZARD_EXTRA_SETTING"
        # main cam settings
        cam_element = cam_settings.get(wizard_setting_name)
        if cam_element is not None:
            # get the skip wizard from the config file
            new_value = cam_element["option_value"].get(wizard_value)
            if new_value is None:
                error = " camera %s %s setting option does not contains value %s in camera module file" % (camera, cam_element.get("option_name"), wizard_value)
                self.logger.error(error)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)

            if self._edit_settings(root_node, cam_element, new_value) == True:
                modification_done = True
            # edit extra settings
            extra_cam_element = cam_settings.get(wizard_extra)
            if extra_cam_element is not None:
                modification_done = self._edit_wizard_custom_settings(root_node, extra_cam_element)
        else:
            self.logger.warning("fail to get configuration info from prefs_setting_option_names.%s module for %s" % (camera, wizard_setting_name))
        return modification_done
