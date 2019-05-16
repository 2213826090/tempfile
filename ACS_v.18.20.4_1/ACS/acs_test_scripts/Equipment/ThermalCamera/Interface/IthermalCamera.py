"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: interface for thermal camera implementation
:since: 08/01/2015
:author: vgombert
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IthermalCamera(object):

    """
    IthermalCamera class.
    """

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)


    def connect(self):
        """
        Establish the connection with the equipment
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_nuc(self):
        """
        Perform a non uniformity correction
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_auto_focus(self):
        """
        Perform an auto focus
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_palette(self, pal_color="default", reverse_pal=False, use_extrem=False):
        """
        Set camera palette

        :type pal_color: str
        :param pal_color: the palette to set: rainbow, iron, gray, or default to apply the other option on current palette

        :type reverse_pal: str
        :param reverse_pal: reverse palette

        :type use_extrem: str
        :param use_extrem: turn on extremes (above/below colors)

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_linear_temperature_mode(self, active, resolution_format=None):
        """
        Activate linear temperature mode
        this allow to get pixels values in Kelvin

        :type active: boolean
        :param active: True = Temperature linear mode
                       False = Raw mode (pixel value linearity)

        :type resolution_format: int
        :param resolution_format: 0 = 100 mK resolution (0 - 6535 K)
                                  1 = 10 mK resolution (0 - 653 K)
                                  leave empty to ignore this option

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_image_geometry(self):
        """
        get the image geometry.
        you can use this function to configure the measurement box to the max image resolution
        :rtype: tuple
        :return: x, y, height, width
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_measurement_box(self, active=None, x=None, y=None, box_height=None, box_width=None):
        """
        configure an area where to perform measurement.

        :type active: boolean
        :param active: True = Active measurement box , mandatory to read its value
                       False = deactivate it

        :type x: int
        :param x: the image x coordinate, 0 is the most left side

        :type y: int
        :param y: the image y coordinate, 0 is the most upper side

        :type box_height: int
        :param box_height: the image height, cannot be set outside the image zone or an error will be raised

        :type box_width: int
        :param box_width: the image width, cannot be set outside the image zone or an error will be raised
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_measurement_from_box(self):
        """
        read the temperature info in defined box

        :rtype: dict
        :return: return a several information seen in the box like average temperature ,max temperature , min temperature
                    each information is a tuple (value, unit)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def take_picture(self, file_name, wanted_format="RADIOMETRIC_JPEG", wanted_option=""):
        """
        Get a thermal picture.

        :type file_name: str
        :param file_name: the file name should not include an extension as this last one will be added depending of wanted_format value

        :type wanted_format: str
        :param wanted_format: the output file format, possible values are: RADIOMETRIC_FFF, RADIOMETRIC_JPEG,
                                                                            VISUAL_JPEG, VISUAL_FFF, SEQUENCE_FFF

        :type wanted_option: str
        :param wanted_option: the file option, possible values are: PNG, JPEG_ONLY (non-radiometric),
                                                                    NO_OVERLAY, CUT_IMAGE (with overlay to IR image size)

        :rtype: str
        :return: the path of the picture with host path separator
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def pull_picture(self, file_name, destination_folder):
        """
        pull a picture

        :type file_name: str
        :param file_name: the file name from the camera

        :type destination_folder: str
        :param destination_folder: the folder on the host side where picture will be stored

        :rtype: str
        :return: the pulled file path

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def pull_all_pictures(self, destination_folder):
        """
        pull all pictures

        :type destination_folder: str
        :param destination_folder: the folder on the host side where picture will be stored
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def delete_picture(self, file_name):
        """
        Delete a given picture

        :type file_name: str
        :param file_name: the file name from the camera
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def delete_all_pictures(self):
        """
        Delete all generated pictures
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
