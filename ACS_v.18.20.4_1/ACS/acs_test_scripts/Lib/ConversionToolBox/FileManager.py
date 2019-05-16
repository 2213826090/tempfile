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
:summary:
:since: 18/11/2010
:author: dgonzalez

"""
from xml.dom import minidom
import xpath

from Lib.ConversionToolBox.DataManagement import FrequencyRangeMgmt, FrequencySetMgmt

class ConversionFileManager:

    """
        ConversionFileManager: class retrieving information from:
            - wcdma_conversion.xml
            - gsm_conversion.xml
            - wifi_conversion.xml
    """

    def __init__(self,
                 wcdma_filename,
                 gsm_filename,
                 wifi_filename,
                 directory='./'
                 ):
        """
        Constructor of the class.

        :type wcdma_filename: str
        :param wcdma_filename: Name of the wcdma conversion's xml file.

        :type gsm_filename: str
        :param gsm_filename: Name of the gsm conversion's xml file.

        :type wifi_filename: str
        :param wifi_filename: Name of the wifi conversion's xml file.

        :type directory: str
        :param directory: Files' path.
        """
        self.__directory = directory
        self.__wcdma_filename = wcdma_filename
        self.__gsm_filename = gsm_filename
        self.__wifi_filename = wifi_filename

    def __get_attribute_value(self,
                              dom_node,
                              attribute_name):
        """
        Returns the value of the given attribute for the given I{DOM} C{Node}.

        :type dom_element: Node
        :param dom_element: the I{DOM} C{Node} among which the attribute's value
            will be looked for.

        :type attribute_name: str
        :param attribute_name: the name of the requested attribute

        :rtype: str
        :return:
            - the requested attribute value if it exists
            - C{None} otherwise
        """
        value = None
        attributes = dom_node.attributes
        for index in range(attributes.length):
            if attributes.item(index).name == attribute_name:
                value = attributes.item(index).value
                break
        return value

    def parse_wcdma_conversion(self):
        """
        Returns retrieved information from wcdma conversion file.

        :rtype: dict
        :return: Dictionary containing wcdma conversion's data.
        """
        try:
            document = minidom.parse(self.__directory +
                                     self.__wcdma_filename)
            dictionnary = {}
            band_node_list = xpath.find('/wcdma_conversion/band', document)
            for band_node in band_node_list:
                band_name = self.__get_attribute_value(band_node, "name")
                dictionnary[band_name] = []
                freq_node_list = xpath.find('frequencies', band_node)
                for freq_node in freq_node_list:
                    freq_value = self.__get_attribute_value(freq_node, "ul_input_channel")
                    ul_formula = self.__get_attribute_value(
                        xpath.find('uplink_formula', freq_node)[0],
                        "value")
                    dl_formula = self.__get_attribute_value(
                        xpath.find('downlink_formula', freq_node)[0],
                        "value")
                    freq_type = self.__get_attribute_value(freq_node, "type")
                    if freq_type == "range":
                        freq_mgmt = FrequencyRangeMgmt(freq_value,
                                                       ul_formula,
                                                       dl_formula)
                        dictionnary[band_name].append(freq_mgmt)
                    elif freq_type == "list":
                        freq_mgmt = FrequencySetMgmt(freq_value,
                                                     ul_formula,
                                                     dl_formula)
                        dictionnary[band_name].append(freq_mgmt)
                    else:
                        raise Exception("Unknown frequencies type value.")
        except Exception as exc:  # pylint: disable=W0703
            raise exc
        return dictionnary

    def parse_gsm_conversion(self):
        """
        Returns retrieved information from gsm conversion file.

        :rtype: dict
        :return: Dictionary containing gsm conversion's data.
        """
        try:
            document = minidom.parse(self.__directory +
                                     self.__gsm_filename)
            dictionnary = {}
            band_node_list = xpath.find('/gsm_conversion/band', document)
            for band_node in band_node_list:
                band_name = self.__get_attribute_value(band_node, "name")
                dictionnary[band_name] = []
                freq_node_list = xpath.find('frequencies', band_node)
                for freq_node in freq_node_list:
                    freq_value = self.__get_attribute_value(freq_node, "input_channel")
                    ul_formula = self.__get_attribute_value(
                        xpath.find('uplink_formula', freq_node)[0],
                        "value")
                    dl_formula = self.__get_attribute_value(
                        xpath.find('downlink_formula', freq_node)[0],
                        "value")
                    freq_type = self.__get_attribute_value(freq_node, "type")
                    if freq_type == "range":
                        freq_mgmt = FrequencyRangeMgmt(freq_value,
                                                       ul_formula,
                                                       dl_formula)
                        dictionnary[band_name].append(freq_mgmt)
                    elif freq_type == "list":
                        freq_mgmt = FrequencySetMgmt(freq_value,
                                                     ul_formula,
                                                     dl_formula)
                        dictionnary[band_name].append(freq_mgmt)
                    else:
                        raise Exception("Unknown frequencies type value.")
        except Exception as exc:  # pylint: disable=W0703
            raise exc
        return dictionnary

    def parse_wifi_conversion(self):
        """
        Returns retrieved information from wifi conversion file.

        :rtype: dict
        :return: Dictionary containing wifi conversion's data.
        """
        try:
            document = minidom.parse(self.__directory +
                                     self.__wifi_filename)
            dictionnary2_4ghz = {}
            band_node_list = xpath.find('/wifi_conversion/band[@name="2.4GHZ"]/channel',
                                        document)
            for band_node in band_node_list:
                band_name = self.__get_attribute_value(band_node, "name")
                band_mid_freq = self.__get_attribute_value(band_node, "mid_range_frequency")
                dictionnary2_4ghz[band_name] = float(band_mid_freq)
            dictionnary5ghz = {}
            band_node_list = xpath.find('/wifi_conversion/band[@name="5.0GHZ"]/channel',
                                        document)
            for band_node in band_node_list:
                band_name = self.__get_attribute_value(band_node, "name")
                band_mid_freq = self.__get_attribute_value(band_node, "mid_range_frequency")
                dictionnary5ghz[band_name] = float(band_mid_freq)
            dictionnary = {}
            dictionnary["2.4GHZ"] = dictionnary2_4ghz
            dictionnary["5.0GHZ"] = dictionnary5ghz

        except Exception as exc:  # pylint: disable=W0703
            raise exc
        return dictionnary
