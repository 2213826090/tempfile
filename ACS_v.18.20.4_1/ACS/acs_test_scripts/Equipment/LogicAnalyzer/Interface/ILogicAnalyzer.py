"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the Interface for logic analyzers.
:since: 2014-11-14
:author: emarchan

"""
from ErrorHandling.TestEquipmentException import TestEquipmentException
# pylint: disable=W0613

class ILogicAnalyzer(object):

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases all resources allocated to equipment
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def connect(self):
        """
        Launches, the server and client and establish a connection between them.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_capture_to_mem(self):
        """
        Starts a capture (destination is memory).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_capture_to_file(self, dest_file):
        """
        Starts a capture (destination is file).

        :type dest_file: string
        :param dest_file: Destination file where the capture will be written.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_capture(self):
        """
        Stops the current capture.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)


    def export_raw_capture_to_csv_file(self, dest_file):
        """
        Exports the previously captured file in a RAW, unformatted CSV file.

        :type dest_file: string
        :param dest_file: CSV Destination file where the capture will be written.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def export_formatted_capture_to_file(self, dest_file, formatter_name):
        """
        Exports the previously captured file in a formatted format.

        :type dest_file: string
        :param dest_file: Destination file where the formatted capture will be written.

        :type formatter_name: string
        :param formatter_name: Name of the formatter that will parse the capture.

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_capture_rate(self, rate):
        """
        Sets the capture rate.

        :param rate: Capture rate
        :type rate: string
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_number_of_samples(self, number):
        """
        Sets the number of samples to capture.

        :param number: Number of samples to capture.
        :type number: string
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_capture_channels(self, capture_channels):
        """
        Sets the channels you want to capture the data from.

        :param capture_channels: List of channels to capture data from.
        :type capture_channels: List of string
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_capture_formatters(self):
        """
        Gets the available capture formatters

        :return: The available formatters
        :rtype: Dict Format {"name": "ID"}
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
