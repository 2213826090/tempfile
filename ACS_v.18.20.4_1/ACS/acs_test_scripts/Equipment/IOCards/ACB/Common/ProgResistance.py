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
:summary: communication tool with the programmable resistance module of Ariane Board
:author: dbatutx
:since: 12/11/2012
"""
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IOCards.ACB.Common import WUsbDio as W
import time


def select_resistance(resistance, feature, usb_dio):
    """
    This function select the resistance to program on the wanted feature
    :type resistance: int
    :param resistance: the value of the resistor in ohm
    :type feature: str
    :param feature: can be "bptherm" or "battid"
    :type usb_dio: An Ariane board equipment
    :param usb_dio: the equipment that uses the ProgResistance
    :rtype: none
    """

    usb_dio.get_logger().info("select a %s ohm resistor on %s" % (str(resistance), feature))
    # check the validity of the resistance
    if resistance > 400000 or resistance < 200:
        raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                            "the resistance should be between 200 and 400000 Ohm")
    # as the resistance is two serial resistance programmable
    # we have to compute the exact resistance value to each
    rdac0 = min(resistance, 250000)
    rdac1 = max(0, resistance - rdac0)
    # write the resistance
    write_resistance(rdac0, feature, 0, usb_dio)
    write_resistance(rdac1, feature, 1, usb_dio)
    # check the feature to enable
    if feature == "bptherm":
        W.Enable(usb_dio, usb_dio.LINES.bptherm_1m)  # ctrl07
        W.Enable(usb_dio, usb_dio.LINES.bptherm_connect)  # ctrl16
    else:
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE,
                            "the ProgResistance are available only for 'bptherm' and 'battid'")


def write_resistance(resistance, feature, rdac, usb_dio):
    """
    This function write the good resistance on the 1024-programmable resistors by spi
    :type resistance: str
    :type resistance: int
    :param resistance: the value of the resistor in ohm
    :param feature: can be "bptherm" or "battid"
    :type rdac: integer
    :param rdac: channel to select the resistor on the chip (0 or 1)
    :type usb_dio: An Ariane board equipment
    :param usb_dio: the equipment that uses the ProgResistance
    :return: none
    """
    res_bit = []
    i = n = 1
    # include all parasitic resistance
    offset = 50

    #
    # adapt the resistance value to the the 250 KOhm resistor
    # with a resolution of 1024
    #
    res_bcd = int(max((resistance - offset) * 1024 / 250000, 0))
    # check the resistor value
    if res_bcd > 1023:
        raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                            "the resistor resolution value %d is not good" % res_bcd)
    while i < 11:
        bit = (res_bcd & n) / n
        res_bit.append(bit)
        i += 1
        n *= 2
    #
    # write the address sequence with 'rdac'
    # then write the resistance value
    #
    # see ADN2850 datasheet for bit definition
    spi_start(feature, usb_dio)
    # commande 0xB0  to write registers
    spi_data(1, usb_dio)  # data1
    spi_data(0, usb_dio)  # data2
    spi_data(1, usb_dio)  # data3
    spi_data(1, usb_dio)  # data4
    spi_data(0, usb_dio)  # data5
    spi_data(0, usb_dio)  # data6
    spi_data(0, usb_dio)  # data7
    spi_data(rdac, usb_dio)  # data8
    # write write to resistance bits
    spi_data(0, usb_dio)  # data9
    spi_data(0, usb_dio)  # data10
    spi_data(0, usb_dio)  # data11
    spi_data(0, usb_dio)  # data12
    spi_data(0, usb_dio)  # data13
    spi_data(0, usb_dio)  # data14
    spi_data(res_bit[9], usb_dio)  # data15
    spi_data(res_bit[8], usb_dio)  # data16
    spi_data(res_bit[7], usb_dio)  # data17
    spi_data(res_bit[6], usb_dio)  # data18
    spi_data(res_bit[5], usb_dio)  # data19
    spi_data(res_bit[4], usb_dio)  # data20
    spi_data(res_bit[3], usb_dio)  # data21
    spi_data(res_bit[2], usb_dio)  # data22
    spi_data(res_bit[1], usb_dio)  # data23
    spi_data(res_bit[0], usb_dio)  # data24
    #
    spi_stop(feature, usb_dio)


def spi_start(feature, usb_dio):
    """
    This function generate a valid sequence in order to
    enable the spi communication
    :type feature: str
    :param feature: can be "bptherm" or "battid"
    :type usb_dio: An Ariane board equipment
    :param usb_dio: the equipment that uses the ProgResistance
    :return: none
    """
    # check the selected feature (bptherm or battid)
    if feature == "bptherm":
        W.Disable(usb_dio, usb_dio.LINES.prog_res_rdy, log=False)  # ctrl08
        W.Enable(usb_dio, usb_dio.LINES.prog_res_cs, log=False)  # ctrl23

        W.Enable(usb_dio, usb_dio.LINES.prog_res_rdy, log=False)  # ctrl08
        W.Enable(usb_dio, usb_dio.LINES.prog_res_cs, log=False)  # ctrl23

        W.Enable(usb_dio, usb_dio.LINES.prog_res_rdy, log=False)  # ctrl08
        W.Disable(usb_dio, usb_dio.LINES.prog_res_cs, log=False)  # ctrl23


def spi_stop(feature, usb_dio):
    """
    This function generate a invalid sequence in order to
    disable the spi communication
    :type feature: str
    :param feature: can be "bptherm" or "battid"
    :type usb_dio: An Ariane board equipment
    :param usb_dio: the equipment that uses the ProgResistance
    :return: none
    """
    # check the selected feature (bptherm or battid)
    if feature == "bptherm":
        W.Enable(usb_dio, usb_dio.LINES.prog_res_rdy, log=False)  # ctrl08
        W.Disable(usb_dio, usb_dio.LINES.prog_res_cs, log=False)  # ctrl23

        W.Enable(usb_dio, usb_dio.LINES.prog_res_rdy, log=False)  # ctrl08
        W.Enable(usb_dio, usb_dio.LINES.prog_res_cs, log=False)  # ctrl23

        W.Disable(usb_dio, usb_dio.LINES.prog_res_rdy, log=False)  # ctrl08
        W.Enable(usb_dio, usb_dio.LINES.prog_res_cs, log=False)  # ctrl23


def spi_data(data, usb_dio):
    """
    This function generate a valid clock during 5 half period
    and a data pulse of 3 half period
    :type data: integer
    :param data: this interger is the bit data (1 or 0)
    :type usb_dio: An Ariane board equipment
    :param usb_dio: the equipment that uses the ProgResistance
    :return: none
    """

    # 1 half period
    time.sleep(0.001)
    W.Disable(usb_dio, usb_dio.LINES.prog_res_sdi, log=False)  # ctrl25
    W.Disable(usb_dio, usb_dio.LINES.prog_res_clk, log=False)  # ctrl24

    # 2 half period
    time.sleep(0.001)
    if data == 1:
        W.Enable(usb_dio, usb_dio.LINES.prog_res_sdi, log=False)  # ctrl25
    elif data == 0:
        W.Disable(usb_dio, usb_dio.LINES.prog_res_sdi, log=False)  # ctrl25
    W.Disable(usb_dio, usb_dio.LINES.prog_res_clk, log=False)  # ctrl24

    # 3 half period
    time.sleep(0.001)
    if data == 1:
        W.Enable(usb_dio, usb_dio.LINES.prog_res_sdi, log=False)  # ctrl25
    elif data == 0:
        W.Disable(usb_dio, usb_dio.LINES.prog_res_sdi, log=False)  # ctrl25
    W.Enable(usb_dio, usb_dio.LINES.prog_res_clk, log=False)  # ctrl24

    # 4 half period
    time.sleep(0.001)
    if data == 1:
        W.Enable(usb_dio, usb_dio.LINES.prog_res_sdi, log=False)  # ctrl25
    elif data == 0:
        W.Disable(usb_dio, usb_dio.LINES.prog_res_sdi, log=False)  # ctrl25
    W.Disable(usb_dio, usb_dio.LINES.prog_res_clk, log=False)  # ctrl24

    # 5 half period
    time.sleep(0.001)
    W.Disable(usb_dio, usb_dio.LINES.prog_res_sdi, log=False)  # ctrl25
    W.Disable(usb_dio, usb_dio.LINES.prog_res_clk, log=False)  # ctrl24
