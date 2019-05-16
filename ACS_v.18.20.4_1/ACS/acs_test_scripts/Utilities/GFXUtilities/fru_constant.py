"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA

provided: GATE Framework partial integration of testing features

:since: 5/26/14
:author: mmaracix
"""
import collections

# XPID and YDPI of MOFD with HDPI
MOFD_HDPI_XDPI = "472"
MOFD_HDPI_YDPI = "473"

MIPI_DISPLAY_FRU = collections.namedtuple('MIPI_DISPLAY_FRU', 'display_fru display_id product_prefix product_prefix_common width height description')

class mipi_fru_data_kk:

    PNC_JDI_7x12 = MIPI_DISPLAY_FRU('PNC_JDI_7x12', \
        '1', \
        'mofd_7x12_kk', '', \
        720, 1280, \
        'JDI 720 x 1280 command mode')
    PNC_SHARP_10x19_1 = MIPI_DISPLAY_FRU('PNC_SHARP_10x19_1', \
        '2', \
        'mofd_10x19-cmdmode_kk', 'mofd_10x19-common_kk', \
        1080, 1920, \
        'Sharp 1080 x 1920 command mode')
    PNV_SHARP_25x16 = MIPI_DISPLAY_FRU('PNV_SHARP_25x16', \
        '3', \
        'mofd_25x16-sharp-vm_kk', 'mofd_25x16-common_kk', \
        2560, 1600, \
        'Sharp 2560 x 1600 video mode')
    PNC_SHARP_10x19_2 = MIPI_DISPLAY_FRU('PNC_SHARP_10x19_2', \
        '4', \
        'mofd_10x19-cmdmode_kk', 'mofd_10x19-common_kk', \
        1080, 1920, \
        'Sharp 1080 x 1920 command mode')
    PNV_JDI_25x16 = MIPI_DISPLAY_FRU('PNV_JDI_25x16', \
        '5', \
        'mofd_25x16-jdi_kk', 'mofd_25x16-common_kk', \
        2560, 1600, \
        'JDI 2560 x 1600 video mode')
    PANEL_AUO = MIPI_DISPLAY_FRU('PANEL_AUO', \
        '6', \
        'mofd_7x12-asus_kk', '', \
        720, 1280, \
        'Asus display')
    PNC_SHARP_25x16 = MIPI_DISPLAY_FRU('PNC_SHARP_25x16', \
        '7', \
        'mofd_25x16-sharp-cm_kk', 'mofd_25x16-common_kk', \
        2560, 1600, \
        'Sharp 2560 x 1600 command mode')
    PNCD_SHARP_10x19 = MIPI_DISPLAY_FRU('PNCD_SHARP_10x19', \
        '8', \
        'mofd_10x19-dual_kk', '', \
        1080, 1920, \
        'Dual Sharp 1080 x 1920 command mode')

FRU_DATA_COUNT = 10
FRU_DIGIT_COUNT = 2
# Display is digit 0
FRU_DISPLAY_INDEX = 0

fru_list = [mipi_fru_data_kk.PNC_JDI_7x12, \
    mipi_fru_data_kk.PNC_SHARP_10x19_1, \
    mipi_fru_data_kk.PNV_SHARP_25x16, \
    mipi_fru_data_kk.PNC_SHARP_10x19_2, \
    mipi_fru_data_kk.PNV_JDI_25x16, \
    mipi_fru_data_kk.PANEL_AUO, \
    mipi_fru_data_kk.PNC_SHARP_25x16, \
    mipi_fru_data_kk.PNCD_SHARP_10x19, \
    ]

