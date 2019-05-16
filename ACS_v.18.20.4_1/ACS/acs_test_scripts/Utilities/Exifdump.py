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
:summary: Implements methods used over the whole ACS solution
:since: 2012-2-08
:author: xzhao23
"""
from ErrorHandling.AcsConfigException import AcsConfigException

"""
     Exif information decoder
     Public Domain
     Since I don't have a copy of the Exif standard, I got most of the
     information from the TIFF/EP draft International Standard:

       ISO/DIS 12234-2
       Photography - Electronic still picture cameras - Removable Memory
       Part 2: Image data format - TIFF/EP

     Another must-read is the TIFF 6.0 specification, which can be
     obtained from Adobe's FTP site, at
     ftp://ftp.adobe.com/pub/adobe/devrelations/devtechnotes/pdffiles/tiff6.pdf
"""

# pylint: disable=missing-docstring, invalid-name

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


EXIF_TAGS = {
    0x100: "ImageWidth",
    0x101: "ImageLength",
    0x102: "BitsPerSample",
    0x103: "Compression",
    0x106: "PhotometricInterpretation",
    0x10A: "FillOrder",
    0x10D: "DocumentName",
    0x10E: "ImageDescription",
    0x10F: "Make",
    0x110: "Model",
    0x111: "StripOffsets",
    0x112: "Orientation",
    0x115: "SamplesPerPixel",
    0x116: "RowsPerStrip",
    0x117: "StripByteCounts",
    0x11A: "XResolution",
    0x11B: "YResolution",
    0x11C: "PlanarConfiguration",
    0x128: "ResolutionUnit",
    0x12D: "TransferFunction",
    0x131: "Software",
    0x132: "DateTime",
    0x13B: "Artist",
    0x13E: "WhitePoint",
    0x13F: "PrimaryChromaticities",
    0x156: "TransferRange",
    0x200: "JPEGProc",
    0x201: "JPEGInterchangeFormat",
    0x202: "JPEGInterchangeFormatLength",
    0x211: "YCbCrCoefficients",
    0x212: "YCbCrSubSampling",
    0x213: "YCbCrPositioning",
    0x214: "ReferenceBlackWhite",
    0x828F: "BatteryLevel",
    0x8298: "Copyright",
    0x829A: "ExposureTime",
    0x829D: "FNumber",
    0x83BB: "IPTC/NAA",
    0x8769: "ExifIFDPointer",
    0x8773: "InterColorProfile",
    0x8822: "ExposureProgram",
    0x8824: "SpectralSensitivity",
    0x8825: "GPSInfoIFDPointer",
    0x8827: "ISOSpeedRatings",
    0x8828: "OECF",
    0x9000: "ExifVersion",
    0x9003: "DateTimeOriginal",
    0x9004: "DateTimeDigitized",
    0x9101: "ComponentsConfiguration",
    0x9102: "CompressedBitsPerPixel",
    0x9201: "ShutterSpeedValue",
    0x9202: "ApertureValue",
    0x9203: "BrightnessValue",
    0x9204: "ExposureBiasValue",
    0x9205: "MaxApertureValue",
    0x9206: "SubjectDistance",
    0x9207: "MeteringMode",
    0x9208: "LightSource",
    0x9209: "Flash",
    0x920A: "FocalLength",
    0x9214: "SubjectArea",
    0x927C: "MakerNote",
    0x9286: "UserComment",
    0x9290: "SubSecTime",
    0x9291: "SubSecTimeOriginal",
    0x9292: "SubSecTimeDigitized",
    0xA000: "FlashPixVersion",
    0xA001: "ColorSpace",
    0xA002: "PixelXDimension",
    0xA003: "PixelYDimension",
    0xA004: "RelatedSoundFile",
    0xA005: "InteroperabilityIFDPointer",
    0xA20B: "FlashEnergy",  # 0x920B in TIFF/EP
    0xA20C: "SpatialFrequencyResponse",  # 0x920C    -  -
    0xA20E: "FocalPlaneXResolution",  # 0x920E    -  -
    0xA20F: "FocalPlaneYResolution",  # 0x920F    -  -
    0xA210: "FocalPlaneResolutionUnit",  # 0x9210    -  -
    0xA214: "SubjectLocation",  # 0x9214    -  -
    0xA215: "ExposureIndex",  # 0x9215    -  -
    0xA217: "SensingMethod",  # 0x9217    -  -
    0xA300: "FileSource",
    0xA301: "SceneType",
    0xA302: "CFAPattern",  # 0x828E in TIFF/EP
    0xA401: "CustomRendered",
    0xA402: "ExposureMode",
    0xA403: "WhiteBalance",
    0xA404: "DigitalZoomRatio",
    0xA405: "FocalLengthIn35mmFilm",
    0xA406: "SceneCaptureType",
    0xA407: "GainControl",
    0xA408: "Contrast",
    0xA409: "Saturation",
    0xA40A: "Sharpness",
    0xA40B: "DeviceSettingDescription",
    0xA40C: "SubjectDistanceRange",
    0xA420: "ImageUniqueID",
}

INTR_TAGS = {
    0x1: "InteroperabilityIndex",
    0x2: "InteroperabilityVersion",
    0x1000: "RelatedImageFileFormat",
    0x1001: "RelatedImageWidth",
    0x1002: "RelatedImageLength",
}

GPS_TAGS = {
    0x0: "GPSVersionID",
    0x1: "GPSLatitudeRef",
    0x2: "GPSLatitude",
    0x3: "GPSLongitudeRef",
    0x4: "GPSLongitude",
    0x5: "GPSAltitudeRef",
    0x6: "GPSAltitude",
    0x7: "GPSTimeStamp",
    0x8: "GPSSatellites",
    0x9: "GPSStatus",
    0xA: "GPSMeasureMode",
    0xB: "GPSDOP",
    0xC: "GPSSpeedRef",
    0xD: "GPSSpeed",
    0xE: "GPSTrackRef",
    0xF: "GPSTrack",
    0x10: "GPSImgDirectionRef",
    0x11: "GPSImgDirection",
    0x12: "GPSMapDatum",
    0x13: "GPSDestLatitudeRef",
    0x14: "GPSDestLatitude",
    0x15: "GPSDestLongitudeRef",
    0x16: "GPSDestLongitude",
    0x17: "GPSDestBearingRef",
    0x18: "GPSDestBearing",
    0x19: "GPSDestDistanceRef",
    0x1A: "GPSDestDistance",
    0x1B: "GPSProcessingMethod",
    0x1C: "GPSAreaInformation",
    0x1D: "GPSDateStamp",
    0x1E: "GPSDifferential"
}


def s2n_motorola(s2n_str):
    x = 0
    for c in s2n_str:
        x = (x << 8) | ord(c)
    return x


def s2n_intel(s2n_str):
    x = 0
    y = 0
    for c in s2n_str:
        x |= ord(c) << y
        y += 8
    return x


class Fraction:

    def __init__(self, num, den):
        self.num = num
        self.den = den

    def __repr__(self):
        # String representation
        return '%d/%d' % (self.num, self.den)


class TIFF_file:

    def __init__(self, data):
        self.data = data
        self.endian = data[0]

    def s2n(self, offset, length, signed=0):
        s2n_slice = self.data[offset:offset + length]
        if self.endian == 'I':
            val = s2n_intel(s2n_slice)
        else:
            val = s2n_motorola(s2n_slice)
        # Sign extension ?
        if signed:
            msb = 1 << (8 * length - 1)
            if val & msb:
                val -= msb << 1
        return val

    def first_IFD(self):
        return self.s2n(4, 4)

    def next_IFD(self, ifd):
        entries = self.s2n(ifd, 2)
        return self.s2n(ifd + 2 + 12 * entries, 4)

    def list_IFDs(self):
        i = self.first_IFD()
        a = []
        while i:
            a.append(i)
            i = self.next_IFD(i)
        return a

    def dump_IFD(self, ifd):
        entries = self.s2n(ifd, 2)
        a = []
        for i in range(entries):
            entry = ifd + 2 + 12 * i
            tag = self.s2n(entry, 2)
            s2n_type = self.s2n(entry + 2, 2)
            if not 1 <= s2n_type <= 10:
                continue  # not handled
            typelen = [1, 1, 2, 4, 8, 1, 1, 2, 4, 8][s2n_type - 1]
            count = self.s2n(entry + 4, 4)
            offset = entry + 8
            if count * typelen > 4:
                offset = self.s2n(offset, 4)
            if s2n_type == 2:
                # Special case: nul-terminated ASCII string
                values = self.data[offset:offset + count - 1]
            else:
                values = []
                signed = (s2n_type == 6 or s2n_type >= 8)
                for _j in range(count):
                    if s2n_type % 5:
                        # Not a fraction
                        value_j = self.s2n(offset, typelen, signed)
                    else:
                        # The s2n_type is either 5 or 10
                        value_j = Fraction(self.s2n(offset, 4, signed),
                                           self.s2n(offset + 4, 4, signed))
                values.append(value_j)
                offset += typelen
            # Now "values" is either a string or an array
            a.append((tag, s2n_type, values))
        return a


class Exifinfo:

    """
    This class will get all the the exif info of a jpg file.
    """

    def __init__(self, jpg_file):
        """
        :param file: String
        """
        try:
            fd = open(jpg_file, 'rb')
        except IOError:
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, "The jpg file does not exist!")
        self.fd = fd
        self.all_value = {}

    def print_IFD(self, cat, fields, exif_dict=None):
        if not isinstance(exif_dict, dict):
            exif_dict = EXIF_TAGS

        self.all_value[cat] = {}
        for (tag, _type, values) in fields:
            try:
                stag = exif_dict[tag]
            except:  # pylint: disable=W0702
                stag = '0x%04X' % tag
            """
            Following code has been commented
            But not clear why it is used. Just keep it for the moment.
            stype = ['B', # BYTE
                     'A', # ASCII
                     'S', # SHORT
                     'L', # LONG
                     'R', # RATIONAL
                     'SB', # SBYTE
                     'U', # UNDEFINED
                     'SS', # SSHORT
                     'SL', # SLONG
                     'SR', # SRATIONAL
                    ] [_type - 1]
            LOGGER_TEST_SCRIPT.debug(' %s(%s)=%s' % (str(stag), str(stype), repr(values)))
            """
            self.all_value[cat][stag] = values

    def process_file(self):
        """
        Process the file.
        """
        data = self.fd.read(12)
        if data[0:4] != '\377\330\377\341' or data[6:10] != 'Exif':
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, "Not a exif file")

        length = ord(data[4]) * 256 + ord(data[5])
        data = self.fd.read(length - 8)
        if 0:
            # that's for debugging only
            open('exif.header', 'wb').write(data)
        # LOGGER_TEST_SCRIPT.info({'I':'Intel', 'M':'Motorola'}[data[0]], 'format')
        T = TIFF_file(data)
        L = T.list_IFDs()
        for i in range(len(L)):
            cat = 'main'
            if i == 0:
                print LOGGER_TEST_SCRIPT.info('(main image)'),
                cat = 'main'
            if i == 1:
                print LOGGER_TEST_SCRIPT.info('(thumbnail)'),
                cat = 'thumbnail'
            IFD = T.dump_IFD(L[i])
            self.print_IFD(cat, IFD)
            exif_off = gps_off = 0
            for tag, _type, values in IFD:
                if tag == 0x8769:
                    exif_off = values[0]
                if tag == 0x8825:
                    gps_off = values[0]
            if exif_off:
                LOGGER_TEST_SCRIPT.info(' Exif SubIFD at offset %d:' % exif_off)
                cat = 'sub'
                IFD = T.dump_IFD(exif_off)
                self.print_IFD(cat, IFD)
                # Recent digital cameras have a little subdirectory
                # here, pointed to by tag 0xA005. Apparently, it's the
                # "Interoperability IFD", defined in Exif 2.1 and DCF.
                intr_off = 0
                for tag, _type, values in IFD:
                    if tag == 0xA005:
                        intr_off = values[0]
                if intr_off:
                    IFD = T.dump_IFD(intr_off)
                    cat = 'intr'
                    self.print_IFD(cat, IFD, exif_dict=INTR_TAGS)
            if gps_off:
                IFD = T.dump_IFD(gps_off)
                cat = 'gps'
                self.print_IFD(cat, IFD, exif_dict=GPS_TAGS)
        return 0

    def get_exif_info(self):
        self.process_file()
        return self.all_value
