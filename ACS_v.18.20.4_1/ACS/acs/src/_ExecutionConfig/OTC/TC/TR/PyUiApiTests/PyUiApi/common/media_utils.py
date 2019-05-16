from PyUiApi.common.shell_utils import ShellUtils
from string import Template
import re


class MediaInfo(object):
    image_property_regex_mapping = {"Complete name": ':\s+(\S+)',
                                    "Format": ':\s+(\S+)',
                                    "File size": ':\s+(\d+)',
                                    "Width": ':\s+(\d+\s*\d+)',
                                    "Height": ':\s+(\d+\s*\d+)',
                                    "Color space": ':\s+(\S+)',
                                    }
    mediainfo_cmd_template = Template("mediainfo $filepath")

    @staticmethod
    def get_media_info(media_file_path):
        mediainfo_cmd = MediaInfo.mediainfo_cmd_template.substitute(filepath=media_file_path)
        info_string = ShellUtils.run_shell_cmd(mediainfo_cmd)
        properties = {}
        for prop_name in MediaInfo.image_property_regex_mapping.keys():
            for line in info_string.splitlines():
                if prop_name in line:
                    prop_value = re.findall(MediaInfo.image_property_regex_mapping[prop_name], line)[0]
                    properties[prop_name] = prop_value.replace(" ", "")
        return properties
