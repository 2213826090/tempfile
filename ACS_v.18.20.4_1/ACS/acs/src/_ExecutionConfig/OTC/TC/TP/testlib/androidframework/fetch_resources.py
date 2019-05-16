from testlib.util.common import g_common_obj
from testlib.util.device import shell_command
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle

class Resources(object):

    def __init__(self):
        self.device = g_common_obj.get_device()
        self.config = TestConfig()
        self.config_handle = ConfigHandle()

    def get_resource_from_atifactory(self, conf_name, section, resource_name):
        #usage: et_resource_from_atifactory("tests.tablet.artifactory.conf", "content_picture", "wbmp")
        cfg_file = conf_name
        cfg_arti = self.config.read(cfg_file, 'artifactory')
        cfg_arti["location"] = self.config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = self.config.read(cfg_file, section)
        arti = Artifactory(cfg_arti.get('location'))
        file_name = cfg.get(resource_name)
        file_path = arti.get(file_name)
        return file_path

    def disable_app_verification(self):
        _cmdstr_disable = "settings put global package_verifier_enable 0"
        _cmdstr_get_status = "settings get global package_verifier_enable"
        _status = shell_command("adb shell settings get  global package_verifier_enable")[1][0].strip()
        if int(_status):
            g_common_obj.adb_cmd(_cmdstr_disable)

resource = Resources()