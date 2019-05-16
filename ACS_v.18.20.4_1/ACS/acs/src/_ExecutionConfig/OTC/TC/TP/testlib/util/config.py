import json
import os
from ConfigParser import ConfigParser, NoSectionError

def _read_conf(file_src):
    _opts = {}
    try:
        file_name, ext_name = os.path.splitext(file_src)
        if ext_name.lower() == '.json':
            _opts = json.loads(open(file_src).read())
        else:
            _config = ConfigParser()
            _config.read(file_src)
            for _sec in _config.sections():
                _opts[_sec] = dict(_config.items(_sec))
    except Exception as e:
        raise Exception('Failed to parse configuration file %s, error: %s!' % (file_src, e))
    return _opts

class TestConfig:
    """
    GlobalContext class is used to hold global settings & configurations
    for test cases in single place.

    The purposes of this class:
    Decouple with nose context object;
    Provide global contexts even not run in nose runner.
    """
    def __init__(self):
        self._sys_conf = '/etc/oat/sys.conf'

    def read(self, file_src='', section=''):
        if file_src:
            if not os.path.isabs(file_src):
                file_src = os.path.join(os.environ['TEST_DATA_ROOT'], file_src)
            if not os.path.exists(file_src):
                raise Exception('Failed to parse conf file %s, it doesn\'t exist!' % (file_src))
            _opts = _read_conf(file_src)
            if section is None:
                return _opts
            if section in _opts:
                return _opts[section]

        if os.path.exists(self._sys_conf):
            _opts = _read_conf(self._sys_conf)
            if section in _opts:
                return _opts[section]
        return {}

    def getConfValue(self, file_src='', section='', key=None):
        if not os.path.isabs(file_src):
            file_src = os.path.join(os.environ['TEST_DATA_ROOT'], file_src)
        _opts = self.read(file_src, section)
        if key and key in _opts:
            return _opts[key]
        else:
            return None
