import os
from time import time
from testlib.util.common import g_common_obj
from acs_test_scripts.Equipment.LOGGER.INSTANTLogger.INSTANTLogger import _get_screenshot_logs

def take_screenshot_for_liverpt():
    #Notice: if this method is being used in case,
    #this case can't be launched by  python -m unittest dir.to.case
    serial = g_common_obj.globalcontext.device_serial
    user_log_dir = os.environ.get('PYUNIT_USER_LOG_DIR', None)
    pic_name = str(time()) + ".png"
    _get_screenshot_logs(tc_path=user_log_dir, fname=pic_name, serial=serial)