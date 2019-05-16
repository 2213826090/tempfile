"""
antutu test lib
"""
import os
import time
import string
from testlib.pnp.pnp_impl import PnpImpl
from testlib.common.common import g_common_obj
from testlib.util.log import Logger


BASE_PATH = os.path.dirname(__file__)
class AntutuImpl(object):
    """
    pnp functions.
    """
    def __init__ (self, cfg):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self.pnp_setting = PnpImpl()
        self.logger = Logger.getlogger(__name__)

    def _parse_antutu_result(self, test_list, resource_dic, result_dic):
        """
        get antutu test result
        """
        for key in test_list:
            if key == 'antutu_2d' or key == 'antutu_3d':
                value_tmp = self.d(resourceId="com.antutu.ABenchMark:id/%s" % resource_dic[key]).text.encode('ascii')
                value = value_tmp.split()[-1]
            else:
                value = self.d(resourceId="com.antutu.ABenchMark:id/%s" % resource_dic[key]).text.encode('ascii')
            result_dic[key].append(string.atoi(value))
        for keys in test_list:
            self.logger.info('%s = %s' % (keys, result_dic[keys]))

    def _launch_antutu_test(self, activity, app_id):
        """
        launch antutu test
        """
        self.logger.info("launch AnTuTu")
        time.sleep(10)
        g_common_obj.back_home()
        time.sleep(3)
        self.d(className="android.widget.TextView", description="Apps").click()
        time.sleep(3)
        self.logger.info("start to run " + app_id)
        os.system("adb shell am start " + activity)
        time.sleep(5)
        self.d(className="android.widget.Button", resourceId="com.antutu.ABenchMark:id/test_btn").click()
        time.sleep(1)
        self.logger.info("sleep 30s to wait the system stable")
        time.sleep(30)
        self.d(className="android.widget.Button", resourceId="com.antutu.ABenchMark:id/test_all_btn").click()
        time.sleep(3)
        self.logger.info("starting and it will take about 400 seconds")
        time.sleep(400)
        g_common_obj.restart_server()

        assert self.d(className="android.widget.TextView", text="Test"), "ERROR: test button is not found"
        self.d(className="android.widget.TextView", text= "Test").click()
        time.sleep(3)
        self.d(className="android.widget.Button", resourceId="com.antutu.ABenchMark:id/detail_btn", text="Details").click()
        time.sleep(3)
        self.pnp_setting.rotate_device_n()

    def test(self):
        """
        test AnTuTu
        """
        result_dic = {}
        log_dic = {}
        resource_dic = {'total':'total_scores_text', 'multitask':'ue_multitask_text', \
        'dalvik':'ue_dalvik_text', 'cpu_int':'cpu_int_text', 'cpu_float':'cpu_float_text', \
        'ram':'mem_text', 'ram_speed':'ram_text', 'io_storage':'io_sdw_text', \
        'io_database':'io_db_text', 'antutu_2d':'gpu_2d_text', 'antutu_3d':'gpu_3d_text', \
        }
        test_list = ['total', 'multitask', 'dalvik', 'cpu_int', 'cpu_float', \
        'ram', 'ram_speed', 'io_storage','io_database', 'antutu_2d', 'antutu_3d']
        rnd = int(self.cfg.get('loop'))
        #init log dic
        for key in test_list:
            log_dic[key] = self.cfg.get(key)
        #init result list
        for key in test_list:
            result_dic[key] = []

        apk_name = self.cfg.get("antutu_apk_name")
        app_id = self.cfg.get('app_id')
        activity = self.cfg.get('activity')
        log_dir = self.cfg.get('log_dir')

        rnd_count = 0
        while(len(result_dic[test_list[0]]) != rnd):
            rnd_count += 1
            self.logger.info("round %d" % (rnd_count))
            self._launch_antutu_test(activity, app_id)
            #drop the first test
            if rnd_count == 1 and rnd != 1:
                self.pnp_setting.clear_cache(activity, apk_name)
                continue
                #get antutu result
            self._parse_antutu_result(test_list, resource_dic, result_dic)
            self.pnp_setting.clear_cache(activity, apk_name)
            g_common_obj.back_home()
        #save result to log file
        self.pnp_setting.save_log(log_dir, test_list, log_dic, result_dic)
