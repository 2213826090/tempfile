import unittest
import datetime
from uiautomator_utils import *
from PyUiApi.log.logging_utils import *
from PyUiApi.screen_recorder.screen_recorder_utils import *


class SingleMethodRunner(object):
    class_name = None
    test_method_name = None

    @staticmethod
    def run_single_test(test_case_class, method_name):
        SingleMethodRunner.test_method_name = method_name
        SingleMethodRunner.class_name = test_case_class.__name__
        LOG.info("running test: " + SingleMethodRunner.class_name + "_" + SingleMethodRunner.test_method_name)
        suite = unittest.TestSuite()
        suite.addTest(test_case_class(method_name))
        runner = unittest.TextTestRunner()
        screen_rec = None
        if ScreenRecorderManager.recording_env_var_name in os.environ:
            screen_rec = ScreenRecorderManager()
            screen_rec.begin_recording()
        result = runner.run(suite)
        if ScreenRecorderManager.recording_env_var_name in os.environ:
            screen_rec_file = SingleMethodRunner.class_name + "_" + SingleMethodRunner.test_method_name + ".mp4"
            screen_rec.end_recording(not result.wasSuccessful(), screen_rec_file)
        return result


class TestUtils(object):
    @staticmethod
    def print_test_result_problems(test_result):
        LOG.info(SingleMethodRunner.class_name + " " + SingleMethodRunner.test_method_name)
        LOG.info("FAILURES:")
        for failure in test_result.failures:
            LOG.info(failure)
        LOG.info("ERRORS:")
        for error in test_result.errors:
            LOG.info(error)

    @staticmethod
    def log_screenshot_and_dump():
        time_string = datetime.datetime.strftime(datetime.datetime.now(), '%Y_%m_%d_%H_%M_%S')
        log_dir = Log.get_log_path()
        test_id = SingleMethodRunner.class_name + "_" + SingleMethodRunner.test_method_name
        screenshot_path = os.path.join(log_dir, test_id + ".png")
        dump_path = os.path.join(log_dir, test_id + ".xml")
        d.screenshot(screenshot_path)
        d.dump(dump_path)
