from _prerequisites import *
from PyUiApi.tests.chrome_tests import *


class SimpleTestWrapper(unittest.TestCase):
    def setUp(self):
        UiAutomatorUtils.unlock_screen()

    def tearDown(self):
        UiAutomatorUtils.close_all_tasks()

    def test_disable_merge_tabs_with_apps(self):
        Chrome.launch()
        Chrome.toggle_merge_tabs_with_apps(False)
        self.assertTrue(d(text="OFF").wait.exists(timeout=3000))

test_result = SingleMethodRunner.run_single_test(SimpleTestWrapper, "test_disable_merge_tabs_with_apps")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
