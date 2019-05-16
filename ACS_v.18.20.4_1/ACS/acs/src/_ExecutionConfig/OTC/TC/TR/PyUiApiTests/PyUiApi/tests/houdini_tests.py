from PyUiApi.app_utils.houdini_utils import *
from PyUiApi.common.test_utils import *
from PyUiApi.common.acs_utils import *
from PyUiApi.common.environment_utils import *
from PyUiApi.app_utils.contacts_utils import *
import zipfile


class HoudiniTests(unittest.TestCase):
    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.screenshoter = ScreenshotUtils()

    def tearDown(self):
        self.log_before_cleanup()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        UiAutomatorUtils.close_all_tasks()
        self.screenshoter.remove_all_screenshots()

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def test_install_houdini_application_from_SD_card(self):
        AcsUtils.copy_acs_artifact_to_dut(HoudiniUtils.houdini_chess_apk_name, Environment.sd_card_path)
        output = AdbUtils.run_adb_cmd("pm install -r -d %s%s" % (Environment.sd_card_path,
                                                                 HoudiniUtils.houdini_chess_apk_name))
        LOG.info("install call yielded: %s" % output)
        self.assertTrue(HoudiniUtils.is_chess_app_is_installed())

    def test_install_houdini_application_with_adb(self):
        self.assertTrue(HoudiniUtils.is_chess_app_is_installed())

    def test_run_houdini_test_tool(self):
        try:
            AcsUtils.copy_acs_artifact_to_host(HoudiniUtils.houdini_sanity_tests_zip_name,
                                               Environment.tmp_dir_path)
            houdini_zip_local_path = os.path.join(Environment.tmp_dir_path, HoudiniUtils.houdini_sanity_tests_zip_name)
            with zipfile.ZipFile(houdini_zip_local_path, "r") as houdini_zip:
                houdini_zip.extractall(Environment.tmp_dir_path)
            serials = AdbUtils.get_adb_devices_serials()
            LOG.info("detected device serials: " + str(serials))
            os.chdir(os.path.join(Environment.tmp_dir_path, "Houdini_sanity_test"))
            houdini_script_cmd = "bash check_houdini.sh L " + serials[0]
            LOG.info(houdini_script_cmd)
            output = subprocess.check_output(houdini_script_cmd, shell=True)
            LOG.info("houdini script output: " + output)
            self.assertTrue("FAIL" not in output)
            expected_missing = ["libstlport.so"]
            for line in output.splitlines():
                hits = [x for x in expected_missing if x in line]
                self.assertTrue(hits is not [] or "MISSING" not in output)
        finally:
            shutil.rmtree(os.path.join(Environment.tmp_dir_path, "Houdini_sanity_test"))
            os.remove(os.path.join(Environment.tmp_dir_path, "Houdini_sanity_tests.zip"))

    def test_play_chess_10_minutes(self):
        # application will always start in portrait mode
        HoudiniUtils.launch_chess_app()
        HoudiniUtils.wait_for_chess_app_to_be_in_focus()
        HoudiniUtils.click_options_button()
        HoudiniUtils.select_new_game()
        moves = [('A2', 'A3'), ('B2', 'B4'), ('C2', 'C3'), ('D2', 'D4'),
                 ('E2', 'E3'), ('F2', 'F4'), ('G2', 'G4'), ('H2', 'H4'),
                 ('C1', 'B2'), ('F1', 'G2'), ('A1', 'A2'), ('H1', 'H2'),
                 ('B1', 'D2'), ('G1', 'E2')]
        self.make_chess_move_sequence(moves)
        HoudiniUtils.click_options_button()
        self.assertTrue(d(text=HoudiniUtils.houdini_new_game_option_txt).wait.exists(timeout=3000))
        d(text=HoudiniUtils.houdini_new_game_option_txt).click()
        time.sleep(4)

    def make_chess_move_sequence(self, moves):
        chessboard = Chessboard()
        for move in moves:
            self.screenshoter.take_screenshot()
            initial_move_list_string = HoudiniUtils.get_current_move_list()
            chessboard.move(move[0], move[1])
            time.sleep(15)
            subsequent_move_list_string = HoudiniUtils.get_current_move_list()
            move_list_changed = False
            if subsequent_move_list_string is not None:
                move_list_changed = initial_move_list_string != subsequent_move_list_string
            self.screenshoter.take_screenshot()
            # the last 2 screenshots must not be the same a move must have changed the display
            self.assertTrue(move_list_changed or not self.screenshoter.same_screenshots(-1, -2))
            time.sleep(5)

    def test_switch_between_arm_and_google_apps(self):
        HoudiniUtils.launch_chess_app()
        HoudiniUtils.wait_for_chess_app_to_be_in_focus()
        HoudiniUtils.click_options_button()
        HoudiniUtils.select_new_game()
        first_session_moves = [('A2', 'A3'), ('B2', 'B4'), ('C2', 'C3'), ('D2', 'D4'), ('E2', 'E3')]
        self.make_chess_move_sequence(first_session_moves)
        Contacts.launch()
        Contacts.search(CONTACTS_AAA_SEARCH_TXT)
        Contacts.click_first_contact()
        self.assertTrue(d(packageName=CONTACTS_PACKAGE_NAME).wait.exists(timeout=3000))
        HoudiniUtils.launch_chess_app()
        second_session_moves = [('F2', 'F4'), ('G2', 'G4'), ('H2', 'H4')]
        self.make_chess_move_sequence(second_session_moves)
        HoudiniUtils.click_options_button()
        self.assertTrue(d(text=HoudiniUtils.houdini_new_game_option_txt).wait.exists(timeout=3000))
        d(text=HoudiniUtils.houdini_new_game_option_txt).click()
        time.sleep(4)

    def test_memory_intensive_app(self):
        RobocopApp.launch()
