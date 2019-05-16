from PyUiApi.common.uiautomator_utils import *


class HoudiniUtils(object):
    houdini_chess_app_name = "Chess Chess"
    houdini_chess_app_package_name = "com.celliecraze.chesschess"
    houdini_chess_apk_name = "com.celliecraze.chesschess-1.apk"
    houdini_sanity_tests_zip_name = "Houdini_sanity_tests.zip"
    houdini_new_game_option_txt = "New Game"
    houdini_chess_board_resid = "com.celliecraze.chesschess:id/chessboard"
    houdini_chess_move_list_resid = "com.celliecraze.chesschess:id/moveList"
    houdini_chess_do_not_rate_txt = "No, thanks"

    @staticmethod
    def is_chess_app_is_installed():
        app_found = UiAutomatorUtils.is_app_in_apps_menu(HoudiniUtils.houdini_chess_app_name)
        if not app_found:
            return False
        UiAutomatorUtils.launch_app_from_apps_menu(HoudiniUtils.houdini_chess_app_name)
        if not d(packageName=HoudiniUtils.houdini_chess_app_package_name).wait.exists(timeout=10000):
            return False
        return True

    @staticmethod
    def launch_chess_app():
        UiAutomatorUtils.launch_app_from_apps_menu(HoudiniUtils.houdini_chess_app_name)
        # handle rating popup
        while d(textContains=HoudiniUtils.houdini_chess_do_not_rate_txt).wait.exists(timeout=4000):
            d(textContains=HoudiniUtils.houdini_chess_do_not_rate_txt).click()
            time.sleep(1)

    @staticmethod
    def click_options_button():
        d.press(KEYCODE_MENU)
        time.sleep(2)

    @staticmethod
    def wait_for_chess_app_to_be_in_focus():
        d(packageName=HoudiniUtils.houdini_chess_app_package_name).wait.exists(timeout=10000)

    @staticmethod
    def select_new_game():
        if d(text=HoudiniUtils.houdini_new_game_option_txt).wait.exists(timeout=3000):
            d(text=HoudiniUtils.houdini_new_game_option_txt).click()

    @staticmethod
    def get_current_move_list():
        if d(resourceId=HoudiniUtils.houdini_chess_move_list_resid).wait.exists(timeout=3000):
            return Info.get_text(d(resourceId=HoudiniUtils.houdini_chess_move_list_resid))
        return None


class Chessboard(object):
    letter_to_number_mapping = {'A': 1, 'B': 2, 'C': 3, 'D': 4, 'E': 5, 'F': 6, 'G': 7, 'H': 8}

    def __init__(self):
        d(resourceId=HoudiniUtils.houdini_chess_board_resid).wait.exists(timeout=3000)
        self.bounds = Bounds(d(resourceId=HoudiniUtils.houdini_chess_board_resid).info)
        self.cell_width = (self.bounds.right - self.bounds.left) / 8
        self.cell_height = (self.bounds.bottom - self.bounds.top) / 8

    def click_cell(self, horiz_index, vert_index):
        horiz_index = int(horiz_index)
        vert_index = int(vert_index)
        # cells are considered from the bottom left of the chessboard, first cell being A1
        vert_coord = self.bounds.bottom - (self.cell_height * (vert_index - 1) + self.cell_height / 2)
        horiz_coord = self.bounds.left + (self.cell_width * (horiz_index - 1) + self.cell_width / 2)
        LOG.info("clicking on: %s %s" % (str(horiz_coord), str(vert_coord)))
        d.click(horiz_coord, vert_coord)
        time.sleep(2)

    def move(self, piece_position, destination_cell):
        LOG.info("moving: " + piece_position)
        self.click_cell(Chessboard.letter_to_number_mapping[piece_position[0]], piece_position[1])
        time.sleep(2)
        LOG.info("to: " + destination_cell)
        self.click_cell(Chessboard.letter_to_number_mapping[destination_cell[0]], destination_cell[1])
        time.sleep(2)

    def execute_moves(self, moves_list, seconds_between_moves):
        for move in moves_list:
            self.move(move[0], move[1])
            time.sleep(seconds_between_moves)


class RobocopApp(object):
    app_shortcut_name = "RoboCop"
    pause_download_button_resid = "com.glu.robocop:id/pauseButton"
    percentage_progress_bar_resid = "com.glu.robocop:id/progressAsPercentage"
    cancel_create_profile_button_resid = "com.google.android.gms:id/back_button"
    cancel_choose_account_text = "Cancel"

    @staticmethod
    def launch():
        UiAutomatorUtils.launch_app_from_apps_menu(RobocopApp.app_shortcut_name)

