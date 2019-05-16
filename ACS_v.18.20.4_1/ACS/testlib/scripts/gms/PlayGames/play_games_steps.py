#!/usr/bin/env python

from testlib.scripts.android.ui import ui_steps
from testlib.base import base_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.browser import browser_utils
from testlib.scripts.android.adb import adb_utils

import time

class open_play_games_settings(ui_step):
    """
        description:
            opens Play Games app Settings

        usage:
            open_play_games_settings()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_app_from_allapps(serial = self.serial,\
                        view_to_find = {"textContains": "Play Games"},
                        view_to_check = {"textContains": "Play Now"})()
        open_navigation_drawer(serial=self.serial)()
        ui_steps.click_button(serial = self.serial,\
                        view_to_find = {"textContains": "Settings"},
                        view_to_check = {"textContains": "Multiplayer notifications"})()
    def check_condition(self):
        self.uidevice.exists(description="Game Profile")

class open_play_games_inbox(ui_step):
    """
        description:
            opens Inbox from Play Games app

        usage:
            open_play_games_inbox()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):

        ui_steps.open_app_from_allapps(serial = self.serial,\
                view_to_find = {"textContains": "Play Games"},
                view_to_check = {"textContains": "Play Now"})()
        open_navigation_drawer(serial=self.serial)()
        ui_steps.click_button(serial = self.serial,\
                view_to_find = {"textContains": "Inbox"},
                view_to_check = {"text" : "Game on! See invitations and matches in progress here."})()
        ui_steps.click_button(serial = self.serial,\
                        view_to_find = {"text": "GIFTS & REQUESTS"},
                        view_to_check = {"textContains" : "Try sending a gift"})()
        ui_steps.click_button(serial = self.serial,\
                    view_to_find = {"text": "QUESTS"},
                    view_to_check = {"text" : "No quests today! Play Games quests that you've accepted appear here."})()

    def check_condition(self):
        self.uidevice.exists(text = "No quests today!")

class open_play_games_help(ui_step):
    """
        description:
            opens Help menu from Play Games app

        usage:
            open_play_games_help()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):

        ui_steps.open_app_from_allapps(serial = self.serial,\
                        view_to_find = {"textContains": "Play Games"},
                        view_to_check = {"textContains": "Play Now"})()
        open_navigation_drawer(serial=self.serial)()
        ui_steps.click_button(serial = self.serial,\
                    view_to_find = {"textContains": "Help & Feedback"},
                    view_to_check = {"text" : "Play games on your device"})()

    def check_condition(self):
        self.uidevice.exists(text = "Request callback")

class open_play_games_play_now(ui_step):
    """
        description:
            opens Help menu from Play Games app

        usage:
            open_play_games_play_now()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_app_from_allapps(serial = self.serial,\
                              view_to_find = {"textContains": "Play Games"},
                              view_to_check = {"textContains": "Play Now"})()
        open_navigation_drawer(serial=self.serial)()
        ui_steps.click_button(serial = self.serial,\
                            view_to_find = {"text": "Play Now",\
                                "resourceId" : "com.google.android.play.games:id/action_text"},
                            view_to_check = {"text" : "Discover new games"})()

    def check_condition(self):
        self.uidevice.exists(text = "Discover new games")

class go_for_free_game(ui_step):
    """
        description:
            opens a free game from Play Games app

        usage:
            go_for_free_game()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.swipe_down_to_view(serial = self.serial,\
             view_to_check = "FREE")

    def check_condition(self):
             self.uidevice.exists(text = "FREE")

class open_play_games_explore(ui_step):
    """
        description:
            opens Explore menu from Play Games app

        usage:
            open_play_games_explore()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_app_from_allapps(serial = self.serial,\
                              view_to_find = {"textContains": "Play Games"},
                              view_to_check = {"textContains": "Play Now"})()
        open_navigation_drawer(serial=self.serial)()
        ui_steps.click_button(serial = self.serial,\
                        view_to_find = {"textContains": "Explore"},
                        view_to_check = {"text" : "FEATURED"})()
    def check_condition(self):
        self.uidevice.exists(text = "FEATURED")

class game_has_specific_properties(ui_step):
    """
        description:
            Checks if a game has icon, title, price and if the "Options",\
                    button has "View in Play Store" option

        usage:
            game_has_specific_properties()()

        tags:
            android, games, play_games
    """
    def __init__(self, game_property = "contentDescription", game_price = "text", **kwargs):
        ui_step.__init__(self, **kwargs)
        self.properties_exist = True
    def do(self):
        index = 0
        play_store_exists = ui_utils.is_view_displayed(serial = self.serial,\
                    view_to_find = {"text" : "Play store"})
        if play_store_exists:
            while not ui_utils.is_view_displayed(serial = self.serial,\
                    view_to_find = {"resourceId" : "com.google.android.play.games:id/root_container",\
                    "index":index,
                    "description" : "Play store"}):
                index += 1
        else:
            index = 1
        index += 1
        title_exists = self.uidevice(resourceId="com.google.android.play.games:id/root_container",\
                index=index).child(resourceId="com.google.android.play.games:id/title").exists
        image_exists = self.uidevice(resourceId="com.google.android.play.games:id/root_container",\
                index=index).child(resourceId="com.google.android.play.games:id/image").exists
        price_exists = self.uidevice(resourceId="com.google.android.play.games:id/root_container",\
                index=index).child(resourceId="com.google.android.play.games:id/primary_label").exists
        self.properties_exist = title_exists and image_exists and price_exists

        ui_steps.click_button(serial = self.serial,\
                view_to_find = {"resourceId" : "com.google.android.play.games:id/overflow_menu"},\
                view_to_check = {"text" : "View in Play Store"})()
        ui_steps.press_back(view_to_check = {"text": "FEATURED"})()
    def check_condition(self):
        return self.properties_exist

class my_games_property(ui_step):
    """
        description:
            Verifies if the game exist and then the game's title

        usage:
            game_has_property()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
    def do(self):
        title_exists = self.uidevice(resourceId="com.google.android.play.games:id/root_container").\
                child(resourceId="com.google.android.play.games:id/title").exists or \
                self.uidevice(text = "No games found").exists
        self.property_exists = title_exists
    def check_condition(self):
        return self.property_exists

class open_navigation_drawer(ui_step):
    """
        description:
            Opens Navigation drawer menu from Play Games app

        usage:
            open_navigation_drawer()()

        tags:
            android, games, play_games
    """
    def __init__(self, wait_time = 20000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
    def do(self):
        if self.uidevice(description = "Open navigation drawer").wait.exists(timeout = self.wait_time):
            ui_steps.click_button(serial = self.serial,\
                    view_to_find = {"descriptionContains": "Open navigation drawer"},
                    view_to_check = {"text": "My Games"})()
    def check_condition(self):
        self.uidevice.exists(text = "My Games")

class open_my_games(ui_step):
    """
        description:
            opens My Games menu from Play Games app

        usage:
            open_my_games()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_app_from_allapps(serial = self.serial,\
                              view_to_find = {"textContains": "Play Games"},
                              view_to_check = {"textContains": "Play Now"})()
        open_navigation_drawer(serial=self.serial)()
        ui_steps.click_button(serial = self.serial,\
                view_to_find = {"textContains": "My Games"},
                view_to_check = {"text" : "RECENT GAMES"})()
    def check_condition(self):
        self.uidevice.exists(text = "RECENT GAMES")

class open_players(ui_step):
    """
        description:
            opens Players menu from Play Games app

        usage:
            open_players()()

        tags:
            android, games, play_games
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_app_from_allapps(serial = self.serial,\
                              view_to_find = {"textContains": "Play Games"},
                              view_to_check = {"textContains": "Play Now"})()
        open_navigation_drawer(serial=self.serial)()
        ui_steps.click_button(serial = self.serial,\
                view_to_find = {"textContains": "Players"},
                view_to_check = {"text" : "FOLLOWING"})()
    def check_condition(self):
        self.uidevice.exists(text = "FOLLOWING")

class check_players_properties(ui_step):
    """
        description:
            Verifies for each player profile picture and name, if the player exists

        usage:
            players_properties()()

        tags:
            android, games, play_games
    """
    def __init__(self, check_follow_button = True, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.check_follow_button = check_follow_button

    def do(self):
        title_exists = self.uidevice(resourceId="com.google.android.play.games:id/avatar").exists or \
                self.uidevice(text = "No players found").exists
        image_exists = self.uidevice(resourceId="com.google.android.play.games:id/avatar").\
                child(resourceId = "com.google.android.play.games:id/avatar_image").exists or \
                self.uidevice(text = "No players found").exists

        if self.check_follow_button:
            follow_button = self.uidevice(resourceId = "com.google.android.play.games:id/button", text = "Follow").exists
        else:
            follow_button = True
        self.property_exists = title_exists and image_exists and follow_button
    def check_condition(self):
        return self.property_exists
