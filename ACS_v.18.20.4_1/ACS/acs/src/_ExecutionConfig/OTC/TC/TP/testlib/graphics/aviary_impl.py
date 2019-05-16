# -*- coding: utf-8 -*-
from testlib.util.common import g_common_obj


class AviaryImpl(object):

    def __init__(self):

        self.device = g_common_obj.get_device()

    fillters = ["Enhance",
                "Effects",
                "Frames",
                "Crop",
                "Focus",
                "Orientation",
                "Brightness",
                "Contrast",
                "Saturation",
                "Warmth",
                "Sharpness",
                "Splash",
                "Draw",
                "Text",
                "Redeye",
                "Whiten",
                "Blur",
                "Blemish",
                "Meme",
                "Stickers"]()

    enhances = ["Hi-Def", "Illuminate", "Color Fix"]
    effects = ["Clyde", "Avenue", "Haas", "Arizona", "Lucky",
               "Dean", "Keylime", "Boardwalk", "Sentosa", "Sage", "Metropolis"]

    def launch_app_am(self):
        g_common_obj.launch_app_am(
            "com.aviary.android.feather", "com.aviary.android.feather.MainActivity")

    def stop_app_am(self):
        g_common_obj.stop_app_am("com.aviary.android.feather")

    def test_fillters(self):
        self.device(resourceId="com.aviary.android.feather:id/squareImageView",
               className="android.widget.ImageView").click

        for t in self.fillters:
            if t == "Enhance":
                self.device(
                    descriptionMatches=t, className="android.widget.LinearLayout").click
                print t
                self.device.wait.update()
                for e in self.enhances:
                    self.device(
                        descriptionMatches=e, className="com.aviary.android.feather.sdk.widget.AviaryHighlightImageButton").click
                    print e
                    self.device.wait.update()
                break
            elif t == "Effect":
                self.device(
                    descriptionMatches=t, className="android.widget.LinearLayout").click
                print t
                self.device.wait.update()
                for ef in self.efffects:
                    self.device(text=ef, className="android.widget.TextView").click
                    print ef
                break
            elif t == "Stickers":
                self.device.press("back")
