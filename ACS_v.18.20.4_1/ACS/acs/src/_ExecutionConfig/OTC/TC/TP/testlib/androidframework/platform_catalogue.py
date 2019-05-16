import re


class Platform(object):
    properties = {}
    name = ""
    screenWidth = None
    screenHeight = None

    def __init__(self, name, properties):
        self.name = name
        self.properties = properties

    def get_name(self):
        return self.name

    def get_property(self, prop_name):
        if not prop_name in self.properties:
            return None
        return self.properties[prop_name]


class PlatformCatalogue(object):
    malata10_0_coho = Platform("malata10_0_coho", {"ro.product.device": "malata10_0_coho",
                               "ro.sf.lcd_density": "160",
                               "ro.sf.lcd_density_info": "1280 x 800px 217mm x 136mm  149 dpi => density: 160",
                               "ro.sf.lcd_density_origin": "drm connector info"})
    manta = Platform("manta", {"ro.product.device": "manta",
                     "ro.sf.lcd_density": "320"})
    manta.screenWidth = 2560
    manta.screenHeight = 1600
    st70408_4_coho = Platform("st70408_4_coho", {"ro.product.device": "st70408_4_coho",
                              "ro.sf.lcd_density": "213",
                              "ro.sf.lcd_density_info": "800 x 1280px 99mm x 177mm  205 dpi => density: 213",
                              "ro.sf.lcd_density_origin": "framebuffer"})
    ecs_e7 = Platform("ecs_e7", {"ro.product.device": "ecs_e7 ???",
                      "ro.sf.lcd_density": "213",
                      "ro.sf.lcd_density_info": "800 x 1280px 94mm x 150mm  216 dpi => density: 213"})
    one7_0_4_coho = Platform("one7_0_4_coho", {"ro.product.device": "one7_0_4_coho",
                             "ro.sf.lcd_density": "213",
                             "ro.sf.lcd_density_info": "800 x 1280px 94mm x 150mm  216 dpi => density: 213"})
    ST70408_4_COHO = Platform("ST70408_4_COHO", {"ro.product.device": "ST70408_4_COHO",
                              "ro.sf.lcd_density": "213",
                              "ro.sf.lcd_density_info": "800 x 1280px 94mm x 150mm  216 dpi => density: 213"})
    ecs27b_0_coho = Platform("ecs27b_0_coho", {"ro.product.device": "ecs27b_0_coho",
                             "ro.sf.lcd_density": "160",
                             "ro.sf.lcd_density_info": "600 x 1024px 89mm x 152mm 171 dpi => density: 160"})
    one695_1_coho = Platform("one695_1_coho", {"ro.product.device": "one695_1_coho",
                             "ro.sf.lcd_density": "160",
                             "ro.sf.lcd_density_info": "600 x 1024px 89mm x 152mm 171 dpi => density: 160"})
    vsi7q_1_coho = Platform("vsi7q_1_coho", {"ro.product.device": "vsi7q_1_coho",
                            "ro.sf.lcd_density": "160",
                            "ro.sf.lcd_density_info": "600 x 1024px 89mm x 152mm 171 dpi => density: 160"})
    ecs28a_0_coho = Platform("ecs28a_0_coho", {"ro.product.device": "ecs28a_0_coho",
                             "ro.sf.lcd_density": "160",
                             "ro.sf.lcd_density_info": "1280 x 800px 182mm x 119mm  178 dpi => density: 160"})
    one8_0_1_coho = Platform("one8_0_1_coho", {"ro.product.device": "one8_0_1_coho",
                             "ro.sf.lcd_density": "160",
                             "ro.sf.lcd_density_info": "1280 x 800px 182mm x 119mm  178 dpi => density: 160"})
    tc80ra3_1_coho = Platform("tc80ra3_1_coho", {"ro.product.device": "tc80ra3_1_coho",
                              "ro.sf.lcd_density": "160",
                              "ro.sf.lcd_density_info": "1280 x 800px 182mm x 119mm  178 dpi => density: 160"})
    vsi8q_1_coho = Platform("vsi8q_1_coho", {"ro.product.device": "vsi8q_1_coho",
                            "ro.sf.lcd_density": "160",
                            "ro.sf.lcd_density_info": "1280 x 800px 182mm x 119mm  178 dpi => density: 160"})
    ecs210a_4_coho = Platform("ecs210a_4_coho", {"ro.product.device": "ecs210a_4_coho",
                              "ro.sf.lcd_density": "160",
                              "ro.sf.lcd_density_info": "1280 x 800px 217mm x 136mm  149 dpi => density: 160"})
    tc10ra3_1_coho = Platform("tc10ra3_1_coho", {"ro.product.device": "tc10ra3_1_coho",
                              "ro.sf.lcd_density": "160",
                              "ro.sf.lcd_density_info": "1280 x 800px 217mm x 136mm  149 dpi => density: 160"})
    malata8_0_coho = Platform("malata8_0_coho", {"ro.product.device": "malata8_0_coho",
                              "ro.sf.lcd_density": "213",
                              "ro.sf.lcd_density_info": "800 x 1280px 107mm x 172mm  189 dpi => density: 213"})
    malata8low_2_coho = Platform("malata8low_2_coho", {"ro.product.device": "malata8low_2_coho",
                                 "ro.sf.lcd_density": "213",
                                 "ro.sf.lcd_density_info": "800 x 1280px 107mm x 172mm  189 dpi => density: 213"})
    A82i_2_COHO = Platform("A82i_2_COHO", {"ro.product.device": "A82i_2_COHO",
                           "ro.sf.lcd_density": "213",
                           "ro.sf.lcd_density_info": "800 x 1280px 107mm x 172mm  189 dpi => density: 213"})
    a105i_1_COHO = Platform("a105i_1_COHO", {"ro.product.device": "a105i_1_COHO",
                            "ro.sf.lcd_density": "160",
                            "ro.sf.lcd_density_info": "1280 x 800px 217mm x 136mm  149 dpi => density: 160"})
    chiphd8_0_coho = Platform("chiphd8_0_coho", {"ro.product.device": "chiphd8_0_coho",
                              "ro.sf.lcd_density": "213",
                              "ro.sf.lcd_density_info": "800 x 1280px 99mm x 177mm 205 dpi => density: 213"})
    s3gr10m6s = Platform("s3gr10m6s", {"ro.product.device": "s3gr10m6s",
                         "ro.sf.lcd_density": "360"})
    s3gr10m6s.screenWidth = 1920
    s3gr10m6s.screenHeight = 1080
    oars7 = Platform("oars7", {"ro.product.device": "oars7",
                     "ro.sf.lcd_density": "160"})
    oars7.screenWidth = 1920
    oars7.screenHeight = 1080
    cht_ffd = Platform("cht-ffd", {"ro.product.device": "cht-ffd",
                       "ro.sf.lcd_density": "240"})
    cht_ffd.screenWidth = 1200
    cht_ffd.screenHeight = 1920

#     r2_cht_ffd = Platform("r2_cht_ffd", {"ro.product.device": "r2_cht_ffd",
#                        "ro.sf.lcd_density": "280"})
#     r2_cht_ffd.screenWidth = 1200
#     r2_cht_ffd.screenHeight = 1920

    platforms = [malata10_0_coho, manta, st70408_4_coho, one7_0_4_coho, ST70408_4_COHO, ecs27b_0_coho, one695_1_coho,
                 vsi7q_1_coho, ecs28a_0_coho, one8_0_1_coho, tc80ra3_1_coho, vsi8q_1_coho, ecs210a_4_coho,
                 tc10ra3_1_coho, malata8_0_coho, malata8low_2_coho, A82i_2_COHO, malata10_0_coho, a105i_1_COHO,
                 chiphd8_0_coho, oars7, cht_ffd]

    @staticmethod
    def find_platform(platform_name):
        for platform in PlatformCatalogue.platforms:
            if platform_name == platform.get_name():
                return platform
