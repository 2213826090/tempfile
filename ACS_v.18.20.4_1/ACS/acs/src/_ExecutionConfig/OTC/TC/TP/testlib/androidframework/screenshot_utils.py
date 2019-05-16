from testlib.util.common import g_common_obj
# from testlib.androidframework.common import UiAutomatorUtils
import os
import datetime
from PIL import Image
from PIL import ImageStat

d = g_common_obj.get_device()

class ScreenshotUtils(object):
    current_screenshot = None
    all_screenshots = []
    screen_width = None
    screen_height = None
    force_keep_screenshots = False

    def take_screenshot(self):
        file_path = self.gen_screenshot_path()
        # print file_path
        success = d.screenshot(file_path)
        self.all_screenshots.append(file_path)
        self.current_screenshot = file_path
        screenshot = Image.open(self.current_screenshot)
        self.screen_width, self.screen_height = screenshot.size
#         screenshot.close()
        return success

    def remove_all_screenshots(self):
        if ScreenshotUtils.force_keep_screenshots:
            return
        for file in self.all_screenshots:
            if os.path.isfile(file):
                # print "deleting ", file
                try:
                    os.remove(file)
                except:
                    print "exception encountered when removing", file

    def get_current_screenshot_stat(self):
        if self.current_screenshot is None:
            return None
        image = Image.open(self.current_screenshot)
        current_stat = ImageStat.Stat(image)
#         image.close()
        return current_stat

    def gen_screenshot_path(self):
        time_string = datetime.datetime.strftime(datetime.datetime.now(), '%Y_%m_%d_%H_%M_%S')
        file_name = time_string + "_" + "screenshot.png"
        dir_path = os.path.dirname(__file__)
        return os.path.join(dir_path, file_name)

    def search_for_first_pixel_of_color_in_column(self, color, column_index, color_error=None):
        if self.current_screenshot is None:
            return None
        resx = None
        resy = None
        screenshot = Image.open(self.current_screenshot)
        width, height = screenshot.size
        for y in range(height):
            pixel = screenshot.getpixel((column_index, y))
            if self.same_color(pixel, color, color_error):
                resx = column_index
                resy = y
                break
#         screenshot.close()
        return resx, resy

    def same_screenshots(self, screenshot1_index, screenshot2_index):
        same_screenshots = True
        screenshot1 = Image.open(self.all_screenshots[screenshot1_index])
        screenshot2 = Image.open(self.all_screenshots[screenshot2_index])
        width, height = screenshot1.size
        for i in range(width):
            for j in range(height):
                if screenshot1.getpixel((i, j)) != screenshot2.getpixel((i, j)):
                    same_screenshots = False
#         screenshot1.close()
#         screenshot2.close()
        return same_screenshots

    def get_first_pixel_of_color_from_bottom_right(self, color, color_error=None):
        if self.current_screenshot is None:
            return None
        resx = None
        resy = None
        screenshot = Image.open(self.current_screenshot)
        width, height = screenshot.size
        # go row by row
        for y in range(height - 1, 1, -1):
            for x in range(width - 1, 1, -1):
                pixel = screenshot.getpixel((x, y))
                if self.same_color(pixel, color, color_error):
                    resx = x
                    resy = y
#                     screenshot.close()
                    return resx, resy
#         screenshot.close()
        return resx, resy

    def get_first_pixel_of_color_from_ref_with_direction(self, color, ref, direction, color_error=None):
        if self.current_screenshot is None:
            return None
        screenshot = Image.open(self.current_screenshot)
        width, height = screenshot.size
        piv_x = ref[0]
        piv_y = ref[1]
        while 0 <= piv_x < width and 0 <= piv_y < height:
            pixel = screenshot.getpixel((piv_x, piv_y))
            if self.same_color(pixel, color, color_error):
                return piv_x, piv_y
            piv_x += direction[0]
            piv_y += direction[1]
        return None, None

    def crop_right_square_pixels(self, side_size):
        if side_size > min(self.screen_width, self.screen_height):
            return None
        screenshot = Image.open(self.current_screenshot)
        center_y = self.screen_height / 2
        top = center_y - side_size / 2
        bottom = center_y + side_size / 2
        left = self.screen_width - side_size
        right = self.screen_width
        print "cropping ", left, top, right, bottom
        crop = screenshot.crop((left, top, right, bottom))
#         screenshot.close()
        return list(crop.getdata())

    def crop_upper_right_corner(self, side_size):
        if side_size > min(self.screen_width, self.screen_height):
            return None
        screenshot = Image.open(self.current_screenshot)
        top = 1
        left = self.screen_width - 1 - side_size
        bottom = side_size
        right = self.screen_width - 1
        print "cropping ", left, top, right, bottom
        crop = screenshot.crop((left, top, right, bottom))
#         screenshot.close()
        return list(crop.getdata())

    def same_color(self, color1, color2, color_error):
        if color_error is None:
            return color1 == color2
        else:
            return (abs(color1[0] - color2[0]) < color_error and
                    abs(color1[1] - color2[1]) < color_error and
                    abs(color1[2] - color2[2]) < color_error)

    def compare_color_lists(self, list_a, list_b, color_error=30):
        for i in range(min(len(list_a), len(list_b))):
            if not self.same_color(list_a[i], list_b[i], color_error):
                return False
        return True

    def search_for_first_pixel_of_color(self, color, color_error=None):
        if self.current_screenshot is None:
            return None
        resx = None
        resy = None
        screenshot = Image.open(self.current_screenshot)
        width, height = screenshot.size
        for x in range(width):
            for y in range(height):
                pixel = screenshot.getpixel((x, y))
                if self.same_color(pixel, color, color_error):
                    resx = x
                    resy = y
                    break
#         screenshot.close()
        return resx, resy

    def search_for_last_pixel_of_color(self, color, color_error=None):
        if self.current_screenshot is None:
            return None
        resx = None
        resy = None
        screenshot = Image.open(self.current_screenshot)
        width, height = screenshot.size
        for x in range(width):
            for y in range(height):
                pixel = screenshot.getpixel((x, y))
                if self.same_color(pixel, color, color_error):
                    resx = x
                    resy = y
#         screenshot.close()
        return resx, resy

    def search_for_pixels(self, filter_function):
        conform_pixels = []
        if self.current_screenshot is None:
            return None
        screenshot = Image.open(self.current_screenshot)
        width, height = screenshot.size
        for x in range(width):
            for y in range(height):
                pixel_color = screenshot.getpixel((x, y))
                if filter_function(x, y, pixel_color):
                    conform_pixels.append(ScreenshotPixel(x, y, pixel_color))
        return conform_pixels

    def get_all_pixels_of_color(self, color, color_error=0, min_x=None, min_y=None, max_x=None, max_y=None):
        conform_pixels = []
        if self.current_screenshot is None:
            return None
        screenshot = Image.open(self.current_screenshot)
        width, height = screenshot.size
        for x in range(width):
            for y in range(height):
                pixel_color = screenshot.getpixel((x, y))
                if self.same_color(pixel_color, color, color_error):
                    # if there are some bounds specified, check that the current pixel is within them
                    if min_x is not None and min_y is not None and max_x is not None and max_y is not None:
                        if min_x <= x <= max_x and min_y <= y <= max_y:
                            conform_pixels.append(ScreenshotPixel(x, y, pixel_color))
                    # if there are no bounds specified, pass the current pixel to the result
                    else:
                        conform_pixels.append(ScreenshotPixel(x, y, pixel_color))
        return conform_pixels

    @staticmethod
    def is_pixel_dark_red(x, y, color):
        red_component_value = color[0]
        green_component_value = color[1]
        blue_component_value = color[2]
        # considering dark reds smaller than middle red value
        return (256 / 2 > red_component_value > green_component_value + 20 and
                red_component_value > blue_component_value + 20)

    def get_dark_red_pixels_from_current_screenshot(self):
        return self.search_for_pixels(ScreenshotUtils.is_pixel_dark_red)

    def get_nr_of_pixels_of_color(self, color, color_error=None):
        if self.current_screenshot is None:
            return None
        nr = 0
        screenshot = Image.open(self.current_screenshot)
        width, height = screenshot.size
        for x in range(width):
            for y in range(height):
                pixel = screenshot.getpixel((x, y))
                if self.same_color(pixel, color, color_error):
                    nr += 1
#         screenshot.close()
        return nr

    def crop_center_square_pixels(self, side_size):
        if side_size > min(self.screen_width, self.screen_height):
            return None
        screenshot = Image.open(self.current_screenshot)
        center_x = self.screen_width / 2
        center_y = self.screen_height / 2
        top = center_y - side_size / 2
        bottom = center_y + side_size / 2
        left = center_x - side_size / 2
        right = center_x + side_size / 2
        print "cropping ", left, top, right, bottom
        crop = screenshot.crop((left, top, right, bottom))
#         screenshot.close()
        return list(crop.getdata())

    def crop(self, top, left, bottom, right):
        if not (0 <= top <= self.screen_width and
                0 <= bottom <= self.screen_width and
                0 <= left <= self.screen_height and
                0 <= right <= self.screen_height and
                (self.current_screenshot is not None)):
            return None
        screenshot = Image.open(self.current_screenshot)
        print "cropping left: %s , top: %s , right :%s , bottom: %s" % (left, top, right, bottom)
        crop = screenshot.crop((left, top, right, bottom))
#         screenshot.close()
        return list(crop.getdata())

    def get_nr_of_pixels_of_color_in_crop(self, color, crop_pixels, color_error=10):
        pixels_found = 0
        for pixel_color in crop_pixels:
            if self.same_color(pixel_color, color, color_error):
                pixels_found += 1
        return pixels_found


class ScreenshotPixel(object):
    def __init__(self, coord_x, coord_y, color):
        self.x = coord_x
        self.y = coord_y
        self.color = color

if __name__ == "__main__":
    ssu = ScreenshotUtils()
    print ssu.gen_screenshot_path()
    ssu.current_screenshot = "screen.png"
    print ssu.search_for_first_pixel_of_color_in_column((255, 255, 255), 1280)
