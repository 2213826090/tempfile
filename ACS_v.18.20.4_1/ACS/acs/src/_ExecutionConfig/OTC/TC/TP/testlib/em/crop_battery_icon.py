#! /usr/bin/python

from PIL import Image

r, g, b = 0, 1, 2

class CropBatteryImage(object):

    def __init__(self, image_file_path, rect = None, back_point = (0, 0)):
        self.im = Image.open(image_file_path)
        if rect:
            self.im = self.im.crop(rect)
        self.back_pixel = self.im.getpixel(back_point)

    def cmp_pixel(self, px1, px2, diff=10):
        if abs(px1[r]-px2[r]) + abs(px1[g]-px2[g]) + abs(px1[b]-px2[b]) < diff:
            return 0
        else:
            return 1

    def cmp_color_v(self, x, ybeg, yend):
        d = 0
        for y in range(ybeg+1, yend):
            d += self.cmp_pixel(self.im.getpixel((x, y)), self.im.getpixel((x, ybeg)))
        return 1.0*d/(yend-ybeg)

    def cmp_color_h(self, y, xbeg, xend):
        d = 0
        for x in range(xbeg+1, xend):
            d += self.cmp_pixel(self.im.getpixel((x, y)), self.im.getpixel((xbeg, y)))
        return 1.0*d/(xend-xbeg)

    def cmp_two_rows(self, row1, row2, beg, end):
        d = 0
        for x in range(beg, end):
            d += self.cmp_pixel(self.im.getpixel((x, row1)), self.im.getpixel((x, row2)))
        return 1.0*d/(end-beg)

    def cmp_two_cols(self, col1, col2, beg, end):
        d = 0
        for y in range(beg, end):
            d += self.cmp_pixel(self.im.getpixel((col1, y)), self.im.getpixel((col2, y)))
        return 1.0*d/(end-beg)

    def crop_battery_h(self):
        X,Y = self.im.size
        y=Y/2
        borders_h = []
        isImage = False
        for x in range(1, X):
            if self.cmp_color_v(x, 0, Y) == 0:
                if isImage:
                    isImage = False
                    borders_h.append(x)
            else:
                if not isImage:
                    borders_h.append(x)
                    isImage = True
        diff1 = diff2 = 0
        x1 = x2 = 0
        for i in borders_h:
            diff = self.cmp_two_cols(i-1, i, 0, Y)
            if diff1 < diff:
                diff2 =  diff1
                x2 = x1
                diff1 =  diff
                x1 = i
            elif diff2 < diff:
                diff2 = diff
                x2 = i
        self.im = self.im.crop((x1, 0, x2 - 1, Y))

    def crop_battery_v(self):
        X,Y = self.im.size
        x = X/2
        y = Y/2
        y1 = 0
        y2 = Y - 1
        while y1 < y:
            if self.cmp_pixel(self.back_pixel, self.im.getpixel((x, y1))) != 0:
                break
            y1 += 1
        while y2 > y:
            if self.cmp_pixel(self.back_pixel, self.im.getpixel((x, y2))) != 0:
                break
            y2 -= 1
        self.im = self.im.crop((0, y1, X, y2))

    def crop_battery(self):
        self.crop_battery_h()
        self.crop_battery_v()

    def check_lightning_mark(self, row_num):
        im_width, _ = self.im.size
        red_max = 0
        red_min = 255
        for x in range(1, im_width - 1):
            rgb = self.im.getpixel((x, row_num))
            if red_max < rgb[0]:
                red_max = rgb[0]
            if red_min > rgb[0]:
                red_min = rgb[0]
        if red_max - red_min < 50:
            return False
        else:
            return True

    def check_charging_status_by_icon(self, status = True):
        _, im_height = self.im.size
        n = 3
        if status:
            for y in range(n):
                if self.check_lightning_mark((y + 1) * (im_height - 1) / (n + 1)):
                    break
            else:
                assert False, "Not charging"
        else:
            for y in range(n):
                assert not self.check_lightning_mark((y + 1) * (im_height - 1) / (n + 1)), "Charging"

    def check_red_exclamation_in_battery_icon(self, status = True):
        im_width, im_height = self.im.size
        x = im_width / 2
        #for x in range(im_width / 2, im_width):
        #    print "[info]--- Check red exclamation in battery icon, pixel_x: %d" % x
        for y in range(im_height):
            rgb = self.im.getpixel((x, y))
            print rgb
            if rgb[0]/rgb[1] >= 2 and rgb[0]/rgb[2] >= 2:
                return
        assert False, "No red exclamation in battery icon"

if __name__ == "__main__":
    import sys
    cbi = CropBatteryImage(sys.argv[1])
    cbi.crop_battery()
    cbi.im.save(sys.argv[2])
    cbi.check_charging_status_by_icon(True)

