from uiautomator import device as android_device


def swipe_right():
    sx = android_device.info[u'displaySizeDpX'] / 2
    sy = android_device.info[u'displaySizeDpY'] / 2
    ex = sx - 100
    ey = sy - 100
    android_device.swipe(sx, sy, ex, ey, steps=5)


def swipe_left():
    sx = android_device.info[u'displaySizeDpX'] / 2
    sy = android_device.info[u'displaySizeDpY'] / 2
    ex = sx + 100
    ey = sy + 100
    android_device.swipe(sx, sy, ex, ey, steps=5)