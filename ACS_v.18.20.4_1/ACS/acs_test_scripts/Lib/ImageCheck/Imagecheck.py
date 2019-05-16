"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL AMPS
:summary: Methods for picture check
:since: 23/01/2013
:author: mmorchex
"""

from ErrorHandling.AcsConfigException import AcsConfigException
from PIL import Image, ImageStat
from PIL.ExifTags import TAGS
import numpy as np
import os


def check_image_corrupt(image_uri):
    """
    Check if a picture is corrupted
    :type image_uri: str
    :param image_uri: uri of the picture
    :rtype: bool
    :return: True if the picture is corrupted False else.
    """
    import cv2
    import cv2.cv as cv

    corrupted = False
    try:
        img = cv2.imread(image_uri)
    except:
        corrupted = True

    if img is None:
        corrupted = True

    return corrupted


def compare_images(first_image_uri, second_image_uri):
    """
    Compare two images
    :type first_image_uri : str
    :param first_image_uri : first image uri
    :type second_image_uri : str
    :param second_image_uri : second image uri
    :rtype: bool
    :return: True if images are the same, False else
    """
    first_img_histogram = Image.open(first_image_uri).histogram()
    second_img_histogram = Image.open(second_image_uri).histogram()

    return first_img_histogram == second_img_histogram


def check_corruption(img):
    """
    Check if a picture is corrupted
    :type img: Image
    :param img: picture to check
    :rtype: bool
    :return: True if the picture is corrupted False else.
    """

    corrupted = False

    # Maximum ratio of lines corrupted
    maximum_lines_corrupted_ration = 0.6

    # Maximum ratio of colons corrupted
    maximum_colons_corrupted_ratio = 0.6

    img_height = np.size(img, 0)
    img_width = np.size(img, 1)

    # Check corrupted lines
    for row in range(img_height):
        tmp_color = img[row, 0]
        tmp_counter = 0
        counter = 0
        for col in range(1, img_width):
            if np.array_equal(img[row, col], tmp_color):
                counter += 1
            else:
                tmp_color = img[row, col]
                tmp_counter = max(counter, tmp_counter)

        counter = max(counter, tmp_counter)
        if counter >= img_width * maximum_lines_corrupted_ration:
            corrupted = True

    # If no line is corrupted
    if not corrupted:
        # Check corrupted colons
        for col in range(img_width):
            tmp_color = img[0, col]
            tmp_counter = 0
            counter = 0
            for row in range(1, img_height):
                if np.array_equal(img[row, col], tmp_color):
                    counter += 1
                else:
                    tmp_color = img[row, col]
                    tmp_counter = max(counter, tmp_counter)

            counter = max(counter, tmp_counter)
            if counter >= img_height * maximum_colons_corrupted_ratio:
                corrupted = True

    return corrupted


def compare_images_brightness(first_image_uri, second_image_uri):
    """
    Compute difference of brightness between two pictures
    :type first_image_uri: str
    :param first_image_uri: uri of the first picture (ref picture)
    :type second_image_uri: str
    :param second_image_uri: uri of the second picture
    :rtype: bool
    :return: True if ref picture has more brightness else False
    """
    return brightness(first_image_uri) < brightness(second_image_uri)


def brightness(im_file):
    """
    Compute Average pixel level of a given picture
    :type im_file: str
    :param im_file: uri of the picture
    :rtype: int
    :return: Average pixel level
    """
    im = Image.open(im_file).convert('L')
    stat = ImageStat.Stat(im)
    return stat.mean[0]


def is_mono(im_file, second_image_uri=None):
    """
    Check if a picture is mono
    :type im_file: str
    :param im_file: uri of the picture
    :rtype: bool
    :return: True if im_file is mono else False
    """
    im = Image.open(im_file)
    result = ImageStat.Stat(im).var
    return result[0] == result[1] == result[2]


def is_negative(first_image_uri, second_image_uri):
    """
    Check if a picture is the negative of another one
    :type first_image_uri: str
    :param first_image_uri: uri of the first picture (ref picture)
    :type second_image_uri: str
    :param second_image_uri: uri of the second picture
    :type percent: int
    :param percent: percent of corrupted pixels
    :rtype: bool
    :return: True if second_image_uri is the negative of first_image_uri  else False
    """
    pixels_ok = 0
    percent = 90

    # Load reference image in grayscale
    image_ref = Image.open(first_image_uri).convert('L')
    image_ref_pixels = image_ref.load()

    # Load test image in grayscale
    image_test = Image.open(second_image_uri).convert('L')
    image_test_pixels = image_test.load()

    # Combine reference and test pictures
    data = []
    for i in range(image_ref.size[0]):
        for j in range(image_ref.size[1]):
            data.append(255 - image_ref_pixels[i, j] - image_test_pixels[i, j])

    # Check result picture is black
    for iter in range(len(data)):
        if data[iter] < 10:
            pixels_ok += 1

    result = (pixels_ok * 100) / len(data)
    return result > percent


def check_picture_resolution(camera_name, resolution):
    """
    Check picture resolution relative to a camera name
    :type camera_name: str
    :param camera_name: camera name (FRONT/BACK)
    :type size: str
    :param size: size of picture
    :rtype: bool
    :return: True if the size is relative to the camera name
    """
    camera_name = camera_name .upper()
    if camera_name == "BACK":
        if resolution not in SCREEN_RESOLUTION_BACK_CAMERA:
            return False
    elif camera_name == "FRONT":
        if resolution not in SCREEN_RESOLUTION_FRONT_CAMERA:
            return False
    return True


def get_size_from_resolution(camera_name, resolution):
    """
    Return width and height for a given resolution
    :type resolution: str
    :param resolution: resolution
    :rtype: (int, int)
    :return: width and height corresponding to a resolution
    """
    camera_name = camera_name.upper()
    if camera_name == "BACK":
        for iter in SCREEN_RESOLUTION_BACK_CAMERA.iterkeys():
            if iter == resolution:
                return SCREEN_RESOLUTION_BACK_CAMERA[iter]
    elif camera_name == "FRONT":
        for iter in SCREEN_RESOLUTION_FRONT_CAMERA.iterkeys():
            if iter == resolution:
                return SCREEN_RESOLUTION_FRONT_CAMERA[iter]


def check_picture_size(camera_name, image_uri, resolution):
    """
    Check if a picture's resolution is the same as a given one
    :type image_uri: str
    :param image_uri: uri of the image to check
    :type resolution: str
    :param resolution: resolution to check
    :rtype: bool
    :return: True if the size of the picture is equal to size given in parameter
    """
    (width, height) = get_size_from_resolution(camera_name, resolution)
    img = Image.open(image_uri)
    if img.size != (width, height):
        return False
    return True


def get_picture_size(image_uri):
    """
    Get picture resolution
    :type image_uri: str
    :param image_uri: uri of the image to check
    :rtype: tuple
    :return: (picture width, picture height)
    """
    try:
        img = Image.open(image_uri)
    except:
        return 0, 0

    return img.size


def resize_picture(picture_uri, new_width, new_height, outout_path, rotate_ref_image=False, crop=False):
    """
    Resize a picture.
    :type picture_uri: str
    :param picture_uri: path to picture to resize
    :type new_width: int
    :param new_width: new width to set
    :type new_height: int
    :param new_height: new height to set
    :type outout_path: str
    :param outout_path: directory where to save picture resized
    :type rotate_ref_image: bool
    :param rotate_ref_image: specify if the reference picture should be rotated or not
    :type crop: bool
    :param crop: crop 20 % of the picture
    :rtype: str
    :return: picture resized's uri
    """
    image = Image.open(picture_uri)
    if rotate_ref_image:
        image = image.transpose(Image.ROTATE_180)
    actual_width, actual_height = float(image.size[0]), float(image.size[1])
    new_width, new_height = float(new_width), float(new_height)

    if actual_width > actual_height:
        coeff = actual_width / new_width
        actual_width = new_width
        actual_height /= coeff

        if actual_height > new_height:
            coeff = actual_height / new_height
            actual_height = new_height
            actual_width /= coeff

    else:
        coeff = actual_height / new_height
        actual_height = new_height
        actual_width /= coeff

        if actual_width > new_width:
            coeff = actual_width / new_width
            actual_width = new_width
            actual_height /= coeff

    result_image_uri = os.path.join(outout_path, picture_uri.split(os.sep)[-1].split('.')[0]+"_resized.bmp")
    result_image = image.resize((int(actual_width), int(actual_height)))
    height, width = result_image.size
    if crop:
        left = int(width * 0.2)
        top = 0
        right = int(width * 0.6)
        bottom = height
        img_crop = result_image.crop((left, top, right, bottom)).save(result_image_uri)
    else:
        result_image.save(result_image_uri)

    return result_image_uri


def set_orientation(picture_uri, orientation, outout_path, to_rgb=False):
    """
    Set picture orientation
    :type orientation: str
    :param orientation: orientation to set
    :type outout_path: str
    :param outout_path: directory where to save picture modified
    :rtype: str
    :return: modified picture's uri
    """
    # Check orientation value
    if orientation.lower() not in ("landscape", "reverse_landscape"):
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Bad orientation value")
    else:
        image = Image.open(picture_uri)

        # Apply rotation
        if orientation.lower() == "landscape":
            desc = "landscape"
            image_modified = image.transpose(Image.ROTATE_270)
        else:
            desc = "reverse_landscape"
            image_modified = image.transpose(Image.ROTATE_90)

        image_modified_uri = os.path.join(outout_path, picture_uri.split(os.sep)[-1].split('.')[0] + "_" + desc
                                                           + ".bmp")
        if to_rgb:
            image_modified = image_modified.convert('RGB')

        image_modified.save(image_modified_uri)
        return image_modified_uri


def get_picture_exif_data(image_uri, element):
    """
    Parse picture's EXIF data and return the value of a specified element given as argument
    :type image_uri: str
    :param image_uri: uri of the image to check
    :type element: str
    :param element: Information to get from picture's EXIF data
    :rtype: int
    :return: value of the specified element of picture's EXIF data
    """
    for (tag, value) in Image.open(image_uri)._getexif().iteritems():
        if TAGS.get(tag) == element:
            return value


treatment_methods = [{'effect': 'flash', 'method': compare_images_brightness, 'attributes': 2}, \
                      {'effect': 'negative', 'method': is_negative, 'attributes': 2}, \
                      {'effect': 'night', 'method': compare_images_brightness, 'attributes': 2}, \
                      {'effect': 'mono', 'method': is_mono, 'attributes': 1}]

# Screen resolution for BACK camera
SCREEN_RESOLUTION_BACK_CAMERA = {
    'QVGA': (320, 240),
    'VGA': (640, 480),
    '1MP': (1024, 768),
    '3MP': (2048, 1536),
    '5MP': (2560, 1920),
    '8MP': (3264, 2448),
}

# Screen resolution for FRONT camera
SCREEN_RESOLUTION_FRONT_CAMERA = {
    'VGA': (640, 480),
    '1.3MP': (1280, 2448),
}

def match_template_in_image(scrennshot, template, threshold_value=0.65):
    """
    Find an image template in a screenshot of the board
    :type screenshot: str
    :param screenshot: path of the screenshot
    :type template: str
    :param template: path of the image template
    :type threshold_value: float
    :param threshold_value: threshold for matching
    :rtype: tuple
    :return: (boolean, int, int) containing (true if the template is found in screenshot, horizontal coordinate in
    pixel, vertical coordinate in pixel.
    """
    #import cv2 from open cv, this library is use for image checking.
    import cv2

    matching = False

    #Open screenshot
    img = cv2.imread(scrennshot, 0)
    #Open template
    temp = cv2.imread(template, 0)

    # 6 algorithms possible, by default use cv2.TM_CCOEFF_NORMED:
    #   - cv2.TM_CCOEFF
    #   - cv2.TM_CCOEFF_NORMED
    #   - cv2.TM_CCORR
    #   - cv2.TM_CCORR_NORMED
    #   - cv2.TM_SQDIFF
    #   - cv2.TM_SQDIFF_NORMED

    methods = 'cv2.TM_CCOEFF_NORMED'

    method = eval(methods)

    # Apply template Matching
    res = cv2.matchTemplate(img, temp, method)
    y, x = np.unravel_index(res.argmax(), res.shape)

    #Set the probability matching
    threshold = threshold_value
    #Search good probability in result matrix
    loc = np.where(res >= threshold)
    if zip(*loc[::-1]):
        matching = True

    return matching, x, y


def generate_dictionary_image_path(image_lib_path):
    """
    Generate a dictionary with key the name of the image, less extension, and in value the path of the image
    :type image_lib_path: str
    :param image_lib_path: path of the image library
    :rtype: dict
    :return: dictionary with key the name of image and value the path
    """
    images = []
    #Find all files in root and directory
    for root, dirs, files in os.walk(image_lib_path):
        for i in files:
            #Check if the extension is .png
            if i.endswith('.png'):
                #Add image to the list
                images.append(os.path.join(root, i))
    image_path_dic = {}
    for image in images:
        #split with os separator
        str = image.split(os.sep)
        name_image = str[-1].split(".")
        #recup only image name
        image_path_dic[name_image[-2]] = image

    return image_path_dic


def face_detection(image_uri):
    """
    Detect faces in a given image
    :type image_uri : str
    :param image_uri : uri of the image to check
    :rtype: int
    :return: number of faces detected
    """
    import cv2

    face_cascade = cv2.CascadeClassifier(os.path.join('_ExecutionConfig', 'haarcascade_frontalface_default.xml'))

    img = cv2.imread(image_uri)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    return len(faces)
