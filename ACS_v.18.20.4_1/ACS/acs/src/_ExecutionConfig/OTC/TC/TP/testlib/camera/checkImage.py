#! /usr/bin/env python

from PIL import Image, ImageStat
from PIL.ExifTags import TAGS
import numpy as np
import os
import cv2
from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger

class CheckImage:

    def check_image_corrupt(self, image_uri):
        """
        Check if a picture is corrupted
        :type image_uri: str
        :param image_uri: uri of the picture
        :rtype: bool
        :return: True if the picture is corrupted False else.
        """
        import cv2
        #import cv2.cv as cv
        corrupted = False
        errMsg = ""
        try:
            img = cv2.imread(image_uri)
        except:
            corrupted = True
            errMsg += "Picture is corrupted."
        if img is None:
            corrupted = True
            errMsg += "Picture is corrupted."
        checkColor = False
        try:
            checkColor = self.check_solid_black_or_green(image_uri)
        except Exception, e:
            checkColor = True
            errMsg += "check solid color, exception occurs: %s" %(e)
        if corrupted==False and checkColor ==True:
            corrupted = True
            errMsg += "Picture is solid black or green."
        if errMsg!="":
            cmd = "cp "+image_uri+" "+g_common_obj.get_user_log_dir()
            CameraLogger.instance().debug("====copy abnormal picure: "+cmd+"======")
            os.system(cmd)      
        return errMsg

    def compare_images(self, first_image_uri, second_image_uri):
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

    def check_corruption(self, img):
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

    def compare_images_brightness(self, first_image_uri, second_image_uri):
        """
        Compute difference of brightness between two pictures
        :type first_image_uri: str
        :param first_image_uri: uri of the first picture (ref picture)
        :type second_image_uri: str
        :param second_image_uri: uri of the second picture
        :rtype: bool
        :return: True if ref picture has more brightness else False
        """
        print first_image_uri + " brightness =%f" %self.brightness(first_image_uri)
        print second_image_uri + " brightness =%f" %self.brightness(second_image_uri)
        return self.brightness(first_image_uri) < self.brightness(second_image_uri)

    def brightness(self, im_file):
        """
        Compute Average pixel level of a given picture
        :type im_file: str
        :param im_file: uri of the picture
        :rtype: int
        :return: Average pixel level
        """
        img = cv2.imread(im_file)
        ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
        y = ycrcb[:,:,0]
        return np.mean(y)
        
        #im = Image.open(im_file).convert('L')
        #stat = ImageStat.Stat(im)
        #return stat.mean[0]

    def is_mono(self, im_file, second_image_uri=None):
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

    def is_negative(self, first_image_uri, second_image_uri):
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
        import PIL.ImageOps
        image = Image.open(first_image_uri)
        inverted_image = None
        if image.mode == 'RGBA':
            r,g,b,a = image.split()
            rgb_image = Image.merge('RGB', (r,g,b))
            inverted_image = PIL.ImageOps.invert(rgb_image)
            r2,g2,b2 = inverted_image.split()
            final_transparent_image = Image.merge('RGBA', (r2,g2,b2,a))
            final_transparent_image.save('temp.jpg')
        else:
            inverted_image = PIL.ImageOps.invert(image)
            inverted_image.save('temp.jpg')
        imageA = cv2.imread("temp.jpg")
        imageB = cv2.imread(second_image_uri)
        try:
            mse = self.mse(imageA, imageB)
            CameraLogger.instance().debug("====mse: "+str(mse)+"======")
        except:
            CameraLogger.instance().debug("====Faile to get mse, ignore it======")
            mse = 1000
        os.system("rm temp.jpg")
        #Check the similarity of the two pictures
        if mse < 2000:
            pixels_ok = 0
            percent = 60
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
    #        print data
            # Check result picture is black
            for iter in range(len(data)):
                if data[iter] < 15:
                    pixels_ok += 1
            result = (pixels_ok * 100) / len(data)
            print result
            return result > percent
        else:
            return False

    def get_picture_size(self, image_uri):
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

    def resize_picture(self, picture_uri, new_width, new_height, outout_path, rotate_ref_image=False, crop=False):
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
        result_image_uri = os.path.join(outout_path, picture_uri.split(os.sep)[-1].split('.')[0] + "_resized.bmp")
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

    def set_orientation(self, picture_uri, orientation, outout_path, to_rgb=False):
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
    #         raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Bad orientation value")
            raise "Bad orientation value"
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

    def get_picture_exif_data(self, image_uri, element):
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
            print tag, value
            if TAGS.get(tag) == element:
                return value

    def match_template_in_image(self, scrennshot, template, threshold_value=0.65):
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
        # import cv2 from open cv, this library is use for image checking.
        import cv2
        matching = False
        # Open screenshot
        img = cv2.imread(scrennshot, 0)
        # Open template
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
    
        # Set the probability matching
        threshold = threshold_value
        # Search good probability in result matrix
        loc = np.where(res >= threshold)
        if zip(*loc[::-1]):
            matching = True
        return matching, x, y

    def generate_dictionary_image_path(self, image_lib_path):
        """
        Generate a dictionary with key the name of the image, less extension, and in value the path of the image
        :type image_lib_path: str
        :param image_lib_path: path of the image library
        :rtype: dict
        :return: dictionary with key the name of image and value the path
        """
        images = []
        # Find all files in root and directory
        for root, dirs, files in os.walk(image_lib_path):
            for i in files:
                # Check if the extension is .png
                if i.endswith('.png'):
                    # Add image to the list
                    images.append(os.path.join(root, i))
        image_path_dic = {}
        for image in images:
            # split with os separator
            str = image.split(os.sep)
            name_image = str[-1].split(".")
            # recup only image name
            image_path_dic[name_image[-2]] = image
        return image_path_dic

    def face_detection(self, image_uri):
        """
        Detect faces in a given image
        :type image_uri : str
        :param image_uri : uri of the image to check
        :rtype: int
        :return: number of faces detected
        """
        import cv2
        face_cascade = cv2.CascadeClassifier('/data/haarcascade_frontalface_default.xml')
        img = cv2.imread(image_uri)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.2, 5)
        return len(faces)

    def detect(self, path):
        img = cv2.imread(path)
        cascade = cv2.CascadeClassifier("/data/haarcascade_frontalface_alt.xml")
        rects = cascade.detectMultiScale(img, 1.3, 4, cv2.CASCADE_SCALE_IMAGE, (20, 20))
    #    print len(rects)
        if len(rects) == 0:
            return [], img
        rects[:, 2:] += rects[:, :2]
        return rects, img

    def box(self, rects, img):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
        cv2.imwrite('/data/detected.jpg', img);
        
    def check_solid_black_or_green(self, image_uri):
        """
        Check if a picture is solid black or green
        :type image_uri: str
        :param image_uri: uri of the picture
        :rtype: bool
        :return: True if the picture is solid black or green False else.
        """
        #import cv2
        #img = cv2.imread(image_uri)
        img=Image.open(image_uri)
        #image.rotate(45).show()
        colors = img.convert('RGB').getcolors(300)
        #print img.convert('RGB').getcolors(img.size[0]*img.size[1])
        if colors==None:
            return False
        else:
            if len(colors)==1:
    #            print colors
    #            print colors[0]
                clolor_count = colors[0][0]
                color =  colors[0][1]
                r = color[0]
                g = color[1]
                b = color[2]
                if r==0 and b==0:
                    return True
        return False

# def main():
#     print brightness("/data/Camera_1_test1.jpeg")
#     print brightness("/data/Camera_1_test2.jpeg")
#     print is_mono("/data/Camera_1_test1.jpeg")
#     print is_negative("/data/Camera_1_test1.jpeg", "/data/Camera_1_test2.jpeg")
#     print face_detection("/data/faces_orig.jpg")
#     print face_detection("/data/IMG_20150430_130846.jpg")
#     # print detect("/data/IMG_20150430_130846.jpg")
#     rects, img = detect("/data/faces_orig.jpg")
#     box(rects, img)
# 
# if __name__ == "__main__":
#     main()


