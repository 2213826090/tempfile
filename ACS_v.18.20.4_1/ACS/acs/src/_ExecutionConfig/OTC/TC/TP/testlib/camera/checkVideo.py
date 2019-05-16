#! /usr/bin/env python

from PIL import Image, ImageStat
import os
import numpy as np
from itertools import izip
from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger

class CheckVideo:
    def check_video_corrupt(self, video_uri):
        """
        Check if a video is corrupted
        :type video_uri: str
        :param video_uri: uri of the video to check
        :rtype: bool
        :return: True if the video is not corrupted False else.
        """
        import cv2
        # Read the video
        video = cv2.VideoCapture(video_uri)
        # Return video status
        ret = video.isOpened()
        if ret==False:
            cmd = "cp "+video_uri+" "+g_common_obj.get_user_log_dir()
            CameraLogger.instance().debug("====copy abnormal video: "+cmd+"======")
            os.system(cmd)
        return ret

    def check_video_brightness(self, video_uri):
        """
        Check video brightness
        :type video_uri: str
        :param video_uri: uri of the video to check
        :rtype: int
        :return: video brightness value
        """
        import cv2
        brightness = 0
        video_frames = []
        # Read the video
        video = cv2.VideoCapture(video_uri)
        # Check if video is correctly opened
        if video.isOpened():
            while True:
                ret, img = video.read()
                if type(img) == type(None):
                    break
                video_frames.append(img)
            # get brightness of each frame of the video
            for iter in range(len(video_frames)):
                img = Image.fromarray(video_frames[iter]).convert('L')
                stat = ImageStat.Stat(img)
                brightness += stat.mean[0]
            # Compute average brightness of video frames as a global
            # Brightness value of the video
            brightness = (brightness / len(video_frames))
        return brightness

    def check_video_resolution(self, video_uri):
        """
        Get video resolution
        :type video_uri: str
        :param video_uri: uri of the video to check
        :rtype: tuple (int, int) & bool
        :return: video resolution & operation status
        """
        import cv2
        video_frames = []
        img_size = (0, 0)
        status = True
        # Read the video
        video = cv2.VideoCapture(video_uri)
        # Check if video is correctly opened
        if video.isOpened():
            while True:
                ret, img = video.read()
                if type(img) == type(None):
                    break
                video_frames.append(img)
            img_size = Image.fromarray(video_frames[0]).size
            # get size of each frame of the video
            for iteration in range(1, len(video_frames)):
                img = Image.fromarray(video_frames[iteration])
                if img_size != img.size:
                    status = False
        return img_size, status

    def compare_videos_size(self, first_video_uri, second_video_uri):
        """
        Compare two video files size
        :type first_video_uri: str
        :param first_video_uri: first video uri
        :type second_video_uri: str
        :param second_video_uri: second video uri
        :rtype: bool
        :return: True if first video's size is higher than the second video's size, False else
        """
        return os.path.getsize(first_video_uri) > os.path.getsize(second_video_uri)

    def get_percent_diff(self, frame1, frame2, percent_threshold=0.10):
        """
        Compare two frame and give the percent of difference between it.
        :type frame1: numpy array
        :param frame1: first frame to be compared
        :type frame2: numpy array
        :param frame2: second frame to be compared
        :type percent_threshold:  float
        :param percent_threshold: percentage difference use in comparison, by default 0.10%
        :rtype: tuple
        :return: a tuple containing percent of difference between two frame as float and match a boolean at True if frame
        are identical, False else.
        """

        # Contruct image from numpy array
        i11 = Image.fromarray(frame1)
        i22 = Image.fromarray(frame2)
        # Resize image and convert in grayscale for faster and easier computation
        i1 = i11.convert("L")
        i2 = i22.convert("L")
        # Fisrst check mode and size
        if i1.mode != i2.mode:
            # Different mode between two image, set variable for match error.
            diff_mode = True
            diff_size = False
            match = False
            percent_diff = 100.0
        elif i1.size != i2.size:
            # Different size between two image, set variable for match error.
            diff_size = True
            diff_mode = False
            match = False
            percent_diff = 100.0
        else:
            diff_mode = False
            diff_size = False
            pairs = izip(i1.getdata(), i2.getdata())
            if len(i1.getbands()) == 1:
                # for grayscale image
                dif = sum(abs(p1 - p2) for p1, p2 in pairs)
            else:
                # for RGB image
                dif = sum(abs(c1 - c2) for p1, p2 in pairs for c1, c2 in zip(p1, p2))
            ncomponents = i1.size[0] * i1.size[1] * 3
            percent_diff = (dif / 255.0 * 100) / ncomponents
            if percent_diff < percent_threshold:
                match = True
            else:
                match = False
        return percent_diff, match, diff_mode, diff_size

    def match_template_on_video_frame(self, frame1, frame2, threshold=0.99):
        """
        Compare two video frame and resize it for faster computation. This comparison use match_template from OpenCv.
        :type frame1: numpy array
        :param frame1: first frame to be compared.
        :type frame2: numpy array
        :param frame2: second frame to be compared.
        :type threshold: float
        :param threshold: percent similarity use in comparison, by default 0.99%
        :rtype: bool
        :return: True if two frame are identical, False else.
        """
        import cv2
        # resize picture for faster computation
        # First check size
        h1, w1, d1 = frame1.shape
        h2, w2, d2 = frame2.shape
        if ((h1 != h2) or (w1 != w2) or (d1 != d2)):
            diff_size = True
            match = False
        else:
            diff_size = False
            # 6 algorithms possible, by default use cv2.TM_CCOEFF_NORMED:
            #   - cv2.TM_CCOEFF
            #   - cv2.TM_CCOEFF_NORMED
            #   - cv2.TM_CCORR
            #   - cv2.TM_CCORR_NORMED
            #   - cv2.TM_SQDIFF
            #   - cv2.TM_SQDIFF_NORMED
            # Check if two image are equal
            res = cv2.matchTemplate(frame1, frame2, cv2.TM_CCOEFF_NORMED)
            loc = np.where(res >= threshold)
            if zip(*loc[::-1]):
                match = True
            else:
                match = False
        return match, diff_size

    def extract_frame_of_video(self, video_uri, box_crop_top, box_crop_bottom, box_crop_left, box_crop_right, size=(128, 128),
                               crop=False):
        """
        Extract frame of a video and return all frame, after resize, in list of numpy array.
        :type video_uri: str
        :param video_uri: uri of the video.
        :type size: tuple
        :param size: Number of pixel (x, y) use is resize.
        :type crop: bool
        :param crop: If True, crop the frame with coordinate in box_crop (box_crop_top, box_crop_bottom, box_crop_left,
                                                                              box_crop_right).
        :type box_crop_top: int
        :param box_crop_top: Pixel number crop at the top of frame.
        :type box_crop_bottom: int
        :param box_crop_bottom: Pixel number crop at the bottom of frame.
        :type box_crop_left: int
        :param box_crop_left: Pixel number crop at the left of frame.
        :type box_crop_right: int
        :param box_crop_right: Pixel number crop at the right of frame.
        :rtype: list
        :return: list of numpy array representing all frame of video.
        """
        import cv2
        video_frame_list = []
        # Open video
        video = cv2.VideoCapture(video_uri)
        if video.isOpened():
            # While video is not finish, extract all frame and add to video_frame_list[].
            while True:
                ret, img = video.read()
                if type(img) == type(None):
                    break
                height, width, depth = img.shape
                # if crop is True, crop the frame with four coordinate.
                if crop:
                    img_crop = img[box_crop_top:(height - box_crop_bottom),
                               box_crop_left: (width - box_crop_right)]
                    img = img_crop
                image = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
                video_frame_list.append(image)
        return video_frame_list

    def get_frame_time(self, num_research_frame, video_frame, video_time):
        """
        Get position in time of a frame in video.
        :type num_research_frame: int
        :param num_research_frame: The position of research frame in video.
        :type video_frame: list
        :param video_frame:List of numpy array representing all frame of video.
        :type video_time: int
        :param video_time: Video duration.
        :rtype: float
        :return: Position in time of num_research_frame.
        """
        time = ((num_research_frame * video_time) / float(len(video_frame)))
        return round(time, 2)

    def compare_two_video_frame_by_frame(self, video1, video2, video1_time, max_consecutive_missing_frame=5, crop=False,
                                         box_crop_top=0, box_crop_bottom=0, box_crop_left=0, box_crop_right=0,
                                         percent_threshold=0.50, match_template_threshold=0.80):
        """
        Compare two video frame by frame.
        :type video1: str
        :param video1: uri of the video1.
        :type video2: str
        :param video2: uri of the video2.
        :type max_missing_frame: int
        :param max_missing_frame: max consecutive missing frame. By default 5.
        :type crop: bool
        :param crop: If True, crop the frame with coordinate in box_crop. By default False.
        :type box_crop_top: int
        :param box_crop_top: Pixel number crop at the top of frame. By default 0.
        :type box_crop_bottom: int
        :param box_crop_bottom: Pixel number crop at the bottom of frame. By default 0.
        :type box_crop_left: int
        :param box_crop_left: Pixel number crop at the left of frame.By default 0.
        :type box_crop_right: int
        :param box_crop_right: Pixel number crop at the right of frame. By default 0.
        :type percent_threshold: float
        :param percent_threshold : threshold for matching pictures pixel by pixel. By default 0.20
        :type match_template_threshold: float
        :param match_template_threshold : threshold for match template. By default 0.80
        :rtype: tuple
        :return: A tuple containing Verdict (True if video are identical, false else) as boolean and msg (message return by
        the method for) as str.
        """
        # Init variable
        matching_frame_counter = 0
        missing_consecutive_frame_counter = 0
        missing_frame_counter = 0
        num_frame = 0
        total_skipped_frame = 0
        warning_msg = ", Warning: "
        verdict = False
        msg = ""

        # Extract frames from first video
        video_frame = self.extract_frame_of_video(self, video1, crop=crop, box_crop_top=box_crop_top, box_crop_bottom=box_crop_bottom,
                                             box_crop_left=box_crop_left, box_crop_right=box_crop_right)

        # Extract frames from second video
        video_frame2 = self.extract_frame_of_video(self, video2, crop=crop, box_crop_top=box_crop_top, box_crop_bottom=box_crop_bottom,
                                              box_crop_left=box_crop_left, box_crop_right=box_crop_right)

        if video_frame == [] or video_frame2 == []:
            raise VideoParameterException("One of video is missed.")

        # Search all frame of first video in second video.
        for i in range(len(video_frame)):
            # Check if the number of missing consecutive frame is lower than max_consecutive_missing_frame, in this case stop
            # the comparison.
            if missing_consecutive_frame_counter == max_consecutive_missing_frame:
                verdict = False
                msg = "Record and reference file are not identical, doesn't match " + str(max_consecutive_missing_frame) + \
                      " consecutive frame at " + str(self.get_frame_time(i, video_frame, video1_time)) + " s."
                break
            for iter in range(num_frame, len(video_frame2)):
                # Use first comparison algorithm match_template
                match, diff_size_match_template = self.match_template_on_video_frame(video_frame[i], video_frame2[iter],
                                                                                match_template_threshold)
                if match:
                    # Use second algorithm percent diff
                    diff_per, match_per, diff_mode, diff_size = self.get_percent_diff(video_frame[i], video_frame2[iter],
                                                                                 percent_threshold=percent_threshold)
                    if match_per:
                        # Claculate the number of skipped frame
                        lag_frame = iter - num_frame
                        # If lag_frame is upper than 0, update warning message.
                        # Screenrecord use a lot of resources to launch and create a lag. To avoid this behaviour, it skip
                        # the first five frame.
                        if ((lag_frame > 0) and (i > 5)):
                            total_skipped_frame += lag_frame
                            warning_msg += str(lag_frame) + " frame skipped at " + \
                                           str(self.get_frame_time(i, video_frame, video1_time)) + " s, "
                        # Update num_frame for restart "for" at last found frame.
                        num_frame = iter + 1
                        # Increment matching_frame_counter
                        matching_frame_counter += 1
                        # Restart missing_frame_counter
                        missing_consecutive_frame_counter = 0
                        break
                    # If two frame have different mode, increment missing_frame_counter
                    elif diff_mode:
                        missing_consecutive_frame_counter += 1
                        missing_frame_counter += 1
                        break
                    # If two frame have different size, increment missing_frame_counter
                    elif diff_size:
                        missing_consecutive_frame_counter += 1
                        missing_frame_counter += 1
                        break
                # If two frame have different size, increment missing_frame_counter
                elif diff_size_match_template:
                    missing_consecutive_frame_counter += 1
                    missing_frame_counter += 1
                    break
                    # If frame i of video1 doesn't match in video2, increment missing_frame_counter
                if iter == (len(video_frame2) - 1):
                    missing_consecutive_frame_counter += 1
                    missing_frame_counter += 1

        # Concatenate result
        if (matching_frame_counter + missing_frame_counter) == len(video_frame) and \
                        total_skipped_frame < (len(video_frame) / 4):
            verdict = True
            msg = "No errors"
            if len(warning_msg) > 12:
                msg += warning_msg
        # Detect if number of skipper frame is upper than length of a quarter video frame.
        elif not verdict and total_skipped_frame > (len(video_frame) / 4):
            verdict = False
            msg = "Too many frame skipped."
        return verdict, msg

class VideoParameterException(Exception):
        def __init__(self, message):
                Exception.__init__(self)
                self.message = message

# def main():
#     print check_video_corrupt("/data/VID_20150430_142022.mp4")
#     print check_video_brightness("/data/VID_20150430_142022.mp4")
#     
# if __name__ == "__main__":
#     main()
