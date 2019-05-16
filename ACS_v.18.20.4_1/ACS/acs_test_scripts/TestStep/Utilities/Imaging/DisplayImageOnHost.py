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

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to suspend execution
:since:31/03/2015
:author: pblunie
"""
from Core.TestStep.TestStepBase import TestStepBase
from Device.DeviceManager import DeviceManager
import UtilitiesFWK.Utilities as Util
from threading import Thread
from threading import Event
import Tkinter
import Image
import ImageTk
import re
import platform


class DisplayImageOnHost(TestStepBase):
    """
    Display image on host screen using Tkinter and PIL
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self.__root = None
        self.__event = Event()

    def __start_tk_thread(self):
        """
        Start the thread of TKinter main loop
        """
        if platform.system() != "Windows":
            self.__root = Tkinter.Tk()
            self.__root.attributes("-fullscreen", True)

        self.__root.mainloop()
        self.__root.destroy()

    def __show_tk_img(self):
        """
        Start the thread that actually shows the image
        """
        geom = self.__root.winfo_geometry()
        self._logger.debug("TK Window geometry : %s" % geom)
        match = re.search("^(?P<width>\d+)x(?P<height>\d+).*", geom)

        wimg = int(match.group("width"))
        himg = int(match.group("height"))

        self._logger.debug("Windows dimension is : %dx%d" % (wimg, himg))

        self._logger.debug("Image to print : %s" % self._pars.image_pathname)
        imgfile = Image.open(self._pars.image_pathname)
        if wimg > himg:
            imgfile = imgfile.rotate(90)

        imgfile = imgfile.resize((wimg, himg), Image.ANTIALIAS)

        tkimg = ImageTk.PhotoImage(imgfile)

        tk_lbl_img = Tkinter.Label(self.__root, image=tkimg)
        tk_lbl_img.place(x=0, y=0, width=wimg, height=himg)

        self._logger.debug("Image loading complete")

        self.__event.wait()
        self.__root.quit()

    def run(self, context):
        """
        Display an image given in parameter on host screen using Tkinter
        """
        TestStepBase.run(self, context)

        host_screen = Util.str_to_bool(DeviceManager().get_device_config("PHONE1").get("DisplayScreenAvailable", "False"))
        if host_screen is not None and host_screen:
            # On windows, Tk main window must be instanciated here, otherwise
            # an exception will be raise.
            if platform.system() == "Windows":
                self.__root = Tkinter.Tk()
                self.__root.attributes("-fullscreen", True)

            wthread = Thread(target=self.__start_tk_thread)
            wthread.start()

            wthread.join(0.2)
            while not self.__root.attributes("-fullscreen"):
                wthread.join(0.1)

            self._logger.debug("Tk main window loaded")
            wimgthread = Thread(target=self.__show_tk_img)
            wimgthread.start()

            context.set_info("DisplayImageOnHost_Event", self.__event)
        else:
            self._logger.info("Host screen not available, nothing will be displayed")


class CloseImageOnHost(TestStepBase):
    """
    Close the image
    """
    def run(self, context):
        """
        Close image Tkinter viewer
        """
        TestStepBase.run(self, context)

        host_screen = Util.str_to_bool(DeviceManager().get_device_config("PHONE1").get("DisplayScreenAvailable", "False"))
        if host_screen is not None and host_screen:
            event = context.get_info("DisplayImageOnHost_Event")
            event.set()
        else:
            self._logger.info("Host screen not available, nothing will be closed")
