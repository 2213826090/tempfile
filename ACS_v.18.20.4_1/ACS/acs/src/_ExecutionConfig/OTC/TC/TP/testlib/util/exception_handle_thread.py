import os,sys,inspect
import time
import threading

class ExceptionHandle(threading.Thread):
    """
    This thread is to handle the exception during the MTBF cases.
    it's started in class common def start_exp_handle() and stopped by
    def stop_exp_handle().
    The spec is to start a thread which searches the right text or widget
    , find it and do the right operation in order not to block the auto test. 
    """
    def __init__(self, deviceObj):
        threading.Thread.__init__(self)
        self.thread_stop = False
        self.d = deviceObj
        self.stopped_exception = False
        self.no_responding_exception = False

    def check_exception(self):
        self.stop_exception()
        self.no_respond_exception()

    def stop_exception(self):
        if self.d(textContains = "has stopped"):
            self.d(text = "OK").click()
            self.stopped_exception = True

    def no_respond_exception(self):
        if self.d(textContains = "isn't responding."):
            self.d(text = "OK").click()
            self.no_responding_exception = True

    def run(self):
        while self.thread_stop == False:
            self.check_exception()
            time.sleep(2)

    def stop(self):
        self.thread_stop = True
