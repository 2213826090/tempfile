import logging
import os
import __builtin__


class Log(object):
    def __init__(self):
        if not hasattr(__builtin__, "ACS_LOGGER"):
            self.logger = logging.getLogger('PyUiApi_Logger')
            # creating file handler to output in file
            log_file_path = os.path.join(self.get_log_path(), "log.txt")
            self.logger.setLevel(logging.INFO)
            hdlr = logging.FileHandler(log_file_path)
            # creating console handler to log in console
            chdlr = logging.StreamHandler()
            chdlr.setLevel(logging.INFO)
            # a comprehensive format for the log entries
            formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
            hdlr.setFormatter(formatter)
            chdlr.setFormatter(formatter)
            self.logger.addHandler(hdlr)
            self.logger.addHandler(chdlr)
        else:
            self.logger = ACS_LOGGER

    def info(self, message, class_ref=None):
        if class_ref is None:
            self.logger.info(str(message))
        else:
            self.logger.info(str(class_ref.__class__.__name__) + " - " + str(message))

    def error(self, message, class_ref=None):
        if class_ref is None:
            self.logger.error(str(message))
        else:
            self.logger.error(str(class_ref.__class__.__name__) + " - " + str(message))

    def warning(self, message, class_ref=None):
        if class_ref is None:
            self.logger.warning(str(message))
        else:
            self.logger.warning(str(class_ref.__class__.__name__) + " - " + str(message))

    @staticmethod
    def get_log_path():
        return os.path.dirname(os.path.realpath(__file__))
