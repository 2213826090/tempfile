import logging


class Log:
    def __init__(self):
        self.debug("Log is online ...")

    @staticmethod
    def debug(message):
        logging.debug(message)
