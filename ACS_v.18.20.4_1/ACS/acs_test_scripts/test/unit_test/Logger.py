'''
Created on Jul 5, 2013

:author: fbo
'''


class Logger():
    '''
    classdocs
    '''

    LEVEL_INFO = "info"
    LEVEL_DEBUG = "debug"
    LEVEL_WARNING = "warning"
    LEVEL_ERROR = "error"

    def __init__(self):
        '''
        Constructor
        '''
        self._msg = []
        self._level = None

    def __is_level(self, msg, level, index):
        """
        """

        msg_ndx = index or len(self._msg) - 1
        return (self._level == level and self._msg[msg_ndx] == msg) \
            if msg_ndx < len(self._msg) else False

    def is_info(self, msg, index=None):
        """
        """

        return self.__is_level(msg, self.LEVEL_INFO, index)

    def is_debug(self, msg, index=None):
        """
        """

        return self.__is_level(msg, self.LEVEL_DEBUG, index)

    def is_warning(self, msg, index=None):
        """
        """

        return self.__is_level(msg, self.LEVEL_WARNING, index)

    def log(self, index):
        """
        """
        return None if index >= len(self._msg) else self._msg[index]

    def info(self, *arg):
        """
        """
        self._log_msg(self.LEVEL_INFO, *arg)

    def debug(self, *arg):
        """
        """
        self._log_msg(self.LEVEL_DEBUG, *arg)

    def warning(self, *arg):
        """
        """
        self._log_msg(self.LEVEL_WARNING, *arg)

    def error(self, *arg):
        """
        """
        self._log_msg(self.LEVEL_ERROR, *arg)

    def _log_msg(self, level, *arg):
        """
        """
        message = arg[0] % (arg[1:])
        self._msg.append(message)
        self._level = level

    def get_level(self):
        """
        """
        return self._level

    def get_msg(self, index=None):
        """
        """
        msg_ndx = index or len(self._msg) - 1
        return None if msg_ndx not in [0, len(self._msg)] else self._msg[msg_ndx]
