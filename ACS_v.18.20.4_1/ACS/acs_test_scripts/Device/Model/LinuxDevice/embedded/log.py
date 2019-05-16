import logging

_instance = logging.Logger("CS", logging.DEBUG)
log_fmt = "%(asctime)s   %(name)s\t%(levelname)5s   %(message)s"
logging.basicConfig(filename="/tmp/bt_edison.log", level=logging.DEBUG, format=log_fmt)


def Logger(name):
    """
    defines a logger
    :param name: name of the logger
    :type name: str
    :return: the logger instance
    :rtype: Logger
    """
    return _instance.getChild(name)

