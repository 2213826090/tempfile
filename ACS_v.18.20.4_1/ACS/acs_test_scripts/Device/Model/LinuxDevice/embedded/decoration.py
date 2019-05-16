import threading
from Queue import Queue

SUCCESS = "success"
FAILURE = "failure"
UNKNWON = "unknown"
DEFERRED = "deferred"


class Inner:
    LOGGER = None


class Status():
    """
    class defining the output of command server executor functions.
    """
    def __init__(self, status=UNKNWON, output=None):
        """
        constructor
        :param status: the status of the output, defaulted to UNKNOWN
        :type status: str
        :param output: the output
        :type output: object
        :return: None
        """
        self.status = status
        self.output = output
        self.thread = None
        self.queue = None


def executor(func):
    """
    decoration that is mandatory for functions allowing to execute a command server request in a sequencial manner
    :param func: the encapsulated function
    :type func: callable
    :return: the encapsulated function output, itself encapsulated as a Status object
    :rtype: Status
    """
    def _executor(*args, **kwargs):
        output = func(*args, **kwargs)
        my_output = Status()
        my_output.output = output
        if type(output).__name__ in ("tuple", "list") and len(output) == 2:
            status, output = output
            if status in (SUCCESS, FAILURE):
                my_output.status = status
                my_output.output = output
        return my_output
    return _executor


def threaded_executor(func):
    """
    decoration that is mandatory for functions allowing to execute a command server request in a thread (avoid blocking
    the command server)
    :param func: the encapsulated function
    :type func: callable
    :return: the encapsulated function output, itself encapsulated as a Status object
    :rtype: Status
    """
    def _executor(q, *args, **kwargs):
        result = func(*args, **kwargs)
        q.put(result)

    def _threaded_executor(*args, **kwargs):
        q = Queue()
        t = threading.Thread(target=_executor, args=(q,)+args, kwargs=kwargs)
        t.start()
        my_output = Status(DEFERRED, t.ident)
        my_output.queue = q
        my_output.thread = t
        return my_output
    return _threaded_executor
