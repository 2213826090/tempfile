import datetime


class CommonUtils(object):

    @staticmethod
    def get_current_time_string():
        return datetime.datetime.strftime(datetime.datetime.now(), '%Y_%m_%d_%H_%M_%S')