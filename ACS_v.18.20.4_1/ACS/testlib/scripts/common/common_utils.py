import datetime


def get_date_diff(date1, date2):
        date1 = int(datetime.datetime.strftime(date1, "%Y%m%d%H%M"))
        date2 = int(datetime.datetime.strftime(date2, "%Y%m%d%H%M"))
        diff_date = abs(date2-date1)
        return diff_date


def get_host_date():
    return datetime.datetime.utcnow()
