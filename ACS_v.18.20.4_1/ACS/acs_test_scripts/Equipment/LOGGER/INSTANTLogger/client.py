import StringIO
import zipfile
import json, hashlib, math, time, threading, sys, os
from tools import logger
AUTH_REQ_TIMEOUT = 3
REQ_TIMEOUT = 3


try:
    import requests
except ImportError, err:
    logger.error("Failed to import 'requests' module")
    logger.error("You can use 'sudo pip install requests'")
    sys.exit(1)

def getContentType(filename):
    '''
    lists and converts supported file extensions to MIME type
    '''
    ext = filename.split('.')[-1].lower()
    if ext == 'png':
        return 'image/png'
    if ext == 'gif':
        return 'image/gif'
    if ext == 'svg':
        return 'image/svg+xml'
    if ext == 'jpg' or ext == 'jpeg':
        return 'image/jpeg'
    if ext == 'zip':
        return 'application/zip'
    return None

def fbuffer(f_src, chunk_size=1024):
    '''
    read source file. default chunk size is 1024b.
    '''
    while True:
        chunk = f_src.read(chunk_size)
        if not chunk:
            break
        yield chunk

class MemoryZip(object):
    """zip memory object into file"""
    def __init__(self):
        # Create the a memory file-like object
        self.memory_zip = StringIO.StringIO()

    def appendFile(self, file_path, file_name=None):
        '''
        read local source file and add into memory
        '''
        if file_name is None:
            _, fname = os.path.split(file_path)
        else:
            fname = file_name
        content = open(file_path, "rb").read()
        self.append(fname, content)
        return self

    def append(self, filename_in_zip, file_contents):
        '''
        Appends a file with name filename_in_zip and contents of
        file_contents to the in-memory zip
        '''
        # Get a handle to the in-memory zip in append mode
        f_zip = zipfile.ZipFile(self.memory_zip, "a",
                           zipfile.ZIP_DEFLATED, False)
        # Write the file to the in-memory zip
        f_zip.writestr(filename_in_zip, file_contents)
        # Mark the files as having been created on Windows so that
        # Unix permissions are not inferred as 0000
        for zfile in f_zip.filelist:
            zfile.create_system = 0
        return self

    def read(self):
        '''
        Returns a string with the contents of the in-memory zip
        '''
        self.memory_zip.seek(0)
        return self.memory_zip.read()

    def writetofile(self, filename):
        '''
        Writes the in-memory zip to a file.
        '''
        f_zip = file(filename, "wb")
        f_zip.write(self.read())
        f_zip.close()

def retry(tries, delay=1, backoff=2):
    '''
    retries a function or method until it returns True.
    delay sets the initial delay, and backoff sets how much the delay should
    lengthen after each failure. backoff must be greater than 1, or else it
    isn't really a backoff. tries must be at least 0, and delay greater than 0.
    @type tries: int
    @param tries: the retry times
    @type delay: int
    @param delay: the retry duration from last request to next request
    @type backoff: int
    @param backoff: used to make the retry duration wait longer
    @rtype: boolean
    @return: True if the function return True. else return False
    '''
    if backoff <= 1:
        raise ValueError("backoff must be greater than 1")
    tries = math.floor(tries)
    if tries < 0:
        raise ValueError("tries must be 0 or greater")
    if delay <= 0:
        raise ValueError("delay must be greater than 0")
    def deco_retry(func):
        """decorator"""
        def f_retry(*args, **kwargs):
            """retry function"""
            mtries, mdelay = tries, delay # make mutable
            ret = func(*args, **kwargs) # first attempt
            while mtries > 0:
                if ret != None or type(ret) == str or type(ret) == dict:
                    return ret
                mtries -= 1      # consume an attempt
                time.sleep(mdelay) # wait...
                mdelay *= backoff  # make future wait longer
                ret = func(*args, **kwargs) # Try again
            logger.error("couldn't connect to report server.")
            return False # Ran out of tries
        return f_retry # true decorator -> decorated function
    return deco_retry  # @retry(arg[, ...]) -> true decorator

@retry(3)
def request(method, url, data=None, **kwargs):
    '''
    Sends a request.
    :param url: URL for the request.
    :param method: http method(get, post, put, delete)
    :param data: (optional) Dictionary, bytes, or file-like object
    :param kwargs: Optional arguments that request takes
    :return: dict or None
    '''
    ret = None
    meth = method.lower()
    if method.lower() in ('get', 'post', 'put', 'delete'):
        req = getattr(requests, meth, None)
    try:
        ret_code = req(url=url, data=data, **kwargs)
        if ret_code:
            ret = ret_code.json()
    except requests.exceptions.Timeout, err:
        #sys.stderr.write(str(e))
        logger.debug(str(err))
    except requests.exceptions.TooManyRedirects, err:
        #sys.stderr.write(str(e))
        logger.debug(str(err))
    except requests.exceptions.RequestException, err:
        logger.debug(str(err))
        #sys.stderr.write(str(e))
    except Exception, err:
        logger.debug(str(err))
        #sys.stderr.write(str(e))
    return ret

class ReportClient(object):
    '''
    client to communicate with server
    '''
    def __init__(self, **kwargs):
        '''init with keywords'''
        if kwargs:
            self.__dict__.update(kwargs)
        self.token = None
        self.session_id = None
        self.created = False

    def regist(self, **kwargs):
        '''get token from server'''
        md5hash = hashlib.md5()
        md5hash.update(self.__dict__['password'])
        values = json.dumps({'subc': 'login',
                             'data': {'appid':'01',
                                      'username': self.__dict__['username'],
                                      'password': md5hash.hexdigest()}
                            })
        headers = {'content-type': 'application/json',
                   'accept': 'application/json'}
        auth_url = self.__dict__['auth']
        try:
            ret = request(method='post', url=auth_url, data=values,
                        headers=headers, timeout=AUTH_REQ_TIMEOUT)
            #{u'msg': u'',
            # u'data': {u'token': u'306fddbabe37011903e8f', u'uid': 2},
            # u'result': u'ok|error'}
            self.token = ret['data']['token']
        except Exception, err:
            logger.debug('error: regist\n%s' % str(err))
        return self.token

    def createSession(self, **kwargs):
        '''
        session_properties = {'sid': self.session_id,\
                              'product': 'p',\
                              'revision': 'r',\
                              'deviceid': 'devid',\
                              'planname': 'test.plan',\
                              'starttime': self.conf.test_start_time
                              }
        '''
        self.session_id = kwargs.pop('sid')
        url = self.__dict__['session_create'] % self.session_id
        headers = {'content-type': 'application/json',
                   'accept': 'application/json'}

        deviceinfo = {'product':self.__dict__['product'],
                    'deviceid':self.__dict__['deviceid'],
                    'uuid':self.__dict__['uuid'],
                    'crash_tool_url':self.__dict__['crash_tool_url'],
                    'revision':self.__dict__['revision'],
                    'width':self.__dict__['screen_width'],
                    'height':self.__dict__['screen_height']}
        values = json.dumps({'subc': 'create',
                             'token': self.token,
                             'data': {'planname':self.__dict__['planname'],
                                     'starttime':kwargs.pop('starttime'),
                                     'deviceinfo':deviceinfo}})
        try:
            ret = request(method='post', url=url, data=values,
                          headers=headers, timeout=REQ_TIMEOUT)
            #{u'msg': u'', u'data': {}, u'result': u'ok'}
            if ret['result'] == 'ok':
                self.created = True
        except Exception, err:
            logger.debug('error: create session\n%s' % str(err))
        return self.created

    def updateTestCase(self, **kwargs):
        """update testcase result"""

        logger.debug('called in updateTestCase')
        print kwargs
        result_url = self.__dict__['case_update'] % self.session_id
        file_url = self.__dict__['file_upload'] % (self.session_id,
                                            kwargs['payload']['tid'])
        kwargs.update({'token':self.token,
                       'result_url': result_url,
                       'file_url': file_url})
        UploadThread(**kwargs).start()

    def updateSession(self, **kwargs):
        '''
        session_properties = {    'sid': self.session_id,
                                  'product': 'p',
                                  'revision': 'r',
                                  'deviceid': 'devid',
                                  'planname': 'test.plan',
                                  'starttime': self.conf.test_start_time
                                 }
        '''
        self.session_id = kwargs.pop('sid')
        url = self.__dict__['session_update'] % self.session_id
        headers = {'content-type': 'application/json',
                   'accept': 'application/json'}

        values = json.dumps({'subc': 'update',
                             'token': self.token,
                             'data' : kwargs})
        try:
            ret = request(method='post', url=url, data=values,
                          headers=headers, timeout=REQ_TIMEOUT)
        #{u'msg': u'', u'data': {}, u'result': u'ok'}

            if ret['result'] == 'ok':
                pass
        except Exception, err:
            logger.debug('error: update session\n%s' % str(err))


class UploadThread(threading.Thread):
    '''
    Thread for uploading result.
    '''
    def __init__(self, callback=None, **kwargs):
        '''
        Init the instance of Sender.
        '''
        super(UploadThread, self).__init__()
        #CHANGE: in order to wait all rquest thread finish.
        #self.daemon = True
        self.kwargs = kwargs
        self.callback = callback

    def run(self):
        '''
        The work method.
        '''
        try:
            if self.kwargs['payload']['result'] == 'pass':
                if self.basicPayloadRequest(**self.kwargs):
                    self.extrasRequest(**self.kwargs)
            elif self.kwargs['payload']['result'] == 'fail':
                if self.basicPayloadRequest(**self.kwargs):
                    self.extrasRequest(**self.kwargs)
            elif self.kwargs['payload']['result'] == 'error':
                if self.basicPayloadRequest(**self.kwargs):
                    self.extrasRequest(**self.kwargs)
        except Exception, err:
            logger.debug('error: upload thread run\n%s' % str(err))
        finally:
            if self.callback:
                self.callback()

    @staticmethod
    def log(output):
        """write test log"""
        try:
            with open('mylog.txt', 'a') as f_log:
                f_log.write('%s%s' % (str(output), os.linesep))
        except:
            logger.debug('error: open log file error')

    @staticmethod
    def basicPayloadRequest(**kwargs):
        """basic payload request"""
        headers = {'content-type': 'application/json',
                   'accept': 'application/json'}
        result_url = kwargs.pop('result_url')
        token = kwargs.pop('token')
        values = json.dumps({'subc':'update',
                             'token':token,
                             'data': kwargs['payload']})
        try:
            ret = request(method='post', url=result_url, data=values,
                          headers=headers, timeout=REQ_TIMEOUT)
            #{u'msg': u'', u'data': {}, u'result': u'ok'}
            return ret['result'] == 'ok'
        except Exception, err:
            logger.debug('error: basicRequest\n%s' % str(err))
        return False

    @staticmethod
    def extrasRequest(**kwargs):
        """extra request"""
        file_url = kwargs.pop('file_url')
        log = kwargs['extras']['log']
        snapshot = kwargs['extras']['screenshot_at_last']
        try:
            files = {'file': open(snapshot, 'rb')}
            headers = {'content-type': 'image/png',
                       'Ext-Type':'%s%s%s' % ('expect', ':', 'step'),
                       'accept': 'application/json'}
            request(method='put', url=file_url,
                           headers=headers, data=files['file'], timeout=10)
        except Exception, err:
            logger.debug('error: extraRequest snapshot\n%s' % str(err))
        headers = {'content-type': 'application/zip',
                   'accept': 'application/json'}
        try:
            files = {'file': open(log, 'rb')}
            request(method='put', url=file_url,
                        headers=headers, data=files['file'], timeout=10)
        except Exception, err:
            logger.debug('error: extraRequest log\n%s' % str(err))
