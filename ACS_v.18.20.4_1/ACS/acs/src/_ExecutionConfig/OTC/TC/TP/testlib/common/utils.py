import sys
import os
import time
import re
try:
    import zbar
except:
    zbar=None
try:
    import Image
except:
    Image=None

base_path = os.path.dirname(__file__)

def getBarCode(pic):
    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')
    pil = Image.open(pic).convert('L')
    width, height = pil.size
    raw = pil.tostring()
    image = zbar.Image(width, height, 'Y800', raw)
    scanner.scan(image)
    result=""
    for symbol in image:
        # do something useful with results
        #print "get bar code:", '%s' % symbol.data
        if result == "":
            result = str(symbol.data)
        else:
            result = result + " " + str(symbol.data)
    return result

def getBarcodeSet(playtime, rePattern, getsc, sleeptime):
    if isinstance(getsc, list):
        barCodeSetList = [set()] *len(getsc)
        getscList = getsc
    else:
        barCodeSetList = [set()]
        getscList = [getsc]
    reCorrectBarCode = re.compile(rePattern)
    start = time.time()
    print "playtime: %d"%playtime
    while(True):
        end = time.time()
        if end - start > playtime:
            break
        #try:
        if True:
            for i in range(len(getscList)):
                sc = getscList[i]()
                code = getBarCode(sc)
                if code:
                    print "get bar code via %s:"%getscList[i].__name__, code
                else:
                    print "can not get bar code via %s"%getscList[i].__name__
                if len(re.findall(reCorrectBarCode, code))==1:
                    barCodeSetList[i].add(code)
#         except BaseException,e:
#             raise e
#             pass
        time.sleep(sleeptime)
    return barCodeSetList
    #return barCodeSetList[0]

def getImageFromHostCamera(path):
        v4l = os.path.join(os.path.dirname(__file__), "..", "tools", "v4l2grab")
        v4l = os.path.normpath(v4l)
        cmd = v4l + ' -o %s'
        os.system(cmd%path)
        return path


def trySomeTimes(func, times=1, *args, **kwargs):
    for i in range(1, times+1):
        try:
            return func(*args, **kwargs)
        except KeyboardInterrupt:
            raise
        except:
            if i>=times:
                raise


def catchException(func, *args, **kwargs):
    e=None
    ret=None
    try:
        ret = func(*args, **kwargs)
    except KeyboardInterrupt:
            raise
    except BaseException, e:
        pass
    return e, ret

if __name__ == "__main__":
    def test():
        print "test"
        raise Exception("test")
        return 1
    print catchException(test)



