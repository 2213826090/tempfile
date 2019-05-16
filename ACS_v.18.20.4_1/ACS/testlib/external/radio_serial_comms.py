__author__ = "Costin C-tin"

import serial as s, time as t

class comm(object):
    """
    class used to communicate with arduino
    """
    ser = s.Serial()
    baud = 9600 # default baud rate
    port = '/dev/ttyUSB0' # default port

    @classmethod
    def comm_init(self, port='', baud=''):
        comm.port = comm.port if not port else port
        comm.baud = comm.baud if not baud else baud
        if not comm.ser.isOpen():
            comm.ser.setPort(comm.port)
            comm.ser.setBaudrate(comm.baud)
            comm.ser.open()
            comm.ser.flushInput()
            comm.ser.flushOutput()
        print "Init Radio: ", comm.comm_read()


    @classmethod
    def comm_read(self):
        data = ''
        while 1:
            if comm.ser.inWaiting():
                b_to_read = comm.ser.inWaiting()
                data += comm.ser.read(b_to_read)
            if "done" in data:
                break
        if data.strip() != "done":
            print data
        return data


    @classmethod
    def comm_write(self, data):
        if comm.ser.isOpen():
            wrote = comm.ser.write(data)
            return wrote
        else:
            return

    @classmethod
    def comm_isopen(self, port=''):
        if port:
            return comm.ser.isOpen()
        else:
            return None

    @classmethod
    def comm_end(self):
        comm.ser.close()
