#!/usr/bin/python

__author__ = "Costin C-tin"

# the main file of the radio control interface

import logging as lg, os, time as t
import argparse
from ConfigParser import SafeConfigParser
from testlib.external import radio_SI4713_prop_defs as defs
from testlib.external import radio_serial_comms as comms

def log_filename():
    lt = t.localtime()
    timestamp = '{}-{}-{}_{}:{}:{}'.format(lt.tm_year, lt.tm_mon, lt.tm_mday, lt.tm_hour,
                                        lt.tm_min, lt.tm_sec)
    filename = "radio_{}.log".format(timestamp)
    return filename

log_file = log_filename()



def get_args_parser(args=None):
    """
    Read the list of arguments from command line
    :return:
    """
    description = "Script to interface with the SI47313 FM RDS capable radio chip via Arduino Uno. Instead of Uno, Arduino Nano can be used."
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('-s', '--scan', action="store_true", dest="scan", help='Scan FM frequency domain 88-108MHz for signal strength. Very useful to chose a frequency where to emit on.')
    parser.add_argument('-p', '--set-rf-level', type = int, dest="signal_level", choices=range(88,115,1), help='Set Tx signal level 88-115dB uV.')
    parser.add_argument('-f', '--set-frequency', type=int, dest="set_f", help='Set Tx frequency')
    # from this point onwards, options are for RDS
    parser.add_argument('-n', '--radio-name', dest="rn", help='Set radio name. By default the name is "AndroidR"')
    parser.add_argument('-t', '--radio-text', dest="rt", help='Set radio text. By default the radio text is "android radio text"')
    parser.add_argument('--TA', dest="TA", default="off", choices=['on', 'off'], help='Set TA(Traffic Announcement) to on/off')
    parser.add_argument('--PI', dest="pi", help="Set PI(Program Identification) code. This is in theory a unique code assigned to each radio station.")
    parser.add_argument('-m', '--mode', dest="m_s", choices=["mono","stereo"], help="Specify if transmission is mono or stereo")
    parser.add_argument('--PS', dest="ps", type=int, help="Set PS(Program Service).")
    parser.add_argument('--PTY', dest="pty", type=int, choices=defs.PTY_rds.keys(), help="Set PTY(Program Type Code).")
    parser.add_argument('-d','--load-defaults', dest="def_cfg", action='store_true', help="Load defaults to SI4713 as described in circuit datasheet")
    parser.add_argument('-l','--load-last-cfg', dest="last_cfg", action='store_true', help="Load config as found in properties.cfg file")

    if args:
        return parser.parse_args(args.split())
    else:
        return parser.parse_args()

def interpret_args(args=None):

    arg_dict = dict()
    args = get_args_parser(args)

    arg_dict['s'] = 1 if args.scan else 0 # 1 if a scan is needed
    arg_dict['f'] = args.set_f if args.set_f else 0
    arg_dict['pwr'] = args.signal_level if args.signal_level else 0
    # from now on related to RDS
    arg_dict['rn'] = args.rn if args.rn else 0 # radio name  if name exists else 0
    arg_dict['rt'] = args.rt if args.rt else 0 # radio text  if text exists else 0
    arg_dict['ps'] = args.ps if args.ps else 0
    arg_dict['pi'] = args.pi if args.pi else 0



    #from here on the TX_RDS_PS_MISC property is affected
    arg_dict['t'] = 1 if args.TA == "on" else 0 # 1 if TA is needed
    arg_dict['m'] = 0 if args.m_s == "mono" else 1 # 1 if stereo
    arg_dict['pty'] = args.pty if args.pty else 0

    #read and load config

    #print arg_dict
    return arg_dict


def compose_properties_dict(args_dict):
    pd = dict()
    ad = args_dict # the dict of parameters sent in command line
    #from here on the TX_RDS_PS_MISC property is affected

    TX_RDS_PS_MISC_value = load_property('TX_RDS_PS_MISC') # using default values for this property
    TX_RDS_PS_MISC_value = modify_property(TX_RDS_PS_MISC_value, bit_no=4, bit_value=ad['t'])
    TX_RDS_PS_MISC_value = modify_property(TX_RDS_PS_MISC_value, bit_no=12, bit_value=ad['m'])
    if ad['pty'] !=0:
        TX_RDS_PS_MISC_value = modify_property(TX_RDS_PS_MISC_value, r_msb=9, r_lsb=5, r_value=ad['pty'])
    pd['TX_RDS_PS_MISC'] = TX_RDS_PS_MISC_value

    #print pd
    return pd


def load_property(pr_name, pr_section='',mode="default"): # loads the property as needed, either from config file or from default
    pr_dict = { # just add the mapping between property name and property dictionary  as done in SI4713_prop_defs.py file
        'TX_RDS_PS_MISC' : defs.TX_RDS_PS_MISC
    }

    pr_value = ''
    if mode == "default":
        def_pr = pr_dict[pr_name]
        for i in reversed(range(16)): # 15 .. 0
            pr_value += def_pr[i]
        return  int(pr_value,2) # will give an int for the bin value
    # this part with loadingfrom file not tested
    else:
        pr_value = rw_cfg_file(pr_name, section=pr_section)
        return int(pr_value, 16) # return the value of the property as found in the properties.cfg file


def modify_property(prop_value, bit_no=-1, bit_value=-1, r_lsb=-1, r_msb=-1, r_value=-1): # gets an integer and modifies it at bit level
    """

    :param prop_value: this is the integer we want to modify, being the value of a property; we are not interested in the prop. name
    :param bit_no: if only 1 bit is to be mod, then this is the no. of it
    :param bit_value: per the above option this is the value of the bit
    :param lsb: for a range of bits, the lsb of the range from the prop. value
    :param msb: for a range of bits, the msb of the range from the prop. value
    :param r_value: the value for the range
    :return:
    """
    word_len = 16
    prop_value_bin = bin(prop_value)[2:] # string representing the value in binary of the given value
                                        # however, keep in mind strings are immutable
    if len(prop_value_bin) != word_len: # it means we need to add extra '0' in front of prop_value_bin
        prop_value_bin = '0'*(word_len - len(prop_value_bin)) + prop_value_bin # now we have the full bin word

    prop_mod_value_bin = ""
    if bit_no !=-1 and bit_value!=-1:
        for i in reversed(range(16)): # we have 16 bits to treat
            if i == bit_no:
                if bit_value == 0:
                    prop_mod_value_bin += '0'
                else:
                    prop_mod_value_bin += '1'
            else:
                prop_mod_value_bin += prop_value_bin[15-i]

    elif r_lsb!=-1 and r_msb!=-1 and r_value!=-1:
        l = (r_msb - r_lsb) + 1
        r_value_bin = bin(r_value)[2:]
        if len(r_value_bin) != l:
            r_value_bin = '0' * (l-len(r_value_bin)) + r_value_bin

        for i in reversed(range(16)):
            if i>=r_lsb and i<=r_msb:
                prop_mod_value_bin += r_value_bin[r_msb - i]
            else:
                prop_mod_value_bin += prop_value_bin[15-i]
    return int(prop_mod_value_bin, 2)


def rw_cfg_file(property, section='',value='',mode='r'):
    parser = SafeConfigParser()
    cfg_file = os.path.join(__path__, "properties.cfg")

    sec = section
    if not sec:
        sec = 'properties'

    if mode == 'r':
        parser.read(cfg_file)
        for name, value in parser.items(sec):
            if property == "name":
                return value
                break
    else:
        #parser.add_section('bug_tracker')
        parser.set(sec, property, value)

def compose_word_to_send(args_dict):
    a_d = args_dict
    wts_list = list()

    for k in ['s', 'f', 'pwr', 'rn', 'rt', 'ps', 'pi']: # this will populate the options that don't modify a chip property
        wts_list.append("{}:{}".format(k, a_d[k]))

    properties_dict = compose_properties_dict(a_d)
    wts_list.append('{}:{}'.format('TX_RDS_PS_MISC', properties_dict["TX_RDS_PS_MISC"]))
    return wts_list

class radio(comms.comm):
    def __init__(self, port):
        self.comm_init(port=port)

    def send_to_radio(self,wts_l):
        """
        we'll prepare a list of split params of max 2 params sent at a time
        The reason for this is the small buffer Arduino has in order to read
        incomming data via serial
        :param wts_l:
        :return:
        """
        def send_word_to_radio(wts):
            w_len = len(wts)
            print "Word to send: ",wts
            #print "Word length: ", w_len
            sent_b = self.comm_write(wts)
            if sent_b == w_len:
                #print "Success: Sent ", sent_b, " bytes to radio."
                pass
            else:
                print "Error: Only ", sent_b, " of ", w_len, " sent to radio."
            r = self.receive_from_radio()
            #print "received", r

        for index, value in enumerate(wts_l):
            if index % 2 == 0:
                wts = value
                if index == len(wts_l) - 1:
                    send_word_to_radio(wts)
            else:
                wts += ":{}".format(value)
                send_word_to_radio(wts)
                wts = ''

        send_word_to_radio("end") # signal end of transmission

    def receive_from_radio(self):
        received = self.comm_read()
        return received

    def __del__(self):
        self.comm_end()

def start_radio(args):
    args=interpret_args(args)
    main(args)


def main(args):
    wts_l = compose_word_to_send(args) # list of param ---> value pairs to set on Radio
    #r = radio("/dev/ttyACM0") # init connection, Arduino Uno
    r = radio("/dev/ttyUSB0") # init connection, Arduino Nano
    r.send_to_radio(wts_l) # the radio is expecting to get a word like:
                            # s:0:pwr:110
                            # f:8900:rn:CTNRadio
                            # rt:radio text here
                            # TX_RDS_PS_MISC:2345:pi:1234
                            # end

if __name__ == "__main__":
    args=interpret_args()
    main(args)
