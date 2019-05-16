#!/usr/bin/env python

########################################################################
#
# @filename:    hal_utils.py
# @description: HAL autodetect helper functions
# @author:      alexandrux.n.branciog@intel.com
#
########################################################################

def getHALBindings(file_name):
    """returns binding dictionary given a file with halctl listing"""
    bindings = []
    with open(file_name, "r") as f:
        lines = f.readlines()
    for line in lines:
        if "Binding #" in line:
            bindings.append({})
        else:
            attr, value = line.strip().split(":", 1)
            bindings[-1][attr] = value
    bindings.sort()
    return bindings

def cmp_bindings(l1, l2, no = None, verbose = False):
    """checks if two bindings are identical"""
    if no == None:
        ret = l1 == l2
    else:
        ret = l1[:no] == l2[:no]
    if not ret and verbose:
        print l1[:no]
        print l2[:no]
    return ret

def check_binding(current, ref, verbose = False):
    """check binding after add/supress"""
    if ref['refcount'] == '0':
        ret = len(current) == 0
    else:
        ret = any([m == ref for m in current])
    if not ret and verbose:
        print ref
        print current
    return ret

def check_mount_entries(current, ref, verbose = False):
    """check mount entries"""
    if verbose:
        print "Current entries:"
        print current
        print "Ref entries:"
        print ref
    if len(current) != len(ref): return False
    matches = 0
    for r_entry in ref:
        for c_entry in current:
            if r_entry in c_entry:
                matches += 1
                break
    return matches == len(ref)

def get_string_for_filter(module):
    """get string for halctl filter"""
    if module.has_key('hal_id'):
        return eval(module['hal_id'])
    if module.has_key('modalias'):
        return "modalias={}*".format(eval(module['modalias']).split(":")[0])

def get_runtime(string):
    """get hald boot impact time"""
    time = 0
    for s in string.split("runtime=")[1:]:
        time += float(s.split("s")[0])
    return time

def check_boot_errors(current_lines, ignore_lines, verbose = False):
    """checks boot erros"""
    for lc in current_lines:
        for li in ignore_lines:
            if li in lc or lc == "":
                break
        else:
            if verbose:
                print "Error at boot - " + lc
            return False
    return True
