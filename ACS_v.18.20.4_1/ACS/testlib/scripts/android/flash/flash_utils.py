#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: Flash utils
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################


def build_url(base_url, project_id, build_no, build_link):
    url = base_url
    url += project_id + build_no + "/"
    url += build_link
    return url
