#!/usr/bin/python

# Copyright @ 2012 Intel Corp .
# All rights reserved
#
# File: test_config.py
# Author: Duan Yin
# Date: 2013-1-21
#
# Description:
#
# Usage:
#
#

#0 is default port
port_relay_0 = '/dev/ttyUSB0'
port_relay_1 = '/dev/ttyUSB1'

port_robot_0 = '/dev/ttyACM0'
port_robot_1 = '/dev/ttyACM1'

relay_switch_map = {'hdmi_hotplug': 3,'usb_hotplug': 4,}

input_keyevent = {'KEYEVENT_UP':'adb shell input keyevent 19',\
                  'KEYEVENT_DOWN':'adb shell input keyevent 20',\
                  'KEYEVENT_ENTER':'adb shell input keyevent 66',\
                  'KEYEVENT_BACK':'adb shell input keyevent 4',\
                  'KEYEVENT_HOME':'adb shell input keyevent 3',\
                  'KEYEVENT_DPAD_LEFT':'adb shell input keyevent 21',\
                  'KEYEVENT_DPAD_RIGHT':'adb shell input keyevent 22',\
                  'KEYCODE_POWER':'adb shell input keyevent 26',\
                  'KEYCODE_MENU':'adb shell input keyevent 82',\
                  'KEYEVENT_TAB':'adb shell input keyevent 61',\
                  'KEYCODE_MOVE_HOME': 'adb shell input keyevent 122',\
                  }

browser_app = {'android':'com.android.browser/.BrowserActivity',\
               'chrome':'com.android.chrome/com.google.android.apps.chrome.Main',\
               'firefox':'org.mozilla.firefox_beta/.App',\
              }

screen_off_timeout_times = {'15s':'15000','30s':'30000','1m':'60000','2m':'120000','5m':'300000','10m':'600000','30m':'1800000',}

screen_off_timeout_times_adjust_offset = {'15s':1,'30s':2,'1m':3,'2m':4,'5m':5,'10m':6,'30m':7,}

cpu_frequency = {'mfld_prx':'1.4',\
                 'ctp':'1.6|2.0',\
                 'rhb':'1.6|2.0',\
                 'mfld_gi':'1.2',\
                 'lex':'1.2',\
                 'salitpa':'1.6',\
                 'merr_vv':'1.2'}

cpu_frequency_kpi = {'mfld_prx':'1.6',\
                     'ctp':'2.0',\
                     'rhb':'2.0',\
                     'mfld_gi':'1.2',\
                     'lex':'1.2',\
                     'yukka':'1.2',\
                     'salitpa':'1.6',\
                     'mfld_pr4':'2.0',\
                     'merr_vv':'1.333'}

cpu_frequency_default = {'mfld_prx':'0.6',\
                         'ctp':'0.8',\
                         'rhb':'0.8',\
                         'mfld_gi':'1.2',\
                         'lex':'1.2',\
                         'yukka':'1.2',\
                         'salitpa':'1.6',\
                         'mfld_pr4':'0.6',\
                         'merr_vv':'0.666'}

rotation_degree = {'add_90':'650','subtract_90':'2350','portrait':'1500'}

evt_device = {'ctp_tap':'/dev/input/event7',\
              'rhb_tap':'/dev/input/event7',\
             }

orng_evt_dict = {'ctp_jb_video_capture_start':'tap 74.9 1188.0 1|tap 108.8 1022.2 1|tap 340.5 1185.0 1',\
                 'ctp_jb_ijetty_start':'tap 186.6 481.6 1',\
                 'ctp_jb_html_play_start':'tap 402.3 244.8 1',\
                 'ctp_jb_player_select':'tap 203.6 699.5 1|tap 519.2 928.2 1',\
                 'rhb_jb2_unlock_screen':'drag 354 1018 646 1018',\
                 'rhb_jb2_video_capture_start':'tap 74.9 1188.0 1|tap 108.8 1022.2 1|tap 340.5 1185.0 1',\
                 'rhb_jb2_ijetty_start':'tap 186.6 481.6 1',\
                 'rhb_jb2_html_play_start':'tap 402.3 244.8 1',\
                 'rhb_jb2_player_select':'tap 203.6 699.5 1|tap 519.2 928.2 1',\
                }
