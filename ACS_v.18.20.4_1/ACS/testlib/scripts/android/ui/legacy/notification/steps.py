#!/usr/bin/env python

##############################################################################
#
# @filename:    steps.py
# @description: UI test steps
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step


class open_notification(ui_step):

     def __init__(self, view_to_check, should_exist = True, **kwargs):
          self.view_to_check = view_to_check
          self.should_exist = should_exist
          ui_step.__init__(self, **kwargs)

     def do(self):
          self.uidevice.open.notification()

     def checkCondition(self):
          if self.uidevice(**self.view_to_check).exists:
              return self.should_exist
          else:
              return not self.should_exist


class open_notif_area(ui_step):
     def do(self):
          ui_steps.open_app_from_allapps(print_error =
                                         "Error - Application could not be "
                                         "openned",
                                         blocking = True,
                                         view_to_find =\
                                            {"text":"NotificationTrigger"},
                                         view_to_check =\
                                            {"text": "Create Notification"})()
          ui_steps.click_button(print_error = "Error - Application coulnd not "
                                              "be openned",
                                blocking = True,
                                view_to_find = {"text":"Create Notification"})()

     def check_condition(self):
          self.uidevice.open.notification()
          return self.uidevice(textContains = "Your test notification title").wait.exists(timeout = 5000)

