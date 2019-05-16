#!/usr/bin/env python

#######################################################################
#
# @filename:    file_steps.py
# @description: Mail steps
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.base.base_step import step as base_step
import smtplib

from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText


class send_mail(base_step):

    """ description:
            Sends mail

        usage:
            file_steps.compare_image(first_file = "file1.png",
                                     second_file = "file2.png",
                                     tolerance = 10)()

        tags:
            file, compare, rms, images
    """

    def __init__(self, sender, recipient, subject, body_text = None, body_html = None, smtp_server = "localhost", **kwargs):
        self.sender = sender
        self.recipient = recipient
        self.smtp_server = smtp_server
        self.subject = subject
        self.text = body_text
        self.html = body_html
        base_step.__init__(self, **kwargs)
        self.set_passm("E-mail with '{0}' subject sent to {1}".format(self.subject, recipient))
        self.set_errorm("", "Error sending e-mail with '{0}' subject sent to {1}".format(self.subject, recipient))

    def do(self):
        msg = MIMEMultipart('alternative')
        msg['Subject'] = self.subject
        msg['From'] = self.sender
        msg['To'] = self.recipient
        if self.text:
            msg.attach(MIMEText(self.text, 'plain'))
        if self.html:
            msg.attach(MIMEText(self.html, 'html'))
        s = smtplib.SMTP(self.smtp_server)
        s.sendmail(self.sender, self.recipient, msg.as_string())
        s.quit()

    def check_condition(self):
        return True

