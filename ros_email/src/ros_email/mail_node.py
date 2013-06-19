#!/usr/bin/python

import smtp_email

import rospy

from ros_email.msg import *
from ros_email.srv import *

class EMAIL_NODE():

    def __init__(self, username, password, server, port):
        self.mail_host = smtp_email.SMTP_PLUS_EMAIL(username, password, server, port)
        rospy.loginfo("Logged into the server successfully")
        self.handler1 = rospy.Service('/ros_email/SendMail', SendMail, self.handler1)
        self.handler2 = rospy.Service('/ros_email/SendMailPlus', SendMailPlus, self.handler2)
        rospy.loginfo("ROS Email node running...")
        while not rospy.is_shutdown():
            rospy.spin()

    def handler1(self, request):
        try:
            self.mail_host.send(request.recipient, request.message_title, request.message_text)
            return 0
        except:
            return 1

    def handler2(self, request):
        try:
            attachments = []
            for index in len(request.attachments):
                attachments.append([request.attachments[index].name, request.attachments[index].data])
            self.mail_host.send(request.recipient, request.message_title, request.message_text, attachments)
            return 0
        except:
            return 1

if __name__ == '__main__':
    rospy.init_node('ros_email')
    username = rospy.get_param("~username", None)
    password = rospy.get_param("~password", None)
    mailserver = rospy.get_param("~mailserver", None)
    serverport = rospy.get_param("~serverport", None)
    if (username is None or password is None or mailserver is None or serverport is None):
        rospy.logerr("*** No/insufficient information provided to the mail node, unable to log in! ***")
    else:
        EMAIL_NODE(username, password, mailserver, serverport)
