import smtplib
import os
from email.MIMEMultipart import MIMEMultipart
from email.MIMEBase import MIMEBase
from email.MIMEText import MIMEText
from email.Utils import COMMASPACE, formatdate
from email import Encoders

class SMTP_EMAIL():

    def __init__(self, username, password, server, port):
        self.username = username
        self.password = password
        self.server = server
        self.port = port
        server = smtplib.SMTP(self.server, self.port)
        try:
            server.starttls()
        except:
            raise BaseException
        try:
            server.login(self.username, self.password)
        except:
            raise BaseException
        server.quit()

    def send(self, recipient, subject, message_text):
        server = smtplib.SMTP(self.server, self.port)
        try:
            server.starttls()
        except:
            raise BaseException
        try:
            server.login(self.username, self.password)
        except:
            raise BaseException
        msg = MIMEText(message_text)
        msg['Subject'] = subject
        msg['From'] = self.username
        msg['To'] = recipient
        try:
            server.sendmail(self.username, recipient, msg.as_string())
        except:
            raise BaseException
        server.quit()

class SMTP_PLUS_EMAIL(SMTP_EMAIL):

    def send(self, recipient, subject, message_text, attachments=[]):
        server = smtplib.SMTP(self.server, self.port)
        try:
            server.starttls()
        except:
            raise BaseException
        try:
            server.login(self.username, self.password)
        except:
            raise BaseException
        #Fill message headers
        msg = MIMEMultipart()
        msg['From'] = self.username
        msg['To'] = recipient
        msg['Date'] = formatdate(localtime=True)
        msg['Subject'] = subject
        #Assemble message
        msg.attach(MIMEText(message_text))
        for attachment in attachments:
            part = MIMEBase('application', "octet-stream")
            part.set_payload(attachment[1])
            Encoders.encode_base64(part)
            part.add_header('Content-Disposition', 'attachment; filename="%s"' % attachment[0])
            msg.attach(part)
        try:
            server.sendmail(self.username, recipient, msg.as_string())
        except:
            raise BaseException
        server.quit()
