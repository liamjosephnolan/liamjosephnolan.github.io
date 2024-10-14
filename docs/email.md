---
layout: page
title: Email
description: >
  Basic email scripting using python
hide_description: true
---

I want to explore Python's ability to send email. I admit it has very limited ethical applications but I mostly wanted to play around and spam friends and classmates. Pretty juvenile but would also be very useful for some sort of notification service for a future project.  

## How it works

I created a Gmail account for dev purposes and enabled 3rd party app support. The source code is below. I found adding random text entry fields was fairly effective in passing spam filters. 

~~~js
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
import random
import time

# Email and password for Gmail account
email = 'EMAIL HERE'
password = 'TEMP PASSWORD HERE'

Subject_list = [ " ADD VARIOUS ENTRIES HERE"
]

text_list = [ " EMAIL BODYY TEXT LIST ENTRIES HERE"
]


while True:

    # Create message container - the correct MIME type is multipart/alternative.
    msg = MIMEMultipart('alternative')
    msg['Subject'] = Subject_list[random.randint(0,19)]
    msg['From'] = email
    msg['To'] = 'EMAIL HERE'  # Replace with recipient email address

    # Create the body of the message (a plain-text and an HTML version).
    text = text_list[random.randint(0,19)]

    # Record the MIME types of both parts - text/plain and text/html.
    part1 = MIMEText(text, 'plain')

    # Attach parts into message container.
    msg.attach(part1)
    

    # Send the message via Gmail's SMTP server.
    try:
        server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
        server.login(email, password)
        server.sendmail(email, msg['To'], msg.as_string())
        server.quit()
        print("Email sent successfully")
    except Exception as e:
        print("Error: Unable to send email - ", e)
    import smtplib
    from email.mime.multipart import MIMEMultipart
    from email.mime.text import MIMEText

    time.sleep(1)
~~~

Source Code
{:.figcaption}
