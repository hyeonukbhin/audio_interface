#!/usr/bin/python3.6
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from pprintpp import pprint
import json
from googletrans import Translator
import time
from termcolor import colored
import os
import rospkg

# translation_service_key.json

while True:
    try:
        translator = Translator(service_urls=['translate.googleapis.com'])

        test_msg_kr = "시작"
        test_msg_en = translator.translate(test_msg_kr, dest='en', src='ko')
        break

    except AttributeError as e:
        print("Cannot connect service url")
        print("Reconnecting URL...")
        time.sleep(1)


print("Connected URL!!")

msg_kr = "시작"
msg_en = translator.translate(msg_kr, dest='en', src='ko').text
print(msg_en)

msg_kr = "그래서 며칠 동안 연락을받지 못했습니다. 반갑습니다"
msg_en = translator.translate(msg_kr, dest='en', src='ko').text
print(msg_en)

msg_kr = "안녕하세요"
msg_en = translator.translate(msg_kr, dest='en', src='ko').text
print(msg_en)