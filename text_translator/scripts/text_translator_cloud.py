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

# [set path]


pack_path = rospkg.RosPack().get_path("text_translator")
service_key_path = pack_path + "/service_key/translation_service_key.json"
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = service_key_path


# from google.cloud import translate_v2 as translate
#
# client = translate.Client()
# result = client.translate('그래서 며칠 동안 연락을받지 못했습니다.', source_language='ko', target_language='en')
# pprint(result)
# print(type(result))
# print(result.get('translatedText'))
from google.cloud import translate_v3beta1 as translate
client = translate.TranslationServiceClient()
location = 'global'
project_id = 'bhin-stt'
parent = client.location_path(project_id, location)


response = client.translate_text(
    parent=parent,
    contents=["그래서 며칠 동안 연락을받지 못했습니다. 반갑습니다"],
    mime_type='text/plain',  # mime types: text/plain, text/html
    source_language_code='ko',
    target_language_code='en')

translation = response.translations[0]
str_translation = str(translation.translated_text)
conv_str_translation = str_translation.replace("n\'t", " not")
print(str_translation)
print(conv_str_translation)

# for translation in response.translations:
#     translation.translated_text


