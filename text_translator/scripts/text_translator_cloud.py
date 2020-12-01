#!/usr/bin/python3.6
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
# from pprintpp import pprint
import json
from googletrans import Translator
import time
from termcolor import colored
import os

# bhin_translation_service_key.json

# [set path]
service_key_path = "/home/kist/bhin_translation_service_key.json"
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = service_key_path


from google.cloud import translate_v2 as translate

client = translate.Client()
result = client.translate('안녕하세요', target_language='en')
print(result['translatedText'])

from google.cloud import translate_v3beta1 as translate


client = translate.TranslationServiceClient()
location = 'global'
project_id = 'bhin-stt'
parent = client.location_path(project_id, location)
client.glossary_path()
response = client.translate_text(
    parent=parent,
    contents=["안녕하세요"],
    mime_type='text/plain',  # mime types: text/plain, text/html
    source_language_code='ko',
    target_language_code='en')

for translation in response.translations:
    print('Translated Text: {}'.format(translation))