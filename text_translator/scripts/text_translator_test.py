#!/usr/bin/python3.6
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
# from pprintpp import pprint
import json
from googletrans import Translator
import time
from termcolor import colored

while True:
    try:
        # translator = Translator(service_urls=[
        #     'translate.google.com',
        #     'translate.google.co.kr',
        # ])
        translator = Translator()
        test_msg_kr = "시작"
        test_msg_en = translator.translate(test_msg_kr, dest='en', src='ko')
        break

    except AttributeError as e:
        # rospy.loginfo("Cannot connect service url")
        # rospy.loginfo("Reconnecting URL...")
        print('[{}] {}'.format(time.time(), colored("Cannot Connect Translator Service URL", 'red', attrs=['bold'])))
        print('[{}] {}'.format(time.time(), colored("Reconnecting Translator Service URL...", 'white', attrs=['bold'])))

        time.sleep(1)
print('[{}] {}'.format(time.time(), colored("Connected URL!!", 'blue', attrs=['bold'])))
#
#
# translator = Translator(service_urls=[
#     'translate.google.com',
#     'translate.google.co.kr',
# ])
# test_msg_kr = "시작"
# test_msg_en = translator.translate(test_msg_kr, dest='en', src='ko')
#
# while True:
#     try:
#         translator = Translator(service_urls=[
#             'translate.google.com',
#             'translate.google.co.kr',
#         ])
#         test_msg_kr = "시작"
#         test_msg_en = translator.translate(test_msg_kr, dest='en', src='ko')
#         break
#
#     except AttributeError as e:
#         print("Cannot connect service url")
#         print("Reconnecting URL...")
#         time.sleep(1)
#
#
# print("Connected URL!!")
#
# while True:
#
#     speech_kr = "반갑습니다."
#     speech_en = translator.translate(speech_kr, dest='en', src='ko')
#     print(speech_en.text)
#     time.sleep(1)


# translations = translator.translate(['The quick brown fox', 'jumps over', 'the lazy dog'], dest='ko')

# print(translations)
# def translate(self, text, dest='en', src='auto', **kwargs):


# def send_translation(name, speech_kr, speech_en):
#     current_time = rospy.get_rostime()
#     msgs_dict = {
#         "header": {
#             "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
#             "source": "perception",
#             "target": ["perception"],
#             "content": ["translation_result"]
#         },
#         "translation_result": {
#             "name": name,
#             "speech_kr": speech_kr,
#             "speech_en": speech_en
#         }
#     }
#     json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
#     # json_string = json.dumps(msgs_dict, ensure_ascii=True, indent=4)
#     pub_translation.publish(json_string)
#     # pub_intent_topic.publish("안녕하세요")
#
#
# def get_header(json_dict):
#     source = json_dict["header"]["source"]
#     target_list = json_dict['header']["target"]
#     content_list = json_dict['header']["content"]
#
#     return source, target_list, content_list
#
# def callback_speech(data):
#     # json_dict = json.loads(data.data.decode('utf-8'))
#     json_dict = json.loads(data.data)
#     source, target_list, content_list = get_header(json_dict)
#     # 사람 위치 추적 및 이름 파라미터 업데이
#     if ("dialog" in target_list) and (source == "perception") and ("human_speech" in content_list):
#
#         name = json_dict["human_speech"]["name"]
#         speech_kr = json_dict["human_speech"]["speech"]
#         info_dict = {}
#         intent = ""
#
#         # -*- coding: utf-8 -*-
#
#         translator = Translator()
#
#         speech_en = translator.translate(speech_kr).text
#
#         # print(speech_kr)
#         # print(speech_en)
#         # print("")
#         send_translation(name, speech_kr, speech_en)
#         # print(translator.translate('안녕하세요'))
#
#
# def text_translator():
#     global pub_translation
#     rospy.init_node('KIST_text_translator', anonymous=False)
#     rospy.Subscriber("recognitionResult", String, callback_speech)
#     pub_translation = rospy.Publisher("translationResult", String, queue_size=100)
#     rospy.spin()
#
# if __name__ == '__main__':
#     text_translator()

