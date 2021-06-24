#!/usr/bin/python3.6
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
# from pprintpp import pprint
import json
from googletrans import Translator
import time
from termcolor import colored
import rospkg
import os
from google.cloud import translate_v2 as translate
from google.cloud import translate_v3beta1 as translate

pack_path = rospkg.RosPack().get_path("text_translator")
service_key_path = pack_path + "/service_key/translation_service_key.json"
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = service_key_path

while True:
    try:
        client = translate.TranslationServiceClient()
        location = 'global'
        project_id = 'bhin-stt'
        parent = client.location_path(project_id, location)
        break

    except AttributeError as e:
        print('[{}] {}'.format(time.time(), colored("Cannot Connect Translator Service URL", 'red', attrs=['bold'])))
        print('[{}] {}'.format(time.time(), colored("Reconnecting Translator Service URL...", 'white', attrs=['bold'])))

        time.sleep(1)
print('[{}] {}'.format(time.time(), colored("Connected URL!!", 'blue', attrs=['bold'])))


def send_translation(name, speech_kr, speech_en):
    current_time = rospy.get_rostime()
    msgs_dict = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "perception",
            "target": ["perception"],
            "content": ["translation_result"]
        },
        "translation_result": {
            "name": name,
            "speech_kr": speech_kr,
            "speech_en": speech_en
        }
    }
    json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
    # json_string = json.dumps(msgs_dict, ensure_ascii=True, indent=4)
    pub_translation.publish(json_string)
    # pub_intent_topic.publish("안녕하세요")


def get_header(json_dict):
    source = json_dict["header"]["source"]
    target_list = json_dict['header']["target"]
    content_list = json_dict['header']["content"]

    return source, target_list, content_list


def callback_speech(data):
    # json_dict = json.loads(data.data.decode('utf-8'))
    try :
        json_dict = json.loads(data.data)
        source, target_list, content_list = get_header(json_dict)
        # 사람 위치 추적 및 이름 파라미터 업데이
        if ("dialog" in target_list) and (source == "perception") and ("human_speech" in content_list):
            name = json_dict["human_speech"]["name"]
            speech_kr = json_dict["human_speech"]["speech"]
            info_dict = {}
            intent = ""
            response = client.translate_text(
                parent=parent,
                contents=[speech_kr],
                mime_type='text/plain',  # mime types: text/plain, text/html
                source_language_code='ko',
                target_language_code='en')

            translation = response.translations[0]
            str_translation = str(translation.translated_text)
            conv_str_translation = str_translation.replace("n\'t", " not")
            speech_en = conv_str_translation
            print(speech_en)
        
            send_translation(name, speech_kr, speech_en)
    except ValueError : 
        pass


def text_translator():
    global pub_translation
    rospy.init_node('KIST_text_translator', anonymous=False)
    rospy.Subscriber("recognitionResult", String, callback_speech)
    pub_translation = rospy.Publisher("translationResult", String, queue_size=100)
    rospy.spin()


if __name__ == '__main__':
    text_translator()
