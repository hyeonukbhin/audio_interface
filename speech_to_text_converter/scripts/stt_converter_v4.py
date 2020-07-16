#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
from __future__ import division

import re
import sys

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from six.moves import queue
import rospkg
import os
import rospy
from std_msgs.msg import String
from audio_msgs.msg import AudioData
import json
import time
from termcolor import colored
from signal import signal, SIGINT
from sys import exit
from hanspell import spell_checker

reload(sys)
sys.setdefaultencoding('utf-8')

# [set path]
pack_path = rospkg.RosPack().get_path("speech_to_text_converter")
service_key_path = pack_path + "/scripts/service_key.json"
os.environ[
    "GOOGLE_APPLICATION_CREDENTIALS"] = service_key_path


WITH_SPELLCHECKER = True

CHANNELS = 1
RATE = 44100
LOOP_RATE = 5
CHUNK = int(RATE / LOOP_RATE)  # 100ms


def sttConverter():
    language_code = 'ko-KR'  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        # single_utterance=True,
        sample_rate_hertz=RATE,
        language_code=language_code)
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    with RosTopicStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        try:
            responses = client.streaming_recognize(streaming_config, requests)
        except Exception as exception:
            print("exception exception exception exception exception ")

        try:
            listen_print_loop(responses)
        except Exception as exception:
            return
    rospy.spin()


class RosTopicStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self, rate, chunk):
        global pub
        self._rate = rate
        self._chunk = chunk
        self._buff = queue.Queue()
        rospy.init_node('KIST_stt_converter', anonymous=False)
        pub = rospy.Publisher('recognitionResult', String, queue_size=100)
        rospy.Subscriber("audio_stream", AudioData, self.packet_callback)
        self.closed = True

    def __enter__(self):
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        self.closed = True
        self._buff.put(None)

    def generator(self):
        while not self.closed:
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)

    def packet_callback(self, packet):
        caller_speech = packet.data
        byte_str = self.make_bytestr(caller_speech)
        robot_speech = get_param("perception/is_speaking_robot/data")
        if robot_speech is not True:
            self._buff.put(byte_str)

    def make_bytestr(self, int16Array):
        byte_str = "".join(map(chr, int16Array))
        return byte_str


#########################################################################

def listen_print_loop(responses):
    num_chars_printed = 0
    test_idx = 0
    for response in responses:

        if not response.results:
            continue

        result = response.results[0]
        if not result.alternatives:
            continue

        transcript = result.alternatives[0].transcript
        overwrite_chars = ' ' * (num_chars_printed - len(transcript))
        if not result.is_final:
            rospy.set_param("perception/is_speaking_human/data", True)
            rospy.set_param("perception/is_speaking_human/timestamp", time.time())

            test_idx += 1
            num_chars_printed = len(transcript)

        else:
            result = str(transcript + overwrite_chars)

            if WITH_SPELLCHECKER is True:
                # word = "안녕 하세요. 안녕 하세요. 안녕 하세요. "
                checked_result = spell_checker.check(strip_one(result)).as_dict()
                dialog = checked_result["checked"]
                delayed_time = checked_result["time"]
                send_topic(result)
                # rospy.loginfo('User Speech : {}'.format(colored(result, 'white', attrs=['bold'])))
                rospy.loginfo('User Speech : {} (Spell Checked, Delayed time : {})'.format(colored(dialog, 'white', attrs=['bold']),colored(delayed_time, 'white', attrs=['bold'])))
            else:
                send_topic(result)
                rospy.loginfo('User Speech : {}'.format(colored(result, 'white', attrs=['bold'])))


            if re.search(r'\b(exit|quit)\b', transcript, re.I):
                print('Exiting..')
                break

            num_chars_printed = 0


def strip_one(s):
    if s.endswith(" ") : s = s[:-1] #마지막이 " "임을 검사
    if s.startswith(" ") : s = s[1:] #첫번째가 " "임을 검사
    return s
# a = "  hello  "
# print strip_one(a)


def get_param(param_name="state1"):
    try:
        output_values = rospy.get_param(param_name)
    except KeyError as e:
        output_values = "KeyError"
        print(colored("KeyError : {}".format(e), 'red'))
    return output_values


def send_topic(sentence):
    current_time = rospy.get_rostime()
    input_string = str(sentence)
    name = get_param("/perception/human_name/data")
    jsonSTTFrame = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "perception",
            "target": ["planning", "dialog"],
            "content": ["human_speech"]
        },
        "human_speech": {
            "name": name,
            "speech": "%s" % input_string
        }
    }

    # se.update_perception('human_name', human_name_data)
    rospy.set_param("perception/is_speaking_human/data", False)
    rospy.set_param("perception/is_speaking_human/timestamp", time.time())
    json_string = json.dumps(jsonSTTFrame, ensure_ascii=False, indent=4)
    pub.publish(json_string)


def handler(signal_received, frame):
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)


if __name__ == '__main__':
    signal(SIGINT, handler)
    while True:
        try:
            sttConverter()
        except Exception as e:
            print(e)
