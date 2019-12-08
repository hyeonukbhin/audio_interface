#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

# Copyright 2017 Google Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import division

import re
import sys

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
from six.moves import queue
from threading import Thread
import time
import rospkg
import os
# Audio recording parameters
# RATE = 16000
# CHUNK = int(RATE / 10)  # 100ms

CHANNELS = 1
RATE = 44100
LOOP_RATE = 10
# CHUNK = 8192  # 100ms
CHUNK = int(RATE / LOOP_RATE)  # 100ms



pack_path = rospkg.RosPack().get_path("speech_to_text_converter")
service_key_path = pack_path + "/scripts/service_key.json"

os.environ[
    "GOOGLE_APPLICATION_CREDENTIALS"] = service_key_path

class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):



        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True
        self.isPause = False




    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()

        device_index = 0
        info = self._audio_interface.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        device_name = "Not Selected"
        for i in range(0, numdevices):
            if (self._audio_interface.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                name = self._audio_interface.get_device_info_by_host_api_device_index(0, i).get('name')
                print "Input Device id ", i, " - ", name
                if "USB Audio Device" in name:
                    device_index = i
                    device_name = name

        # if rospy.has_param('~DEVICE_INDEX'):
        #     INPUT_DEVICE = rospy.get_param('~DEVICE_INDEX')
        #     CHANNELS = rospy.get_param('~CHANNELS')
        #     RATE = rospy.get_param('~RATE')
        #     LOOP_RATE = rospy.get_param('~LOOP_RATE')
        #     CHUNK = rospy.get_param('~CHUNK')
        #     # CHUNK = int(RATE / LOOP_RATE)  # 100ms
        #
        # else:
        INPUT_DEVICE = device_index
        print("\x1b[1;33m[Device Information] : {}\x1b[1;m".format(device_name))
        CHANNELS = 1
        RATE = 44100
        LOOP_RATE = 10


        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            channels=1, rate=self._rate,
            input_device_index=INPUT_DEVICE,
            input=True, frames_per_buffer=self._chunk,
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()


        self.closed = True
        self._buff.put(None)
        self._audio_interface.terminate()

    def pause(self):
        if self.isPause == False:
            self.isPause = True
            print("isPause")


    def resume(self):
        if self.isPause == True:
            self.isPause = False


    def status(self):
        return self.isPause

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        if self.isPause == False:
            self._buff.put(in_data)
        #else
        return None, pyaudio.paContinue

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


# [END audio_stream]



class Gspeech(Thread):
    def __init__(self):
        Thread.__init__(self)

        self.language_code = 'ko-KR'  # a BCP-47 language tag

        self._buff = queue.Queue()

        self.client = speech.SpeechClient()
        self.config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=self.language_code)
        self.streaming_config = types.StreamingRecognitionConfig(
            config=self.config,
            interim_results=True)

        self.mic = None
        self.status = True

        self.daemon = True
        self.start()

    def __eixt__(self):
        self._buff.put(None)

    def run(self):
        with MicrophoneStream(RATE, CHUNK) as stream:
            self.mic = stream
            audio_generator = stream.generator()
            requests = (types.StreamingRecognizeRequest(audio_content=content)
                        for content in audio_generator)

            responses = self.client.streaming_recognize(self.streaming_config, requests)

            # Now, put the transcription responses to use.
            self.listen_print_loop(responses, stream)
        self._buff.put(None)
        self.status = False

    def pauseMic(self):
        if self.mic is not None:
            self.mic.pause()
            print("pauseMIC")

    def resumeMic(self):
        if self.mic is not None:
            self.mic.resume()
            print("resumeMic")


    # 인식된 Text 가져가기
    def getText(self, block = True):
        print("getText")

        return self._buff.get(block=block)

    # 음성인식 처리 루틴
    def listen_print_loop(self, responses, mic):
        num_chars_printed = 0
        try:
            for response in responses:
                if not response.results:
                    continue

                result = response.results[0]
                if not result.alternatives:
                    continue
                print("...")
                transcript = result.alternatives[0].transcript
                overwrite_chars = ' ' * (num_chars_printed - len(transcript))
                if not result.is_final:
                    sys.stdout.write(transcript + overwrite_chars + '\r')
                    sys.stdout.flush()
                    #### 추가 ### 화면에 인식 되는 동안 표시되는 부분.
                    num_chars_printed = len(transcript)
                else:
                    # 큐에 넣는다.
                    self._buff.put(transcript+overwrite_chars)
                    num_chars_printed = 0
        except:
            return



def main():
    gsp = Gspeech()
    while True:
        # 음성 인식 될때까지 대기 한다.
        stt = gsp.getText()
        if stt is None:
            break
        print(stt)
        time.sleep(0.01)
        # if ('끝내자' in stt):
        #     break


if __name__ == '__main__':
    main()