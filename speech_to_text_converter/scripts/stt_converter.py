#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
from std_msgs.msg import String
from audio_msgs.msg import AudioData
import threading
import json
import contextlib
import re
import signal
import google.auth
import google.auth.transport.grpc
import google.auth.transport.requests
from google.cloud.proto.speech.v1beta1 import cloud_speech_pb2
from google.rpc import code_pb2
import grpc
import pyaudio
from six.moves import queue
import os
import sounddevice as sd
import rospkg

pack_path = rospkg.RosPack().get_path("speech_to_text_converter")
service_key_path = pack_path + "/scripts/service_key.json"

# sd.default.device
# sd.default.samplerate = 16000
os.environ[
    "GOOGLE_APPLICATION_CREDENTIALS"] = service_key_path

CHANNELS = 1
RATE = 44100
LOOP_RATE = 20
CHUNK = int(RATE / LOOP_RATE)  # 100ms
# print CHUNK



# The Speech API has a streaming limit of 60 seconds of audio*, so keep the
# connection alive for that long, plus some more to give the API time to figure
# out the transcription.
# * https://g.co/cloud/speech/limits#content
DEADLINE_SECS = 60 * 3 + 10
# DEADLINE_SECS = 60
SPEECH_SCOPE = 'https://www.googleapis.com/auth/cloud-platform'

import sys

reload(sys)
sys.setdefaultencoding('utf-8')



class rosSpinThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        print "Starting " + self.name
        rospy.spin()
        print "Exiting " + self.name

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def make_channel(host, port):
    """Creates a secure channel with auth credentials from the environment."""
    # Grab application default credentials from the environment
    credentials, _ = google.auth.default(scopes=[SPEECH_SCOPE])

    # Create a secure channel using the credentials.
    http_request = google.auth.transport.requests.Request()
    target = '{}:{}'.format(host, port)

    return google.auth.transport.grpc.secure_authorized_channel(
        credentials, http_request, target)


def _audio_data_generator(buff):
    """A generator that yields all available data in the given buffer.

    Args:
        buff - a Queue object, where each element is a chunk of data.
    Yields:
        A chunk of data that is the aggregate of all chunks of data in `buff`.
        The function will block until at least one data chunk is available.
    """
    stop = False
    while not stop:
        data = [buff.get()]
        # print len(data)
        # Now consume whatever other data's still buffered.
        while True:
            try:
                data.append(buff.get(block=False))
            except queue.Empty:
                break

        # `None` in the buffer signals that the audio stream is closed. Yield
        # the final bit of the buffer and exit the loop.
        if None in data:
            stop = True
            data.remove(None)

        yield b''.join(data)


def _fill_buffer(buff, in_data, frame_count, time_info, status_flags):
    """Continuously collect data from the audio stream, into the buffer."""
    buff.put(in_data)
    return None, pyaudio.paContinue

def makeByteStr(int16Array):

    byte_str = "".join(map(chr, int16Array))
    # byte_str = struct.pack('>%si' % len(int16Array), *int16Array)
    # print "test"
    # print len(int16Array)
    # print "test1111"
    # print len(byte_str)
    return byte_str

def packetCallback(packetData, buff):
    global callerSpeech, idx, callerArray, frameIdx, endFlag, lastFrameIdx
    callerSpeech = packetData.data
    # print len(callerSpeech)
    robot_speech = get_param("perception/robot_speech")
    if robot_speech != "ON":
        # print(robot_speech)
        byte_str = makeByteStr(callerSpeech)
        buff.put(byte_str)

def get_param(input_key="condition_rule/social_distance/no_definition"):
    # input_key = "condition_rule/social_distance/no_definition"
    output_values = rospy.get_param(input_key)
    return output_values

# [START audio_stream]
@contextlib.contextmanager
def record_audio(rate, chunk, buff):
    """Opens a recording stream in a context manager."""
    yield _audio_data_generator(buff)



# [END audio_stream]

def request_stream(data_stream, rate, interim_results=True):
    """Yields `StreamingRecognizeRequest`s constructed from a recording audio
    stream.

    Args:
        data_stream: A generator that yields raw audio data to send.
        rate: The sampling rate in hertz.
        interim_results: Whether to return intermediate results, before the
            transcription is finalized.
    """
    # The initial request must contain metadata about the stream, so the
    # server knows how to interpret it.
    recognition_config = cloud_speech_pb2.RecognitionConfig(
        # There are a bunch of config options you can specify. See
        # https://goo.gl/KPZn97 for the full list.
        encoding='LINEAR16',  # raw 16-bit signed LE samples
        sample_rate=rate,  # the rate in hertz
        # See http://g.co/cloud/speech/docs/languages
        # for a list of supported languages.
        language_code='ko-KR',  # a BCP-47 language tag
    )
    streaming_config = cloud_speech_pb2.StreamingRecognitionConfig(
        interim_results=interim_results,
        config=recognition_config,
    )

    yield cloud_speech_pb2.StreamingRecognizeRequest(
        streaming_config=streaming_config)

    for data in data_stream:
        # Subsequent requests can all just have the content
        yield cloud_speech_pb2.StreamingRecognizeRequest(audio_content=data)




def listen_print_loop(recognize_stream):
    """Iterates through server responses and prints them.

    The recognize_stream passed is a generator that will block until a response
    is provided by the server. When the transcription response comes, print it.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """
    num_chars_printed = 0
    for resp in recognize_stream:
        if resp.error.code != code_pb2.OK:
            # raise RuntimeError('Server error: ' + resp.error.message)
            continue
        if not resp.results:
            continue

        # Display the top transcription
        result = resp.results[0]
        transcript = result.alternatives[0].transcript

        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        #
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        overwrite_chars = ' ' * max(0, num_chars_printed - len(transcript))

        if not result.is_final:
            sys.stdout.write(transcript + overwrite_chars + '\r')
            sys.stdout.flush()
            num_chars_printed = len(transcript)

        else:
            recognitionResult = transcript + overwrite_chars
            callbackFromGoogle(recognitionResult)

            # Exit recognition if any of the transcribed phrases could be
            # one of our keywords.
            if re.search(r'\b(exit|quit)\b', transcript, re.I):
                print('Exiting..')
                break

            num_chars_printed = 0


def callbackFromGoogle(recognitionWord):

    current_time = rospy.get_rostime()
    inputString = recognitionWord
    name = ""
    jsonSTTFrame = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "KIST",
            "target": ["UOS", "HY"],
            "content": ["human_speech"]
        },
        "human_speech": {
            "name": name,
            "speech": "%s" % inputString
        }
    }

    jsonString = json.dumps(jsonSTTFrame, ensure_ascii=False, indent=4)
    pub.publish(jsonString)
    print "===================================================================="
    print " Recognized Dialog : \x1b[1;33m%s\x1b[1;m" % inputString
    print "===================================================================="
    # print "===================================================================="
    # print " Topic Published : \x1b[1;33m[/recognitionResult]\x1b[1;m"
    # print " \x1b[1;34mKIST_stt_converter -> KIST_data_collector\x1b[1;m"
    # print "===================================================================="
    # print("\x1b[1;36mcyan\x1b[1;m")
    # print("\x1b[1;33mblue\x1b[1;m")


def sttConverter():
    buff = queue.Queue()
    rospy.init_node('KIST_stt_converter', anonymous=None)
    global pub
    pub = rospy.Publisher('recognitionResult', String, queue_size=100)
    rospy.Subscriber("audio_stream", AudioData, packetCallback, buff)
    # rospy.Subscriber("inputCMD", String, callbackFromCMD, pub)

    service = cloud_speech_pb2.SpeechStub(
        make_channel('speech.googleapis.com', 443))

    # For streaming audio from the microphone, there are three threads.
    # First, a thread that collects audio data as it comes in
    with record_audio(RATE, CHUNK, buff) as buffered_audio_data:
        # Second, a thread that sends requests wi th that data
        requests = request_stream(buffered_audio_data, RATE)
        # print buffered_audio_data
        # Third, a thread thatlistens for transcription responses

        # print requests
        recognize_stream = service.StreamingRecognize(
            requests, DEADLINE_SECS)

        # print recognize_stream
        # Exit things cleanly on interrupt
        signal.signal(signal.SIGINT, lambda *_: recognize_stream.cancel())

        # Now, put the transcription responses to use.
        try:
            listen_print_loop(recognize_stream)
            recognize_stream.cancel()
        except grpc.RpcError as e:
            code = e.code()
            # CANCELLED is caused by the interrupt handler, which is expected.
            if code is not code.CANCELLED:
                raise

    #signal.pause()
    #sys.exit()
    rospy.spin()



if __name__ == '__main__':
    # thread1 = rosSpinThread(1, "rosThread")
    # thread1.start()

    while True:
        try:
            sttConverter()
        except Exception as e:
            print(e)