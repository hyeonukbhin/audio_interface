#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from audio_msgs.msg import AudioData
import numpy as np
import sounddevice as sd
# import noisereduce as nr

SAMPLING_FREQUENCY = 44100
PLAY_OVERLAP_CHUNK = 10000
# PLAY_OVERLAP_CHUNK = SAMPLING_FREQUENCY * 1


class AudioStreamPlayer:
    def __init__(self):
        rospy.init_node('audio_stream_player')
        rospy.Subscriber("audio_stream", AudioData, self.cb_audio_stream, queue_size=50)
        self.play_buff = []

        self.sampling_frequency = int(get_setting_from_launch("sampling_frequency", SAMPLING_FREQUENCY))
        self.play_overlap_chunk = int(get_setting_from_launch("play_overlap_chunk", PLAY_OVERLAP_CHUNK))
        rospy.spin()

    def cb_audio_stream(self, topic):
        audio_stream = topic.data
        # reduced_audio_stream = nr.reduce_noise(audio_stream, sr=SAMPLING_FREQUENCY)
        self.play_buff.extend(audio_stream)
        self.play_buff = self.play_buff[-self.play_overlap_chunk:]
        int16_buff = np.array(self.play_buff, dtype='int16')
        play_audio(int16_buff, self.sampling_frequency)

# audio = pyaudio.PyAudio()
# stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, output=True, frames_per_buffer=CHUNK)

# try:
#     while True:
#         data = s.recv(CHUNK)
#         stream.write(data)


def play_audio(chunk_array, fs):
    if chunk_array != []:
        sd.play(chunk_array, samplerate=fs, blocking=False)


def get_setting_from_launch(arg_name, default_arg):

    try:
        output = rospy.get_param('~{}'.format(arg_name))
    except KeyError:
        output = default_arg

    return output


if __name__ == '__main__':
    AudioStreamPlayer()
