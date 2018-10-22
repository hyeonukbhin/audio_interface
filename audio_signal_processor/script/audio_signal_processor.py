#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
from std_msgs.msg import String
from audio_msgs.msg import AudioData
# import pyaudio
from six.moves import queue
import os
import sounddevice as sd
import parselmouth
import numpy as np
import pandas as pd
# import parselmouth
# buff = []


def main():
    buff = queue.Queue()
    rospy.init_node('audio_signal_processor', anonymous=None)
    global pub
    pub = rospy.Publisher('sound_information', String, queue_size=100)
    rospy.Subscriber("audio_stream", AudioData, packetCallback, buff)
    rospy.spin()


def packetCallback(packetData, buff):
    global callerSpeech, idx, callerArray, frameIdx, endFlag, lastFrameIdx
    callerSpeech = packetData.data
    byte_str = makeByteStr(callerSpeech)
    # print(byte_str)
    buff.put(byte_str)
    print([buff.get()])


    # list1 = [10, 1, 246, 0, 235, 0, 6, 1, 32, 1, 36, 1, 39, 1, 47, 1, 20, 1, 5, 1]
    # list2 = [235, 255, 57, 0, 153, 0, 219, 0, 3, 1, 221, 0, 154, 0, 157, 0, 210, 0, 35, 1]
    # list3 = [105, 1, 112, 1, 126, 1, 123, 1, 64, 1, 250, 0, 199, 0, 178, 0, 162, 0, 114, 0]
    # list4 = [46, 0, 36, 0, 46, 0, 44, 0, 36, 0, 50, 0, 23, 0, 250, 255, 213, 255, 218]
    # snd = parselmouth.Sound("docs/examples/audio/the_north_wind_and_the_sun.wav")
    # snd = parselmouth.Sound()
    #
    #
    #
    # 1.
    # __init__(self: parselmouth.Sound, values: numpy.ndarray[float64], sampling_frequency: Positive[
    #     float] = 44100.0, start_time: float = 0.0) -> None

    # plt.figure()
    # plt.plot(snd.xs(), snd.values.T)
    # plt.xlim([snd.xmin, snd.xmax])
    # plt.xlabel("time [s]")
    # plt.ylabel("amplitude")
    # plt.show()


    # 1.
    # _(self: parselmouth.Sound, values: numpy.ndarray[float64], sampling_frequency: Positive[
    #     float] = 44100.0, start_time: float = 0.0) -> None


#    callerSpeech = packetData.data
#    byte_str = makeByteStr(callerSpeech)
#    buff.put(byte_str)

def makeByteStr(int16Array):

    byte_str = "".join(map(chr, int16Array))
    return byte_str


if __name__ == '__main__':
    main()

