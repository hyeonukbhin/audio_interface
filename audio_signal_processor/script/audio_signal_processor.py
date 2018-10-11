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


def main():
    buff = queue.Queue()
    rospy.init_node('audio_signal_processor', anonymous=None)
    global pub
    pub = rospy.Publisher('sound_information', String, queue_size=100)
    rospy.Subscriber("audio_stream", AudioData, packetCallback, buff)
    rospy.spin()


def packetCallback(packetData, buff):
    global callerSpeech, idx, callerArray, frameIdx, endFlag, lastFrameIdx

    snd = parselmouth.Sound("docs/examples/audio/the_north_wind_and_the_sun.wav")
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

if __name__ == '__main__':
    main()

