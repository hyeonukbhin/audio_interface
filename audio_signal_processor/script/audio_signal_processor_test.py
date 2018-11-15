#!/usr/bin/python2
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
import parselmouth
import rospy
from audio_msgs.msg import AudioData, FeatureData
from six.moves import queue
from std_msgs.msg import String
import sounddevice as sd
import json
import Queue


test = 0

print(test % 5)

que = queue.Queue(100)
print(que)
que2 = Queue.Queue(100)
print(que2)


que2.put([1,2,3,4])
que2.put([2])
que2.put(3)
print(que2.get())
print(que2.qsize())
print(que2.get())
print(que2.qsize())
print(que2.get())
print(que2.qsize())
print(que2.get())

# print("ttt", que.get())


# print(que)

