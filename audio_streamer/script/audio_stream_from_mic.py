#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyaudio
import rospy
from audio_msgs.msg import AudioData


def main():
    rospy.init_node('audio_streamer')
    scan_pub = rospy.Publisher('audio_stream', AudioData, queue_size=50)

    ##### Initial Setting #####
    FORMAT = pyaudio.paInt16

    if rospy.has_param('~DEVICE_INDEX'):
        INPUT_DEVICE = rospy.get_param('~DEVICE_INDEX')
        CHANNELS = rospy.get_param('~CHANNELS')
        RATE = rospy.get_param('~RATE')
        LOOP_RATE = rospy.get_param('~LOOP_RATE')
        CHUNK = int(RATE / LOOP_RATE)  # 100ms

    else:
        INPUT_DEVICE = 5
        CHANNELS = 1
        RATE = 44100
        LOOP_RATE = 10
        CHUNK = int(RATE / LOOP_RATE)  # 100ms

    audio_interface = pyaudio.PyAudio()
    r = rospy.Rate(LOOP_RATE)

    stream = audio_interface.open(format=FORMAT,
                                  input_device_index=INPUT_DEVICE,
                                  channels=CHANNELS,
                                  rate=RATE,
                                  input=True,
                                  output=True,
                                  frames_per_buffer=CHUNK)

    # INPUT_DEVICE_INFO = audio_interface.get_default_input_device_info()
    INPUT_DEVICE_INFO = audio_interface.get_device_info_by_index(INPUT_DEVICE)
    print INPUT_DEVICE_INFO

    count = 0
    while not rospy.is_shutdown():

        current_time2 = rospy.get_rostime()
        if count >= 10:
            rospy.loginfo("Current time %i %i", current_time2.secs, current_time2.nsecs)
            count = 0
        else:
            count += 1

        try:
            byte_buff = stream.read(CHUNK)
            print(byte_buff)
            int16_buff = map(ord, byte_buff)
            scan_pub.publish(int16_buff)
            # print type(int16_buff)
            # print int16_buff
            print int16_buff[:20]
            # print len(int16_buff)
        except IOError, e:
            if e[1] == pyaudio.paInputOverflowed:
                print e
                # x = '\x00' * 16 * 256 * 2  # value*format*chunk*nb_channels

        r.sleep()

if __name__ == '__main__':
    main()
