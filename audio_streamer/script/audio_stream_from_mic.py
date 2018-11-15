#!/usr/bin/python2
# -*- coding: utf-8 -*-

import pyaudio
import rospy
from audio_msgs.msg import AudioData

def main():
    rospy.init_node('audio_streamer')
    scan_pub = rospy.Publisher('audio_stream', AudioData, queue_size=100)

    '''
    Initial Setting for Py audio interface
    '''

    format = pyaudio.paInt16
    audio_interface = pyaudio.PyAudio()

    info = audio_interface.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    device_index = 0
    device_name = "Not Selected"
    for i in range(0, numdevices):
        if (audio_interface.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            name = audio_interface.get_device_info_by_host_api_device_index(0, i).get('name')
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
    # CHUNK = 8192  # 100ms
    CHUNK = RATE/LOOP_RATE  # 100ms

    r = rospy.Rate(LOOP_RATE)
    stream = make_stream(audio_interface, format, INPUT_DEVICE, CHANNELS, RATE, CHUNK)
    INPUT_DEVICE_INFO = audio_interface.get_device_info_by_index(INPUT_DEVICE)

    print INPUT_DEVICE_INFO

    msg_sequence = 0

    audio_stream = AudioData()

    while not rospy.is_shutdown():
        # current_time2 = rospy.get_rostime()
        # if msg_sequence % LOOP_RATE == 0:
        #     rospy.loginfo("Current time %i %i", current_time2.secs, current_time2.nsecs)
        #     msg_sequence = 0
        # else:

        try:
            audio_stream.header.seq = msg_sequence
            audio_stream.header.frame_id = device_name
            audio_stream.header.stamp = rospy.Time.now()

            byte_buff = stream.read(CHUNK)
            int16_buff = map(ord, byte_buff)
            audio_stream.data = int16_buff
            scan_pub.publish(audio_stream)
            msg_sequence += 1

            # print(byte_buff[:20])
            # print(int16_buff[:20])
        except IOError, e:
            print("\x1b[1;31m[Error Massage] : %s\x1b[1;m" % e)
            stream.close()
            stream = make_stream(audio_interface, format, INPUT_DEVICE, CHANNELS, RATE, CHUNK)

        r.sleep()


def make_stream(audio_interface, format, input_device_index, channels, rate, frames_per_buffer, input=True,
                output=True):
    stream = audio_interface.open(format=format,
                                  input_device_index=input_device_index,
                                  channels=channels,
                                  frames_per_buffer=frames_per_buffer,
                                  rate=rate,
                                  input=input,
                                  output=output
                                  )

    return stream


if __name__ == '__main__':
    main()
