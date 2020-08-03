#!/usr/bin/python2
# -*- coding: utf-8 -*-

import pyaudio
import rospy
from audio_msgs.msg import AudioData
import numpy as np
from termcolor import colored

CHANNELS = 1
SAMPLING_FREQUENCY = 44100
LOOP_RATE = 5
# PCM_FORMAT = "int16"
DEVICE_NAME = "USB Audio Device"
OUTPUT_PLAY = False


def main():
    rospy.init_node('audio_streamer')
    scan_pub = rospy.Publisher('audio_stream', AudioData, queue_size=100)

    device_name = get_setting_from_launch("device_name", DEVICE_NAME)
    channels = int(get_setting_from_launch("channels", CHANNELS))
    sampling_frequency = int(get_setting_from_launch("sampling_frequency", SAMPLING_FREQUENCY))
    loop_rate = int(get_setting_from_launch("loop_rate", LOOP_RATE))
    # pcm_format = get_setting_from_launch("pcm_format", PCM_FORMAT)
    chunk = int(sampling_frequency / loop_rate)  # 100ms

    audio_interface = pyaudio.PyAudio()
    # pa_format = get_pa_format(pcm_format)
    dvc_idx = get_device_idx(audio_interface, device_name)
    r = rospy.Rate(LOOP_RATE)

    # print("[Binding Information] " + colored("Device Name : {}, Channels : {} , Sampling Frequency : {}, Loop Rate : {}".format(device_name, channels, sampling_frequency, loop_rate), 'blue', attrs=['bold']))
    print(colored("[Binding Information] ", 'blue', attrs=['bold']) + "Device Name : {}, Channels : {} , Sampling Frequency : {}, Loop Rate : {}".format(device_name, channels, sampling_frequency, loop_rate))
    stream = make_stream(audio_interface, pyaudio.paInt16, dvc_idx, channels, sampling_frequency, chunk)
    msg_sequence = 0
    audio_stream = AudioData()
    while not rospy.is_shutdown():
        try:
            byte_buff = stream.read(chunk)
            int16_buff = np.fromstring(byte_buff, dtype=np.int16)
            audio_stream.header.seq = msg_sequence
            audio_stream.header.frame_id = DEVICE_NAME
            audio_stream.header.stamp = rospy.Time.now()
            audio_stream.data = int16_buff
            msg_sequence += 1
            scan_pub.publish(audio_stream)

        except IOError as e:
            stream.close()
            stream = make_stream(audio_interface, pyaudio.paInt16, dvc_idx, channels, sampling_frequency, chunk)

        r.sleep()

# def play_audio(myarray):
#     sd.default.samplerate = 44100
#     # sd.default.device =
#     sd.play(myarray)


def make_stream(audio_interface, format, input_device_index, channels, rate, frames_per_buffer, input=True, output=True):
    stream = audio_interface.open(format=format,
                                  input_device_index=input_device_index,
                                  channels=channels,
                                  frames_per_buffer=frames_per_buffer,
                                  rate=rate,
                                  input=input,
                                  output=output)

    return stream


def get_pa_format(format_name):
    if format_name == "int16":
        pa_format = pyaudio.paInt16
    elif format_name == "paInt24":
        pa_format = pyaudio.paInt24
    elif format_name == "paInt32":
        pa_format = pyaudio.paInt32
    elif format_name == "paFloat32":
        pa_format = pyaudio.paFloat32
    else:
        pa_format = pyaudio.paInt16
    return pa_format


def get_setting_from_launch(arg_name, default_arg):

    try:
        output = rospy.get_param('~{}'.format(arg_name))
    except KeyError:
        output = default_arg

    return output




def get_device_idx(audio_interface, device_name):
    cnt_avl_dvc = audio_interface.get_host_api_info_by_index(0).get('deviceCount')

    output = 0
    print(colored("========== Available Alsa Mic Device List ==========", 'yellow', attrs=['bold']))
    for idx_dvc in range(cnt_avl_dvc):
        dvc_info = audio_interface.get_device_info_by_host_api_device_index(0, idx_dvc)
        dvc_info_channel = dvc_info.get('maxInputChannels')
        dvc_info_name = dvc_info.get('name')
        if dvc_info_channel > 0:
            print("Device ID : {}, {}".format(idx_dvc, dvc_info_name))
            if device_name in dvc_info_name:
                # print("Device ID : {}, {}".format(colored(idx_dvc, 'white', attrs=['bold']), colored(dvc_info_name, 'yellow', attrs=['bold'])))
                output = idx_dvc
            # else:

    return output


if __name__ == '__main__':
    main()
