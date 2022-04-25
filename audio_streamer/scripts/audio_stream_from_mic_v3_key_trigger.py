#!/usr/local/bin/python3.6
# -*- coding: utf-8 -*-

import pyaudio
import rospy
from audio_msgs.msg import AudioData
import numpy as np
from termcolor import colored
from pynput import mouse, keyboard
import noisereduce as nr
import time

CHANNELS = 1
SAMPLING_FREQUENCY = 44100
LOOP_RATE = 5
# PCM_FORMAT = "int16"
DEVICE_NAME = "USB Audio Device"
OUTPUT_PLAY = False
key_state = "released"

C_BOLD = "\033[1m"
C_GREEN = "\033[32m"
C_RED = "\033[31m"
C_END = "\033[0m"
# ○ ●

off_icon = C_BOLD + C_RED + "○" + C_END
on_icon = C_BOLD + C_GREEN + "●" + C_END


def on_click(x, y, button, pressed):
    global key_state
    # print("Pressed")
    key_state = "pressed"
    # if button == mouse.Button.left:
    # return False

    if not pressed:
        # print("Released")
        key_state = "released"
        # return False


def on_press(key):
    global key_state
    if str(key) == "Key.page_down" or str(key) == "Key.page_up":
        # print(key)
        key_state = "pressed"


def on_release(key):
    global key_state
    # print("Pressed")
    key_state = "released"


def main():
    rospy.init_node('audio_streamer')
    scan_pub = rospy.Publisher('audio_stream', AudioData, queue_size=100)

    audio_streamer_conf = {
        "mic_volume": 30,
        "playback_sound": False,
        "noise_reduce": True,
        "key_control": True,
        "off_volume": 0.,
        "last_on_time": 0.
    }
    rospy.set_param("/audio_streamer/conf", audio_streamer_conf)

    device_name = get_setting_from_launch("device_name", DEVICE_NAME)
    channels = int(get_setting_from_launch("channels", CHANNELS))
    sampling_frequency = int(get_setting_from_launch("sampling_frequency", SAMPLING_FREQUENCY))
    loop_rate = int(get_setting_from_launch("loop_rate", LOOP_RATE))
    # pcm_format = get_setting_from_launch("pcm_format", PCM_FORMAT)
    chunk = int(sampling_frequency / loop_rate)  # 100ms
    audio_interface = pyaudio.PyAudio()
    # pa_format = get_pa_format(pcm_format)
    dvc_idx = get_device_idx(audio_interface, device_name)
    if dvc_idx != 0:
        print(colored("[Binding Information] ", 'blue', attrs=['bold']) + "Device Name : {}, Channels : {} , Sampling Frequency : {}, Loop Rate : {}".format(device_name, channels, sampling_frequency, loop_rate))
    else:
        print(colored("[Binding Information] ", 'blue', attrs=['bold']) + colored("Binding Failure", 'red', attrs=['bold']))

    r = rospy.Rate(LOOP_RATE)

    # print("[Binding Information] " + colored("Device Name : {}, Channels : {} , Sampling Frequency : {}, Loop Rate : {}".format(device_name, channels, sampling_frequency, loop_rate), 'blue', attrs=['bold']))
    stream = make_stream(audio_interface, pyaudio.paInt16, dvc_idx, channels, sampling_frequency, chunk)
    msg_sequence = 0
    audio_stream = AudioData()
    listener2 = keyboard.Listener(on_press=on_press, on_release=on_release)

    # listener1.start()
    listener2.start()
    while not rospy.is_shutdown():
        try:
            byte_buff = stream.read(chunk)
            int16_buff = np.fromstring(byte_buff, dtype=np.int16)

            audio_conf = rospy.get_param("/audio_streamer/conf")

            sound_level = (int(audio_conf['mic_volume']) / 100.)
            int16_buff = int16_buff * sound_level
            int16_buff = int16_buff.astype(np.int16)

            if audio_conf["noise_reduce"] is True:
                int16_buff = nr.reduce_noise(int16_buff, sr=SAMPLING_FREQUENCY)

            audio_stream.header.seq = msg_sequence
            audio_stream.header.frame_id = DEVICE_NAME
            audio_stream.header.stamp = rospy.Time.now()
            msg_sequence += 1
            # scan_pub.publish(audio_stream)

            if audio_conf["key_control"] is True:
                if key_state == "pressed":
                    sound_level = (int(audio_conf['mic_volume']) / 100.)
                    int16_buff = int16_buff * sound_level
                    int16_buff = int16_buff.astype(np.int16)

                    print("\n" * 40)
                    print("MiC Streaming... {}".format(on_icon))
                    audio_stream.data = int16_buff

                    scan_pub.publish(audio_stream)
                    if audio_conf["playback_sound"] is True:
                        playback_buff = int16_buff.astype(np.int16).tobytes()
                        stream.write(playback_buff)

                    # mem_key_control = True
                    rospy.set_param("/audio_streamer/conf/last_on_time", time.time())

                else:
                    last_on_time = rospy.get_param("/audio_streamer/conf/last_on_time", time.time())
                    if float(time.time()) - float(last_on_time) < 2.:
                        sound_level = (int(audio_conf['mic_volume']) / 100.)
                        int16_buff = int16_buff * sound_level
                        int16_buff = int16_buff.astype(np.int16)

                        print("\n" * 40)
                        print("MiC Streaming... {}".format(on_icon))
                        audio_stream.data = int16_buff

                        scan_pub.publish(audio_stream)
                        if audio_conf["playback_sound"] is True:
                            playback_buff = int16_buff.astype(np.int16).tobytes()
                            stream.write(playback_buff)
                    else:

                        sound_level = (audio_conf['off_volume'] / 100.)
                        int16_buff = int16_buff * sound_level
                        int16_buff = int16_buff.astype(np.int16)

                        if audio_conf['off_volume'] != 0:
                            audio_stream.data = int16_buff

                        else:
                            audio_stream.data = [0] * 8820

                        print("\n" * 40)
                        print("MiC Off          {}".format(off_icon))

                    scan_pub.publish(audio_stream)

            else:
                audio_stream.data = int16_buff
                scan_pub.publish(audio_stream)

                if audio_conf["playback_sound"] is True:
                    playback_buff = int16_buff.astype(np.int16).tobytes()
                    stream.write(playback_buff)

        except IOError as e:
            stream.close()
            stream = make_stream(audio_interface, pyaudio.paInt16, dvc_idx, channels, sampling_frequency, chunk)

        r.sleep()


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
