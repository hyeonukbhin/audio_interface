#!/usr/bin/python2
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
import parselmouth
import rospy
from audio_msgs.msg import AudioData
from six.moves import queue
from std_msgs.msg import String
import sounddevice as sd

wav_array = np.array([])
count = 0

time_step = 0.1
pitch_floor = 75.0
max_number_of_candidates = 15
very_accurate = True
silence_threshold = 0.03
voicing_threshold = 0.45
octave_cost = 0.01
octave_jump_cost = 0.35
voiced_unvoiced_cost = 0.14
ceiling = 600.0


def main():
    global pub
    buff = queue.Queue()
    rospy.init_node('audio_signal_processor', anonymous=None)
    pub = rospy.Publisher('sound_information', String, queue_size=100)
    rospy.Subscriber("audio_stream", AudioData, callback_packet, buff)
    rospy.spin()


def callback_packet(packetData, buff):
    global callerSpeech, idx, callerArray, frameIdx, endFlag, lastFrameIdx, count, wav_array
    callerSpeech = packetData.data
    byte_str = make_byte_str(callerSpeech)
    buff.put(byte_str)
    wav_r = np.fromstring(buff.get(), dtype=np.int16).astype(np.float64) / 32768

    if count >= 20:
        amplitude = amp_calculation(wav_array)
        pitch = pitch_calculation(wav_array)
        intensity = intensity_calculation(wav_array)
        print(amplitude)
        print(len(amplitude))
        play_audio(amplitude)
        # print(pitch)
        # print(len(pitch))
        # print(intensity)
        # print(len(intensity))
        count = 0
        wav_array = np.array([])
    else:
        wav_array = np.append(wav_array, wav_r)
        count += 1


def make_byte_str(int16Array):
    byte_str = "".join(map(chr, int16Array))
    return byte_str

def amp_calculation(wav_array):
    snd = parselmouth.Sound(wav_array)
    return snd.values.T

def pitch_calculation(wav_array):
    snd = parselmouth.Sound(wav_array)
    pitch = snd.to_pitch_ac(time_step=time_step,
                            pitch_floor=pitch_floor,
                            max_number_of_candidates=max_number_of_candidates,
                            very_accurate=very_accurate,
                            silence_threshold=silence_threshold,
                            voicing_threshold=voicing_threshold,
                            octave_cost=octave_cost,
                            octave_jump_cost=octave_jump_cost,
                            voiced_unvoiced_cost=voiced_unvoiced_cost,
                            pitch_ceiling=ceiling
                            )
    pitch_valuses = np.array(pitch.selected_array['frequency'])
    pitch_xs = np.round(np.array(pitch.xs()), 4)

    return pitch_valuses


def intensity_calculation(wav_array):
    snd = parselmouth.Sound(wav_array)
    intensity = snd.to_intensity(minimum_pitch=pitch_floor,
                                 time_step=time_step,
                                 subtract_mean=True)

    intensity_values_valuses = np.array(intensity.values.T)
    intensity_xs = np.round(np.array(intensity.xs()), 4)

    return intensity_values_valuses


def interval_calculation(wav_array):
    snd = parselmouth.Sound(wav_array)
    return ""

def play_audio(myarray):
    sd.default.samplerate = 44100
    # sd.default.device =
    sd.play(myarray)


if __name__ == '__main__':
    main()
