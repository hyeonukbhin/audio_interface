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
import json

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

class AudioSignalProcessor:
    # Calibration pair distance parameter
    def __init__(self):
        rospy.init_node("audio_signal_processor", anonymous=False)
        rospy.Subscriber("audio_stream", AudioData, self.callback_packet, queue_size=50)
        self.pub_request = rospy.Publisher("voice_feature", String, queue_size=50)
        self.buff = queue.Queue()
        self.count = 0
        self.wav_array = np.array([])
        rospy.spin()

    def callback_packet(self, packetData):
        # global callerSpeech, idx, callerArray, frameIdx, endFlag, lastFrameIdx, count, wav_array
        callerSpeech = packetData.data
        byte_str = self.make_byte_str(callerSpeech)
        self.buff.put(byte_str)
        wav_r = np.fromstring(self.buff.get(), dtype=np.int16).astype(np.float64) / 32768

        if self.count >= 20:
            amplitude = self.amp_calculation(self.wav_array)
            pitch = self.pitch_calculation(self.wav_array)
            intensity = self.intensity_calculation(self.wav_array)
            self.play_audio(amplitude)
            self.count = 0
            self.wav_array = np.array([])
        else:
            self.wav_array = np.append(self.wav_array, wav_r)
            self.count += 1

    def make_byte_str(self, int16Array):
        byte_str = "".join(map(chr, int16Array))
        return byte_str

    def amp_calculation(self, wav_array):
        snd = parselmouth.Sound(wav_array)
        return snd.values.T

    def pitch_calculation(self, wav_array):
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

    def intensity_calculation(self, wav_array):
        snd = parselmouth.Sound(wav_array)
        intensity = snd.to_intensity(minimum_pitch=pitch_floor,
                                     time_step=time_step,
                                     subtract_mean=True)

        intensity_values_valuses = np.array(intensity.values.T)
        intensity_xs = np.round(np.array(intensity.xs()), 4)

        return intensity_values_valuses

    def interval_calculation(self, wav_array):
        snd = parselmouth.Sound(wav_array)
        return ""

    def play_audio(self, myarray):
        sd.default.samplerate = 44100
        # sd.default.device =
        sd.play(myarray)



if __name__ == '__main__':
    try:
        AudioSignalProcessor()
    except rospy.ROSInterruptException:
        pass