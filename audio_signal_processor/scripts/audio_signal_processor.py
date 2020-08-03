#!/usr/bin/python3.5
# -*- coding: utf-8 -*-
# from __future__ import division

import numpy as np
# import parselmouth
import rospy
from audio_msgs.msg import AudioData, FeatureData
import sounddevice as sd
from modules import etc_functions as ef


time_step = 0.05
pitch_floor = 75.0
max_number_of_candidates = 15
very_accurate = True
silence_threshold = 0.03
voicing_threshold = 0.45
octave_cost = 0.01
octave_jump_cost = 0.35
voiced_unvoiced_cost = 0.14
ceiling = 600.0

LOOP_RATE = 10



class AudioSignalProcessor:
    # Calibration pair distance parameter
    def __init__(self):
        rospy.init_node("audio_signal_processor", anonymous=False)
        rospy.Subscriber("audio_stream", AudioData, self.callback_packet, queue_size=50)
        self.pub_voice_feature = rospy.Publisher("voice_feature", FeatureData, queue_size=50)
        self.wav_array = np.array([])
        self.voice_feature = FeatureData()
        rospy.spin()

    def callback_packet(self, topic):
        audio_stream = topic.data
        msg_sequence = topic.header.seq
        byte_str = ef.make_byte_str(audio_stream)
        wav_raw = ef.make_wav_raw(byte_str)
        self.wav_array = np.append(self.wav_array, wav_raw)

        if msg_sequence > 0 and msg_sequence % LOOP_RATE == 0:

            amplitude = ef.amp_calculation(self.wav_array)
            pitch = ef.pitch_calculation(self.wav_array)
            intensity = ef.intensity_calculation(self.wav_array)

            self.voice_feature.header.seq = msg_sequence
            self.voice_feature.header.frame_id = "voice_feature"
            self.voice_feature.header.stamp = rospy.Time.now()
            self.voice_feature.amplitude = amplitude[:30]
            self.voice_feature.pitch = pitch
            self.voice_feature.intensity = intensity
            human_speech = rospy.get_param("perception/is_speaking_human/data")

            if human_speech is True:
                self.pub_voice_feature.publish(self.voice_feature)
                ef.play_audio(amplitude)
            self.wav_array = np.array([])





if __name__ == '__main__':
    try:
        AudioSignalProcessor()
    except rospy.ROSInterruptException:
        pass
