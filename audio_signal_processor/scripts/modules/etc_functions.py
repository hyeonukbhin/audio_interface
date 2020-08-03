#!/usr/bin/python3.5
# -*- coding: utf-8 -*-

import numpy as np
import parselmouth
import rospy
import sounddevice as sd


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


def convert_db(data):
    np.seterr(divide='ignore')
    data_abs = np.abs(data)
    output = 20 * np.log10(data_abs)
    return output


def make_byte_str(int16Array):
    byte_str = "".join(map(chr, int16Array))
    return byte_str


def make_wav_raw(byte_str):
    # wav_raw = np.fromstring(byte_str, dtype=np.int16).astype(np.float64) / 32768
    # wav_raw = np.fromstring(byte_str.encode('raw_unicode_escape'), dtype=np.int16) / 32768
    wav_raw = np.fromstring(byte_str.encode('raw_unicode_escape'), dtype=np.int16)
    # fromstring(data.encode('raw_unicode_escape'), dtype='int32')

    return wav_raw


def get_param(param_name=""):
    try:
        output_values = rospy.get_param(param_name)
    except KeyError as e:
        output_values = ""
    if output_values == "None" or None:
        output_values = ""
    return output_values





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
    pitch_values = np.array(pitch.selected_array['frequency'])
    pitch_xs = np.round(np.array(pitch.xs()), 4)

    return pitch_values

def intensity_calculation(wav_array):
    snd = parselmouth.Sound(wav_array)
    intensity = snd.to_intensity(minimum_pitch=pitch_floor,
                                 time_step=time_step,
                                 subtract_mean=True)

    intensity_values = np.array(intensity.values.T)
    intensity_xs = np.round(np.array(intensity.xs()), 4)

    return intensity_values

def interval_calculation(wav_array):
    snd = parselmouth.Sound(wav_array)
    return ""

def play_audio(myarray):
    sd.default.samplerate = 44100
    # sd.default.device =
    sd.play(myarray)

