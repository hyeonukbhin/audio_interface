#!/usr/bin/python3

import numpy as np
import pyaudio
import socket
import select
import noisereduce as nr

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 4096
audio = pyaudio.PyAudio()

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind(('127.0.0.1', 4444))
serversocket.listen(5)


def callback(in_data, frame_count, time_info, status):
    for s in read_list[1:]:
        s.send(in_data)
    return (None, pyaudio.paContinue)


# start Recording
stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK, stream_callback=callback)
# byte_buff = stream.read(CHUNK)
# int16_buff = np.fromstring(byte_buff, dtype=np.int16) / 1.0
# reduced_noise_data = nr.reduce_noise(int16_buff, sr=RATE)

# stream = reduced_noise_data.astype(np.int16).tobytes()

# stream.start_stream()


# def __init__(self, output_to_file=True):
#     self.vad = webrtcvad.Vad(int(self.vad_aggressiveness))
#     self.deepspeech_model = self.load_deepspeech_model()
#     self.noise_sample_data = self.load_noise_sample_data()

# def reduce_audio_noise(self, data: bytes) -> bytes:
#     np_data = np.frombuffer(data, np.int16) / 1.0
#     reduced_noise_data = reduce_noise(audio_clip=np_data, noise_clip=self.noise_sample_data)
#     return reduced_noise_data.astype(np.int16).tobytes()

# def load_noise_sample_data(self) -> np.ndarray:
#     path = os.path.join(os.path.dirname(__file__), "../../../assets/deepspeech/noise_sample.wav")
#     with wave.open(path, "rb") as wf:
#         frames = wf.getnframes()
#         return np.frombuffer(wf.readframes(frames), np.int16) / 1.0


read_list = [serversocket]
print("recording...")

try:
    while True:
        readable, writable, errored = select.select(read_list, [], [])
        for s in readable:
            if s is serversocket:
                (clientsocket, address) = serversocket.accept()
                read_list.append(clientsocket)
                print("Connection from", address)
            else:
                data = s.recv(1024)
                if not data:
                    read_list.remove(s)
except KeyboardInterrupt:
    pass


print("finished recording")

serversocket.close()
# stop Recording
stream.stop_stream()
stream.close()
audio.terminate()
