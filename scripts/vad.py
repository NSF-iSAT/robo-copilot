import sys
import time
import sounddevice as sd
import numpy as np # required to avoid crashing in assigning the callback input which is a numpy object
import webrtcvad

# SOURCE: viduraakalanka on github https://github.com/viduraakalanka

channels = [4]
# translate channel numbers to be 0-indexed
mapping  = [c - 1 for c in channels]

# get the default audio input device and its sample rate
device_info = sd.query_devices(None, 'input')
# sample_rate = int(device_info['default_samplerate'])
sample_rate = 48000

interval_size = 30 # audio interval size in ms
downsample = 1

block_size = sample_rate * interval_size / 1000

# get an instance of webrtc's voice activity detection
vad = webrtcvad.Vad(2)

print("reading audio stream from default audio input device:\n" + str(sd.query_devices()) + '\n')
print(F"audio input channels to process: {channels}")
print(F"sample_rate: {sample_rate}")
print(F"window size: {interval_size} ms")
print(F"datums per window: {block_size}")
print()


def voice_activity_detection(audio_data):
    return vad.is_speech(audio_data, sample_rate)


def audio_callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(F"underlying audio stack warning:{status}", file=sys.stderr)

    assert frames == block_size
    audio_data = indata[::downsample, mapping]        # possibly downsample, in a naive way
    audio_data = map(lambda x: (x+1)/2, audio_data)   # normalize from [-1,+1] to [0,1], you might not need it with different microphones/drivers
    audio_data = np.fromiter(audio_data, np.float16)  # adapt to expected float type

    # uncomment to debug the audio input, or run sounddevice's mic input visualization for that
    #print(f'{sum(audio_data)} \r', end="")
    #print(f'min: {min(audio_data)}, max: {max(audio_data)}, sum: {sum(audio_data)}')

    audio_data = audio_data.tobytes()
    detection = voice_activity_detection(audio_data)
    print(f'{detection} \r', end="") # use just one line to show the detection status (speech / not-speech)


with sd.InputStream(
    device=None,  # the default input device
    channels=max(channels),
    samplerate=sample_rate,
    blocksize=int(block_size),
    callback=audio_callback):

    # avoid shutting down for endless processing of input stream audio
    while True:
        time.sleep(0.1)  # intermittently wake up