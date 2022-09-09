import sys
import time
import sounddevice as sd
import numpy as np # required to avoid crashing in assigning the callback input which is a numpy object
import webrtcvad

import rospy

from std_msgs.msg import Bool

class VADNode:
    def __init__(self):
        channels = [2]
        self.mapping = [c - 1 for c in channels]

        self.device_info = sd.query_devices(11, 'input')
        # self.sample_rate = int(self.device_info['default_samplerate'])
        self.sample_rate = 48000
        self.interval_size = 30 # audio interval size in ms
        self.downsample = 1

        self.block_size = self.sample_rate * self.interval_size / 1000
        self.vad = webrtcvad.Vad(1)

        self.pub = rospy.Publisher('/vad/speech_detected', Bool, queue_size=1)
        rospy.init_node('vad_node', anonymous=True)

        with sd.InputStream(
            device=11,
            channels=max(channels),
            samplerate=self.sample_rate,
            blocksize=int(self.block_size),
            callback=self.audio_callback):

            print("reading audio stream from default audio input device:\n" + str(sd.query_devices()) + '\n')
            print(F"audio input channels to process: {channels}")
            print(F"sample_rate: {self.sample_rate}")
            print(F"window size: {self.interval_size} ms")
            print(F"datums per window: {self.block_size}")
            print()
            
            while not rospy.is_shutdown():
                time.sleep(0.1)

    def voice_activity_detection(self, audio_data):
        return self.vad.is_speech(audio_data, self.sample_rate)
    
    def audio_callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(F"underlying audio stack warning:{status}", file=sys.stderr)

        assert frames == self.block_size
        audio_data = indata[::self.downsample, self.mapping]        # possibly downsample, in a naive way
        audio_data = map(lambda x: (x+1)/2, audio_data)   # normalize from [-1,+1] to [0,1], you might not need it with different microphones/drivers
        audio_data = np.fromiter(audio_data, np.float16)  # adapt to expected float type

        # uncomment to debug the audio input, or run sounddevice's mic input visualization for that
        #print(f'{sum(audio_data)} \r', end="")
        #print(f'min: {min(audio_data)}, max: {max(audio_data)}, sum: {sum(audio_data)}')

        audio_data = audio_data.tobytes()
        detection = self.voice_activity_detection(audio_data)
        print(f'{detection} \r', end="") # use just one line to show the detection status (speech / not-speech)
        self.pub.publish(detection)


if __name__ == "__main__":
    vad = VADNode()