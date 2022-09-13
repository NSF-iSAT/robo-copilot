import sys
import time
import sounddevice as sd
import numpy as np # required to avoid crashing in assigning the callback input which is a numpy object
import webrtcvad
import rospy

from std_msgs.msg import Bool
from robo_copilot.msg import Speech

class VADNode:
    def __init__(self):
        rospy.init_node('vad_node', anonymous=True)

        self.audio_idx          = rospy.get_param('~audio_idx', 6)
        self.sample_rate        = rospy.get_param('~sample_rate', 48000)
        self.vad_filter_level   = rospy.get_param('~vad_filter_level', 3)

        channels = [2]
        self.mapping = [c - 1 for c in channels]

        rospy.loginfo("Available audio devices: \n" + str(sd.query_devices()))
        rospy.loginfo("Using audio device: " + str(sd.query_devices(self.audio_idx, 'input')))
        rospy.loginfo("Available audio devices: ")
        rospy.loginfo(sd.query_devices())
        self.device_info = sd.query_devices(self.audio_idx, 'input')
        # self.sample_rate = int(self.device_info['default_samplerate'])
        self.interval_size = 30 # audio interval size in ms
        self.downsample = 1

        self.block_size = self.sample_rate * self.interval_size / 1000
        self.vad = webrtcvad.Vad(self.vad_filter_level)

        self.utterance_timeout = rospy.Duration(1.0)
        self.is_speaking = False

        self.detect_pub    = rospy.Publisher('/vad/speech_detected', Bool, queue_size=1)
        self.utterance_pub = rospy.Publisher('/vad/utterance', Speech, queue_size=2)

        with sd.InputStream(
            device=self.audio_idx,
            channels=max(channels),
            samplerate=self.sample_rate,
            blocksize=int(self.block_size),
            callback=self.audio_callback):

            rospy.loginfo(F"audio input channels to process: {channels}")
            rospy.loginfo(F"sample_rate: {self.sample_rate}")
            rospy.loginfo(F"window size: {self.interval_size} ms")
            rospy.loginfo(F"datums per window: {self.block_size}")
            
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
        #rospy.loginfo(f'{sum(audio_data)} \r', end="")
        #rospy.loginfo(f'min: {min(audio_data)}, max: {max(audio_data)}, sum: {sum(audio_data)}')

        audio_data = audio_data.tobytes()
        detection = self.voice_activity_detection(audio_data)
        # rospy.loginfo(f'{detection} \r') # use just one line to show the detection status (speech / not-speech)
        self.detect_pub.publish(detection)
        if detection:
            if not self.is_speaking:
                self.is_speaking = True
                self.utterance_start_time = rospy.Time.now()
                self.utterance_pub.publish(Speech(Speech.STARTED, 0))
                rospy.loginfo("Utterance STARTED")
            self.last_heard = rospy.Time.now()

        elif self.is_speaking and (rospy.Time.now() - self.last_heard) > self.utterance_timeout:
            rospy.loginfo("Utterance STOPPED")
            duration = (self.last_heard - self.utterance_start_time).to_sec()
            self.utterance_pub.publish(Speech(Speech.STOPPED, duration))
            self.is_speaking = False



if __name__ == "__main__":
    vad = VADNode()