import random
import rospy
from misty_wrapper.msg import MoveHead, MoveArm, MoveArms
from gaze_tracking_ros.msg import GazeState
from ros_speech2text.msg import Event
from std_msgs.msg import String

# idk about some of these parameters, I just based it on https://github.com/MistySampleSkills/Misty-Concierge-Template/blob/master/JavaScript/conciergeBaseTemplate/conciergeBaseTemplate.js

# movement limits, in degrees
PITCH_UP = -35
PITCH_DOWN = 20
YAW_LEFT = 70
YAW_RIGHT = -70
ROLL_LEFT = -35
ROLL_RIGHT = 35

# BEAR_RANGE = 6 # -13 right and +13 left
# ELEVATION_RANGE = 12 # -13 up and +13 down
WIDTH_RANGE = 120
HEIGHT_RANGE = 120

DELTA_MIN = 0.1

class GazeFollower:
    def __init__(self, misty_id, img_w, img_h):
        rospy.init_node("gaze_follower", anonymous=True)

        self.img_dims = (img_w, img_h)

        self.head_roll  = 0
        self.head_pitch = 0
        self.head_yaw   = 0

        self.do_idle = True
        self.timeout = rospy.Duration(1.5)
        self.reset_timeout = rospy.Duration(10)
        self.idle_timeout = rospy.Duration(5)
        self.joint_attn_timeout = rospy.Duration(5)
        self.joint_attn_offset = 20 # degrees, yaw

        self.gaze_last_seen = rospy.Time.now()
        self.gaze_last_changed = rospy.Time.now()
        self.movement_start = rospy.Time.now()

        self.look_dir = 1 # for when scanning for faces

        # pubs and subs
        gaze_sub = rospy.Subscriber("/gaze_state", GazeState, self.gaze_callback)
        speech_sub = rospy.Subscriber("speech_to_text/log", Event, self.speaking_callback)
        self.head_pub = rospy.Publisher("/misty/id_0/head", MoveHead, queue_size=1)
        self.face_pub = rospy.Publisher("/misty/id_0/face_img", String, queue_size=1)
        self.arms_pub = rospy.Publisher("/misty/id_0/arms", MoveArms, queue_size=1)
        self.speech_pub = rospy.Publisher("/misty/id_0/speech", String, queue_size=1)

        # reset head position
        rospy.sleep(5)
        self.reset()

        # is user actively speaking?
        self.is_listening = False
        # should robot do idle behaviors?
        self.do_idle = True
        self.spoken_backchannel_options = [
            "Hmmmm",
            "I see",
        ]

        while not rospy.is_shutdown():
            # print("looping: ", (rospy.Time.now() - self.movement_start).to_sec())
            if self.is_listening:
                self.do_idle = False

            elif rospy.Time.now() - self.gaze_last_seen > self.reset_timeout and rospy.Time.now() - self.movement_start > self.timeout:
                self.look_for_face()

            # elif rospy.Time.now() - self.movement_start > self.idle_timeout :
            #     self.do_idle_motion()

            # elif self.listening:
            #     # TODO add more interesting things to do when listening
            #     self.do_idle_motion()

            rospy.sleep(self.timeout)
    
    def spoken_backchannel(self):
        bc = random.choice(self.spoken_backchannel_options)
        self.speech_pub.publish(bc)

    def arm_waggle(self):
        l_arm = MoveArm(value=10, velocity=100)
        r_arm = MoveArm(value=0, velocity=100)
        self.arms_pub.publish(MoveArms(leftArm=l_arm, rightArm=r_arm))

        rospy.sleep(0.5)

        l_arm = MoveArm(value=0, velocity=100)
        r_arm = MoveArm(value=10, velocity=100)
        self.arms_pub.publish(MoveArms(leftArm=l_arm, rightArm=r_arm))

        rospy.sleep(1.0)

        straight_arm = MoveArm(value=5, velocity=50)
        self.arms_pub.publish(MoveArms(leftArm=straight_arm, rightArm=straight_arm))

    def nod(self):
        pitch_factor = -20
        if self.head_pitch + 20 <= PITCH_DOWN:
            pitch_factor = 20

        self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=self.head_pitch-pitch_factor, yaw=self.head_yaw,
            velocity=100, units="degrees"))
        self.movement_start = rospy.Time.now()

        rospy.sleep(1.0)
        self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=self.head_pitch, yaw=self.head_yaw,
            velocity=100, units="degrees"))
        self.movement_start = rospy.Time.now()

    def speaking_callback(self, msg):
        if msg.event == msg.STARTED:
            self.is_listening = True
            # tilt head and change expression to indicate listening
            self.head_pub.publish(MoveHead(roll=self.head_roll+(25 * random.choice([-1, 1])), pitch=self.head_pitch, yaw=self.head_yaw, velocity=90))
            self.face_pub.publish(String("e_Joy.jpg"))

        elif msg.event == msg.STOPPED:
            self.is_listening = False
            # reset head position to original (before tilt)
            self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=self.head_pitch, yaw=self.head_yaw, velocity=90))
            self.face_pub.publish(String("e_DefaultContent.jpg"))
            self.nod()
            self.arm_waggle()

    def look_for_face(self):
        yaw = self.head_yaw
        if abs(yaw + 30 * self.look_dir) > 75:
            self.look_dir *= -1

        yaw = yaw + (30 * self.look_dir)

        self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=0, yaw=yaw, velocity = 95, units="degrees"))
        self.head_yaw = yaw
        self.head_pitch = 0

    def do_idle_motion(self):
        id = random.randint(0, 3)
        if id == 0:
            self.head_pub.publish(MoveHead(roll=20, pitch=self.head_pitch, yaw=self.head_yaw, velocity=100, units="degrees"))
            rospy.sleep(self.timeout)
            self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=self.head_pitch, yaw=self.head_yaw, velocity=100))

        elif id == 1:
            self.face_pub.publish(String("e_Joy.jpg"))
            rospy.sleep(self.timeout)
            self.face_pub.publish(String("e_DefaultContent.jpg"))

        elif id == 2:
            self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=-10, yaw=self.head_yaw, velocity=90))
            rospy.sleep(self.timeout)
            self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=self.head_pitch, yaw=self.head_yaw, velocity=90))
            self.movement_start = rospy.Time.now()

        self.movement_start = rospy.Time.now()

    def reset(self):
        self.head_pub.publish(MoveHead(roll=0, pitch=0, yaw=0,
            velocity=100, units="degrees"))
        self.head_yaw = 0
        self.head_pitch = 0
        self.head_roll = 0
        self.movement_start = rospy.Time.now()

    def reset(self):
        self.head_pub.publish(MoveHead(roll=0, pitch=0, yaw=0,
            velocity=100, units="degrees"))
        self.head_yaw = 0
        self.head_pitch = 0
        self.head_roll = 0
        self.movement_start = rospy.Time.now()

    def gaze_callback(self, msg):
        self.gaze_last_seen = rospy.Time.now()
        if rospy.Time.now() - self.movement_start > self.timeout:
            avg_pupil_x = (msg.pupil_left_coords.x + msg.pupil_right_coords.x) / 2.0
            avg_pupil_y = (msg.pupil_left_coords.y + msg.pupil_right_coords.y) / 2.0
            avg_pupil_x = avg_pupil_x / self.img_dims[0] - 0.5
            avg_pupil_y = avg_pupil_y / self.img_dims[1] - 0.5

            # calculate head movement
            pitch = self.head_pitch
            yaw = self.head_yaw

            if (abs(avg_pupil_x) + abs(avg_pupil_y)) > DELTA_MIN:
                # self.head_yaw -= ((YAW_LEFT - YAW_RIGHT)/33 * (avg_pupil_x*BEAR_RANGE))
                # self.head_pitch += ((PITCH_DOWN - PITCH_UP)/33 * (avg_pupil_y*ELEVATION_RANGE))
                # # self.head_pub.publish(MoveHead(roll = self.head_roll, pitch = self.head_pitch + ((PITCH_DOWN - PITCH_UP)/33 * (avg_pupil_y*13)), velocity=10.0,
                #     # yaw = self.head_yaw - ((YAW_LEFT - YAW_RIGHT)/33 * (avg_pupil_x*13)), duration=1, units="degrees"))
                self.head_yaw -= (WIDTH_RANGE / 2 * avg_pupil_x)
                self.head_pitch += (HEIGHT_RANGE / 2 * avg_pupil_y)

                if self.head_yaw < YAW_RIGHT:
                    self.head_yaw = YAW_RIGHT
                elif self.head_yaw > YAW_LEFT:
                    self.head_yaw = YAW_LEFT

                if self.head_pitch > PITCH_DOWN:
                    self.head_pitch = PITCH_DOWN
                elif self.head_pitch < PITCH_UP:
                    self.head_pitch = PITCH_UP

                self.head_pub.publish(MoveHead(roll = self.head_roll, pitch = self.head_pitch, yaw = self.head_yaw,
                    velocity=100, units="degrees"))
                self.movement_start = rospy.Time.now()
            
if __name__ == "__main__":
    GazeFollower(0, 480, 640)